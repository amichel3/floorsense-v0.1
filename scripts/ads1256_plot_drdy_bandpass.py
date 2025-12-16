#!/usr/bin/env python3
import argparse
import collections
import math
import statistics
import time

import numpy as np
import spidev, RPi.GPIO as GPIO
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Pins
DRDY_PIN_DEFAULT = 17  # BCM (physical pin 11)

FULL_SCALE_COUNTS = (1 << 23) - 1
GLITCH_ABS_COUNTS_DEFAULT = int(0.98 * FULL_SCALE_COUNTS)

# ADS1256 commands/registers
CMD_RESET=0xFE; CMD_WREG=0x50; CMD_RDATA=0x01; CMD_SDATAC=0x0F
CMD_SELFCAL=0xF0; CMD_SYNC=0xFC; CMD_WAKEUP=0x00
REG_STATUS=0x00; REG_MUX=0x01; REG_ADCON=0x02; REG_DRATE=0x03

def wr(s, r, v): s.xfer2([CMD_WREG | (r & 0x0F), 0x00, v]); time.sleep(0.001)
def to_i24(b0,b1,b2):
    v=(b0<<16)|(b1<<8)|b2
    if v & 0x800000: v -= 1<<24
    return v

def wait_drdy(pin: int, timeout=0.5):
    t0=time.time()
    while time.time()-t0 < timeout:
        if GPIO.input(pin)==0:
            return
        time.sleep(0.0002)
    raise TimeoutError("DRDY timeout")


def mux_from_mode(mode: str) -> int:
    # ADS1256 MUX: high nibble = positive input, low nibble = negative input.
    # 0x18 = AIN1 - AINCOM (single-ended)
    # 0x08 = AIN0 - AINCOM (single-ended)
    # 0x01 = AIN0 - AIN1  (differential)
    mode = mode.lower().strip()
    if mode in {"ain1_gnd", "ain1_aincom", "ch1"}:
        return 0x18
    if mode in {"ain0_gnd", "ain0_aincom", "ch0"}:
        return 0x08
    if mode in {"ain0_ain1", "diff01", "diff"}:
        return 0x01
    raise ValueError(f"unknown --input-mode '{mode}'")


def adcon_from_pga(pga: int) -> int:
    # ADS1256 ADCON: lower 3 bits set PGA: 1,2,4,8,16,32,64
    pga_map = {1: 0, 2: 1, 4: 2, 8: 3, 16: 4, 32: 5, 64: 6}
    if pga not in pga_map:
        raise ValueError("PGA must be one of: 1,2,4,8,16,32,64")
    return pga_map[pga]


parser = argparse.ArgumentParser(description="ADS1256 DRDY-synchronized plot with band-pass + RMS")
parser.add_argument("--drdy-pin", type=int, default=DRDY_PIN_DEFAULT, help="BCM pin for DRDY (default: 17)")
parser.add_argument("--input-mode", type=str, default="ain1_gnd", help="ain1_gnd | ain0_gnd | ain0_ain1")
parser.add_argument("--pga", type=int, default=1, help="ADC PGA gain: 1,2,4,8,16,32,64")
parser.add_argument("--sps", type=float, default=1000.0, help="Assumed sample rate for filters/plot scaling (default 1000)")
parser.add_argument("--hp", type=float, default=5.0, help="High-pass cutoff Hz (default 5)")
parser.add_argument("--lp", type=float, default=50.0, help="Low-pass cutoff Hz (default 50)")
parser.add_argument("--rms-ms", type=float, default=10.0, help="RMS window length in ms (default 10)")
parser.add_argument("--mean-sec", type=float, default=0.5, help="Moving mean window seconds for detrend (default 0.5)")
parser.add_argument("--glitch-abs", type=int, default=GLITCH_ABS_COUNTS_DEFAULT, help="Reject samples with abs(counts) >= this")
parser.add_argument("--no-glitch-reject", action="store_true", help="Disable glitch rejection (useful to confirm clipping)")
parser.add_argument("--score", action="store_true", help="Print simple event score (RMS/baseline) once per interval")
parser.add_argument("--score-interval", type=float, default=1.0, help="Seconds between score prints (default 1.0)")
parser.add_argument("--score-window-sec", type=float, default=0.25, help="Window seconds for current RMS stats (default 0.25)")
parser.add_argument("--score-baseline-sec", type=float, default=2.0, help="Window seconds for baseline RMS stats (default 2.0)")
parser.add_argument("--event", action="store_true", help="Count events when ratio_peak crosses a threshold")
parser.add_argument("--event-threshold", type=float, default=3.0, help="Trigger threshold on ratio_peak (default 3.0)")
parser.add_argument("--event-min-interval", type=float, default=0.5, help="Minimum seconds between events (default 0.5)")
parser.add_argument("--event-reset-threshold", type=float, default=1.5, help="Reset threshold on ratio_peak to end an event (default 1.5)")
parser.add_argument("--event-reset-hold", type=float, default=0.25, help="Seconds ratio_peak must stay below reset threshold to end an event (default 0.25)")
args = parser.parse_args()

# --- Setup GPIO + SPI ---
GPIO.setmode(GPIO.BCM)
GPIO.setup(args.drdy_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

s=spidev.SpiDev(); s.open(0,0); s.max_speed_hz=1_000_000; s.mode=0b01
s.xfer2([CMD_RESET]); time.sleep(0.05); s.xfer2([CMD_SDATAC]); time.sleep(0.001)
wr(s,REG_STATUS,0x03)   # MSB-first, BUFEN=1
wr(s,REG_MUX,   mux_from_mode(args.input_mode))
wr(s,REG_ADCON, adcon_from_pga(args.pga))
wr(s,REG_DRATE, 0xA1)   # ~1000 SPS
s.xfer2([CMD_SELFCAL]); time.sleep(0.1)
s.xfer2([CMD_SYNC]); time.sleep(0.001); s.xfer2([CMD_WAKEUP]); time.sleep(0.001)

# --- Filters: band-pass (first-order HPF + LPF) ---
FS = float(args.sps)
F_HP = float(args.hp)
F_LP = float(args.lp)
a_hp = math.exp(-2.0*math.pi*F_HP/FS); x1=0.0; y1=0.0
a_lp = math.exp(-2.0*math.pi*F_LP/FS); yl1=0.0
def bandpass_step(sample):
    global x1,y1,yl1
    yh = a_hp*(y1 + sample - x1); y1=yh; x1=sample
    yl = (1.0 - a_lp)*yh + a_lp*yl1; yl1=yl
    return yl

# Detrend raw (center around 0)
MEAN_WIN = max(1, int(float(args.mean_sec) * FS))
from collections import deque
mwin = deque(); msum=0.0
def detrend(v):
    global msum
    mwin.append(v); msum += v
    if len(mwin) > MEAN_WIN:
        msum -= mwin.popleft()
    mean = msum / len(mwin) if mwin else 0.0
    return v - mean

# --- Moving RMS smoother for filtered signal ---
RMS_MS = float(args.rms_ms)
RMS_N = max(1, int(FS * RMS_MS / 1000.0))
rwin = deque(); rsum_sq = 0.0
def rms_smooth(x):
    global rsum_sq
    rwin.append(x); rsum_sq += x*x
    if len(rwin) > RMS_N:
        old = rwin.popleft()
        rsum_sq -= old*old
    n = len(rwin) if rwin else 1
    return math.sqrt(max(0.0, rsum_sq) / n)

# --- Plot buffers ---
WINDOW_SEC = 2
N = int(FS*WINDOW_SEC)
buf_raw = collections.deque([0]*N, maxlen=N)
buf_bp  = collections.deque([0]*N, maxlen=N)
buf_rms = collections.deque([0]*N, maxlen=N)

plt.style.use("seaborn-v0_8-darkgrid")
fig,(ax1,ax2)=plt.subplots(2,1,figsize=(10,6),sharex=True)
lr, = ax1.plot(range(N), list(buf_raw), lw=1.0, color="#1f77b4", label="raw (detrended)")
ax1.set_ylabel("Counts"); ax1.legend(loc="upper left")
lf, = ax2.plot(range(N), list(buf_bp),  lw=1.0, color="#d62728", alpha=0.6, label=f"band-pass {F_HP:g}–{F_LP:g} Hz")
lrms,= ax2.plot(range(N), list(buf_rms),lw=1.6, color="#ff7f0e", label=f"RMS {int(RMS_MS)} ms")
levents = ax2.scatter([], [], marker='s', color='red', s=36, zorder=6)
ax2.set_xlabel("Samples (last ~2 s)"); ax2.set_ylabel("Filtered"); ax2.legend(loc="upper left")

def read_sample():
    wait_drdy(args.drdy_pin)
    s.xfer2([0x01])       # CMD_RDATA
    time.sleep(0.0005)    # t6 settle
    b0,b1,b2 = s.readbytes(3)
    return to_i24(b0,b1,b2)


glitch_count = 0
clip_count = 0
last_good = None

last_score_t = 0.0
event_count = 0

# Event grouping state (hysteresis)
in_event = False
event_peak_ratio = 0.0
event_peak_rms = 0.0
event_peak_index = 0
event_below_since = None
last_event_end_t = 0.0

# Track absolute sample index for plotting markers in the rolling window
sample_count = 0

event_markers = collections.deque(maxlen=200)  # (sample_index, event_id, peak_rms, peak_ratio)
event_texts = []

CHUNK=20   # ~20 ms/frame
def update(_):
    global glitch_count, clip_count, last_good, last_score_t, event_count
    global in_event, event_peak_ratio, event_peak_rms, event_peak_index, event_below_since, last_event_end_t
    global sample_count, event_texts
    for _ in range(CHUNK):
        v  = read_sample()

        if abs(v) >= FULL_SCALE_COUNTS:
            clip_count += 1

        if abs(v) >= args.glitch_abs:
            glitch_count += 1
            if (not args.no_glitch_reject) and (last_good is not None):
                v = last_good
        else:
            last_good = v

        vr = detrend(v)
        vf = bandpass_step(vr)
        vlo = rms_smooth(vf)
        buf_raw.append(vr)
        buf_bp.append(vf)
        buf_rms.append(vlo)
        sample_count += 1

    # --- Score + event detection (shared computation) ---
    rms_list = list(buf_rms)
    n_cur = max(1, int(args.score_window_sec * FS))
    n_base = max(1, int(args.score_baseline_sec * FS))

    # buf_rms starts pre-filled with zeros; ignore zeros so baseline doesn't start at 0.
    cur = [x for x in rms_list[-n_cur:] if x > 0]
    base = [x for x in rms_list[-n_base:] if x > 0]

    # Require enough baseline samples to avoid noisy/meaningless early ratios.
    baseline_ready = len(base) >= min(n_base, int(0.5 * FS))

    if baseline_ready:
        cur_mean = sum(cur) / len(cur) if cur else 0.0
        cur_peak = max(cur) if cur else 0.0
        base_med = statistics.median(base) if base else 0.0
        eps = 1e-9
        ratio_mean = cur_mean / max(base_med, eps)
        ratio_peak = cur_peak / max(base_med, eps)
    else:
        cur_mean = cur_peak = base_med = ratio_mean = ratio_peak = 0.0

    if args.score and baseline_ready and (time.time() - last_score_t) >= max(0.1, args.score_interval):
        last_score_t = time.time()
        print(
            f"score cur_mean={cur_mean:.2f} cur_peak={cur_peak:.2f} baseline_med={base_med:.2f} "
            f"ratio_mean={ratio_mean:.2f} ratio_peak={ratio_peak:.2f}",
            flush=True,
        )

    if args.event and baseline_ready:
        now = time.time()

        if not in_event:
            if (ratio_peak >= float(args.event_threshold)) and (now - last_event_end_t >= float(args.event_min_interval)):
                in_event = True
                event_peak_ratio = ratio_peak
                event_peak_rms = cur_peak
                event_peak_index = max(0, sample_count - 1)
                event_below_since = None
        else:
            # Track the strongest point during the event
            if ratio_peak >= event_peak_ratio:
                event_peak_ratio = ratio_peak
                event_peak_rms = cur_peak
                event_peak_index = max(0, sample_count - 1)

            # End event once ratio has been low for long enough
            if ratio_peak <= float(args.event_reset_threshold):
                if event_below_since is None:
                    event_below_since = now
                elif (now - event_below_since) >= float(args.event_reset_hold):
                    event_count += 1
                    last_event_end_t = now
                    event_markers.append((event_peak_index, event_count, event_peak_rms, event_peak_ratio))
                    print(
                        f"EVENT {event_count} peak_ratio={event_peak_ratio:.2f} peak_rms={event_peak_rms:.2f}",
                        flush=True,
                    )
                    in_event = False
                    event_below_since = None
            else:
                event_below_since = None

    # --- Render event markers in the current window ---
    window_start = max(0, sample_count - N)
    xs = []
    ys = []
    ids = []
    for idx, eid, peak_rms, peak_ratio in list(event_markers):
        if window_start <= idx < sample_count:
            xs.append(idx - window_start)
            ys.append(peak_rms)
            ids.append(eid)

    levents.set_offsets(np.column_stack((xs, ys)) if xs else np.empty((0, 2)))

    # Rebuild text labels each frame (small count; keeps logic simple)
    for t in event_texts:
        try:
            t.remove()
        except Exception:
            pass
    event_texts = []
    for x, y, eid in zip(xs, ys, ids):
        event_texts.append(ax2.text(x, y, str(eid), color='red', fontsize=9, ha='center', va='bottom'))

    y1=list(buf_raw); y2=list(buf_bp); y3=list(buf_rms)
    lr.set_ydata(y1)
    lf.set_ydata(y2)
    lrms.set_ydata(y3)
    for ax,y in ((ax1,y1),(ax2,y2)):
        ymin,ymax=min(y),max(y); margin=0.1*max(1,ymax-ymin)
        ax.set_ylim(ymin-margin, ymax+margin); ax.set_xlim(0,len(y1)-1)
    # ensure RMS trace is also in view
    ymin2=min(min(y2),min(y3)); ymax2=max(max(y2),max(y3)); margin2=0.1*max(1,ymax2-ymin2)
    ax2.set_ylim(ymin2-margin2, ymax2+margin2)

    fig.suptitle(
        f"DRDY=on | input={args.input_mode} | PGA={args.pga} | clip={clip_count} | glitches={glitch_count}"
        + (f" | events={event_count}" if args.event else "")
        + (" | glitch-reject=off" if args.no_glitch_reject else "")
    )
    return lr, lf, lrms

ani = animation.FuncAnimation(fig, update, interval=20, blit=False)
plt.tight_layout()
try:
    plt.show()
finally:
    try: s.xfer2([CMD_SDATAC]); time.sleep(0.001)
    except: pass
    s.close(); GPIO.cleanup()
