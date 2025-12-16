#!/usr/bin/env python3
import time, math, collections
import spidev, RPi.GPIO as GPIO
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Pins
DRDY_PIN = 17  # BCM (pin 11)

# ADS1256 commands/registers
CMD_RESET=0xFE; CMD_WREG=0x50; CMD_RDATA=0x01; CMD_SDATAC=0x0F
CMD_SELFCAL=0xF0; CMD_SYNC=0xFC; CMD_WAKEUP=0x00
REG_STATUS=0x00; REG_MUX=0x01; REG_ADCON=0x02; REG_DRATE=0x03

def wr(s, r, v): s.xfer2([CMD_WREG | (r & 0x0F), 0x00, v]); time.sleep(0.001)
def to_i24(b0,b1,b2):
    v=(b0<<16)|(b1<<8)|b2
    if v & 0x800000: v -= 1<<24
    return v

def wait_drdy(timeout=0.5):
    t0=time.time()
    while time.time()-t0 < timeout:
        if GPIO.input(DRDY_PIN)==0: return
        time.sleep(0.0002)
    raise TimeoutError("DRDY timeout")

# --- Setup GPIO + SPI ---
GPIO.setmode(GPIO.BCM)
GPIO.setup(DRDY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

s=spidev.SpiDev(); s.open(0,0); s.max_speed_hz=1_000_000; s.mode=0b01
s.xfer2([CMD_RESET]); time.sleep(0.05); s.xfer2([CMD_SDATAC]); time.sleep(0.001)
wr(s,REG_STATUS,0x03)   # MSB-first, BUFEN=1
wr(s,REG_MUX,   0x18)   # AIN1 vs GND (use 0x08 if you're on AIN0)
wr(s,REG_ADCON, 0x00)   # PGA=1
wr(s,REG_DRATE, 0xA1)   # ~1000 SPS
s.xfer2([CMD_SELFCAL]); time.sleep(0.1)
s.xfer2([CMD_SYNC]); time.sleep(0.001); s.xfer2([CMD_WAKEUP]); time.sleep(0.001)

# --- Filters: 5–50 Hz band-pass (first-order HPF + LPF) ---
FS = 1000.0
F_HP = 5.0
F_LP = 50.0
a_hp = math.exp(-2.0*math.pi*F_HP/FS); x1=0.0; y1=0.0
a_lp = math.exp(-2.0*math.pi*F_LP/FS); yl1=0.0
def bandpass_step(sample):
    global x1,y1,yl1
    yh = a_hp*(y1 + sample - x1); y1=yh; x1=sample
    yl = (1.0 - a_lp)*yh + a_lp*yl1; yl1=yl
    return yl

# Detrend raw (center around 0)
MEAN_WIN = int(0.5*FS)  # 0.5 s moving mean
from collections import deque
mwin = deque([], maxlen=MEAN_WIN); msum=0.0
def detrend(v):
    global msum
    mwin.append(v); msum += v
    if len(mwin) == mwin.maxlen:
        msum -= mwin[0]
    mean = msum / len(mwin) if mwin else 0.0
    return v - mean

# --- Moving RMS smoother for filtered signal ---
# Set window between 5–20 ms. Default 10 ms:
RMS_MS = 10.0
RMS_N = max(1, int(FS * RMS_MS / 1000.0))
rwin = deque([], maxlen=RMS_N); rsum_sq = 0.0
def rms_smooth(x):
    global rsum_sq
    rwin.append(x); rsum_sq += x*x
    if len(rwin) == rwin.maxlen:
        # next append will drop oldest; simulate rolling sum now
        rsum_sq -= rwin[0]*rwin[0]
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
lf, = ax2.plot(range(N), list(buf_bp),  lw=1.0, color="#d62728", alpha=0.6, label="band-pass 5–50 Hz")
lrms,= ax2.plot(range(N), list(buf_rms),lw=1.6, color="#ff7f0e", label=f"RMS {int(RMS_MS)} ms")
ax2.set_xlabel("Samples (last ~2 s)"); ax2.set_ylabel("Filtered"); ax2.legend(loc="upper left")

def read_sample():
    wait_drdy()
    s.xfer2([0x01])       # CMD_RDATA
    time.sleep(0.0005)    # t6 settle
    b0,b1,b2 = s.readbytes(3)
    return to_i24(b0,b1,b2)

CHUNK=20   # ~20 ms/frame
def update(_):
    for _ in range(CHUNK):
        v  = read_sample()
        vr = detrend(v)
        vf = bandpass_step(vr)
        vlo = rms_smooth(vf)
        buf_raw.append(vr)
        buf_bp.append(vf)
        buf_rms.append(vlo)

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
    return lr, lf, lrms

ani = animation.FuncAnimation(fig, update, interval=20, blit=False)
plt.tight_layout()
try:
    plt.show()
finally:
    try: s.xfer2([CMD_SDATAC]); time.sleep(0.001)
    except: pass
    s.close(); GPIO.cleanup()
