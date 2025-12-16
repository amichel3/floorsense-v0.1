#!/usr/bin/env python3
import time, math, collections
import spidev
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ---------- ADS1256 (CH1 vs GND, CE0) ----------
CMD_RESET=0xFE; CMD_WREG=0x50; CMD_RDATA=0x01; CMD_RDATAC=0x03; CMD_SDATAC=0x0F
CMD_SELFCAL=0xF0; CMD_SYNC=0xFC; CMD_WAKEUP=0x00
REG_STATUS=0x00; REG_MUX=0x01; REG_ADCON=0x02; REG_DRATE=0x03
def wr(spi, reg, val): spi.xfer2([CMD_WREG | (reg & 0x0F), 0x00, val]); time.sleep(0.001)
def to_i24(b): 
    v = (b[0] << 16) | (b[1] << 8) | b[2]
    return v - (1<<24) if (v & 0x800000) else v

spi = spidev.SpiDev(); spi.open(0,0); spi.max_speed_hz=1_000_000; spi.mode=0b01
spi.xfer2([CMD_RESET]); time.sleep(0.05); spi.xfer2([CMD_SDATAC]); time.sleep(0.001)
wr(spi, REG_STATUS, 0x03)   # BUFEN=1, MSB first
wr(spi, REG_MUX,    0x18)   # AIN1 vs GND (use 0x08 for AIN0)
wr(spi, REG_ADCON,  0x00)   # PGA=1
wr(spi, REG_DRATE,  0x83)   # ~500 SPS on most clones (quieter)
spi.xfer2([CMD_SELFCAL]); time.sleep(0.1); spi.xfer2([CMD_SYNC]); time.sleep(0.001); spi.xfer2([CMD_WAKEUP]); time.sleep(0.001)
for _ in range(3): spi.xfer2([CMD_RDATA]); time.sleep(0.001); _=spi.readbytes(3)
spi.xfer2([CMD_RDATAC]); time.sleep(0.001)

# ---------- Filters: 10–80 Hz band + 60 Hz notch ----------
FS = 500.0          # match DRATE above
F_HP = 10.0         # high-pass
F_LP = 80.0         # low-pass (tight for steps/taps)
F_NOTCH = 60.0      # mains
Q_NOTCH = 30.0      # notch sharpness (higher = narrower)

# One-pole HPF
a_hp = math.exp(-2.0*math.pi*F_HP/FS)
x1 = 0.0; y1 = 0.0

# One-pole LPF
a_lp = math.exp(-2.0*math.pi*F_LP/FS)
yl1 = 0.0

# Notch biquad @ 60 Hz
# Standard biquad (RBJ cookbook)
def biquad_notch(f0, Q, fs):
    w0 = 2.0*math.pi*f0/fs
    alpha = math.sin(w0)/(2.0*Q)
    b0 = 1.0; b1 = -2.0*math.cos(w0); b2 = 1.0
    a0 = 1.0 + alpha; a1 = -2.0*math.cos(w0); a2 = 1.0 - alpha
    # normalize
    return (b0/a0, b1/a0, b2/a0, 1.0, a1/a0, a2/a0)

b0,b1,b2,_,a1,a2 = biquad_notch(F_NOTCH, Q_NOTCH, FS)
zn1 = 0.0; zn2 = 0.0

def step_band(sample):
    global x1,y1,yl1,zn1,zn2
    # HPF
    yh = a_hp*(y1 + sample - x1); y1 = yh; x1 = sample
    # Notch
    # Direct Form I (transposed) using normalize above (a0=1)
    out = b0*yh + zn1
    zn1 = b1*yh - a1*out + zn2
    zn2 = b2*yh - a2*out
    # LPF
    yl = (1.0 - a_lp)*out + a_lp*yl1; yl1 = yl
    return yl

# ---------- Plot ----------
WINDOW_SEC = 3
N = int(FS*WINDOW_SEC)
buf_raw = collections.deque([0]*N, maxlen=N)
buf_bp  = collections.deque([0]*N, maxlen=N)

plt.style.use("seaborn-v0_8-darkgrid")
fig, (ax1, ax2) = plt.subplots(2,1, figsize=(10,6), sharex=True)
line1, = ax1.plot(range(N), list(buf_raw), lw=0.9, color="#1f77b4", label="raw")
ax1.set_ylabel("Raw"); ax1.legend(loc="upper left")
line2, = ax2.plot(range(N), list(buf_bp),  lw=1.2, color="#d62728", label="10–80 Hz + 60 Hz notch")
ax2.set_xlabel("Samples (last ~3 s)")
ax2.set_ylabel("Filtered"); ax2.legend(loc="upper left")

CHUNK = 25   # ~50 ms/frame at 500 SPS
def update(_):
    for _ in range(CHUNK):
        v = to_i24(spi.readbytes(3))
        buf_raw.append(v)
        buf_bp.append(step_band(v))
    y1d = list(buf_raw); y2d = list(buf_bp)
    line1.set_ydata(y1d); line2.set_ydata(y2d)
    for ax, y in ((ax1,y1d),(ax2,y2d)):
        ymin, ymax = min(y), max(y); margin = 0.15*max(1, ymax-ymin)
        ax.set_ylim(ymin-margin, ymax+margin); ax.set_xlim(0, len(y)-1)
    return line1, line2

ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
plt.tight_layout()
try:
    plt.show()
finally:
    try: spi.xfer2([CMD_SDATAC]); time.sleep(0.001)
    except Exception: pass
    spi.close()
