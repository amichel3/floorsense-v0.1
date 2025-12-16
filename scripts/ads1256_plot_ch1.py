#!/usr/bin/env python3
import time, spidev, collections
import matplotlib
matplotlib.use('TkAgg')  # fall back to Agg if no desktop; use Tk on Pi desktop
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ADS1256 commands/registers
CMD_RESET=0xFE; CMD_WREG=0x50; CMD_RDATA=0x01; CMD_RDATAC=0x03; CMD_SDATAC=0x0F
CMD_SELFCAL=0xF0; CMD_SYNC=0xFC; CMD_WAKEUP=0x00
REG_STATUS=0x00; REG_MUX=0x01; REG_ADCON=0x02; REG_DRATE=0x03

def wr(spi, reg, val):
    spi.xfer2([CMD_WREG | (reg & 0x0F), 0x00, val])
    time.sleep(0.001)

def to_i24(b):
    v = (b[0] << 16) | (b[1] << 8) | b[2]
    return v - (1<<24) if (v & 0x800000) else v

# Open SPI CE0, configure CH1 single-ended vs GND with input buffer on
spi = spidev.SpiDev(); spi.open(0,0); spi.max_speed_hz=1_000_000; spi.mode=0b01
spi.xfer2([CMD_RESET]); time.sleep(0.05); spi.xfer2([CMD_SDATAC]); time.sleep(0.001)
wr(spi, REG_STATUS, 0x03)  # MSB-first, BUFEN=1
wr(spi, REG_MUX,    0x18)  # AIN1 vs AINCOM (your "GND AIN1" pair)
wr(spi, REG_ADCON,  0x00)  # PGA=1
wr(spi, REG_DRATE,  0xA1)  # ~1 kSPS
spi.xfer2([CMD_SELFCAL]); time.sleep(0.1); spi.xfer2([CMD_SYNC]); time.sleep(0.001); spi.xfer2([CMD_WAKEUP]); time.sleep(0.001)
for _ in range(3): spi.xfer2([CMD_RDATA]); time.sleep(0.001); _=spi.readbytes(3)
spi.xfer2([CMD_RDATAC]); time.sleep(0.001)

# Rolling buffer for the plot (e.g., last 2 seconds at 1 kSPS)
SPS = 1000
WINDOW_SEC = 2
N = SPS * WINDOW_SEC
buf = collections.deque([0]*N, maxlen=N)
x = list(range(N))  # sample indices (we don't need absolute time for live view)

# Matplotlib setup
plt.style.use("seaborn-v0_8-darkgrid")
fig, ax = plt.subplots(figsize=(10,4))
line, = ax.plot(x, list(buf), lw=1.2, color="#1f77b4")
ax.set_title("ADS1256 CH1 live stream — tap to see spikes")
ax.set_xlabel("Samples (last ~2 s)")
ax.set_ylabel("Raw counts")
ax.set_ylim(-500000, 500000)  # adjust if your gain is different

# Optional peak indicator (simple absolute peak hold over short window)
peak_text = ax.text(0.02, 0.92, "", transform=ax.transAxes)

# Animation function: read a chunk, update the line
CHUNK = 50  # read 50 samples per frame for efficiency (~20 FPS at 1 kSPS)
def update(_):
    # Read CHUNK samples
    for _ in range(CHUNK):
        raw = spi.readbytes(3)
        buf.append(to_i24(raw))
    y = list(buf)
    line.set_ydata(y)
    # Autoscale vertically if out of bounds
    ymin, ymax = min(y), max(y)
    cur_ymin, cur_ymax = ax.get_ylim()
    margin = 0.1 * max(1, ymax - ymin)
    if ymin <= cur_ymin or ymax >= cur_ymax:
        ax.set_ylim(ymin - margin, ymax + margin)
    # Update simple peak display (abs over last 200 ms)
    window = y[-200:] if len(y) >= 200 else y
    peak = max(abs(v) for v in window) if window else 0
    peak_text.set_text(f"peak |value| ≈ {peak}")
    return line, peak_text

ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
plt.tight_layout()
try:
    plt.show()
finally:
    try: spi.xfer2([CMD_SDATAC]); time.sleep(0.001)
    except Exception: pass
    spi.close()
