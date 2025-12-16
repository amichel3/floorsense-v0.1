#!/usr/bin/env python3
import time, math, collections
import spidev
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ---------- ADS1256 setup (CH1 vs GND on CE0) ----------
CMD_RESET=0xFE; CMD_WREG=0x50; CMD_RDATA=0x01; CMD_RDATAC=0x03; CMD_SDATAC=0x0F
CMD_SELFCAL=0xF0; CMD_SYNC=0xFC; CMD_WAKEUP=0x00
REG_STATUS=0x00; REG_MUX=0x01; REG_ADCON=0x02; REG_DRATE=0x03
def wr(spi, reg, val): spi.xfer2([CMD_WREG | (reg & 0x0F), 0x00, val]); time.sleep(0.001)
def to_i24(b):
	v = (b[0] << 16) | (b[1] << 8) | b[2]
	return v - (1<<24) if (v & 0x800000) else v

spi = spidev.SpiDev(); spi.open(0,0); spi.max_speed_hz=1_000_000; spi.mode=0b01
spi.xfer2([CMD_RESET]); time.sleep(0.05); spi.xfer2([CMD_SDATAC]); time.sleep(0.001)
wr(spi, REG_STATUS, 0x03)   # MSB-first, BUFEN=1
wr(spi, REG_MUX,    0x18)   # AIN1 vs GND (use 0x08 for AIN0)
wr(spi, REG_ADCON,  0x00)   # PGA=1
wr(spi, REG_DRATE,  0xA1)   # ~1 kSPS
spi.xfer2([CMD_SELFCAL]); time.sleep(0.1); spi.xfer2([CMD_SYNC]); time.sleep(0.001); spi.xfer2([CMD_WAKEUP]); time.sleep(0.001)
for _ in range(3): spi.xfer2([CMD_RDATA]); time.sleep(0.001); _=spi.readbytes(3)
spi.xfer2([CMD_RDATAC]); time.sleep(0.001)

# ---------- Band-pass filter (first-order HPF then first-order LPF) ----------
FS = 1000.0         # samples per second
F_HP = 10.0         # high-pass cutoff (Hz)
F_LP = 300.0        # low-pass cutoff (Hz)

# One-pole HPF: y[n] = a*(y[n-1] + x[n] - x[n-1]) with a = exp(-2*pi*F_HP/FS)
a_hp = math.exp(-2.0*math.pi*F_HP/FS)
x1 = 0.0
y1 = 0.0

# One-pole LPF: y[n] = (1-a)*x[n] + a*y[n-1] with a = exp(-2*pi*F_LP/FS)
a_lp = math.exp(-2.0*math.pi*F_LP/FS)
yl1 = 0.0

def bandpass_step(sample):
	global x1, y1, yl1
	# HPF
	yh = a_hp*(y1 + sample - x1)
	y1 = yh
	x1 = sample
	# LPF
	yl = (1.0 - a_lp)*yh + a_lp*yl1
	yl1 = yl
	return yl

# ---------- Plot setup ----------
WINDOW_SEC = 2
N = int(FS*WINDOW_SEC)
buf_raw = collections.deque([0]*N, maxlen=N)
buf_bp  = collections.deque([0]*N, maxlen=N)

plt.style.use("seaborn-v0_8-darkgrid")
fig, (ax1, ax2) = plt.subplots(2,1, figsize=(10,6), sharex=True)

line1, = ax1.plot(range(N), list(buf_raw), lw=1.0, color="#1f77b4", label="raw")
ax1.set_ylabel("Raw counts"); ax1.legend(loc="upper left")
line2, = ax2.plot(range(N), list(buf_bp),  lw=1.2, color="#d62728", label="band-pass 10–300 Hz")
ax2.set_xlabel("Samples (last ~2 s)")
ax2.set_ylabel("Filtered"); ax2.legend(loc="upper left")

CHUNK = 50   # ~50 ms per redraw
def update(_):
	for _ in range(CHUNK):
		v = to_i24(spi.readbytes(3))
		buf_raw.append(v)
		buf_bp.append(bandpass_step(v))

	y1d = list(buf_raw)
	y2d = list(buf_bp)

	# Update lines
	line1.set_ydata(y1d); line2.set_ydata(y2d)
	# Autoscale to content
	for ax, y in ((ax1,y1d),(ax2,y2d)):
		ymin, ymax = min(y), max(y)
		margin = 0.1*max(1, ymax-ymin)
		ax.set_ylim(ymin-margin, ymax+margin)
	ax1.set_xlim(0, len(y1d)-1); ax2.set_xlim(0, len(y1d)-1)
	return line1, line2

ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
plt.tight_layout()
try:
	plt.show()
finally:
	try: spi.xfer2([CMD_SDATAC]); time.sleep(0.001)
	except Exception: pass
	spi.close()
