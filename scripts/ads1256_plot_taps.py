#!/usr/bin/env python3
import time, spidev, collections, math
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ---------- ADS1256 setup (CH1 vs GND, CE0) ----------
CMD_RESET=0xFE; CMD_WREG=0x50; CMD_RDATA=0x01; CMD_RDATAC=0x03; CMD_SDATAC=0x0F
CMD_SELFCAL=0xF0; CMD_SYNC=0xFC; CMD_WAKEUP=0x00
REG_STATUS=0x00; REG_MUX=0x01; REG_ADCON=0x02; REG_DRATE=0x03
def wr(spi, reg, val):
	spi.xfer2([CMD_WREG | (reg & 0x0F), 0x00, val]); time.sleep(0.001)
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

# ---------- Simple signal processing (no extra libs) ----------
SPS = 1000                  # samples per second from ADC
RAW_WINDOW_SEC = 2          # show last 2 s of raw
ENV_WINDOW_MS = 120         # envelope window (~tap energy)
HPF_WINDOW_MS = 400         # DC removal window (moving mean)
DOWNSAMPLE = 5              # plot every Nth sample for clarity (~200 Hz plot)

N_RAW = SPS * RAW_WINDOW_SEC
raw_buf = collections.deque([0]*N_RAW, maxlen=N_RAW)

# Moving mean for DC removal
mean_win = collections.deque([], maxlen=max(1, int(SPS * HPF_WINDOW_MS/1000)))
mean_sum = 0

# Moving RMS envelope
env_win_len = max(1, int(SPS * ENV_WINDOW_MS/1000))
env_win = collections.deque([], maxlen=env_win_len)
env_sum_sq = 0

# Tap detection
THRESH = 30000              # adjust live with up/down arrow if needed
REFRACT_S = 0.25
last_tap_ts = 0

# ---------- Plot setup ----------
plt.style.use("seaborn-v0_8-darkgrid")
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10,6), sharex=True)
x = list(range(0, N_RAW, DOWNSAMPLE))
line_raw, = ax1.plot(x, [0]*len(x), lw=1.0, color="#1f77b4", label="raw (detrended, decimated)")
ax1.set_ylabel("Counts")
ax1.legend(loc="upper left")
ax1.set_title("ADS1256 live — visible spikes and envelope")

line_env, = ax2.plot(x, [0]*len(x), lw=1.2, color="#d62728", label="envelope (RMS)")
thresh_line = ax2.axhline(THRESH, color="#ff7f0e", ls="--", lw=1.0, label="threshold")
tap_text = ax2.text(0.02, 0.88, "", transform=ax2.transAxes, color="#d62728")
ax2.set_xlabel("Samples (last ~2 s)")
ax2.set_ylabel("Envelope")
ax2.legend(loc="upper left")

def detrend_and_env(sample):
	global mean_sum, env_sum_sq
	# update moving mean for DC removal
	mean_win.append(sample)
	mean_sum += sample
	if len(mean_win) == mean_win.maxlen:
		mean = mean_sum / len(mean_win)
		# pop oldest from sum on next call
	else:
		mean = sample if len(mean_win)==0 else (mean_sum / len(mean_win))
	# prepare mean_sum for next call
	if len(mean_win) == mean_win.maxlen:
		# next append will pop oldest; emulate rolling sum now
		pass
	# detrend
	d = sample - mean
	# envelope via RMS over short window
	env_win.append(d)
	env_sum_sq += d*d
	if len(env_win) == env_win.maxlen:
		# approximate rolling: remove oldest^2 from sum at next call
		pass
	rms = math.sqrt(env_sum_sq / max(1, len(env_win)))
	# maintain rolling sums by peeking when we will overflow next append
	if len(env_win) == env_win.maxlen:
		# prepare to subtract the oldest next time
		old_raw = env_win[0]
	# maintain moving mean sum when at capacity
	if len(mean_win) == mean_win.maxlen:
		old_mean = mean_win[0]
	else:
		old_mean = None
	return d, rms, old_mean, old_raw if len(env_win)==env_win.maxlen else None

def update(_):
	global mean_sum, env_sum_sq, last_tap_ts, THRESH
	# Read a chunk for smoother UI
	CHUNK = 50
	detrended = []
	envelopes = []
	for _ in range(CHUNK):
		v = to_i24(spi.readbytes(3))
		d, rms, old_mean, old_env0 = detrend_and_env(v)
		detrended.append(d)
		envelopes.append(rms)
		# roll the raw buffer (decimate later for plotting)
		raw_buf.append(d)
		# upkeep rolling sums (after append)
		if old_mean is not None and len(mean_win)==mean_win.maxlen:
			# emulate removal of oldest on next append
			mean_sum -= old_mean
		if old_env0 is not None and len(env_win)==env_win.maxlen:
			env_sum_sq -= old_env0*old_env0

	# Prepare decimated arrays for plot
	y_raw = list(raw_buf)[::DOWNSAMPLE]
	# Downsample envelope to same length as y_raw by simple stride
	env_stride = max(1, int(len(envelopes) / max(1,len(y_raw))))
	y_env_full = [envelopes[-1]] * len(raw_buf)  # simple “last value” fill
	y_env = y_env_full[::DOWNSAMPLE]

	line_raw.set_ydata(y_raw)
	line_env.set_ydata(y_env)
	ax1.set_xlim(0, len(y_raw)-1); ax2.set_xlim(0, len(y_raw)-1)

	# autoscale raw around content
	ymin, ymax = min(y_raw), max(y_raw)
	margin = 0.1 * max(1, ymax - ymin)
	ax1.set_ylim(ymin - margin, ymax + margin)

	# adaptive envelope scale
	emin, emax = min(y_env), max(y_env)
	ax2.set_ylim(0, max(THRESH*1.5, emax*1.2 + 1))

	# simple tap detection (envelope crossing with refractory)
	now = time.time()
	if y_env[-1] > THRESH and (now - last_tap_ts) > REFRACT_S:
		last_tap_ts = now
		tap_text.set_text("TAP!")
	else:
		# fade text if no tap recently
		if (now - last_tap_ts) > 0.5:
			tap_text.set_text("")

	# keep threshold line updated (may be changed by keys)
	thresh_line.set_ydata([THRESH, THRESH])
	return line_raw, line_env, thresh_line, tap_text

def on_key(event):
	global THRESH
	if event.key == 'up':
		THRESH = int(THRESH * 1.2)
	elif event.key == 'down':
		THRESH = max(1000, int(THRESH / 1.2))
	elif event.key == 'r':
		# reset threshold to a heuristic based on recent envelope
		try:
			# recent env approx = abs of recent raw
			recent = [abs(v) for v in list(raw_buf)[-500:]]
			THRESH = max(10000, int(sum(recent)/max(1,len(recent)) * 4))
		except Exception:
			pass

fig.canvas.mpl_connect('key_press_event', on_key)
ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
plt.tight_layout()
try:
	plt.show()
finally:
	try: spi.xfer2([CMD_SDATAC]); time.sleep(0.001)
	except Exception: pass
	spi.close()
