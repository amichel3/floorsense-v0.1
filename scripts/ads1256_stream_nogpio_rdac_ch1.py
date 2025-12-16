#!/usr/bin/env python3
import time, spidev
CMD_WAKEUP=0x00; CMD_RDATA=0x01; CMD_RDATAC=0x03; CMD_SDATAC=0x0F
CMD_RREG=0x10; CMD_WREG=0x50; CMD_SYNC=0xFC; CMD_RESET=0xFE; CMD_SELFCAL=0xF0
REG_STATUS=0x00; REG_MUX=0x01; REG_ADCON=0x02; REG_DRATE=0x03
DRATE_1000SPS=0xA1
def wr(s,r,v): s.xfer2([CMD_WREG|(r&0x0F),0x00,v]); time.sleep(0.001)
def to_i24(b): v=(b[0]<<16)|(b[1]<<8)|b[2]; return v-(1<<24) if v&0x800000 else v
s=spidev.SpiDev(); s.open(0,0); s.max_speed_hz=1_000_000; s.mode=0b01
try:
  s.xfer2([CMD_RESET]); time.sleep(0.05); s.xfer2([CMD_SDATAC]); time.sleep(0.001)
  wr(s,REG_STATUS,0x03)   # MSB-first, input buffer ON
  wr(s,REG_MUX,0x18)      # AIN1 vs AINCOM (your GND next to AIN1)
  wr(s,REG_ADCON,0x00)    # PGA=1
  wr(s,REG_DRATE,DRATE_1000SPS)
  s.xfer2([CMD_SELFCAL]); time.sleep(0.1); s.xfer2([CMD_SYNC]); time.sleep(0.001); s.xfer2([CMD_WAKEUP]); time.sleep(0.001)
  for _ in range(3): s.xfer2([CMD_RDATA]); time.sleep(0.001); _=s.readbytes(3)
  s.xfer2([CMD_RDATAC]); time.sleep(0.001)
  print("Streaming CH1 with input buffer on. Ctrl+C to stop.")
  per=1/1000.0; nxt=time.perf_counter()
  while True:
    raw=s.readbytes(3); print(to_i24(raw))
    nxt+=per; dt=nxt-time.perf_counter()
    if dt>0: time.sleep(dt)
    else: nxt=time.perf_counter()
except KeyboardInterrupt:
  pass
finally:
  try: s.xfer2([CMD_SDATAC]); time.sleep(0.001)
  except Exception: pass
  s.close()
