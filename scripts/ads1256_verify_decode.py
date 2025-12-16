#!/usr/bin/env python3
import time, spidev, RPi.GPIO as GPIO
DRDY_PIN=17
CMD_RESET=0xFE; CMD_WREG=0x50; CMD_RDATA=0x01; CMD_SDATAC=0x0F
CMD_SELFCAL=0xF0; CMD_SYNC=0xFC; CMD_WAKEUP=0x00
REG_STATUS=0x00; REG_MUX=0x01; REG_ADCON=0x02; REG_DRATE=0x03
def wr(s,r,v): s.xfer2([CMD_WREG|(r&0x0F),0x00,v]); time.sleep(0.001)
def wait_drdy(t=0.5):
  t0=time.time()
  while time.time()-t0<t:
    if GPIO.input(DRDY_PIN)==0: return
    time.sleep(0.0002)
  raise TimeoutError("DRDY timeout")
def to_i24(b0,b1,b2):
  v=(b0<<16)|(b1<<8)|b2
  if v & 0x800000: v -= 1<<24
  return v
GPIO.setmode(GPIO.BCM); GPIO.setup(DRDY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
s=spidev.SpiDev(); s.open(0,0); s.max_speed_hz=1_000_000; s.mode=0b01
s.xfer2([CMD_RESET]); time.sleep(0.05); s.xfer2([CMD_SDATAC]); time.sleep(0.001)
wr(s,REG_STATUS,0x03); wr(s,REG_MUX,0x18); wr(s,REG_ADCON,0x00); wr(s,REG_DRATE,0xA1)
s.xfer2([CMD_SELFCAL]); time.sleep(0.1); s.xfer2([CMD_SYNC]); time.sleep(0.001); s.xfer2([CMD_WAKEUP]); time.sleep(0.001)
print("Reading 200 DRDY-gated samples…")
vals=[]
for _ in range(200):
  wait_drdy()
  s.xfer2([CMD_RDATA]); time.sleep(0.0005)
  b0,b1,b2=s.readbytes(3)
  vals.append(to_i24(b0,b1,b2))
print("First 20:", vals[:20])
s.close(); GPIO.cleanup()
