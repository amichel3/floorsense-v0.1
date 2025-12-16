#!/usr/bin/env python3
import time
import spidev
import RPi.GPIO as GPIO

# ==== Pin configuration (BCM numbers) ====
CS_PIN   = 8    # GPIO8  (pin 24)
DRDY_PIN = 17   # GPIO17 (pin 11)
RESET_PIN = None  # set to a GPIO number if you wired RESET/PDWN

# ==== ADS1256 register addresses ====
REG_STATUS = 0x00
REG_MUX    = 0x01
REG_ADCON  = 0x02
REG_DRATE  = 0x03

# ==== ADS1256 commands ====
CMD_WAKEUP  = 0x00
CMD_RDATA   = 0x01
CMD_RREG    = 0x10
CMD_WREG    = 0x50
CMD_SYNC    = 0xFC
CMD_RESET   = 0xFE
CMD_SELFCAL = 0xF0

# Data rates from datasheet (we'll use 1000 SPS = 0x82 or slower)
DRATE_1000SPS = 0x82  # typical
DRATE_500SPS  = 0x83

def gpio_setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(CS_PIN, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(DRDY_PIN, GPIO.IN)
    if RESET_PIN is not None:
        GPIO.setup(RESET_PIN, GPIO.OUT, initial=GPIO.HIGH)

def spi_setup():
    spi = spidev.SpiDev()
    spi.open(0, 0)        # bus 0, device 0 -> /dev/spidev0.0
    spi.max_speed_hz = 1000000  # 1 MHz is safe
    spi.mode = 0b01       # CPOL=0, CPHA=1 (ADS1256 mode 1)
    return spi

def cs_low():
    GPIO.output(CS_PIN, GPIO.LOW)

def cs_high():
    GPIO.output(CS_PIN, GPIO.HIGH)

def wait_drdy(timeout=1.0):
    """Wait for DRDY to go low."""
    t0 = time.time()
    while GPIO.input(DRDY_PIN) == 1:
        if time.time() - t0 > timeout:
            raise TimeoutError("DRDY timeout")
        time.sleep(0.001)

def reset_chip(spi):
    if RESET_PIN is not None:
        GPIO.output(RESET_PIN, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(RESET_PIN, GPIO.HIGH)
        time.sleep(0.05)
    else:
        cs_low()
        spi.xfer2([CMD_RESET])
        cs_high()
        time.sleep(0.05)

def write_reg(spi, reg, value):
    cs_low()
    spi.xfer2([CMD_WREG | reg, 0x00, value])  # write single register
    cs_high()
    time.sleep(0.001)

def read_reg(spi, reg):
    cs_low()
    spi.xfer2([CMD_RREG | reg, 0x00])
    val = spi.xfer2([0xFF])[0]
    cs_high()
    return val

def read_data(spi):
    """Issue RDATA and read 24-bit value."""
    wait_drdy()
    cs_low()
    spi.xfer2([CMD_RDATA])
    time.sleep(0.001)
    raw = spi.readbytes(3)
    cs_high()

    # Convert 3 bytes to signed 24-bit integer
    value = (raw[0] << 16) | (raw[1] << 8) | raw[2]
    if value & 0x800000:  # negative number
        value -= 1 << 24
    return value

def configure_ads1256(spi):
    reset_chip(spi)

    # STATUS: default 0x01 (buffer disabled). We'll keep buffer off.
    write_reg(spi, REG_STATUS, 0x01)

    # MUX: AINP = AIN0, AINN = AINCOM (0b0000_1000 = 0x08)
    #   Bits 7-4 = AINP (0000 = AIN0)
    #   Bits 3-0 = AINN (1000 = AINCOM)
    write_reg(spi, REG_MUX, 0x08)

    # ADCON: Clock out off, PGA gain = 1 (0b0000_0000)
    write_reg(spi, REG_ADCON, 0x00)

    # DRATE: choose 1000 samples/s (or 500 SPS if you prefer)
    write_reg(spi, REG_DRATE, DRATE_500SPS)

    # Optional: self-calibration
    cs_low()
    spi.xfer2([CMD_SELFCAL])
    cs_high()
    time.sleep(0.1)

    # A few dummy reads to settle
    for _ in range(3):
        _ = read_data(spi)

def main():
    gpio_setup()
    spi = spi_setup()
    try:
        configure_ads1256(spi)
        print("ADS1256 configured. Reading AIN0 (single-ended)...")
        while True:
            v = read_data(spi)
            print(v)
            time.sleep(0.05)  # 20 samples per second printed
    except KeyboardInterrupt:
        print("Stopping.")
    finally:
        cs_high()
        spi.close()
        GPIO.cleanup()

if __name__ == "__main__":
    main()

