#!/usr/bin/env python3
import time
import spidev

# ADS1256 commands
CMD_RDATA   = 0x01
CMD_RREG    = 0x10
CMD_WREG    = 0x50
CMD_RESET   = 0xFE
CMD_SELFCAL = 0xF0

# Registers
REG_STATUS = 0x00
REG_MUX    = 0x01
REG_ADCON  = 0x02
REG_DRATE  = 0x03

# Choose a conservative data rate: 500 samples/sec
DRATE_500SPS = 0x83
SAMPLE_PERIOD_S = 1.0 / 500.0  # ~2 ms

def write_reg(spi, reg, val):
    spi.xfer2([CMD_WREG | (reg & 0x0F), 0x00, val])
    time.sleep(0.001)

def reset_chip(spi):
    spi.xfer2([CMD_RESET])
    time.sleep(0.05)

def read_data_once(spi):
    # Issue RDATA and read 24-bit value
    spi.xfer2([CMD_RDATA])
    time.sleep(0.001)
    raw = spi.readbytes(3)
    v = (raw[0] << 16) | (raw[1] << 8) | raw[2]
    if v & 0x800000:
        v -= 1 << 24
    return v

def configure(spi):
    reset_chip(spi)
    # STATUS: MSB-first, buffer off (=0x01)
    write_reg(spi, REG_STATUS, 0x01)
    # MUX: AIN0 vs AINCOM (0x08)
    write_reg(spi, REG_MUX, 0x08)
    # ADCON: PGA=1
    write_reg(spi, REG_ADCON, 0x00)
    # DRATE: 500 SPS
    write_reg(spi, REG_DRATE, DRATE_500SPS)
    # Self-cal
    spi.xfer2([CMD_SELFCAL])
    time.sleep(0.1)
    # A few dummy reads to settle
    for _ in range(3):
        _ = read_data_once(spi)

def main():
    spi = spidev.SpiDev()
    spi.open(0, 0)               # SPI0 CE0 (auto CS)
    spi.max_speed_hz = 1_000_000
    spi.mode = 0b01               # ADS1256 mode 1

    try:
        configure(spi)
        print("Streaming CH0 integers (no GPIO). Ctrl+C to stop.")
        next_t = time.perf_counter()
        while True:
            v = read_data_once(spi)
            print(v)
            next_t += SAMPLE_PERIOD_S
            dt = next_t - time.perf_counter()
            if dt > 0:
                time.sleep(dt)
            else:
                next_t = time.perf_counter()
    except KeyboardInterrupt:
        pass
    finally:
        spi.close()

if __name__ == "__main__":
    main()
