import spidev
import time

# Ouvre SPI bus 0, device 0 (CE0)
spi = spidev.SpiDev()
spi.open(0, 0)

spi.max_speed_hz = 100000   # 100 kHz (safe pour breadboard)
spi.mode = 0                # SPI mode 0

time.sleep(0.5)

print("Sending 0xAA ...")

# Envoie 0xAA et lit la réponse
response = spi.xfer2([0xAA])

print("Received:", response)

spi.close()
