import time
import board
import busio
import digitalio
import adafruit_gps
import adafruit_bmp3xx
import adafruit_rfm9x
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT
i2c = busio.I2C(board.GP13, board.GP12)
spi = busio.SPI(clock=board.GP2, MOSI=board.GP3, MISO=board.GP4)
cs = digitalio.DigitalInOut(board.GP5)
reset = digitalio.DigitalInOut(board.GP27)
uart = busio.UART(board.GP0, board.GP1, baudrate=9600, bits=8, parity=None, stop=1, timeout=0.1)
gps = adafruit_gps.GPS(uart, debug=False)
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
gps.send_command(b"PMTK220,100")
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)
bmp.sea_level_pressure = 1001
rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, 433)
rfm9x.tx_power = 20

pc = 0
while(True):
    pc += 1
    alt = bmp.altitude
    gps.update()
    msg = (
        f"PKT={pc},"
        f"ALT={bmp.altitude},"
        f"LAT={gps.latitude},"
        f"LON={gps.longitude}"
    )
    print(msg)
    try:
        rfm9x.send(bytes(msg, "utf-8"))
    except Exception as e:
        print("LoRa send Failed", e)
    time.sleep(0.05)
