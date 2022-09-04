import time
import board
import busio
import adafruit_lidarlite

# Create library object using our Bus I2C port
i2c = busio.I2C(board.SCL, board.SDA)

# Default configuration, with only i2c wires
sensor = adafruit_lidarlite.LIDARLite(i2c)

while True:
    try:
        # We print tuples so you can plot with Mu Plotter
        print((sensor.distance))
    except RuntimeError as e:
        # If we get a reading error, just print it and keep truckin'
        print(e)
    time.sleep(0.01) # you can remove this for ultra-fast measurements!þ