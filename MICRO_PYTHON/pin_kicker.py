from machine import Pin
import time

pin = Pin(1, Pin.OUT)

while True:
    pin.high()
    time.sleep(0.5)
    pin.low()
    time.sleep(0.5)