from machine import Pin, I2C
import time

# I2C on GPIO0 (SDA) and GPIO1 (SCL)
i2c = I2C(1, sda=Pin(10), scl=Pin(11), freq=100000)

while True:
    devices = i2c.scan()
    
    if devices:
        print("I2C devices found:")
        for addr in devices:
            print(" - 0x{:02X}".format(addr))
    else:
        print("No I2C devices found")
    
    print()
    time.sleep(2)