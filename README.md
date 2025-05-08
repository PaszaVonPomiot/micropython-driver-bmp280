# MicroPython driver for BMP280 over I2C

MicroPython driver for BMP280 temperature and pressure sensor over I2C interface.
![BMP280](docs/img/bmp280-pinout.webp)

## Hardware

-   [Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/)
-   [BMP280 Datasheet](docs/bst-bmp280-ds001.pdf)

## Software

-   [MicroPython](https://micropython.org/download/RPI_PICO/) - Pico firmware (RP2 port)

## Installation

-   Copy the `bmp280.py` file or `lib` folder to the root of your MicroPython device filesystem.

## Code examples

Working code example can be found in [main.py](main.py) file.

```py
from machine import Pin, I2C
from bmp280 import BMP280

i2c = I2C(0, scl=Pin(9), sda=Pin(8))
sensor = BMP280(i2c=i2c)
print(sensor.read_measurements())  # (temperature, pressure)
```

# Acknowledgements

-   [flrrth](https://github.com/flrrth) @ [https://github.com/flrrth/pico-bmp280](https://github.com/flrrth/pico-bmp280) - for the inspiration
