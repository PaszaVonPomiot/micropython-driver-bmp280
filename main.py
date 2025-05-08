from machine import Pin, I2C
from lib import BMP280, BMP280Config
from utime import sleep_ms


def main():
    i2c = I2C(
        0,
        scl=Pin(9),
        sda=Pin(8),
    )

    sensor = BMP280(
        i2c=i2c,
        config=BMP280Config(
            pressure_oversampling=BMP280Config.PRESSURE_OVERSAMPLING_X16,
            temperature_oversampling=BMP280Config.TEMPERATURE_OVERSAMPLING_X2,
            filter_coefficient=BMP280Config.FILTER_COEFFICIENT_2,
        ),
    )

    while True:
        print(sensor.read_measurements())
        sleep_ms(200)


if __name__ == "__main__":
    main()
