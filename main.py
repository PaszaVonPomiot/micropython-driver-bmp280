from machine import Pin, I2C
from lib.bmp280 import (
    BMP280,
    BMP280Configuration,
    PRESSURE_OVERSAMPLING_X16,
    TEMPERATURE_OVERSAMPLING_X2,
    FILTER_COEFFICIENT_2,
)
from utime import sleep_ms


def main():
    i2c = I2C(
        0,
        scl=Pin(9),
        sda=Pin(8),
    )

    sensor = BMP280(
        i2c=i2c,
        configuration=BMP280Configuration(
            pressure_oversampling=PRESSURE_OVERSAMPLING_X16,
            temperature_oversampling=TEMPERATURE_OVERSAMPLING_X2,
            filter_coefficient=FILTER_COEFFICIENT_2,
        ),
    )

    while True:
        print(sensor.get_measurements())
        sleep_ms(200)


if __name__ == "__main__":
    main()
