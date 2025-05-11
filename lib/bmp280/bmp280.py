# https://github.com/PaszaVonPomiot/micropython-driver-bmp280

from micropython import const
from machine import I2C
import struct
from utime import sleep_ms


class Settings:
    """Sensor settings for BMP280Configuration object."""

    ## CTRL_MEAS register settings
    POWER_MODE_SLEEP = const(0b00)
    POWER_MODE_FORCED = const(0b01)
    POWER_MODE_NORMAL = const(0b11)

    TEMPERATURE_OVERSAMPLING_SKIP = const(0b000)
    TEMPERATURE_OVERSAMPLING_X1 = const(0b001)
    TEMPERATURE_OVERSAMPLING_X2 = const(0b010)
    TEMPERATURE_OVERSAMPLING_X4 = const(0b011)
    TEMPERATURE_OVERSAMPLING_X8 = const(0b100)
    TEMPERATURE_OVERSAMPLING_X16 = const(0b101)

    PRESSURE_OVERSAMPLING_SKIP = const(0b000)
    PRESSURE_OVERSAMPLING_X1 = const(0b001)
    PRESSURE_OVERSAMPLING_X2 = const(0b010)
    PRESSURE_OVERSAMPLING_X4 = const(0b011)
    PRESSURE_OVERSAMPLING_X8 = const(0b100)
    PRESSURE_OVERSAMPLING_X16 = const(0b101)

    ## CONFIG register settings
    STANDBY_TIME_0_5_MS = const(0b000)
    STANDBY_TIME_62_5_MS = const(0b001)
    STANDBY_TIME_125_MS = const(0b010)
    STANDBY_TIME_250_MS = const(0b011)
    STANDBY_TIME_500_MS = const(0b100)
    STANDBY_TIME_1000_MS = const(0b101)
    STANDBY_TIME_2000_MS = const(0b110)
    STANDBY_TIME_4000_MS = const(0b111)

    FILTER_COEFFICIENT_OFF = const(0b000)
    FILTER_COEFFICIENT_2 = const(0b001)
    FILTER_COEFFICIENT_4 = const(0b010)
    FILTER_COEFFICIENT_8 = const(0b011)
    FILTER_COEFFICIENT_16 = const(0b100)

    SPI3W_EN = const(0b01)  # Enable 3-wire SPI
    SPI3W_DIS = const(0b00)  # Disable 3-wire SPI


class _Const:
    """Other constants."""

    DEFAULT_I2C_ADDRESS = const(0x76)  # Default I2C address for BMP280
    STATUS_MASK = const(0b00001001)
    RESET_CODE = const(0xB6)


class _Reg:
    """BMP280 register addresses (complete memory map)."""

    TEMP_XLSB = const(0xFC)  # 4-bits (7-4)
    TEMP_LSB = const(0xFB)  # 8-bits
    TEMP_MSB = const(0xFA)  # 8-bits
    PRESS_XLSB = const(0xF9)  # 4-bits (7-4)
    PRESS_LSB = const(0xF8)  # 8-bits
    PRESS_MSB = const(0xF7)  # 8-bits
    CONFIG = const(0xF5)  # Configuration register
    CTRL_MEAS = const(0xF4)  # Control measurement register
    STATUS = const(0xF3)  # Status register
    RESET = const(0xE0)  # Reset register
    ID = const(0xD0)  # Chip ID register
    CALIB25 = const(0xA1)  # Calibration data end
    CALIB00 = const(0x88)  # Calibration data start


class BMP280Configuration:
    """Configuration object for the BMP280 sensor."""

    def __init__(
        self,
        power_mode: int = Settings.POWER_MODE_FORCED,
        temperature_oversampling: int = Settings.TEMPERATURE_OVERSAMPLING_X1,
        pressure_oversampling: int = Settings.PRESSURE_OVERSAMPLING_X1,
        filter_coefficient: int = Settings.FILTER_COEFFICIENT_OFF,
        standby_time: int = Settings.STANDBY_TIME_1000_MS,
        spi3w: int = Settings.SPI3W_DIS,
    ):
        self._power_mode = power_mode
        self._temperature_oversampling = temperature_oversampling
        self._pressure_oversampling = pressure_oversampling
        self._filter_coefficient = filter_coefficient
        self._standby_time = standby_time
        self._spi3w = spi3w

    @property
    def ctrl_meas(self) -> bytes:
        return struct.pack(
            "B",
            self._temperature_oversampling << 5
            | self._pressure_oversampling << 2
            | self._power_mode,
        )

    @property
    def config(self) -> bytes:
        return struct.pack(
            "B",
            self._standby_time << 5 | self._filter_coefficient << 2 | self._spi3w,
        )


class BMP280:
    def __init__(
        self,
        i2c: I2C,
        i2c_address: int = _Const.DEFAULT_I2C_ADDRESS,
        configuration: BMP280Configuration = BMP280Configuration(),
    ) -> None:
        self._i2c = i2c
        self._i2c_address = i2c_address
        self._ctrl_meas = configuration.ctrl_meas
        self._config = configuration.config
        self._power_mode = configuration._power_mode
        self._read_calibration_data()
        self._configure_sensor(ctrl_meas=self._ctrl_meas, config=self._config)

    def _read_register(self, register: int, burst: int = 1) -> bytes:
        """
        Read a single (burst=1) or multiple (burst>1) bytes from a register
        on the BMP280 sensor.
        """
        data = self._i2c.readfrom_mem(self._i2c_address, register, burst, addrsize=8)
        return data

    def _write_register(self, register: int, value: bytes) -> None:
        """Write a single byte to a register on the BMP280 sensor."""
        self._i2c.writeto_mem(self._i2c_address, register, value, addrsize=8)

    def _read_calibration_data(self) -> None:
        """Read and store 26 bytes of calibration data from the sensor."""
        data = self._read_register(register=_Reg.CALIB00, burst=26)
        self._dig_T1 = struct.unpack("<H", data[0:2])[0]
        self._dig_T2 = struct.unpack("<h", data[2:4])[0]
        self._dig_T3 = struct.unpack("<h", data[4:6])[0]
        self._dig_P1 = struct.unpack("<H", data[6:8])[0]
        self._dig_P2 = struct.unpack("<h", data[8:10])[0]
        self._dig_P3 = struct.unpack("<h", data[10:12])[0]
        self._dig_P4 = struct.unpack("<h", data[12:14])[0]
        self._dig_P5 = struct.unpack("<h", data[14:16])[0]
        self._dig_P6 = struct.unpack("<h", data[16:18])[0]
        self._dig_P7 = struct.unpack("<h", data[18:20])[0]
        self._dig_P8 = struct.unpack("<h", data[20:22])[0]
        self._dig_P9 = struct.unpack("<h", data[22:24])[0]

    def _configure_sensor(self, ctrl_meas: bytes, config: bytes) -> None:
        """Initial sensor configuration."""
        self._write_register(register=_Reg.CONFIG, value=config)
        self._write_register(register=_Reg.CTRL_MEAS, value=ctrl_meas)

    def _get_status(self) -> int:
        """Get byte from the STATUS register."""
        status = self._read_register(_Reg.STATUS)
        return status[0]

    def _compensate_temperature(self, raw_temperature: int) -> float:
        """Compensate the raw temperature value using calibration data."""
        var1 = ((raw_temperature / 16384.0) - (self._dig_T1 / 1024.0)) * self._dig_T2
        var2 = (
            ((raw_temperature / 131072.0) - (self._dig_T1 / 8192.0)) ** 2
        ) * self._dig_T3
        self._t_fine = int(var1 + var2)  # Store t_fine for pressure compensation
        temperature = (var1 + var2) / 5120.0
        return temperature

    def _compensate_pressure(self, raw_pressure: int) -> float:
        """Compensate the raw pressure value using calibration data."""
        var1 = (self._t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * self._dig_P6 / 32768.0
        var2 = var2 + (var1 * self._dig_P5 * 2.0)
        var2 = (var2 / 4.0) + (self._dig_P4 * 65536.0)
        var1 = (self._dig_P3 * var1 * var1 / 524288.0 + self._dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self._dig_P1
        if var1 == 0:
            return 0  # Avoid division by zero
        pressure = 1048576.0 - raw_pressure
        pressure = ((pressure - (var2 / 4096.0)) * 6250.0) / var1
        var1 = self._dig_P9 * pressure * pressure / 2147483648.0
        var2 = pressure * self._dig_P8 / 32768.0
        pressure = pressure + (var1 + var2 + self._dig_P7) / 16.0
        return pressure

    def _ensure_forced_mode_measurement(self) -> None:
        """
        Ensure a measurement is triggered if the sensor is in FORCED mode.
        """
        if self._power_mode == Settings.POWER_MODE_FORCED:
            self._force_measurement()

    def _force_measurement(self, milliseconds: int = 44) -> None:
        """
        Forcing meaurement requires a write to the CTRL_MEAS register.
        Measurement takes from 7 to 65 ms depending on oversampling configuration.
        P x16, T x2, F2 ~ 41 ms
        """
        self._write_register(register=_Reg.CTRL_MEAS, value=self._ctrl_meas)
        sleep_ms(milliseconds)

    def _get_raw_temperature(self) -> int:
        """
        Read the raw temperature value from the register.
        For combined read of temperature and pressure, use read_raw() instead.
        """
        self._ensure_forced_mode_measurement()
        burst_data = self._read_register(register=_Reg.TEMP_MSB, burst=3)
        raw_temp = (burst_data[0] << 12) | (burst_data[1] << 4) | (burst_data[2] >> 4)
        return raw_temp

    def _get_raw_pressure(self) -> int:
        """
        Read the raw pressure value from the register.
        For combined read of temperature and pressure, use read_raw() instead.
        """
        self._ensure_forced_mode_measurement()
        burst_data = self._read_register(register=_Reg.PRESS_MSB, burst=3)
        raw_pressure = (
            (burst_data[0] << 12) | (burst_data[1] << 4) | (burst_data[2] >> 4)
        )
        return raw_pressure

    def _get_raw_measurements(self) -> tuple[int, int]:
        """
        Read the raw temperature and pressure values from the sensor registers.
        If the power mode is set to FORCED, a measurement will be triggered before reading.
        """
        self._ensure_forced_mode_measurement()
        burst_data = self._read_register(register=_Reg.PRESS_MSB, burst=6)
        raw_temperature = (
            (burst_data[3] << 12) | (burst_data[4] << 4) | (burst_data[5] >> 4)
        )
        raw_pressure = (
            (burst_data[0] << 12) | (burst_data[1] << 4) | (burst_data[2] >> 4)
        )
        return raw_temperature, raw_pressure

    def is_idle(self) -> bool:
        """Check if sensor is not performing a measurement or updating its registers."""
        status = self._get_status()
        return (status & _Const.STATUS_MASK) == 0

    def get_chip_id(self) -> str:
        """Get byte from the ID register."""
        chip_id = self._read_register(_Reg.ID)
        return hex(chip_id[0])

    def reset(self) -> None:
        """Reset the sensor."""
        reset_code = struct.pack("B", _Const.RESET_CODE)
        self._write_register(register=_Reg.RESET, value=reset_code)
        sleep_ms(100)

    def get_temperature(self) -> float:
        """
        Get compensated temperature (°C) measurement.
        For combined read of temperature and pressure, use read_raw_measurements() instead.
        """
        raw_temperature = self._get_raw_temperature()
        temperature = self._compensate_temperature(raw_temperature)
        return temperature

    def get_pressure(self) -> float:
        """
        Get compensated pressure (Pa) measurement.
        For combined read of temperature and pressure, use read_raw_measurements() instead.
        """
        raw_pressure = self._get_raw_pressure()
        pressure = self._compensate_pressure(raw_pressure)
        return pressure

    def get_measurements(self) -> tuple[float, float]:
        """Get compensated temperature (°C) and pressure (Pa) measurements."""
        raw_temperature, raw_pressure = self._get_raw_measurements()
        temperature = self._compensate_temperature(raw_temperature)
        pressure = self._compensate_pressure(raw_pressure)
        return temperature, pressure
