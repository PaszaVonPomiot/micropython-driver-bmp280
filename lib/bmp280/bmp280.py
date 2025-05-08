from micropython import const
from machine import I2C
import struct
from utime import sleep_ms


class BMP280Config:
    # CTRL_MEAS register settings
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

    # CONFIG register settings
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

    def __init__(
        self,
        power_mode: int = POWER_MODE_FORCED,
        temperature_oversampling: int = TEMPERATURE_OVERSAMPLING_X1,
        pressure_oversampling: int = PRESSURE_OVERSAMPLING_X1,
        filter_coefficient: int = FILTER_COEFFICIENT_OFF,
        standby_time: int = STANDBY_TIME_1000_MS,
        spi3w: int = SPI3W_DIS,
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
    # BMP280 register addresses (complete memory map)
    REG_TEMP_XLSB = const(0xFC)  # 4-bits (7-4)
    REG_TEMP_LSB = const(0xFB)  # 8-bits
    REG_TEMP_MSB = const(0xFA)  # 8-bits
    REG_PRESS_XLSB = const(0xF9)  # 4-bits (7-4)
    REG_PRESS_LSB = const(0xF8)  # 8-bits
    REG_PRESS_MSB = const(0xF7)  # 8-bits
    REG_CONFIG = const(0xF5)  # Configuration register
    REG_CTRL_MEAS = const(0xF4)  # Control measurement register
    REG_STATUS = const(0xF3)  # Status register
    REG_RESET = const(0xE0)  # Reset register
    REG_ID = const(0xD0)  # Chip ID register
    REG_CALIB25 = const(0xA1)  # Calibration data end
    REG_CALIB00 = const(0x88)  # Calibration data start

    def __init__(
        self,
        i2c: I2C,
        config: BMP280Config = BMP280Config(),
        i2c_address: int = 0x76,
    ) -> None:
        self._i2c = i2c
        self._i2c_address = i2c_address
        self._ctrl_meas = config.ctrl_meas
        self._config = config.config
        self._power_mode = config._power_mode
        self._read_calibration_data()
        self._configure_sensor()

    def _read_register(self, register: int, burst: int = 1) -> bytes:
        """Read a single or multiple bytes from a register on the BMP280 sensor."""
        data = self._i2c.readfrom_mem(self._i2c_address, register, burst, addrsize=8)
        return data

    def _write_register(self, register: int, value: bytes) -> None:
        """Write a single byte to a register on the BMP280 sensor."""
        self._i2c.writeto_mem(self._i2c_address, register, value, addrsize=8)

    def _read_calibration_data(self) -> None:
        """Read and store 26 bytes of calibration data from the sensor."""
        data = self._read_register(register=self.REG_CALIB00, burst=26)
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

    def _configure_sensor(self) -> None:
        """Initial sensor configuration."""
        self._write_register(register=self.REG_CTRL_MEAS, value=self._ctrl_meas)
        self._write_register(register=self.REG_CONFIG, value=self._config)

    def _get_status(self) -> int:
        """Get byte from the STATUS register."""
        status = self._read_register(self.REG_STATUS)
        return status[0]

    def _get_ctrl_meas(self) -> int:
        """Get byte from the CTRL_MEAS register."""
        ctrl_meas = self._read_register(self.REG_CTRL_MEAS)
        return ctrl_meas[0]

    def _get_config(self) -> int:
        """Get byte from the CONFIG register."""
        config = self._read_register(self.REG_CONFIG)
        return config[0]

    def _dump_registers(self) -> None:
        """Dump key registers to the console."""
        for register in [0xD0, 0xE0] + list(range(0xF3, 0xFD)):
            value = self._read_register(register=register)
            print(f"0x{register:02X}: {value[0]:08b}")

    def _compensate_temperature(self, raw_temp: int) -> float:
        """Compensate the raw temperature value using calibration data."""
        var1 = ((raw_temp / 16384.0) - (self._dig_T1 / 1024.0)) * self._dig_T2
        var2 = (((raw_temp / 131072.0) - (self._dig_T1 / 8192.0)) ** 2) * self._dig_T3
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

    def _is_measurement_ready(self) -> bool:
        """"""
        status = self._get_status()
        return (status & 0b00001001) == 0

    def _force_measurement(self, miliseconds: int = 44) -> None:
        """
        Forcing meaurement requires a write to the CTRL_MEAS register.
        Measurement takes from 7 to 65 ms depending on oversampling configuration.
        P x16, T x2, F2 ~ 41 ms
        """
        self._write_register(register=self.REG_CTRL_MEAS, value=self._ctrl_meas)
        sleep_ms(miliseconds)

    def set_power_mode(self, power_mode: int) -> None:
        """Change the power mode of the sensor. Forced mode goes to sleep after measurement."""
        self._power_mode = power_mode
        new_ctrl_meas: int = struct.unpack("B", self._ctrl_meas)[0]
        new_ctrl_meas = (new_ctrl_meas & 0b11111100) | power_mode
        self._ctrl_meas = struct.pack("B", new_ctrl_meas)
        self._write_register(register=self.REG_CTRL_MEAS, value=self._ctrl_meas)

    def reset(self) -> None:
        """Reset the sensor."""
        reset_code = struct.pack("B", 0xB6)
        self._write_register(register=self.REG_RESET, value=reset_code)
        sleep_ms(100)

    def read_raw_temperature(self) -> int:
        """
        Read the raw temperature value from the register.
        For combined read of temperature and pressure, use read_raw() instead.
        """
        if self._power_mode == BMP280Config.POWER_MODE_FORCED:
            self._force_measurement()
        burst_data = self._read_register(register=self.REG_TEMP_MSB, burst=3)
        raw_temp = (burst_data[0] << 12) | (burst_data[1] << 4) | (burst_data[2] >> 4)
        return raw_temp

    def read_raw_pressure(self) -> int:
        """
        Read the raw pressure value from the register.
        For combined read of temperature and pressure, use read_raw() instead.
        """
        if self._power_mode == BMP280Config.POWER_MODE_FORCED:
            self._force_measurement()
        burst_data = self._read_register(register=self.REG_PRESS_MSB, burst=3)
        raw_pressure = (
            (burst_data[0] << 12) | (burst_data[1] << 4) | (burst_data[2] >> 4)
        )
        return raw_pressure

    def read_raw(self) -> tuple[int, int]:
        """Read the raw temperature and pressure values from the register."""
        if self._power_mode == BMP280Config.POWER_MODE_FORCED:
            self._force_measurement()
        burst_data = self._read_register(register=self.REG_PRESS_MSB, burst=6)
        raw_temperature = (
            (burst_data[3] << 12) | (burst_data[4] << 4) | (burst_data[5] >> 4)
        )
        raw_pressure = (
            (burst_data[0] << 12) | (burst_data[1] << 4) | (burst_data[2] >> 4)
        )
        return raw_temperature, raw_pressure

    def read_calibrated(self) -> tuple[float, float]:
        """Read and return the calibrated temperature (°C) and pressure (Pa)."""
        raw_temperature, raw_pressure = self.read_raw()
        temperature = self._compensate_temperature(raw_temperature)
        pressure = self._compensate_pressure(raw_pressure)
        return temperature, pressure
