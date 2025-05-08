# BMP280 Register Memory Map

## ID and Reset

| Address | Register | Description              |
| ------- | -------- | ------------------------ |
| 0xD0    | ID       | Chip ID (should be 0x58) |
| 0xE0    | RESET    | Soft reset (write 0xB6)  |

## Status and Control

| Address | Register  | Bits                                   | Description                                      |
| ------- | --------- | -------------------------------------- | ------------------------------------------------ |
| 0xF3    | STATUS    | [3] measuring, [0] im_update           | Sensor status: measuring or updating NVM         |
| 0xF4    | CTRL_MEAS | [7:5] osrs_t, [4:2] osrs_p, [1:0] mode | Controls temp/pressure oversampling & power mode |
| 0xF5    | CONFIG    | [7:5] t_sb, [4:2] filter, [0] spi3w_en | Inactive duration, filter, SPI 3-wire enable     |

### CTRL_MEAS bits (0xF4)

- `osrs_t`: temperature oversampling
- `osrs_p`: pressure oversampling
- `mode`: 00 = sleep, 01/10 = forced, 11 = normal

### CONFIG bits (0xF5)

- `t_sb`: standby time between measurements
- `filter`: IIR filter coefficient
- `spi3w_en`: 1 = enable 3-wire SPI

## Calibration Data (Read-only)

| Address Range | Description                               |
| ------------- | ----------------------------------------- |
| 0x88 - 0xA1   | Calibration data (temperature & pressure) |
| 0xE1 - 0xF0   | NVM mirror of config/calibration          |

## Raw Data Output (Read-only)

| Address | Register   | Description            |
| ------- | ---------- | ---------------------- |
| 0xF7    | PRESS_MSB  | Pressure MSB           |
| 0xF8    | PRESS_LSB  | Pressure LSB           |
| 0xF9    | PRESS_XLSB | Pressure XLSB [7:4]    |
| 0xFA    | TEMP_MSB   | Temperature MSB        |
| 0xFB    | TEMP_LSB   | Temperature LSB        |
| 0xFC    | TEMP_XLSB  | Temperature XLSB [7:4] |

## Notes

- All registers are 8-bit.
- Temperature and pressure readings are 20-bit (combine MSB, LSB, XLSB).
