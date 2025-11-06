## Overview

This is a general-purpose Cooling Control Module that relies on a single digital temperature reading, and provides two independent fan trigger signals. User-set thresholds trigger either a LOW or HIGH speed, with a physical switch to override temperature-dependent switching.

The underlying algorithm is straightforward, with only a few conditions:

- Temperature is below `LOW` threshold : system is heating, so do nothing until `OPTIMAL` or higher temperature is reached.
- Temperature grows above `LOW` or `HIGH` threshold -> trigger `LOW` or `HIGH` speed fan, and `BUFFER` at the highest level reached until the sytem temperature returns to `OPTIMAL` or lower.

---Overrides---
- An external request, eg. A/C fan turns on, triggering a `LOW` speed condition
- A manual override, via physical switch, directly applies a signal to either the `LOW` or `HIGH` fans


The Coolant Temperature Controller Module consists of the following hardware and software features:

	- OLED displays (one hub modules for all inputs and outputs, and a remote module with UI buttons, manual overrides, and a hardware reset button)
	- Historical reporting (a software UI bar graph that plots temperature data over time)
	- `LOW` / `HIGH` speed Trigger Temperature (user-configurable thresholds)
	--	`LOW` setting is constrained from `optimalTemperature` sensor temperature to current `highSpeedTriggerTemperature`
	--	`HIGH` setting is constrained from `lowSpeedTriggerTemperature` sensor temperature to current `SENSOR_MAX_TEMPERATURE`

## Hardware

### Sensor

DS18B20 temperature sensor

### Microcontroller

Name: `SEEEDUINO XIAO RP2040`

### Displays

OLED Display : PRIMARY : 128x64 I2C : Address: 0x3C
OLED Display : SECONDARY : 128x64 I2C : Address: 0x3D
OLED Pin Mapping:
	- SDA:  `D4`
	- SCL:  `D5`

* XIAO nuance: use 'D[number]' for pins eg. 'D1'

### Others

- Momentary push buttons
- Two-position switch
- Screw Terminals
- RJ45 Connector
- 12V mini relays
- Resistors, diodes, LEDs, NPN transistors
- Housing


### Build System

#### Board Manager
Source: https://github.com/earlephilhower/arduino-pico
Board Manager: https://arduino-pico.github.io/package_rp2040_index.json (add to preferences in Arduino IDE)
Board Manager Name: Raspberry Pi Pico/RP2040 by Earle Philhower
Version: 5.4.2

#### Housing and Circuit

(Coming Soon)

#### Software Libraries

`ezButton`: 1.0.6
`DallasTemperature` : 4.0.5
`Adafruit_GFX`: 1.12.3
`Adafruit_SSD1306`: 2.5.15
