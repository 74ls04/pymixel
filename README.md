# Introduction
Pymixel provides a 1-to-1 mapping of the control tables of the Dynamixel MX-28. Pymixel is a transparent helper layer and does not do any data conversion or validation.

Property names are the same as the data names in each [control table](https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/#control-table-of-eeprom-area) but are all lowercase with spaces replaced by the underscore ( _ ) character. **The only exception is "Secondary(Shadow) ID" which is represented as "secondary_id".**

Refer to the [e-manual](https://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/#present-temperature) for a full description of the properties and the data ranges/enumerations.


# Dependencies

This module depends on:

* [dynamixel-sdk](https://pypi.org/project/dynamixel-sdk/)

# Usage Example

```python
import pymixel
import time

motor = Pymixel("/dev/ttyUSB0", 1, baudrate=57600)
motor.torque_enable = 1

while True:
    print(f'Voltage is: {motor.present_input_voltage / 10} V')
    print(f'Temperature is {motor.present_temperature} degrees C')
    time.sleep(1)

```