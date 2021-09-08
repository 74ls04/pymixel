# The MIT License (MIT)
#
# Copyright (c) 2021 Paul Bupe Jr
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""
Helper library for the Dynamixel MX actuators using Protocol 2.0
Provides a 1-to-1 mapping of the entire control table as properties of an object.

* Author(s): Paul Bupe Jr
"""
from dynamixel_sdk.robotis_def import COMM_SUCCESS
from dynamixel_sdk import PacketHandler
from dynamixel_sdk import PortHandler

# MX-28 Control Tables

# fmt: off
# EEPROM table
# Name                (Address, Bytes)
MX_MODEL_NUMBER           = (0, 2)      # R
MX_MODEL_INFORMATION      = (2, 4)      # R
MX_FIRMWARE_VERSION       = (6, 1)     # R
MX_ID                     = (7, 1)     # RW
MX_BAUD_RATE              = (8, 1)     # RW
MX_RETURN_DELAY_TIME      = (9, 1)     # RW
MX_DRIVE_MODE             = (10, 1)     # RW
MX_OPERATING_MODE         = (11, 1)     # RW
MX_SECONDARY_ID           = (12, 1)     # RW
MX_PROTOCOL_TYPE          = (13, 1)     # RW
MX_HOMING_OFFSET          = (20, 4)     # RW
MX_MOVING_THRESHOLD       = (24, 4)     # RW
MX_TEMPERATURE_LIMIT      = (31, 1)     # RW
MX_MAX_VOLTAGE_LIMIT      = (32, 2)     # RW
MX_MIN_VOLTAGE_LIMIT      = (34, 2)     # RW
MX_PWM_LIMIT              = (36, 2)     # RW
MX_ACCELERATION_LIMIT     = (40, 4)     # RW
MX_VELOCITY_LIMIT         = (44, 4)     # RW
MX_MAX_POSITION_LIMIT     = (48, 4)     # RW
MX_MIN_POSITION_LIMIT     = (52, 4)     # RW
MX_SHUTDOWN               = (63, 1)     # RW

# RAM table
MX_TORQUE_ENABLE          = (64, 1)     # RW
MX_LED                    = (65, 1)     # RW
MX_STATUS_RETURN_LEVEL    = (68, 1)     # RW
MX_REGISTERED_INSTRUCTION = (69, 1)     # R
MX_HARDWARE_ERROR_STATUS  = (70, 1)     # R
MX_VELOCITY_I_GAIN        = (76, 2)     # RW
MX_VELOCITY_P_GAIN        = (78, 2)     # RW
MX_POSITION_D_GAIN        = (80, 2)     # RW
MX_POSITION_I_GAIN        = (82, 2)     # RW
MX_POSITION_P_GAIN        = (84, 2)     # RW
MX_FEEDFORWARD_2ND_GAIN   = (88, 2)     # RW
MX_FEEDFORWARD_1ST_GAIN   = (90, 2)     # RW
MX_BUS_WATCHDOG           = (98, 1)     # RW
MX_GOAL_PWM               = (100, 2)    # RW
MX_GOAL_VELOCITY          = (104, 4)    # RW
MX_PROFILE_ACCELERATION   = (108, 4)    # RW
MX_PROFILE_VELOCITY       = (112, 4)    # RW
MX_GOAL_POSITION          = (116, 4)    # RW
MX_REALTIME_TICK          = (120, 2)    # R
MX_MOVING                 = (122, 1)    # R
MX_MOVING_STATUS          = (123, 1)    # R
MX_PRESENT_PWM            = (124, 2)    # R
MX_PRESENT_LOAD           = (126, 2)    # R
MX_PRESENT_VELOCITY       = (128, 4)    # R
MX_PRESENT_POSITION       = (132, 4)    # R
MX_VELOCITY_TRAJECTORY    = (136, 4)    # R
MX_POSITION_TRAJECTORY    = (140, 4)    # R
MX_PRESENT_INPUT_VOLTAGE  = (144, 2)    # R
MX_PRESENT_TEMPERATURE    = (146, 1)    # R
# fmt: on


class Pymixel:
    """Helper class for the Dynamixel MX-Series actuators
    using Protocol 2.0
    """

    # SDK classes
    port_handler = None
    packet_handler = None
    port = None
    baudrate = None

    def __init__(self, port, mxl_id, baudrate=57600, protocol=2):
        """Create an instance of the Pymixel class"""
        self._id = mxl_id
        self.port = port
        self.baudrate = baudrate
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(protocol)

        # Open port
        if not self.port_handler.openPort():
            raise Exception(f"ERROR: Could not open port {self.port}")

        if not self.port_handler.setBaudRate(self.baudrate):
            raise Exception("ERROR: Failed to change the baudrate")

    def _read(self, register):
        """Read data from a register"""

        addr, num_bytes = register
        data = response = error = None
        if num_bytes == 1:
            data, response, error = self.packet_handler.read1ByteTxRx(
                self.port_handler, self._id, addr
            )
        elif num_bytes == 2:
            data, response, error = self.packet_handler.read2ByteTxRx(
                self.port_handler, self._id, addr
            )
        else:
            data, response, error = self.packet_handler.read4ByteTxRx(
                self.port_handler, self._id, addr
            )

        # Check response
        self._error_handler(response, error)

        return data

    def _write(self, register, value):
        """write to a register"""

        addr, num_bytes = register
        response = error = None
        if num_bytes == 1:
            response, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, self._id, addr, value
            )
        elif num_bytes == 2:
            response, error = self.packet_handler.write2ByteTxRx(
                self.port_handler, self._id, addr, value
            )
        else:
            response, error = self.packet_handler.write4ByteTxRx(
                self.port_handler, self._id, addr, value
            )

        # Check response
        self._error_handler(response, error)

    @property
    def model_number(self):
        """Return the model number."""
        return self._read(MX_MODEL_NUMBER)

    @property
    def model_information(self):
        """Return the model information."""
        return self._read(MX_MODEL_INFORMATION)

    @property
    def firmware_version(self):
        """Return the firmware version."""
        return self._read(MX_FIRMWARE_VERSION)

    @property
    def id(self):  # pylint: disable=invalid-name
        """Return the DYNAMIXEL ID."""
        return self._read(MX_ID)

    @property
    def baud_rate(self):
        """Return the serial baud rate."""
        return self._read(MX_BAUD_RATE)

    @property
    def return_delay_time(self):
        """Return the response delay."""
        return self._read(MX_RETURN_DELAY_TIME)

    @property
    def drive_mode(self):
        """Return the drive mode."""
        return self._read(MX_DRIVE_MODE)

    @property
    def operating_mode(self):
        """Return the operating mode."""
        return self._read(MX_OPERATING_MODE)

    @property
    def secondary_id(self):
        """Return the secondary ID."""
        return self._read(MX_SECONDARY_ID)

    @property
    def protocol_type(self):
        """Return the protocol type."""
        return self._read(MX_PROTOCOL_TYPE)

    @property
    def homing_offset(self):
        """Return the home position offset."""
        return self._read(MX_HOMING_OFFSET)

    @property
    def moving_threshold(self):
        """Return the velocity threshold for
        movement detection."""
        return self._read(MX_MOVING_THRESHOLD)

    @property
    def temperature_limit(self):
        """Return the maximum internal temperature limit."""
        return self._read(MX_TEMPERATURE_LIMIT)

    @property
    def max_voltage_limit(self):
        """Return the maximum input voltage limit."""
        return self._read(MX_MAX_VOLTAGE_LIMIT)

    @property
    def min_voltage_limit(self):
        """Return the minimum input voltage limit."""
        return self._read(MX_MIN_VOLTAGE_LIMIT)

    @property
    def pwm_limit(self):
        """Return the maximum PWM limit."""
        return self._read(MX_PWM_LIMIT)

    @property
    def acceleration_limit(self):
        """Return the maximum acceleration limit."""
        return self._read(MX_ACCELERATION_LIMIT)

    @property
    def velocity_limit(self):
        """Return the maximum velocity limit."""
        return self._read(MX_VELOCITY_LIMIT)

    @property
    def max_position_limit(self):
        """Return the maximum position limit."""
        return self._read(MX_MAX_POSITION_LIMIT)

    @property
    def min_position_limit(self):
        """Return the minimum position limit."""
        return self._read(MX_MIN_POSITION_LIMIT)

    @property
    def shutdown(self):
        """Return the shutdown error information."""
        return self._read(MX_SHUTDOWN)

    @property
    def torque_enable(self):
        """Return the motor torque status."""
        return self._read(MX_TORQUE_ENABLE)

    @property
    def led(self):
        """Return the status LED status."""
        return self._read(MX_LED)

    @property
    def status_return_level(self):
        """Return the status return level."""
        return self._read(MX_STATUS_RETURN_LEVEL)

    @property
    def registered_instruction(self):
        """Return the REG_WRITE instruction flag."""
        return self._read(MX_REGISTERED_INSTRUCTION)

    @property
    def hardware_error_status(self):
        """Return the hardware error status."""
        return self._read(MX_HARDWARE_ERROR_STATUS)

    @property
    def velocity_i_gain(self):
        """Return the I gain of velocity."""
        return self._read(MX_VELOCITY_I_GAIN)

    @property
    def velocity_p_gain(self):
        """Return the P gain of velocity."""
        return self._read(MX_VELOCITY_P_GAIN)

    @property
    def position_d_gain(self):
        """Return the D gain of position."""
        return self._read(MX_POSITION_D_GAIN)

    @property
    def position_i_gain(self):
        """Return the I gain of position."""
        return self._read(MX_POSITION_I_GAIN)

    @property
    def position_p_gain(self):
        """Return the P gain of position."""
        return self._read(MX_POSITION_P_GAIN)

    @property
    def feedforward_2nd_gain(self):
        """Return the 2nd gain of feed-forward."""
        return self._read(MX_FEEDFORWARD_2ND_GAIN)

    @property
    def feedforward_1st_gain(self):
        """Return the 1st gain of feed-forward."""
        return self._read(MX_FEEDFORWARD_1ST_GAIN)

    @property
    def bus_watchdog(self):
        """Return the dynamixel bus watchdog."""
        return self._read(MX_BUS_WATCHDOG)

    @property
    def goal_pwm(self):
        """Return the desired pwm value."""
        return self._read(MX_GOAL_PWM)

    @property
    def goal_velocity(self):
        """Return the desired velocity value."""
        return self._read(MX_GOAL_VELOCITY)

    @property
    def profile_acceleration(self):
        """Return the acceleration value of profile."""
        return self._read(MX_PROFILE_ACCELERATION)

    @property
    def profile_velocity(self):
        """Return the velocity value of profile."""
        return self._read(MX_PROFILE_VELOCITY)

    @property
    def goal_position(self):
        """Return the desired position."""
        return self._read(MX_GOAL_POSITION)

    @property
    def realtime_tick(self):
        """Return the count time in millisecond."""
        return self._read(MX_REALTIME_TICK)

    @property
    def moving(self):
        """Return the movement flag."""
        return self._read(MX_MOVING)

    @property
    def moving_status(self):
        """Return detailed information of movement status."""
        return self._read(MX_MOVING_STATUS)

    @property
    def present_pwm(self):
        """Return the present pwm value."""
        return self._read(MX_PRESENT_PWM)

    @property
    def present_load(self):
        """Return the present load value."""
        return self._read(MX_PRESENT_LOAD)

    @property
    def present_velocity(self):
        """Return the present velocity value."""
        return self._read(MX_PRESENT_VELOCITY)

    @property
    def present_position(self):
        """Return the present position value."""
        return self._read(MX_PRESENT_POSITION)

    @property
    def velocity_trajectory(self):
        """Return the desired velocity trajectory from profile."""
        return self._read(MX_VELOCITY_TRAJECTORY)

    @property
    def position_trajectory(self):
        """Return the desired position trajectory from profile."""
        return self._read(MX_POSITION_TRAJECTORY)

    @property
    def present_input_voltage(self):
        """Return the present input voltage."""
        return self._read(MX_PRESENT_INPUT_VOLTAGE)

    @property
    def present_temperature(self):
        """Return the present internal temperature."""
        return self._read(MX_PRESENT_TEMPERATURE)

    @id.setter
    def id(self, value):  # pylint: disable=invalid-name
        """Set the DYNAMIXEL ID."""
        self._write(MX_ID, value)

    @baud_rate.setter
    def baud_rate(self, value):
        """Set the serial baud rate."""
        self._write(MX_BAUD_RATE, value)

    @return_delay_time.setter
    def return_delay_time(self, value):
        """Set the response delay."""
        self._write(MX_RETURN_DELAY_TIME, value)

    @drive_mode.setter
    def drive_mode(self, value):
        """Set the drive mode."""
        self._write(MX_DRIVE_MODE, value)

    @operating_mode.setter
    def operating_mode(self, value):
        """Set the operating mode."""
        self._write(MX_OPERATING_MODE, value)

    @secondary_id.setter
    def secondary_id(self, value):
        """Set the secondary ID."""
        self._write(MX_SECONDARY_ID, value)

    @protocol_type.setter
    def protocol_type(self, value):
        """Set the protocol type."""
        self._write(MX_PROTOCOL_TYPE, value)

    @homing_offset.setter
    def homing_offset(self, value):
        """Set the home position offset."""
        self._write(MX_HOMING_OFFSET, value)

    @moving_threshold.setter
    def moving_threshold(self, value):
        """Set the velocity threshold for
        movement detection."""
        self._write(MX_MOVING_THRESHOLD, value)

    @temperature_limit.setter
    def temperature_limit(self, value):
        """Set the maximum internal temperature limit."""
        self._write(MX_TEMPERATURE_LIMIT, value)

    @max_voltage_limit.setter
    def max_voltage_limit(self, value):
        """Set the maximum input voltage limit."""
        self._write(MX_MAX_VOLTAGE_LIMIT, value)

    @min_voltage_limit.setter
    def min_voltage_limit(self, value):
        """Set the minimum input voltage limit."""
        self._write(MX_MIN_VOLTAGE_LIMIT, value)

    @pwm_limit.setter
    def pwm_limit(self, value):
        """Set the maximum PWM limit."""
        self._write(MX_PWM_LIMIT, value)

    @acceleration_limit.setter
    def acceleration_limit(self, value):
        """Set the maximum acceleration limit."""
        self._write(MX_ACCELERATION_LIMIT, value)

    @velocity_limit.setter
    def velocity_limit(self, value):
        """Set the maximum velocity limit."""
        self._write(MX_VELOCITY_LIMIT, value)

    @max_position_limit.setter
    def max_position_limit(self, value):
        """Set the maximum position limit."""
        self._write(MX_MAX_POSITION_LIMIT, value)

    @min_position_limit.setter
    def min_position_limit(self, value):
        """Set the minimum position limit."""
        self._write(MX_MIN_POSITION_LIMIT, value)

    @shutdown.setter
    def shutdown(self, value):
        """Set the shutdown error information."""
        self._write(MX_SHUTDOWN, value)

    @torque_enable.setter
    def torque_enable(self, value):
        """Set the motor torque status."""
        self._write(MX_TORQUE_ENABLE, value)

    @led.setter
    def led(self, value):
        """Set the status LED status."""
        self._write(MX_LED, value)

    @status_return_level.setter
    def status_return_level(self, value):
        """Set the status return level."""
        self._write(MX_STATUS_RETURN_LEVEL, value)

    @velocity_i_gain.setter
    def velocity_i_gain(self, value):
        """Set the I gain of velocity."""
        self._write(MX_VELOCITY_I_GAIN, value)

    @velocity_p_gain.setter
    def velocity_p_gain(self, value):
        """Set the P gain of velocity."""
        self._write(MX_VELOCITY_P_GAIN, value)

    @position_d_gain.setter
    def position_d_gain(self, value):
        """Set the D gain of position."""
        self._write(MX_POSITION_D_GAIN, value)

    @position_i_gain.setter
    def position_i_gain(self, value):
        """Set the I gain of position."""
        self._write(MX_POSITION_I_GAIN, value)

    @position_p_gain.setter
    def position_p_gain(self, value):
        """Set the P gain of position."""
        self._write(MX_POSITION_P_GAIN, value)

    @feedforward_2nd_gain.setter
    def feedforward_2nd_gain(self, value):
        """Set the 2nd gain of feed-forward."""
        self._write(MX_FEEDFORWARD_2ND_GAIN, value)

    @feedforward_1st_gain.setter
    def feedforward_1st_gain(self, value):
        """Set the 1st gain of feed-forward."""
        self._write(MX_FEEDFORWARD_1ST_GAIN, value)

    @bus_watchdog.setter
    def bus_watchdog(self, value):
        """Set the dynamixel bus watchdog."""
        self._write(MX_BUS_WATCHDOG, value)

    @goal_pwm.setter
    def goal_pwm(self, value):
        """Set the desired pwm value."""
        self._write(MX_GOAL_PWM, value)

    @goal_velocity.setter
    def goal_velocity(self, value):
        """Set the desired velocity value."""
        self._write(MX_GOAL_VELOCITY, value)

    @profile_acceleration.setter
    def profile_acceleration(self, value):
        """Set the acceleration value of profile."""
        self._write(MX_PROFILE_ACCELERATION, value)

    @profile_velocity.setter
    def profile_velocity(self, value):
        """Set the velocity value of profile."""
        self._write(MX_PROFILE_VELOCITY, value)

    @goal_position.setter
    def goal_position(self, value):
        """Set the desired position."""
        self._write(MX_GOAL_POSITION, value)

    def __del__(self):
        self._close()

    def _close(self):
        # Closes the port
        self.port_handler.closePort()

    def _error_handler(self, res, err):
        # Checks each request
        if res != COMM_SUCCESS:
            print(f"Abnormal response: {self.packet_handler.getTxRxResult(res)}")
        elif err != 0:
            print(f"Error: {self.packet_handler.getRxPacketError(err)}")
