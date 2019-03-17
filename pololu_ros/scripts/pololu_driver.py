#!/usr/bin/python
from __future__ import division
import serial
import struct
import time


"""Pololu driver module for motor controllers using pyserial

This module handles the lower level logic of writing to a pololu motor
controller using python.

Serial reference: https://www.pololu.com/docs/0J44/6.2.1
Get Variable reference: https://www.pololu.com/docs/0J44/6.4

Example:
Here is an example of usage. 
::
        motor0 = Pololu(0)
        motor1 = Pololu(1)
        motor2 = Pololu(2)

        motor1.forward(1600) # drive forward the motor with device number 1

"""
def clip(val, minval, maxval):
    """clip val between minval,maxval"""
    return max(min(maxval, val), minval)

class CMD:
    BAUD_SYNC = chr(0x80)  # Required to sync the baud rate for older devices
    PROTOCOL = chr(0xAA)  # First part of command to write using Pololu Protocol
    FORWARD = chr(0x05)  # Drive motor forward command
    BACKWARD = chr(0x06)  # Drive motor reverse command
    START = chr(0x03)  # Exit Safe-Start mode so the motor can run
    STOP = chr(0x60)   # Stops the motor and enters Safe-Start mode
    MOTOR_LIMIT = chr(0x22) # Set motor limits

class VAR_ID:
    ERROR_STATUS = 0
    ERROR_OCCUR = 1
    SERIAL_ERROR = 2
    LIMIT_STATUS = 3
    AN1_UNLIMITED = 12
    AN1_RAW = 13
    AN1_SCALED = 14
    AN2_UNLIMITED = 16
    AN2_RAW = 17
    AN2_SCALED = 18
    TARGET_SPEED = 20
    SPEED = 21
    BRAKE_AMOUNT = 22
    INPUT_VOLTAGE = 23
    TEMPERATURE = 24
    RC_PER = 26
    BAUDRATE = 27
    SYSTIME_LOW = 28
    SYSTIME_HIGH = 29
    MAX_SPEED_FORWARD = 30
    MAX_ACCEL = 31
    MAX_DECEL = 32
    BRAKE_DURATION_FORWARD = 33
    MAX_SPEED_BACK = 36
    MAX_ACCEL_BACK = 37
    MAX_DECEL_BACK = 38
    BRAKE_DURATION_BACKWARD = 39


class Pololu(object):
    """Represents a single Pololu Simple Motor Controller in a daisy chain

    Attributes:
        dev_num (int): Device number of Pololu board to be commanded
        flip (bool): Polarity of motor. False if normal configuration
        port (str): Serial port of all devices chained
    """
    def __init__(self, dev_num, flip=False, port=None, timeout=0.1):
        """Set motor id and open serial connection if not already open"""
        if dev_num < 0 or dev_num > 127:
            raise Exception("Invalid motor id, must set to id of motor (0-127) for daisy chaining")

        self.dev_num = dev_num  # set device number to use in other commands
        self.port = "/dev/pololu{}".format(dev_num)
        #self.crc_enabled = crc_enabled # NOT FUNCTIONAL
        self.ser = serial.Serial(port=self.port, baudrate=19200, timeout=timeout)
        self.ser.write(CMD.BAUD_SYNC)  # sync old devices by writing 0x80

        self.flip = flip
        self._exit_safe_start()  # make it so pololu reacts to commands

    def __del__(self):
        """Decrement count, stop motor, and if it's the last connection, close the port"""
        self._stop_motor()  # safely stop current motor
        self.ser.close()

    # INTERNAL METHODS
    def _send_command(self, command, databyte3, databyte4):
        """Sends a two-byte command using the Pololu protocol."""
        cmd = CMD.PROTOCOL + chr(self.dev_num) + command + chr(databyte3) + chr(databyte4)
        self.ser.write(cmd)

    def _send_command_single(self, command):
        """Sends a one-byte command using the Pololu protocol."""
        cmd = CMD.PROTOCOL + chr(self.dev_num) + command
        self.ser.write(cmd)

    def _send_command_three(self, command, databyte3, databyte4, databyte5):
        """Sends a three-byte command using the Pololu protocol"""
        cmd = CMD.PROTOCOL + chr(self.dev_num) + command + chr(databyte3) + chr(databyte4) + chr(databyte5)
        self.ser.write(cmd)

    def _exit_safe_start(self):
        """Exit safe start so you can freely send commands to Pololu.
        This must be run before run other commands."""
        self._send_command_single(CMD.START)

    def _stop_motor(self):
        """Immediately stops the motor and enters safe start mode"""
        self._send_command_single(CMD.STOP)

    def _get_variable(self, variable_id):
        """Get variable from pololu device"""
        cmd = CMD.PROTOCOL + chr(self.dev_num) + chr(0x21) + chr(variable_id)
        self.ser.write(cmd)

        low_byte = self.ser.read()
        # if the board didn't write anything to serial (due to timeout probably), bail
        if (low_byte == ''): 
            self.ser.flushInput()
            return None
        low_byte = ord(low_byte) # variable low byte
        high_byte = 256 * ord(self.ser.read())
        return high_byte + low_byte


    # USER METHODS
    # VARIABLE GETTERS
    def get_error_status(self):
        """Get the error status of the device. Return string `OK` if all good,
        else return the string of the exact error."""
        val = self._get_variable(VAR_ID.ERROR_STATUS)
        if val == 0:
            return "OK"
        else:
            error_msg = "ERROR STATUS: "
            error_msg += "safe start mode, " if val & 2**0 else ""
            error_msg += "request channel invalid, " if val & 2**1 else ""
            error_msg += "serial error, " if val & 2**2 else ""
            error_msg += "command timeout, " if val &  2**3 else ""
            error_msg += "limit/kill switch, " if val & 2**4 else ""
            error_msg += "low vin, " if val & 2**5 else ""
            error_msg += "high vin, " if val & 2**6 else ""
            error_msg += "over temperature, " if val & 2**7 else ""
            error_msg += "motor drive error, " if val & 2**8 else ""
            error_msg += "err line high, " if val & 2**9 else ""
            error_msg = error_msg[:-2]
            return error_msg

    def get_serial_errors(self):
        """Check if there were any serial errors, if none return "OK, else
        give return a listing the error that occured, such as a FRAME error"""
        val = self._get_variable(VAR_ID.SERIAL_ERROR)
        if val == 0:
            return "OK" # No errors
        else:
            # Create an error message. If any of the bits are set, their
            # error message gets appended, else an empty string is appended
            error_msg = "SERIAL ERROR: "
            error_msg += "frame, " if val & 2**1 else ""
            error_msg += "noise, " if val & 2**2 else ""
            error_msg += "rx overrun, " if val & 2**3 else ""
            error_msg += "format, " if val & 2**4 else ""
            error_msg += "crc, " if val & 2**5 else ""
            error_msg = error_msg[:-2] # remove the last comma and space
            return error_msg

    def get_limit_statuses(self):
        """Return a list of two bools, which are true if the limit switches
        (AN1 and AN2) are active"""
        val = self._get_variable(VAR_ID.LIMIT_STATUS)
        an1_active = bool(val & 2**7) # bit mask to check if 7th bit is set
        an2_active = bool(val & 2**8) # bit mask to check if 8th bit is set
        return an1_active, an2_active

    def get_target_speed(self):
        """Motor target speed (-3200 to +3200) requested by the controlling interface."""
        unsigned = self._get_variable(VAR_ID.TARGET_SPEED)
        if (unsigned > 3200):
            signed = unsigned - 2**16
        else:
            signed = unsigned
        return signed

    def get_speed(self):
        """Current speed of the motor (-3200 to +3200), 16 bit"""
        unsigned = self._get_variable(VAR_ID.SPEED)
        if (unsigned > 3200):
            signed = unsigned - 2**16
        else:
            signed = unsigned
        return unsigned

    def get_input_voltage(self):
        """Get the measured voltage on the VIN pin"""
        return self._get_variable(VAR_ID.INPUT_VOLTAGE) / 994.0 # some arbitrary constant from the boards

    def get_board_temperature(self):
        """Board temperature as measured by a temperature sensor near
        the motor driver. Returned in degrees Celsius"""
        return self._get_variable(VAR_ID.TEMPERATURE)

    def get_an1(self):
        """Read analog voltage level. 65535 means Error Max/Min or disconnect"""
        return self._get_variable(VAR_ID.AN1_RAW)

    def get_an2(self):
        """Read analog voltage level. 65535 means Error Max/Min or disconnect"""
        return self._get_variable(VAR_ID.AN2_RAW)

    def get_baud(self):
        """Read the baudrate of the pololu"""
        return self._get_variable(VAR_ID.BAUDRATE)

    def set_motor_limits(self, max_accel, max_deccel):
        """Set limits on motor acceleration"""
        if not isinstance(max_accel, int) or not isinstance(max_deccel, int):
            raise ValueError("Must use int set limits")

        max_accel = clip(max_accel, 0, 3200)
        max_deccel = clip(max_deccel, 0, 3200)

        accel_b1 = max_accel % 128
        accel_b2 = max_accel // 128

        deccel_b1 = max_deccel % 128
        deccel_b2 = max_deccel // 128

        # max speed = self._send_command_three(CMD.MOTOR_LIMIT, 0, 0, 0)
        self._send_command_three(CMD.MOTOR_LIMIT, 1, accel_b1, accel_b2)
        self._send_command_three(CMD.MOTOR_LIMIT, 2, deccel_b1, deccel_b2)

    # USER CONTROL
    def forward(self, speed):
        """Drive motor forward at specified speed (0 to 3200)"""
        if not isinstance(speed, int):
            raise ValueError("Must use int to drive motor")

        speed = clip(speed, 0, 3200)
        self._exit_safe_start()
        # This is how the documentation recommends doing it.
        # The 1st byte will be from 0 to 31
        # The 2nd byte will be from 0 to 100
        cmd1 = speed % 32
        cmd2 = speed // 32
        if self.flip:
            self._send_command(CMD.BACKWARD, cmd1, cmd2)  # low bytes, high bytes
        else:
            self._send_command(CMD.FORWARD, cmd1, cmd2)  # low bytes, high bytes

    def backward(self, speed):
        """Drive motor backward (reverse) at specified speed (0 to 3200)"""
        if not isinstance(speed, int):
            raise ValueError("Must use int to drive motor")

        speed = clip(speed, 0, 3200)
        self._exit_safe_start()
        cmd1 = speed % 32
        cmd2 = speed // 32
        if self.flip:
            self._send_command(CMD.FORWARD, cmd1, cmd2)  # low bytes, high bytes
        else:
            self._send_command(CMD.BACKWARD, cmd1, cmd2)  # low bytes, high bytes

    def drive(self, speed):
        """Drive motor in direction based on speed (-3200, 3200)"""
        if speed < 0:
            self.backward(-speed)
        else:
            self.forward(speed)

    def stop(self):
        """Stop the motor"""
        self._stop_motor()


##    def __bitrev(self,bytes):
##        """
##        Creates a lookup table of reversed bit orders
##
##        Input:
##           bytes -- tuple -- tuple of 1 byte values to be reversed
##        Output:
##           bitrev_table -- dict
##        """
##        bytes = sum(bytes)         # Sums the bytes
##        bin_repr = bin(bytes)[2:]  # Convert to binary string, remove "0b" at the beginning of the string
##        bin_repr = bin_repr[::-1]  # Reverse all digits
##        bin_repr = "0b%s" % bin_repr
##
##        return int(bin_repr,2)     # Convert back to int, and return
##
##
##    def crc7(self,cmd_str):
##        """
##        Calculates and appends the Cyclic Redundancy Check (CRC7) byte for error checking
##        """
##        l = len(cmd_str)
##
##        int_tuple = struct.unpack('B'*len(cmd_str), cmd_str)
##        divd = self.__bitrev(int_tuple)
##
##        if(l>4):
##            print " This CRC function currently does not support strings > 4 chars"
##            return 0
##
##        divd = self.__bitrev(ord(cmd_str[0]))
##
##            # put the chars in an integer
##        for i in range(1,l):
##              new = self.__bitrev(ord(cmd_str[i]))
##              divd <<= 8
##              divd = divd | new
##
##                #crc = 0b10001001<<(8*(l-1))
##              #hex instead
##              crc = int('10001001',2)<<(8*(l-1)) #J binary literals don't work in python 2.5
##              lsbcheck = 0x80 << (8*(l-1))
##
##              for i in range(0,8*l):
##                  if(divd & lsbcheck == lsbcheck):
##                      divd = divd ^ crc
##                      divd = divd << 1
##                  else:
##                      divd = divd<<1
##
##              divd = divd>>(8*(l-1))
##              divd = self.__bitrev(divd)
##              s = chr(divd & 0xff)
##        return cmd_str + s

