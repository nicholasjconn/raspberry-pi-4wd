#!/usr/bin/python
import time
from typing import Any
from dataclasses import dataclass

import numpy as np
import board
import busio
import digitalio
from adafruit_motor import servo, motor
from adafruit_pca9685 import PCA9685
from adafruit_bus_device import i2c_device
import adafruit_hcsr04


class ADS7830:

    def __init__(self, i2c_bus: busio.I2C, address: int = 0x48) -> None:
        """Initialize ADC object and detect the chip type."""
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)
        self.ADS7830_CMD = 0x84
        self._BUFFER = bytearray(1)

    def read_adc(self, channel: int) -> float:
        """Read voltage from the detected ADC chip.

        Args:
            channel (int): Channel number

        Returns:
            float: voltage value
        """
        command_set = self.ADS7830_CMD | ((((channel << 2) | (channel >> 1)) & 0x07) << 4)

        with self.i2c_device as i2c:
            i2c.write_then_readinto(bytes([command_set]), self._BUFFER)
            value = self._BUFFER[0]

        voltage = round(value / 255 * 3.3, 2)
        return voltage


@dataclass
class Calibration:
    zero_neck_rotation: int = 90
    zero_neck_nod: int = 90


class Robot:

    def __init__(self):
        """Initialize Robot object and setup all required devices."""

        # Initialize I2C and all I2C devices
        self.i2c = board.I2C()
        self.pwm_controller = PCA9685(self.i2c, address=0x40)
        self.pwm_controller.frequency = 50
        self.adc = ADS7830(self.i2c)
        # Initialize GPIO devices
        self.sonar = adafruit_hcsr04.HCSR04(trigger_pin=board.D27, echo_pin=board.D22)
        self.buzzer = digitalio.DigitalInOut(board.D17)
        self.buzzer.direction = digitalio.Direction.OUTPUT

        # Setup servo motors for neck
        self.neck_rotate = servo.Servo(self.pwm_controller.channels[8])
        self.neck_nod = servo.Servo(self.pwm_controller.channels[9])
        # Setup DC motors for each wheel
        self.fl_wheel = motor.DCMotor(self.pwm_controller.channels[1], self.pwm_controller.channels[0])
        self.bl_wheel = motor.DCMotor(self.pwm_controller.channels[2], self.pwm_controller.channels[3])
        self.br_wheel = motor.DCMotor(self.pwm_controller.channels[5], self.pwm_controller.channels[4])
        self.fr_wheel = motor.DCMotor(self.pwm_controller.channels[7], self.pwm_controller.channels[6])

        # Set motor to active braking mode to improve performance
        self.fl_wheel.decay_mode = motor.SLOW_DECAY
        self.bl_wheel.decay_mode = motor.SLOW_DECAY
        self.br_wheel.decay_mode = motor.SLOW_DECAY
        self.fr_wheel.decay_mode = motor.SLOW_DECAY

        # Read in calibration and set all motors to 0
        self.calibration = Calibration()
        self.all_motors(0)

    def reset(self) -> None:
        """Reset all servo motors to their zero position."""
        self.neck_rotate.angle = self.calibration.zero_neck_rotation
        self.neck_nod.angle = self.calibration.zero_neck_nod

    def set_neck(self, rotate: int = None, nod: int = None):
        """Set neck rotation and nod angle.

        Args:
            rotate (int): Rotation angle from -180 to 180
            nod (int): Nod angle from -180 to 180
        """
        if rotate is not None:
            self.neck_rotate.angle = rotate + self.calibration.zero_neck_rotation
        if nod is not None:
            self.neck_nod.angle = nod + self.calibration.zero_neck_nod

    def spin(self, speed: float, clockwise: bool):
        """Spin the robot in place.

        Args:
            speed (float): Speed value from 0 to 1
            clockwise (bool): True to spin clockwise, False to spin counter-clockwise
        """
        self.fl_wheel.throttle = speed if clockwise else -speed
        self.bl_wheel.throttle = speed if clockwise else -speed
        self.br_wheel.throttle = -speed if clockwise else speed
        self.fr_wheel.throttle = -speed if clockwise else speed

    def all_motors(self, speed: float) -> None:
        """Set speed for all motors.

        Args:
            speed (float): Speed value from -1 to 1
        """
        self.fl_wheel.throttle = speed
        self.bl_wheel.throttle = speed
        self.br_wheel.throttle = speed
        self.fr_wheel.throttle = speed

    @property
    def battery_voltage(self) -> float:
        """Get battery voltage.

        Returns:
            float: Battery voltage
        """
        return self.adc.read_adc(2) * 3

    @property
    def sonar_distance(self) -> float:
        """Get the distance from the ultrasonic sensor.

        Returns:
            float: Distance in centimeters or None if measurement failed
        """
        try:
            distance = self.sonar.distance
        except RuntimeError:
            distance = np.nan
        return distance

    def set_buzzer(self, is_on: bool) -> None:
        """Control the state of the buzzer.

        Args:
            is_on (bool): True to turn on the buzzer, False to turn it off
        """
        self.buzzer.value = is_on

    def beep(self, duration: float) -> None:
        """Beep the buzzer for a given duration.

        Args:
            duration (float): Duration in seconds
        """
        self.set_buzzer(True)
        time.sleep(duration)
        self.set_buzzer(False)

    def deinit(self):
        """Deinit the objects in Robot."""
        self.sonar.deinit()
        self.pwm_controller.deinit()

    def __enter__(self) -> 'Robot':
        """Enter the runtime context related to this object."""
        return self

    def __exit__(self, exc_type: Any, exc_value: Any, traceback: Any) -> None:
        """Exit the runtime context related to this object."""
        self.deinit()


if __name__ == '__main__':
    with Robot() as robot:
        print(f"Battery voltage: {robot.battery_voltage}")
        print(f"Sonar distance: {robot.sonar_distance}")

        robot.neck_rotate.angle = 90
        robot.neck_nod.angle = 90
        robot.all_motors(0)

        robot.set_buzzer(True)
        time.sleep(0.1)
        robot.set_buzzer(False)

        print("Done")
