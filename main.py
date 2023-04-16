import time
import cmd
import json
import os
from dataclasses import asdict

import numpy as np
import getch
import robot


def sweep_distances(robot_obj: robot.Robot, hold_time: float = 0.2) -> tuple[list]:
    """Rotate the head of the robot and get the distance at each 10 degrees.

    The function will rotate the robot's head horizontally from -45 to 45 degrees,
    at intervals of 10 degrees, and measure the distance to the nearest object
    using the robot's sonar sensor. After the sweep is completed, the robot's head
    is returned to the center position.

    Args:
        robot_obj (robot.Robot): The robot object representing the physical robot.
        hold_time (float): The time to wait after rotating the head to a new angle
                           before measuring the distance. This is to allow the sonar
                           sensor to settle. The default value is 0.2 seconds.

    Returns:
        tuple[list]: A tuple containing two lists. The first list contains the
                     distances measured at each angle, and the second list contains
                     the corresponding angles at which the distances were measured.
                     The two lists have the same length.

    Example:
        >>> robot_obj = robot.Robot()
        >>> distances, angles = sweep_distances(robot_obj)
        >>> print(distances)
        [32.1, 28.5, 25.7, 23.4, 21.8, 21.2, 21.8, 23.3, 25.6, 28.3]
        >>> print(angles)
        [-45, -35, -25, -15, -5, 5, 15, 25, 35, 45]
    """
    distances = []
    angles = range(-45, 55, 10)
    for d in angles:
        robot_obj.set_neck(d, 0)
        time.sleep(hold_time)
        distances.append(robot_obj.sonar_distance)
    robot_obj.set_neck(0, 0)
    return distances, angles


def get_furthest_distance(robot_obj: robot.Robot, threshold: float = 20) -> tuple[float]:
    """
    Find the furthest distance and corresponding angle during a sweep of the robot's head.

    The function performs a sweep of the robot's head, measures distances, and finds the
    maximum distance. If the maximum distance is less than the given threshold, the function
    checks for NaN distances and returns the angle corresponding to the first or last NaN
    distance, depending on which angle is larger in magnitude.

    Args:
        robot_obj (robot.Robot): The robot object representing the physical robot.
        threshold (float, optional): The minimum distance for considering a valid maximum
                                      distance. Defaults to 20.

    Returns:
        tuple[float]: A tuple containing the furthest distance and the corresponding angle
                      during the sweep.

    Example:
        >>> robot_obj = robot.Robot()
        >>> distance, angle = get_furthest_distance(robot_obj)
        >>> print(distance)
        32.1
        >>> print(angle)
        -45
    """
    distances, angles = sweep_distances(robot_obj)
    max_index = np.nanargmax(distances)

    # If we still have a max distance < threshold, check to see if any of the distances were NaN
    if distances[max_index] < threshold:
        if np.isnan(distances).any():
            # Find the index of the first and the last nan
            first_nan_index = np.where(np.isnan(distances))[0][0]
            last_nan_index = np.where(np.isnan(distances))[0][-1]

            # If the angle is bigger on the last NaN, then set that to the max index,
            # otherwise, do the first
            if np.abs(angles[last_nan_index]) > np.abs(angles[first_nan_index]):
                max_index = last_nan_index
            else:
                max_index = first_nan_index

    return distances[max_index], angles[max_index]


def rotate_to_target_distance(robot_obj: robot.Robot,
                              target_angle: int,
                              target_distance: float,
                              retry_timeout: float = 5,
                              max_duration: float = 15) -> None:
    """
    Rotate the robot to face the target distance and angle.

    The function rotates the robot in the direction of the target_angle until it gets
    the target_distance. If the target_distance is not reached within the retry_timeout,
    the robot will beep and update the target_distance. The function has an absolute
    timeout of max_duration, after which the robot will give up and beep twice.

    Args:
        robot_obj (robot.Robot): The robot object representing the physical robot.
        target_angle (int): The angle to face the target.
        target_distance (float): The target distance to maintain from the target.
        retry_timeout (float, optional): The timeout to retry distance update in seconds. Defaults to 5.
        max_duration (float, optional): The absolute timeout in seconds. Defaults to 15.

    Returns:
        None
    """
    # If target_distance is nan, set it to a distance of 20
    if np.isnan(target_distance) or target_distance < 20:
        target_distance = 20

    # Rotate the whole robot in the direction of the target_angle until we get the target_distance
    robot_obj.spin(speed=1, clockwise=(target_angle < 0))
    furthest_distance = 0
    start_time = time.time()
    absolute_start_time = time.time()

    # Every 100 ms check distance
    while True:
        distance = robot_obj.sonar_distance
        # Update current furthest_distance, in case we can't find the target_distance
        if distance > furthest_distance:
            furthest_distance = distance
        # If we found the target distance, stop and return
        if distance > target_distance:
            robot_obj.all_motors(0)
            return
        # If we haven't found the target distance, check if we've timed out before resetting target
        if time.time() - start_time > retry_timeout:
            robot_obj.beep(0.1)
            start_time = time.time()
            target_distance = furthest_distance - 5
            furthest_distance = 0
        # Check for absolute timeout
        if time.time() - absolute_start_time > max_duration:
            robot_obj.beep(0.1)
            time.sleep(0.1)
            robot_obj.beep(0.1)
            return

        time.sleep(0.1)


class RobotCLI(cmd.Cmd):
    intro = "Welcome to the Robot CLI. Type 'help' or '?' to list commands.\n"
    prompt = "(robot) "

    def __init__(self, robot_obj: robot.Robot):
        super().__init__()
        self.robot = robot_obj

    def do_reset(self, arg):
        """Reset the robot."""
        print("Resetting the robot...")
        self.robot.reset()

    def do_wander(self, arg):
        """Start the robot wandering."""

        print("Robot is starting to wander...")
        while True:
            # Go straight checking distance ever 100 ms, move forward until we are 20 cm from an object
            self.robot.all_motors(0.5)
            while True:
                distance = self.robot.sonar_distance
                if distance < 20:
                    self.robot.all_motors(0)
                    break
                time.sleep(0.1)
            # Rotate head and get furthest distance to the left or right, and rotate towards it
            distance, angle = get_furthest_distance(self.robot)
            # If too close, we are stuck
            if distance > 20:
                print("I'm stuck!")
                for _ in range(3):
                    robot_obj.beep(0.1)
                    time.sleep(0.5)
                    return
            print(f'Max distance: {distance} at {angle} degrees')
            rotate_to_target_distance(self.robot, angle, distance)

    def do_calibration(self, arg):
        """Calibrate the robot."""
        print("Press return without entering any text to finish a step.")

        print("Press 'wasd' keys to move the neck left, right, up or down and 'Enter' to exit.")
        # Repeatedly ask the user to move the neck left or right until Enter is pressed
        while True:
            direction = getch.getch()  # Get a single character without waiting for Enter
            if direction == '\n' or direction == '\r':  # Break the loop if Enter is pressed
                break
            elif direction == 'a':
                robot_obj.neck_rotate.angle -= 1
            elif direction == 'd':
                robot_obj.neck_rotate.angle += 1
            elif direction == 's':
                robot_obj.neck_nod.angle -= 1
            elif direction == 'w':
                robot_obj.neck_nod.angle += 1
            # Clear the line and print the current angle without echoing the character
            print(
                f"\rCurrent neck angles: {self.robot.neck_rotate.angle:0.0f} H {self.robot.neck_nod.angle:0.0f} V",
                end="    ")
        print(f"\nFinal neck angles: {self.robot.neck_rotate.angle:0.0f} H {self.robot.neck_nod.angle:0.0f} V")
        self.robot.calibration.zero_neck_rotation = self.robot.neck_rotate.angle
        self.robot.calibration.zero_neck_nod = self.robot.neck_nod.angle

        # Save the calibration to a JSON file
        with open("calibration.json", "w") as f:
            json.dump(asdict(self.robot.calibration), f)

    def do_state(self, arg):
        """Print the state of the robot."""
        print(f"Neck rotation: {self.robot.neck_rotate.angle:0.0f}")
        print(f"Neck nod: {self.robot.neck_nod.angle:0.0f}")
        print(f"Sonar distance: {self.robot.sonar_distance:0.0f}")
        print(f"Battery voltage: {self.robot.battery_voltage:0.2f}")

    def do_EOF(self, arg):
        """Exit the command line interface."""
        return True

    def do_exit(self, arg):
        """Exit the command line interface."""
        return True


def load_calibration(robot_obj: robot.Robot) -> None:
    """
    Load calibration data from a JSON file and apply it to the robot object.

    This function checks if a "calibration.json" file exists. If it does, the function
    loads the calibration data, applies it to the robot object, and resets the robot.
    If the file does not exist, the function does nothing.

    Args:
        robot_obj (robot.Robot): The robot object representing the physical robot.

    Returns:
        None
    """
    if os.path.exists("calibration.json"):
        with open("calibration.json", "r") as f:
            calibration_data = json.load(f)
        robot_obj.calibration = robot.Calibration(**calibration_data)
        robot_obj.reset()


if __name__ == "__main__":
    with robot.Robot() as robot_obj:
        load_calibration(robot_obj)
        RobotCLI(robot_obj).cmdloop()
