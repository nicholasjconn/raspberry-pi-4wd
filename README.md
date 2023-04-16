# Freenove 4WD Smart Car Kit for Raspberry Pi

A simple 4-wheeled robot built using the Freenove 4WD Smart Car Kit for Raspberry Pi. The wheels are mounted to a custom PCB that includes motor drivers and a few other circuits. Two servos control the "head" which contains a spot for an ultrasonic distance sensor (HC-SR04) and a Raspberry Pi camera. The Raspberry Pi connects to the circuit board with a custom jumper board. This kit requires a Raspberry Pi, and this repository uses a Raspberry Pi 4. You can purchase the robot (without the Raspberry Pi) at https://amzn.to/3KJOR2E.

Freenove is incredibly helpful, and I highly recommend this kit for learning purposes or as a proof of concept. The mechanical performance may not be as high quality as expected, but it is a great starting point. Freenove also provides a Github repository (https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi) containing all instructions and the code needed to set up and run the robot.

Instead of using the official repository, this project was built (with the help of Chat-GPT) to use the Adafruit CircuitPython libraries on the Raspberry Pi. It does not use the IR sensors on the robot, which are a bit too low to the ground to work effectively in my house. It also does not use the camera.

This repository contains a CLI tool that allows the robot to be calibrated and wander around a house (albeit, ineffectively). The code is not super smart and is used to illustrate super simple behavior.

## Setup

1. Follow the setup for CircuitPython first, as it installs some required site packages: https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/installing-circuitpython-on-raspberry-pi

2. Set up a virtual environment and activate it:

```
python -m venv --system-site-packages .env
source .env/bin/activate
```

3. Install the required Python packages:

```
pip install -r requirements.txt
```


## Code Structure

1. **Robot class**: The `robot.py` file contains the `Robot` class that represents the physical robot. This class includes methods to control the robot's motors, servos, and sensors.

2. **Command Line Interface (CLI)**: The `main.py` file contains the `RobotCLI` class, a command-line interface for interacting with the robot. The CLI allows you to calibrate the robot, view its state, and control its movement.

3. **Robot behavior**: The `main.py` file also contains functions that define the robot's behavior, such as `sweep_distances`, `get_furthest_distance`, and `rotate_to_target_distance`. These functions are used by the CLI to enable the robot to wander around a house.

## Usage

After setting up the environment, run the CLI tool with the following command:

```
python main.py
```

This will open the Robot CLI, where you can enter commands to control the robot, such as calibrating its servos or making it wander around.

### Available CLI Commands

1. `calibrate`: Calibrate the robot's servos to ensure they are centered properly.
2. `state`: Display the current state of the robot, including motor speeds and servo positions.
3. `wander`: Make the robot wander around the house while avoiding obstacles.
4. `exit`: Exit the Robot CLI and stop the robot.

### Example Usage

<pre>
$ python main.py
Welcome to the Robot CLI!
> calibrate
Calibrating servos...
> state
Motor speeds: left=0, right=0 | Servo positions: left=90, right=90
> wander
The robot is now wandering around the house. Press Ctrl+C to stop.
> exit
Exiting the Robot CLI. Goodbye!
</pre>

## Contributing

Feel free to submit pull requests or open issues if you have any improvements, bug fixes, or suggestions. Your contributions are always welcome.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
