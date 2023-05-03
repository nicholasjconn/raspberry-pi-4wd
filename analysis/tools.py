import json
from dataclasses import dataclass, field
from typing import Optional, Tuple, Union

import pandas as pd
import numpy as np
from scipy import signal, optimize
from scipy.spatial.transform import Rotation as R
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
import matplotlib.pyplot as plt
import plotly.graph_objs as go

GRAVITY = 9.80339


def run_butter_filter(in_signal: np.ndarray,
                      sample_rate: float,
                      highpass: Optional[float] = None,
                      lowpass: Optional[float] = None,
                      order: int = 2) -> np.ndarray:
    """Apply a Butterworth filter to the input signal.

    Args:
        in_signal (np.ndarray): Input signal to be filtered.
        sample_rate (float): Sampling rate of the input signal.
        highpass (Optional[float], optional): Highpass filter cutoff frequency. Defaults to None.
        lowpass (Optional[float], optional): Lowpass filter cutoff frequency. Defaults to None.
        order (int, optional): Order of the Butterworth filter. Defaults to 2.

    Raises:
        NotImplementedError: Raised when neither highpass nor lowpass cutoff frequencies are provided.

    Returns:
        np.ndarray: Filtered output signal.
    """
    # Determine the filter type and cutoff frequency based on provided parameters
    if lowpass is None and highpass is not None:
        filter_cutoff = [highpass]
        f_type = 'highpass'
    elif highpass is None and lowpass is not None:
        filter_cutoff = [lowpass]
        f_type = 'lowpass'
    elif highpass is not None and lowpass is not None:
        filter_cutoff = [highpass, lowpass]
        f_type = 'bandpass'
    else:
        raise NotImplementedError

    # Design the Butterworth filter
    sos = signal.butter(order, filter_cutoff, f_type, fs=sample_rate, output='sos')

    # Apply the filter to the input signal using forward-backward (filtfilt) filtering
    return signal.sosfiltfilt(sos, in_signal)


def read_data_file(file_path: str,
                   lowpass: Optional[float] = None,
                   radians: bool = True,
                   settle_time: float = 2) -> Tuple[np.ndarray, np.ndarray, Optional[np.ndarray], Optional[float]]:
    """
    Read data from a JSON file and preprocess the accelerometer and gyroscope data.

    Args:
        file_path (str): Path to the JSON data file.
        lowpass (Optional[float], optional): Lowpass filter cutoff frequency. Defaults to None.
        radians (bool, optional): If True, convert gyroscope data to radians. Defaults to True.
        settle_time (float, optional): Time in seconds to discard at the beginning of the data. Defaults to 2.

    Returns:
        Tuple[np.ndarray, np.ndarray, Optional[np.ndarray], Optional[float]]: Preprocessed accelerometer, gyroscope, and temperature data, and the temperature sample rate.
    """
    with open(file_path) as f:
        data = json.load(f)

    accelerometer = np.array(data['accelerometer'])
    gyroscope = np.array(data['gyro'])

    # Convert gyroscope data to radians if requested
    if radians:
        gyroscope = gyroscope * np.pi / 180

    # Get sample rate from data
    if 'sample_rate' in data:
        sample_rate = data['sample_rate']

    # Filter the accelerometer data
    if lowpass is not None:
        accelerometer[:, 0] = run_butter_filter(accelerometer[:, 0], sample_rate, lowpass=lowpass, order=3)
        accelerometer[:, 1] = run_butter_filter(accelerometer[:, 1], sample_rate, lowpass=lowpass, order=3)
        accelerometer[:, 2] = run_butter_filter(accelerometer[:, 2], sample_rate, lowpass=lowpass, order=3)

    # Remove first seconds of data
    accelerometer = accelerometer[int(sample_rate * settle_time):]
    gyroscope = gyroscope[int(sample_rate * settle_time):]

    # Get temperature data if available
    temperature = None
    temperature_sample_rate = None
    if 'temperature' in data and 'temperature_sample_rate' in data:
        temperature = np.array(data['temperature'])
        temperature_sample_rate = data['temperature_sample_rate']
        temperature = temperature[int(temperature_sample_rate * settle_time):]

    return accelerometer, gyroscope, sample_rate, temperature, temperature_sample_rate


def plot_data(accelerometer: np.ndarray,
              gyro: np.ndarray,
              sample_rate: int,
              title: Optional[str] = None,
              temperature: np.ndarray = None,
              temperature_sample_rate: int = None) -> None:
    """Plot accelerometer and gyroscope data as a function of time.

    Args:
        accelerometer (np.ndarray): Accelerometer data, a 2D array with shape (N, 3), where N is the number of samples.
        gyro (np.ndarray): Gyroscope data, a 2D array with shape (N, 3), where N is the number of samples.
        sample_rate (int): Sampling rate of the IMU data in Hz.
        title (Optional[str], optional): Title for the plot. Defaults to None.

    Returns:
        None
    """
    # Create time vector based on sample rate and data length
    t = np.arange(len(accelerometer)) / sample_rate

    # Create a figure with two subplots sharing the same x-axis
    if temperature is not None and temperature_sample_rate is not None:
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(8, 5))
    else:
        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(8, 4))
    # Plot accelerometer data in the first subplot
    ax1.plot(t, accelerometer[:, 0], label='x')
    ax1.plot(t, accelerometer[:, 1], label='y')
    ax1.plot(t, accelerometer[:, 2], label='z')
    ax1.set_ylabel('Acceleration (m/s^2)')
    ax1.legend()
    # Plot gyroscope data in the second subplot
    ax2.plot(t, gyro[:, 0] * 360 / (2 * np.pi), label='x')
    ax2.plot(t, gyro[:, 1] * 360 / (2 * np.pi), label='y')
    ax2.plot(t, gyro[:, 2] * 360 / (2 * np.pi), label='z')
    ax2.set_ylabel('Angular velocity (dps)')
    ax2.legend()

    # Temperature plot (if temperature data is provided)
    if temperature is not None and temperature_sample_rate is not None:
        t_temp = np.arange(len(temperature)) / temperature_sample_rate
        ax3.plot(t_temp, temperature, label='Temperature', color='k')
        ax3.set_ylabel('Temperature (°C)')
        ax3.legend()
        ax3.set_xlabel('Time (s)')
    else:
        ax2.set_xlabel('Time (s)')

    # Add a title to the figure if provided
    if title is not None:
        fig.suptitle(title)
    fig.tight_layout()
    plt.show()


class Location3d:
    # Names to use in the state vector
    state_names = [
        'x_w', 'y_w', 'z_w', # Position (world frame)
        'vx_w', 'vy_w', 'vz_w', # Velocity (world frame)
        'ax_b', 'ay_b', 'az_b', # Acceleration (body frame)
        'qa_w', 'qb_w', 'qc_w', 'qd_w', # Orientation quaternions (world frame)
        'wpi_b', 'wro_b', 'wya_b' # Angular velocity (body frame)
    ]

    def __init__(self, sample_rate, init_r: R = None):
        self.dt = 1/sample_rate
        self.dim_x = len(Location3d.state_names)
        init_state = [0]*self.dim_x
        if init_r is None:
            init_r = R.identity()
        # We need an initial rotation quaternion and gravity
        init_state[9:13] = init_r.as_quat()
        init_state[8] = GRAVITY
        self.state_history = [init_state]

    def add_measurement(self, accelerometer, gyroscope):
        # Unpack the last state
        x_w, y_w, z_w, vx_w, vy_w, vz_w, ax_b, ay_b, az_b, qa_w, qb_w, qc_w, qd_w, wpi_b, wro_b, wya_b = self.state_history[-1]
        # Get the measurements from the body
        ax_b, ay_b, az_b = accelerometer
        wpi_b, wro_b, wya_b = gyroscope
        # Insert them into the state
        x = x_w, y_w, z_w, vx_w, vy_w, vz_w, ax_b, ay_b, az_b, qa_w, qb_w, qc_w, qd_w, wpi_b, wro_b, wya_b
        # Update the state
        x_new = Location3d.state_transition_function(x, self.dt)
        self.state_history.append(x_new)

    @staticmethod
    def state_transition_function(x, dt):
        # Extract state variables
        x_w, y_w, z_w, vx_w, vy_w, vz_w, ax_b, ay_b, az_b, qa_w, qb_w, qc_w, qd_w, wpi_b, wro_b, wya_b = x

        # Convert the acceleration in the body frame to the world frame
        rotation_w = R.from_quat([qa_w, qb_w, qc_w, qd_w])
        # TODO don't know if this is inverse or not
        # ax_w, ay_w, az_w = rotation_w.inv().apply(np.array([ax_b, ay_b, az_b]))
        ax_w, ay_w, az_w = rotation_w.apply(np.array([ax_b, ay_b, az_b]))

        # Update the linear velocity
        vx_w += ax_w * dt
        vy_w += ay_w * dt
        vz_w += (az_w - GRAVITY) * dt
        # Update the position
        x_w += vx_w * dt
        y_w += vy_w * dt
        z_w += vz_w * dt

        # Calculate the change in rotation
        angular_velocity = np.array([wpi_b, wro_b, wya_b])
        rotation_vector = angular_velocity * dt
        delta_rotation = R.from_rotvec(rotation_vector)

        # Update the current orientation by multiplying with the delta rotation
        rotation_w = rotation_w * delta_rotation
        qa_w, qb_w, qc_w, qd_w = rotation_w.as_quat()

        # New state vector (x_new)
        return np.array([
            x_w, y_w, z_w, vx_w, vy_w, vz_w, ax_b, ay_b, az_b, qa_w, qb_w, qc_w, qd_w, wpi_b, wro_b, wya_b
        ])

    def get_state_history_df(self):
        df = pd.DataFrame(self.state_history, columns=Location3d.state_names)
        # Convert quantenions to euler angles in radidans
        df['ya_w'], df['pi_w'], df['ro_w'] = R.from_quat(df[['qa_w', 'qb_w', 'qc_w', 'qd_w']].values).as_euler('xyz').T
        return df

    @staticmethod
    def initial_rotation_from_accelerometer(accelerometer, sample_rate, length=0.1):
        # Assuming the robot is stationary, the acceleration vector should be pointing opposite to the gravity vector
        if length == -1:
            mean_accelerometer = np.mean(accelerometer, axis=0)
        else:
            mean_accelerometer = np.mean(accelerometer[:int(length*sample_rate)], axis=0)
        return Location3d.get_rotation_from_mean_accelerometer(mean_accelerometer)

    @staticmethod
    def get_rotation_from_mean_accelerometer(accelerometer_value):
        assert len(accelerometer_value) == 3
        magnitude = np.linalg.norm(accelerometer_value)

        def _get_error(c):
            pitch, roll = c
            angle = R.from_euler('zyx', [0, pitch, roll], degrees=False)
            error = angle.apply(accelerometer_value) - [0, 0, magnitude]
            return np.linalg.norm(error)

        res = optimize.differential_evolution(_get_error, [(-np.pi, np.pi), (-np.pi/2, np.pi/2)])
        if res.success == False:
            raise RuntimeError('Failed to find the rotation')
        pitch, roll = res.x
        return R.from_euler('zyx', [0, pitch, roll], degrees=False)


@dataclass
class UkfConfig:
    # Number of state dimensions
    state_n: int
    # Number of measurement dimensions
    measurement_n: int
    # Alpha determines the spread of the sigma points, with smaller values resulting in a tighter spread
    alpha: float = 1e-3
    # Set beta to 2 for Gaussian distributions
    beta: float = 2
    # Set kappa to 0 initially (secondary scaling parameter)
    kappa: float = 0
    # Init state
    init_r: R = None

    # When an single value, the following parameters define the diagonals, otherwise they define the full matrix
    # Measurement noise covariance
    R: Union[float, np.ndarray] = field(default_factory=lambda: 0)
    # Process noise covariance (Too slow to react? Increase Q; Too sensitive to noise? Decrease the Q)
    Q: Union[float, np.ndarray] = field(default_factory=lambda: 1e-4)
    # Initial state covariance (Small values if confident, large values between 1-10 suggestaed when not)
    P: Union[float, np.ndarray] = field(default_factory=lambda: 1e-5)

    def __post_init__(self):
        if np.isscalar(self.R):
            self.R = np.eye(self.measurement_n)*self.R
        if np.isscalar(self.Q):
            self.Q = np.eye(self.state_n)*self.Q
        if np.isscalar(self.P):
            self.P = np.eye(self.state_n)*self.P


class Location3dUkf(Location3d):
    def __init__(self, sample_rate, config: UkfConfig) -> None:
        super().__init__(sample_rate, init_r=config.init_r)

        self.ukf = UnscentedKalmanFilter(
            dim_x=self.dim_x, dim_z=6, dt=self.dt,
            fx=Location3d.state_transition_function, hx=Location3dUkf.measurement_function,
            points=MerweScaledSigmaPoints(self.dim_x, alpha=config.alpha, beta=config.beta, kappa=config.kappa)
        )

        # Initial state
        self.ukf.x = self.state_history[0]
        # Matrix configurations
        self.ukf.R = config.R
        self.ukf.Q = config.Q
        self.ukf.P = config.P

    @staticmethod
    def measurement_function(x):
        # Extract state variables
        # x_w, y_w, z_w, vx_w, vy_w, vz_w, ax_b, ay_b, az_b, pi_w, ro_w, ya_w, wpi_b, wro_b, wya_b = x
        x_w, y_w, z_w, vx_w, vy_w, vz_w, ax_b, ay_b, az_b, qa_w, qb_w, qc_w, qd_w, wpi_b, wro_b, wya_b = x
        # Measurement vector (z)
        return np.array([ax_b, ay_b, az_b, wpi_b, wro_b, wya_b])

    def add_measurement(self, accelerometer, gyroscope):
        # The measurement vector (z) from the body frame
        z_b = np.hstack((accelerometer, gyroscope))
        # Call the `predict` and `update` methods of the UnscentedKalmanFilter class
        self.ukf.predict()
        self.ukf.update(z_b)
        self.state_history.append(self.ukf.x.copy())


def rotation_to_pitch_and_roll(rotation_list: list[R]) -> tuple[list[float], list[float], list[float]]:
    """Convert a list of rotation objects to a list of pitch and roll angles

    Args:
        rotation_list (list[R]): List of rotation objects.

    Returns:
        tuple[list[float], list[float], list[float]]: List of pitch, roll, and yaw angles.
    """
    # Calculate the pitch and roll list from the rotation object list
    euler_list = np.array([rotation.as_euler('xyz', degrees=False) for rotation in rotation_list])
    pitch_list = euler_list[:, 0]
    roll_list = euler_list[:, 1]
    yaw_list = euler_list[:, 2]
    return list(pitch_list), list(roll_list), list(yaw_list)


def angles_to_cartesian(pitch, roll, yaw) -> tuple[float, float, float]:
    """Convert yaw, pitch, and roll angles to cartesian coordinates on a unit sphere

    Args:
        pitch (float): Pitch angle.
        roll (float): Roll angle.
        yaw (float): Yaw angle.

    Returns:
        tuple[float, float, float]: The x, y, and z coordinates on the unit sphere.
    """
    x = np.cos(yaw) * np.sin(pitch) - np.sin(yaw) * np.cos(pitch) * np.sin(roll)
    y = np.sin(yaw) * np.sin(pitch) + np.cos(yaw) * np.cos(pitch) * np.sin(roll)
    z = np.cos(pitch) * np.cos(roll)
    return x, y, z


def plot_rotations_on_sphere(pitch_list, roll_list, yaw_list, color_list):
    """Visualize the paired pitch and roll angles on a sphere

    Args:
        pitch_list (list[float]): List of pitch angles.
        roll_list (list[float]): List of roll angles.
        yaw_list (list[float]): List of yaw angles.
        color_list (list[str]): List of colors for each point.
    """
    # Plot the unit sphere
    pitch = np.linspace(0, 2 * np.pi, 100)
    roll = np.linspace(0, 2 * np.pi, 100)
    pitch, roll = np.meshgrid(pitch, roll)
    # Create the surface plot for the sphere
    x, y, z = angles_to_cartesian(pitch, roll, 0)
    sphere = go.Surface(x=x, y=y, z=z, opacity=0.1, showscale=False, colorscale=[[0, 'blue'], [1, 'blue']])

    # Plot red points for each pitch and roll measurement and draw lines connecting them to the center
    points_x, points_y, points_z = [], [], []
    lines = []
    for pitch, roll, yaw in zip(pitch_list, roll_list, yaw_list):
        x_point, y_point, z_point = angles_to_cartesian(pitch, roll, yaw)
        line = go.Scatter3d(
            x=[0, x_point], y=[0, y_point], z=[0, z_point],
            mode='lines',
            line=dict(width=2, color='black'),
            showlegend=False
        )
        points_x.append(x_point)
        points_y.append(y_point)
        points_z.append(z_point)
        lines.append(line)

    points = go.Scatter3d(x=points_x, y=points_y, z=points_z, mode='markers', marker=dict(size=5, color=color_list))

    # Create the plot
    layout = go.Layout(scene=dict(xaxis_title='X', yaxis_title='Y', zaxis_title='Z'))
    fig = go.Figure(data=[sphere, points] + lines, layout=layout)
    fig.show()


def find_largest_gap(pitch_list, roll_list, yaw_list) -> tuple[float, float, float]:
    """Find the angle that fills in the largest gap between all of the provided angles

    Args:
        pitch_list (list[float]): List of pitch angles.
        roll_list (list[float]): List of roll angles.
        yaw_list (list[float]): List of yaw angles.

    Returns:
        tuple[float, float, float]: The pitch, roll, and yaw angles that fill in the largest gap.
    """
    # Calculate the pitch and roll list from the rotation object list
    points = np.array([
        angles_to_cartesian(pitch, roll, yaw)
        for pitch, roll, yaw in zip(pitch_list, roll_list, yaw_list)
    ])

    def _get_loss(c):
        """ Calculate the loss, in search of the largest empty area defined by the new point at the center"""
        new_roll, new_pitch, new_yaw = c
        # Find the distance to the nearest point
        smallest_distance = np.min(np.linalg.norm(points - angles_to_cartesian(new_pitch, new_roll, new_yaw), axis=1))
        # We want to find the largest smallest distance, so I'm going to 1/smallest_distance for the loss function
        return 1/smallest_distance

    # Minimize the loss function using differential evolution
    res = optimize.differential_evolution(_get_loss, [(0, 2*np.pi)]*3, tol=1e-6, popsize=100, maxiter=1000)
    new_roll, new_pitch, new_yaw = res.x
    return new_roll, new_pitch, new_yaw
