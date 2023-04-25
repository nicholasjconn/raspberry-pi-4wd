import os
import traceback
import json
import argparse
import time
import signal
from multiprocessing import Process, Manager, Lock, Condition

import numpy as np
import matplotlib.pyplot as plt

import board
from adafruit_lsm6ds import Rate, AccelRange, GyroRange

import hardware


def signal_handler(signum, frame, stop_event):
    print("\nReceived signal:", signum)
    print("Stopping processes...")
    stop_event.set()


def setup():
    i2c = board.I2C()  # uses board.SCL and board.SDA
    # i2c = busio.I2C( board.SCL, board.SDA, frequency=400_000)
    sensor = hardware.Imu(i2c)

    sensor.reset()
    # Configure accelerometer and gyroscope
    sensor.accelerometer_range = AccelRange.RANGE_2G
    sensor.gyro_range = GyroRange.RANGE_250_DPS
    return i2c, sensor


def plot(accelerometer, gyro):
    # Remove first 100 ms of data
    accelerometer = accelerometer[int(417*0.1):]
    gyro = gyro[int(417*0.1):]

    # Plot the accelerometer data in one subplot and the gyro data in another with shared x axis, time axis has a sample rate of 417 Hz
    t = np.arange(len(accelerometer)) / 417
    accelerometer = np.array(accelerometer)
    gyro = np.array(gyro)
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    ax1.plot(t, accelerometer[:, 0], label='x')
    ax1.plot(t, accelerometer[:, 1], label='y')
    ax1.plot(t, accelerometer[:, 2], label='z')
    ax1.set_ylabel('Acceleration (m/s^2)')
    ax1.legend()
    ax2.plot(t, gyro[:, 0], label='x')
    ax2.plot(t, gyro[:, 1], label='y')
    ax2.plot(t, gyro[:, 2], label='z')
    ax2.set_ylabel('Angular velocity (dps)')
    ax2.set_xlabel('Time (s)')
    ax2.legend()
    fig.tight_layout()
    fig.savefig(os.path.join('tmp', 'fig.png'))


def sensor_thread(accelerometer_list, gyro_list, condition, stop_event):
    try:
        print("Starting sensor thread...")
        i2c, sensor = setup()

        print(f'Gyro Range: {sensor._gyro_range}')
        print(f'Gyro Range 125 dps: {sensor._gyro_range_125dps}')
        print(f'Cached: {sensor._cached_gyro_range}')
        print(f'LSB: {GyroRange.lsb[sensor._cached_gyro_range]}')

        sensor.fifo_start(Rate.RATE_416_HZ)

        read_period = 0.25
        read_duration = 0
    except:
        stop_event.set()
        traceback.print_exc()
        sensor.fifo_stop()
        i2c.deinit()

    while not stop_event.is_set():
        try:
            if read_duration < read_period:
                time.sleep(read_period - read_duration)
            read_start_time = time.time()
            a, g = sensor.fifo_read()

            with condition:
                accelerometer_list.extend(a)
                gyro_list.extend(g)
                condition.notify_all()

            read_duration = time.time() - read_start_time
        except:
            traceback.print_exc()
            print("Error reading sensor data")
            stop_event.set()

    sensor.fifo_stop()
    i2c.deinit()
    with condition:
        condition.notify_all()


def data_consumer(accelerometer_list, gyro_list, condition, stop_event, duration):
    print("Starting data consumer...")
    print()
    while not stop_event.is_set():
        with condition:
            condition.wait()
            print(f"\rAccel: X={accelerometer_list[-1][0]:0.2f}, "
                    f"Y={accelerometer_list[-1][1]:0.2f}, "
                    f"Z={accelerometer_list[-1][2]:0.2f}, "
                    f"Gyro: X={gyro_list[-1][0]:0.2f}, "
                    f"Y={gyro_list[-1][1]:0.2f}, "
                    f"Z={gyro_list[-1][2]:0.2f}", end='      ', flush=True)

            if duration and len(accelerometer_list) > duration * 417:
                stop_event.set()
                print("\nStopping data consumer...")


def main(duration, output):
    with Manager() as manager:
        accelerometer_list = manager.list()
        gyro_list = manager.list()
        condition = manager.Condition()
        stop_event = manager.Event()

        # Set the signal handler for SIGINT
        signal.signal(signal.SIGINT, lambda signum, frame: signal_handler(signum, frame, stop_event))

        producer_process = Process(target=sensor_thread, args=(accelerometer_list, gyro_list, condition, stop_event))
        consumer_process = Process(target=data_consumer, args=(accelerometer_list, gyro_list, condition, stop_event, duration))
        producer_process.start()
        consumer_process.start()
        producer_process.join()
        consumer_process.join()

        accelerometer_list = list(accelerometer_list)
        gyro_list = list(gyro_list)

    # Truncate to duration if specified
    if duration is not None:
        accelerometer_list = accelerometer_list[:duration * 417]
        gyro_list = gyro_list[:duration * 417]
    # Save accelerometer_list and gyro_list to json file
    if output is not None:
        with open(os.path.join("tmp", f"{output}.json"), "w") as f:
            json.dump({"accelerometer": accelerometer_list, "gyro": gyro_list, "sample_rate": 417}, f)

    plot(accelerometer_list, gyro_list)


if __name__ == '__main__':
    # Parse args for output filename and duration
    parser = argparse.ArgumentParser(description='Record IMU data')
    parser.add_argument('-o', '--output', type=str, default=None,
                        help='Output filename')
    parser.add_argument('-d', '--duration', type=int, default=None,
                        help='Duration in seconds')

    args = parser.parse_args()
    duration = args.duration
    output = args.output

    try:
        main(duration, output)
    except KeyboardInterrupt:
        print("Exiting...")
