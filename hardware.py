import struct
import time

from micropython import const
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_register.i2c_struct import ROUnaryStruct, UnaryStruct
from adafruit_register.i2c_bits import RWBits

# TODO add comments, docstrings, and typehints
# TODO run autoformatter too


class Binary:
    BIT0 = 0b00000001
    BIT1 = 0b00000010
    BIT2 = 0b00000100
    BIT3 = 0b00001000
    BIT4 = 0b00010000
    BIT5 = 0b00100000
    BIT6 = 0b01000000
    BIT7 = 0b10000000


class ImuDefinitions:
    TAG_GYRO = 0x01
    TAG_ACCELEROMETER = 0x02

    FIFO_MODE_BYPASS = 0
    FIFO_MODE_RUN = 1

    _LSM6DS_FIFO_CTRL1 = const(0x07)
    _LSM6DS_FIFO_CTRL3 = const(0x09)
    _LSM6DS_FIFO_CTRL4 = const(0x0A)
    _LSM6DS_FIFO_STATUS1 = const(0x3A)
    _LSM6DS_FIFO_STATUS2 = const(0x3B)
    _LSM6DS_FIFO_DATA = const(0x78)


class Imu(LSM6DSOX):

    _fifo_watermark_threshold = UnaryStruct(ImuDefinitions._LSM6DS_FIFO_CTRL1, "<B")

    _fifo_mode = RWBits(3, ImuDefinitions._LSM6DS_FIFO_CTRL4, 0)

    _fifo_batch_accelerometer = RWBits(4, ImuDefinitions._LSM6DS_FIFO_CTRL3, 0)
    _fifo_batch_gyro = RWBits(4, ImuDefinitions._LSM6DS_FIFO_CTRL3, 4)

    _fifo_status1 = ROUnaryStruct(ImuDefinitions._LSM6DS_FIFO_STATUS1, "<B")
    _fifo_status2 = ROUnaryStruct(ImuDefinitions._LSM6DS_FIFO_STATUS2, "<B")

    def fifo_status(self):
        fifo_status1 = self._fifo_status1
        fifo_status2 = self._fifo_status2
        samples = fifo_status1 + ((0b11 & fifo_status2) << 8)
        overrun = fifo_status2 & Binary.BIT3 == Binary.BIT3
        full = fifo_status2 & Binary.BIT5 == Binary.BIT5
        watermark = fifo_status2 & Binary.BIT7 == Binary.BIT7
        return {"samples": samples, "overrun": overrun, "watermark": watermark, "full": full}

    def fifo_start(self, rate):
        # Start by stopping the FIFO, since this will clear any existing data
        self._fifo_mode = ImuDefinitions.FIFO_MODE_BYPASS
        self.gyro_data_rate = rate
        self.accelerometer_data_rate = rate
        self._fifo_batch_accelerometer = rate
        self._fifo_batch_gyro = rate
        self._fifo_mode = ImuDefinitions.FIFO_MODE_RUN

    def fifo_stop(self):
        self._fifo_mode = 0x1

    def fifo_read(self, read_threshold=2):
        status = self.fifo_status()
        if status['full']:
            raise RuntimeError('FIFO is full')
        if status['overrun']:
            raise RuntimeError('FIFO overrun')
        total_bytes_to_read = 7 * status['samples']
        # # If we have an odd number of bytes, we are between samples, and set total_byte_to_read to be one smaller
        # if total_bytes_to_read % 2 == 1:
        #     total_bytes_to_read -= 1
        if total_bytes_to_read < read_threshold:
            return [], []

        buffer = bytearray(1 + total_bytes_to_read)
        buffer[0] = ImuDefinitions._LSM6DS_FIFO_DATA
        start_time = time.time()
        with self.i2c_device as i2c:
            i2c.write_then_readinto(buffer, buffer, out_end=1, in_start=1)
        # print(f'Read {total_bytes_to_read} bytes in {time.time() - start_time:.3f} seconds.')

        accelerometer = []
        gyro = []
        for n in range(status['samples']):
            index = 1 + n * 7
            tag_byte = buffer[index]
            tag = tag_byte >> 3
            data = struct.unpack_from("<hhh", memoryview(buffer)[index + 1 : index + 7])
            if tag == ImuDefinitions.TAG_ACCELEROMETER:
                accelerometer.append([self._scale_xl_data(d) for d in data])
            elif tag == ImuDefinitions.TAG_GYRO:
                gyro.append([self._scale_gyro_data(d) for d in data])
            else:
                raise RuntimeError(f'Unknown tag: {tag}')
        assert len(accelerometer) == len(gyro)
        return accelerometer, gyro



# def read_data(sensor, total_time=5, read_period=0.5):
#     accelerometer = []
#     gyro = []
#     read_duration = 0
#     total_read_time = 0
#     while time.time() - start_time < total_time:
#         if read_duration < read_period:
#             time.sleep(read_period - read_duration)
#         read_start_time = time.time()
#         a, g = sensor.fifo_read()
#         accelerometer.extend(a)
#         gyro.extend(g)
#         read_duration = time.time() - read_start_time
#         total_read_time += read_duration
#     duration = time.time() - start_time
#     a, g = sensor.fifo_read()
#     accelerometer.extend(a)
#     gyro.extend(g)
#     print(f'Read {len(accelerometer)} samples in {duration:.3f} sec, active for {total_read_time:0.3f} sec ({len(accelerometer) / duration:.0f} sps @ duty {total_read_time / duration:.0%}).')
#     return accelerometer, gyro
