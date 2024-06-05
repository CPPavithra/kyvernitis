import os
import sys
import ctypes
import serial
import time


# Define the DiffDriveStatus struct
class DiffDriveStatus(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("heading", ctypes.c_float),
        ("linear", ctypes.c_float),
        ("angular", ctypes.c_float),
    ]


# Define the DiffDriveTwist struct
class DiffDriveTwist(ctypes.Structure):
    _fields_ = [("linear_x", ctypes.c_float), ("angular_z", ctypes.c_float)]


# Define the mother_cmd_msg struct
class mother_cmd_msg(ctypes.Structure):
    _fields_ = [
        ("drive_cmd", DiffDriveTwist),
        ("arm_joint", ctypes.c_float * 3),
        ("adaptive_sus_cmd", ctypes.c_uint8 * 4),
    ]


# Define the mother_status_msg struct
class mother_status_msg(ctypes.Structure):
    _fields_ = [("odom", DiffDriveStatus), ("arm_joint_status", ctypes.c_float * 3), ("timestamp", ctypes.c_uint64)]


# Define the mother_msg struct
class mother_msg(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_uint16),
        ("status", mother_status_msg),
        ("cmd", mother_cmd_msg),
        ("info", ctypes.c_char * 100),
        ("crc", ctypes.c_uint32),
    ]


class CobsDecodeStatus(ctypes.c_int):
    COBS_DECODE_OK = 0x00
    COBS_DECODE_NULL_POINTER = 0x01
    COBS_DECODE_OUT_BUFFER_OVERFLOW = 0x02
    COBS_DECODE_ZERO_BYTE_IN_INPUT = 0x04
    COBS_DECODE_INPUT_TOO_SHORT = 0x08


# Define the cobs_decode_result struct
class CobsDecodeResult(ctypes.Structure):
    _fields_ = [("out_len", ctypes.c_size_t), ("status", CobsDecodeStatus)]


# Load the shared library
lib_path = os.path.join(os.path.dirname(__file__), "libcobs.so.2.0.0")
cobs_lib = ctypes.CDLL(lib_path)
crc_path  = os.path.join(os.path.dirname(__file__), "crc.so.1.0.0")
libcrc = ctypes.CDLL(crc_path)

# Prototype for cobs_encode
cobs_lib.cobs_encode.argtypes = [
    ctypes.POINTER(ctypes.c_uint8),
    ctypes.c_size_t,
    ctypes.POINTER(ctypes.c_uint8),
]
cobs_lib.cobs_encode.restype = None

# Prototype for cobs_decode
cobs_lib.cobs_decode.argtypes = [
    ctypes.POINTER(ctypes.c_uint8),
    ctypes.c_size_t,
    ctypes.POINTER(ctypes.c_uint8),
    ctypes.c_size_t,
]
cobs_lib.cobs_decode.restype = CobsDecodeResult


def read_from_serial(ser):
            data = ser.read(178)
            # Check if we got enough bytes
            if len(data) == 178:
                # Prepare input and output buffers for decoding
                input_buffer = (ctypes.c_uint8 * len(data)).from_buffer_copy(data)
                output_buffer = (ctypes.c_uint8 * 178)()

                # Call the cobs_decode function
                result = cobs_lib.cobs_decode(
                    output_buffer, 178, input_buffer, len(data) - 1
                )


                if result.status.value == CobsDecodeStatus.COBS_DECODE_OK:
                    decoded_data = bytes(output_buffer[: result.out_len])
                    decoded_msg = ctypes.cast(decoded_data, ctypes.POINTER(mother_msg))

                    if decoded_msg.contents.type == 3:
                        print(
                            "[STATUS] [{}] | Odom: x: {}, y: {}, heading: {}° | Arm Joint Status: {} {} {}".format(
                                decoded_msg.contents.status.timestamp,
                                decoded_msg.contents.status.odom.x,
                                decoded_msg.contents.status.odom.y,
                                decoded_msg.contents.status.odom.heading,
                                decoded_msg.contents.status.arm_joint_status[0],
                                decoded_msg.contents.status.arm_joint_status[1], 
                                decoded_msg.contents.status.arm_joint_status[2] 
                            )
                        )
                    if decoded_msg.contents.type == 4:
                        print(f"[ERROR]: {decoded_msg.contents.info.decode()}")
                    if decoded_msg.contents.type == 5:
                        print(f"[INFO]: {decoded_msg.contents.info.decode()}")
                else:
                    print(f"COBS decode error: {result.status.value}")
            else:
                print(f"Read {len(data)} bytes, which is less than expected.")

def write_to_serial(ser, data):
        # Open the serial port
    encoded_msg = bytearray(data)
    
    # Prepare input and output buffers for encoding
    input_buffer = (ctypes.c_uint8 * len(encoded_msg)).from_buffer_copy(encoded_msg)
    output_buffer = (ctypes.c_uint8 * 178)()

    # Call the cobs_encode function
    cobs_lib.cobs_encode(output_buffer, 178, input_buffer,len(encoded_msg))

    # Write the encoded data to the serial port
    ser.write(bytes(output_buffer))
    print(f"[CMD]:  {data.cmd.drive_cmd.linear_x} | {data.cmd.drive_cmd.angular_z} to the serial port")  

def calculate_crc(data):
    libcrc.crc32_ieee.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t]
    libcrc.crc32_ieee.restype = ctypes.c_uint32
    data_bytes = bytes(data)
    data_array = (ctypes.c_uint8 * len(data_bytes)).from_buffer_copy(data_bytes)
    crc = libcrc.crc32_ieee(data_array, len(data_bytes) - ctypes.sizeof(ctypes.c_uint32))
    return crc


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <serial_port>")
        sys.exit(1)

    port = sys.argv[1]

    
    data = mother_msg()
    data.type = 0

    # data.status.odom.x = 2.0
    # data.status.odom.y = 2.0
    # data.status.odom.heading = 2.0
    # data.status.odom.linear = 2.0
    # data.status.odom.angular = 2.0
    #
    # data.status.arm_joint_status[0] = 2.0
    # data.status.arm_joint_status[1] = 2.0
    # data.status.arm_joint_status[2] = 2.0
    # data.status.timestamp = 0

    data.cmd.drive_cmd.linear_x = 1.5
    data.cmd.drive_cmd.angular_z = 0.0

    data.cmd.arm_joint[0] = 0.0
    data.cmd.arm_joint[1] = 0.0
    data.cmd.arm_joint[2] = 0.0 

    data.cmd.adaptive_sus_cmd[0] = 0
    data.cmd.adaptive_sus_cmd[1] = 0
    data.cmd.adaptive_sus_cmd[2] = 0
    data.cmd.adaptive_sus_cmd[3] = 0

    data.crc = calculate_crc(data)
    
    #data.info = "Sending drive command"
    try:
        ser = serial.Serial(port, 921600, timeout=5)
        print(f"Opened serial port {port}")
       
        while True:

            write_to_serial(ser, data)
            read_from_serial(ser)
            time.sleep(1);
        
    except serial.SerialException as e:
        print(f"Error opening or reading from serial port: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        ser.close()

