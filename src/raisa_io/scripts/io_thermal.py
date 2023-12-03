#!/usr/bin/python3

from rclpy.node import Node
from std_msgs.msg import Float32
from struct import unpack
import crcmod.predefined
import numpy as np
import rclpy
import serial
import serial.tools.list_ports
import threading


class IOThermal(Node):
    def __init__(self):
        super().__init__('io_thermal')
        # ----Timer
        self.tim_50hz = self.create_timer(0.02, self.cllbck_tim_50hz)
        # ----Publisher
        self.pub_thermal = self.create_publisher(Float32, 'thermal', 10)

        if self.io_thermal_init() == False:
            self.get_logger().error('IO Thermal init failed')
            exit()

    # ==================================

    def cllbck_tim_50hz(self):
        if self.io_thermal_routine() == False:
            self.get_logger().error('IO Thermal routine failed')
            exit()

    # ==================================

    def io_thermal_init(self):
        ports = list(serial.tools.list_ports.comports())
        portname = None
        for p in ports:
            if ":5740" in p[2]:
                self.get_logger().info("EvoThermal found on port " + p[0])
                portname = p[0]
        if portname is None:
            self.get_logger().error("Sensor not found. Please Check connections.")
            return False
        self.port = serial.Serial(
            port=portname,  # To be adapted if using UART backboard
            baudrate=115200,  # 460800 for UART backboard
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        self.port.isOpen()
        self.serial_lock = threading.Lock()
        ### CRC functions ###
        self.crc32 = crcmod.predefined.mkPredefinedCrcFun('crc-32-mpeg')
        self.crc8 = crcmod.predefined.mkPredefinedCrcFun('crc-8')
        ### Activate sensor USB output ###
        self.activate_command = (0x00, 0x52, 0x02, 0x01, 0xDF)
        self.deactivate_command = (0x00, 0x52, 0x02, 0x00, 0xD8)
        self.send_command(self.activate_command)

        return True

    def io_thermal_routine(self):
        thermals_raw = self.get_thermals()
        thermals_center = thermals_raw[8:24, 8:24]
        thermals_max = np.max(thermals_center)

        msg_thermal = Float32()
        msg_thermal.data = thermals_max
        self.pub_thermal.publish(msg_thermal)

        return True

    # ==================================

    def get_thermals(self):
        got_frame = False
        while not got_frame:
            with self.serial_lock:
                ### Polls for header ###
                header = self.port.read(2)
                # header = unpack('H', str(header))
                header = unpack('H', header)
                if header[0] == 13:
                    ### Header received, now read rest of frame ###
                    data = self.port.read(2068)
                    ### Calculate CRC for frame (except CRC value and header) ###
                    calculatedCRC = self.crc32(data[:2064])
                    data = unpack("H" * 1034, data)
                    receivedCRC = (data[1032] & 0xFFFF) << 16
                    receivedCRC |= data[1033] & 0xFFFF
                    TA = data[1024]
                    data = data[:1024]
                    data = np.reshape(data, (32, 32))
                    ### Compare calculated CRC to received CRC ###
                    if calculatedCRC == receivedCRC:
                        got_frame = True
                    else:
                        self.get_logger().error("Bad CRC. Dropping frame")
        self.port.flushInput()
        ### Data is sent in dK, this converts it to celsius ###
        data = (data / 10.0) - 273.15
        TA = (TA / 10.0) - 273.15

        return data

    def send_command(self, command):
        ### This avoid concurrent writes/reads of serial ###
        with self.serial_lock:
            self.port.write(command)
            ack = self.port.read(1)
            ### This loop discards buffered frames until an ACK header is reached ###
            while ord(ack) != 20:
                ack = self.port.read(1)
            else:
                ack += self.port.read(3)
            ### Check ACK crc8 ###
            crc8 = self.crc8(ack[:3])
            if crc8 == ack[3]:
                ### Check if ACK or NACK ###
                if ack[2] == 0:
                    self.get_logger().info("Command acknowledged")
                    return True
                else:
                    self.get_logger().warn("Command not acknowledged")
                    return False
            else:
                self.get_logger().error("Error in ACK checksum")
                return False

    def run(self):
        ### Get frame and print it ###
        frame = self.get_thermals()
        self.get_logger().info(frame)

    def stop(self):
        ### Deactivate USB VCP output and close port ###
        self.send_command(self.deactivate_command)
        self.port.close()


def main(args=None):
    rclpy.init(args=args)

    node_io_thermal = IOThermal()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node_io_thermal)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
