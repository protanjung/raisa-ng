#!/usr/bin/python3

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import numpy as np
import rclpy
import serial
import serial.tools.list_ports
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class IOMB1R2T(Node):
    def __init__(self):
        super().__init__('io_mb1r2t')
        # ----Parameter
        self.declare_parameter('port', rclpy.parameter.Parameter.Type.STRING)
        self.declare_parameter('baud', rclpy.parameter.Parameter.Type.INTEGER)
        self.declare_parameter('frame_id', rclpy.parameter.Parameter.Type.STRING)
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value
        # ----Timer
        self.tim_50hz = self.create_timer(0.000001, self.cllbck_tim_50hz)
        # ----Publisher
        self.pub_points_xyz = self.create_publisher(PointCloud2, 'points_xyz', 1)
        self.pub_points_xyzi = self.create_publisher(PointCloud2, 'points_xyzi', 1)
        self.pub_laser_scan = self.create_publisher(LaserScan, 'laser_scan', 1)

        self.state = 0
        self.package_type = 0
        self.package_size = 0
        self.package_start = 0
        self.package_stop = 0
        self.last_angle = 0

        self.points_xyz_array = []
        self.points_xyzi_array = []
        self.points_xyz = PointCloud2()
        self.points_xyzi = PointCloud2()
        self.laser_scan = LaserScan()

        if self.io_mb1r2t_init() == False:
            self.get_logger().error('IO MB1R2T init failed')
            exit()

    # ==================================

    def cllbck_tim_50hz(self):
        if self.io_mb1r2t_routine() == False:
            self.get_logger().error('IO MB1R2T routine failed')
            exit()

    # ==================================

    def io_mb1r2t_init(self):
        rclpy.logging.get_logger('rclpy.node').info('Port: ' + str(self.port))
        rclpy.logging.get_logger('rclpy.node').info('Baud: ' + str(self.baud))
        rclpy.logging.get_logger('rclpy.node').info('Frame ID: ' + str(self.frame_id))

        self.com = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )

        self.laser_scan.header.frame_id = self.frame_id
        self.laser_scan.angle_min = 0.0
        self.laser_scan.angle_max = 2 * np.pi
        self.laser_scan.range_min = 0.1
        self.laser_scan.range_max = 10.0

        return True

    def io_mb1r2t_routine(self):
        if self.state == 0:
            sync = self.com.read(1)

            if len(sync) < 1:
                self.state = 0
                return True

            if sync[0] == 0xAA:
                self.state = 1
            else:
                self.state = 0
        # ==============================
        elif self.state == 1:
            sync = self.com.read(1)

            if len(sync) < 1:
                self.state = 0
                return True

            if sync[0] == 0x55:
                self.state = 2
            else:
                self.state = 0
        # ==============================
        elif self.state == 2:
            header = self.com.read(8)

            if len(header) < 8:
                self.state = 0
                return True

            self.state = 3

            self.package_type = header[0]
            self.package_size = header[1]
            self.package_start = (header[3] << 8) | header[2]
            self.package_stop = (header[5] << 8) | header[4]
        # ==============================
        elif self.state == 3:
            if self.package_size > 0:
                data = self.com.read(self.package_size * 3)
            else:
                self.state = 0
                return True

            if len(data) < self.package_size * 3:
                self.state = 0
                return True

            self.state = 0

            angle_diff = self.package_stop - self.package_start
            if self.package_stop < self.package_start:
                angle_diff = 0xb400 - self.package_start + self.package_stop
            angle_step = 0
            if angle_diff > 1:
                angle_step = angle_diff / (self.package_size - 1)

            for i in range(self.package_size):
                intensity = data[i * 3 + 0]
                intensity = intensity / 255.0

                distance = (data[i * 3 + 2] << 8) | data[i * 3 + 1]
                distance = distance / 4000.0

                angle = (self.package_start + angle_step * i) % 0xb400
                angle = angle / 0xb400 * 2 * np.pi

                self.laser_scan.ranges.append(distance)
                self.laser_scan.intensities.append(intensity)

                x, y, z = distance * np.cos(angle), distance * np.sin(angle), 0.0
                if np.sqrt(x * x + y * y) > 0.1 and np.sqrt(x * x + y * y) < 10.0:
                    self.points_xyz_array.append([x, y, z])
                    self.points_xyzi_array.append([x, y, z, intensity])

                if angle < self.last_angle:
                    self.points_xyz = PointCloud2()
                    self.points_xyz.header.frame_id = self.frame_id
                    self.points_xyz.height = 1
                    self.points_xyz.width = len(self.points_xyz_array)
                    self.points_xyz.fields = [
                        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
                    ]
                    self.points_xyz.is_bigendian = False
                    self.points_xyz.point_step = 12
                    self.points_xyz.row_step = 12 * len(self.points_xyz_array)
                    self.points_xyz.is_dense = True
                    self.points_xyz.data = np.asarray(self.points_xyz_array, np.float32).tostring()
                    self.points_xyz.header.stamp = self.get_clock().now().to_msg()
                    self.pub_points_xyz.publish(self.points_xyz)

                    self.points_xyzi = PointCloud2()
                    self.points_xyzi.header.frame_id = self.frame_id
                    self.points_xyzi.height = 1
                    self.points_xyzi.width = len(self.points_xyzi_array)
                    self.points_xyzi.fields = [
                        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
                    ]
                    self.points_xyzi.is_bigendian = False
                    self.points_xyzi.point_step = 16
                    self.points_xyzi.row_step = 16 * len(self.points_xyzi_array)
                    self.points_xyzi.is_dense = True
                    self.points_xyzi.data = np.asarray(self.points_xyzi_array, np.float32).tostring()
                    self.points_xyzi.header.stamp = self.get_clock().now().to_msg()
                    self.pub_points_xyzi.publish(self.points_xyzi)

                    self.laser_scan.header.stamp = self.get_clock().now().to_msg()
                    self.laser_scan.angle_increment = 2 * np.pi / len(self.laser_scan.ranges)
                    self.pub_laser_scan.publish(self.laser_scan)

                    self.points_xyz_array = []
                    self.points_xyzi_array = []
                    self.laser_scan.ranges = []
                    self.laser_scan.intensities = []

                self.last_angle = angle

        return True


def main(args=None):
    rclpy.init(args=args)

    node_io_mb1r2t = IOMB1R2T()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node_io_mb1r2t)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
