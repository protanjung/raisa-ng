#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from raisa_interfaces.msg import Face, Faces
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2 as cv
import mediapipe as mp
import numpy as np
import rclpy
import threading


class FaceDetector(Node):
    def __init__(self):
        super().__init__('face_detector')
        # ----Timer
        self.tim_10hz = self.create_timer(0.1, self.cllbck_tim_10hz)
        # ----Subscriber
        self.sub_image_bgr = self.create_subscription(Image, 'image_bgr', self.cllbck_sub_image_bgr, 1)
        self.sub_image_gray = self.create_subscription(Image, 'image_gray', self.cllbck_sub_image_gray, 1)
        # ----Publisher
        self.pub_faces = self.create_publisher(Faces, 'faces', 10)
        self.pub_faces_display = self.create_publisher(Image, 'faces_display', 1)
        # ----Mutex
        self.mtx_image_bgr = threading.Lock()
        self.mtx_image_gray = threading.Lock()

        # Image processing
        # ================
        self.image_bgr = np.zeros((360, 640, 3), np.uint8)
        self.image_gray = np.zeros((360, 640, 1), np.uint8)
        self.face_display = np.zeros((360, 640, 3), np.uint8)

        # Mediapipe
        # =========
        self.BaseOptions = mp.tasks.BaseOptions
        self.FaceDetector = mp.tasks.vision.FaceDetector
        self.FaceDetectorOptions = mp.tasks.vision.FaceDetectorOptions
        self.FaceDetectorResult = mp.tasks.vision.FaceDetectorResult
        self.VisionRunningMode = mp.tasks.vision.RunningMode

        model_path = get_package_share_directory('raisa_middleware') + '/assets/blaze_face_short_range.tflite'

        options = self.FaceDetectorOptions(
            base_options=self.BaseOptions(model_asset_path=model_path),
            running_mode=self.VisionRunningMode.LIVE_STREAM,
            result_callback=self.mediapipe_print_result,
            min_detection_confidence=0.375,
            min_suppression_threshold=0.3
        )

        self.detector = self.FaceDetector.create_from_options(options)

        # Faces
        # =====
        self.faces = Faces()

        if self.face_detector_init() == False:
            self.get_logger().error('Face Detector init failed')
            exit()

    # ==================================

    def cllbck_tim_10hz(self):
        if self.face_detector_routine() == False:
            self.get_logger().error('Face Detector routine failed')
            exit()

    # ==================================

    def cllbck_sub_image_bgr(self, msg):
        image = CvBridge().imgmsg_to_cv2(msg, 'bgr8')

        self.mtx_image_bgr.acquire()
        self.image_bgr = cv.resize(image, (640, 360))
        self.mtx_image_bgr.release()

    def cllbck_sub_image_gray(self, msg):
        image = CvBridge().imgmsg_to_cv2(msg, 'mono8')

        self.mtx_image_gray.acquire()
        self.image_gray = cv.resize(image, (640, 360))
        self.mtx_image_gray.release()

    # ==================================

    def face_detector_init(self):

        return True

    def face_detector_routine(self):
        self.mtx_image_bgr.acquire()
        self.face_display = self.image_bgr.copy()
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=self.face_display)
        self.mtx_image_bgr.release()

        # Detect faces
        self.detector.detect_async(mp_image, int(self.get_clock().now().nanoseconds / 1000000))

        # Display faces
        for face in self.faces.faces:
            cv.rectangle(self.face_display, (face.origin_x, face.origin_y), (face.origin_x + face.width, face.origin_y + face.height), (0, 255, 0), 2)
        msg_face_display = CvBridge().cv2_to_imgmsg(self.face_display, 'bgr8')
        self.pub_faces_display.publish(msg_face_display)

        return True

    # ==================================

    def mediapipe_print_result(self, result, output_image, timestamp_ms):
        msg_faces = Faces()
        for detection in result.detections:
            face = Face()
            face.origin_x = detection.bounding_box.origin_x
            face.origin_y = detection.bounding_box.origin_y
            face.width = detection.bounding_box.width
            face.height = detection.bounding_box.height
            msg_faces.faces.append(face)
        self.pub_faces.publish(msg_faces)

        self.faces = msg_faces


def main(args=None):
    rclpy.init(args=args)

    node_face_detector = FaceDetector()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node_face_detector)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
