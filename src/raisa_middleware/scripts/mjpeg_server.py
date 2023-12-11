#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from flask import Flask, Response, render_template_string, request
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2 as cv
import numpy as np
import rclpy
import threading


class MJPEGServer(Node):
    def __init__(self):
        super().__init__('mjpeg_server')
        # ----Parameter
        self.declare_parameter('mjpeg_server.topics', rclpy.parameter.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('mjpeg_server.port', rclpy.parameter.Parameter.Type.INTEGER)
        self.mjpeg_server_topics = self.get_parameter('mjpeg_server.topics').value
        self.mjpeg_server_port = self.get_parameter('mjpeg_server.port').value
        # ----Timer
        self.tim_1hz = self.create_timer(1.0, self.cllbck_tim_1hz)
        # ----Subscriber
        self.subscriber_topics = []
        self.subscriber_objects = {}

        # Image processing
        # ================
        self.mtx_images = {}
        self.images = {}

        if self.mjpeg_server_init() == False:
            self.get_logger().error('MJPEG Server init failed')
            exit()

    # ==================================

    def cllbck_tim_1hz(self):
        if self.mjpeg_server_routine() == False:
            self.get_logger().error('MJPEG Server routine failed')
            exit()

    # ==================================

    def cllbck_sub_image(self, msg, topic):
        if self.mtx_images[topic].acquire(False):
            self.images[topic] = CvBridge().imgmsg_to_cv2(msg, 'bgr8')
            self.mtx_images[topic].release()

    # ==================================

    def mjpeg_server_init(self):
        self.get_logger().info('Topics: ' + str(self.mjpeg_server_topics))
        self.get_logger().info('Port: ' + str(self.mjpeg_server_port))

        self.flask_app = Flask(__name__)
        self.flask_thread = threading.Thread(target=self.flask_app_run)
        self.flask_thread.daemon = True
        self.flask_thread.start()

        @self.flask_app.route('/')
        def index():
            with open(get_package_share_directory('raisa_middleware') + '/assets/index.html', 'r') as f:
                return render_template_string(f.read(), topics=self.mjpeg_server_topics)

        @self.flask_app.route('/<path:topic>')
        def display(topic):
            if '/' + topic not in self.mjpeg_server_topics:
                return Response('Topic not available')

            quality = 50
            if 'quality' in request.args:
                quality = min(max(int(request.args['quality']), 0), 100)
            scale = 0.5
            if 'scale' in request.args:
                scale = min(max(float(request.args['scale']), 0.1), 10.0)

            return Response(self.flask_display('/' + topic, quality, scale), mimetype='multipart/x-mixed-replace; boundary=frame')

        return True

    def mjpeg_server_routine(self):
        for topic in self.mjpeg_server_topics:
            if topic not in self.subscriber_topics:
                self.register_subscriber(topic)
                self.subscriber_topics.append(topic)
                self.mtx_images[topic] = threading.Lock()
                self.images[topic] = np.zeros((64, 64, 3), np.uint8)

        for topic in self.subscriber_topics:
            if topic not in self.mjpeg_server_topics:
                self.unregister_subscriber(topic)
                self.subscriber_topics.remove(topic)
                del self.mtx_images[topic]
                del self.images[topic]

        return True

    # ==================================

    def register_subscriber(self, topic):
        self.subscriber_objects[topic] = self.create_subscription(Image, topic, lambda msg, topic=topic: self.cllbck_sub_image(msg, topic), 1)

    def unregister_subscriber(self, topic):
        self.destroy_subscription(self.subscriber_objects[topic])
        del self.subscriber_objects[topic]

    # ==================================

    def flask_app_run(self):
        self.flask_app.run(host='0.0.0.0', port=self.mjpeg_server_port)

    def flask_app_stop(self):
        self.flask_app.stop()

    def flask_display(self, topic, quality, scale):
        rate = self.create_rate(15.0)
        while True:
            rate.sleep()

            if topic not in self.mtx_images or topic not in self.images:
                continue

            self.mtx_images[topic].acquire()
            image = self.images.get(topic, np.zeros((64, 64, 3), np.uint8))
            self.mtx_images[topic].release()

            image = cv.resize(image, (0, 0), fx=scale, fy=scale)
            _, image = cv.imencode('.jpg', image, [cv.IMWRITE_JPEG_QUALITY, quality])

            try:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + image.tobytes() + b'\r\n')
            except GeneratorExit:
                break


def main(args=None):
    rclpy.init(args=args)

    node_mjpeg_server = MJPEGServer()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node_mjpeg_server)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
