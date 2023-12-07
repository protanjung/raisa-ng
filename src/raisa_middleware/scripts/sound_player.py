#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String
import rclpy
import pygame
import os


class SoundPlayer(Node):
    def __init__(self):
        super().__init__('sound_player')
        # ----Subscriber
        self.sub_sound = self.create_subscription(String, 'sound', self.cllbck_sub_sound, 1)

        pygame.init()

        # Sound processing
        # ================
        self.mixers = {}

        if self.sound_player_init() == False:
            self.get_logger().error('Sound Player init failed')
            exit()

    # ==================================

    def cllbck_sub_sound(self, msg):
        filename = get_package_share_directory('raisa_middleware') + '/assets/' + msg.data

        if msg.data in self.mixers:
            self.mixers[msg.data].play(fade_ms=100)
        else:
            self.mixers[msg.data] = pygame.mixer.Sound(filename)
            self.mixers[msg.data].play(fade_ms=100)
            self.mixers[msg.data].set_volume(1.0)

    # ==================================

    def sound_player_init(self):
        sound_noise = get_package_share_directory('raisa_middleware') + '/assets/sound_noise.wav'

        if not os.path.exists(sound_noise):
            self.get_logger().error('Sound noise not found')
            return False

        self.mixers['sound_noise.wav'] = pygame.mixer.Sound(get_package_share_directory('raisa_middleware') + '/assets/sound_noise.wav')
        self.mixers['sound_noise.wav'].play(loops=-1)
        self.mixers['sound_noise.wav'].set_volume(0.1)

        return True


def main(args=None):
    rclpy.init(args=args)

    node_sound_player = SoundPlayer()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node_sound_player)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
