#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pygame
from mujoco_rl.msg import Joymsg

class JoystickSubscriber(Node):
    def __init__(self):
        super().__init__('joystick_subscriber')
        self.subscription = self.create_subscription(
            Joymsg,
            'joystick_state',
            self.listener_callback,
            10)

        self.shutdown_requested = False
        self.should_print = True
        self.get_logger().info('Joystick 구독자 노드가 시작되었습니다...')

    def listener_callback(self, msg):
        if msg.cross and msg.cross[0] == -1:
            self.get_logger().info('종료 버튼 입력 감지! 노드를 종료합니다...')
            self.shutdown_requested = True
            return

        if msg.cross and msg.cross[1] == -1:
            self.should_print = False
            print("출력 중지됨.")
        elif msg.cross and msg.cross[1] == 1:
            self.should_print = True

        if self.should_print:
            axes_str = ', '.join([f'{axis:.3f}' for axis in msg.axes])
            self.get_logger().info(
                f'\n'
                f'--- 조이스틱 상태 수신 ---\n'
                f'  Axes:    [{axes_str}]\n'
                f'  Buttons: {msg.buttons}\n'
                f'  Cross:   {msg.cross}\n'
                f'----------------------------'
            )

def main(args=None):
    rclpy.init(args=args)
    joystick_subscriber = JoystickSubscriber()
    
    while rclpy.ok() and not joystick_subscriber.shutdown_requested:
        rclpy.spin_once(joystick_subscriber, timeout_sec=0.1)

    print("종료 절차를 시작합니다...")
    joystick_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()