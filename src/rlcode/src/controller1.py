#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pygame
import numpy as np
import time
import os
import threading

from msg_interfaces.msg import Joymsg

# os.environ['ROS_DOMAIN_ID'] = '45'

class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_node')
        self.publisher_ = self.create_publisher(Joymsg, 'joystick_state', 10)

        self.timer_period = 0.02 # 50hz
        self.timer = self.create_timer(self.timer_period, self.joy_callback)

        pygame.init()

        self.joystick = None
        self.reconnect_counter = 0
        self.reconnect_interval = 1.0 

        self.shutdown_requested = False

        self.get_logger().info("조이스틱 연결을 시도합니다...")
        self.attempt_reconnect()

    def attempt_reconnect(self):
        """조이스틱 재연결을 시도하는 함수"""
        pygame.joystick.quit()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.get_logger().info(f"'{self.joystick.get_name()}' 컨트롤러가 연결되었습니다.")
        else:
            self.joystick = None

    def joy_callback(self):
        if self.shutdown_requested:
            return
        
        pygame.event.pump()

        msg = Joymsg()

        if self.joystick:
            try:
                msg.axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
                msg.buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
                hats_list = []
                for i in range(self.joystick.get_numhats()):
                    hat_state = self.joystick.get_hat(i)
                    hats_list.extend([hat_state[0], hat_state[1]])
                msg.cross = hats_list
            except pygame.error as e:
                self.get_logger().warn(f"조이스틱 에러: {e}. 재연결을 시도합니다.")
                self.joystick = None
                msg.axes, msg.buttons, msg.cross = [], [], []
        else:
            msg.axes, msg.buttons, msg.cross = [], [], []
            self.reconnect_counter += self.timer_period
            if self.reconnect_counter >= self.reconnect_interval:
                self.get_logger().info("조이스틱을 찾을 수 없습니다. 다시 연결을 시도합니다...")
                self.attempt_reconnect()
                self.reconnect_counter = 0.0
        
        self.publisher_.publish(msg)

        if len(msg.cross) > 1 and msg.cross[0] == -1:
            self.get_logger().info("joy노드 종료를 요청합니다.")
            self.shutdown_requested = True


def main(args=None):
    rclpy.init(args=args)
    domain_id = os.getenv('ROS_DOMAIN_ID', '기본값 0')
    print(f"--- 현재 ROS_DOMAIN_ID: {domain_id} ---")
    
    node = None
    spin_thread = None
    try:
        node = JoystickPublisher()
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        while not node.shutdown_requested:
            time.sleep(0.1)

    except KeyboardInterrupt:
        if node:
            node.get_logger().info("키보드 인터럽트로 노드를 종료합니다.")
            node.shutdown_requested = True
    finally:
        if rclpy.ok():
            rclpy.shutdown()

        if spin_thread and spin_thread.is_alive():
            spin_thread.join()
        
        if node:
            node.destroy_node()
            
        pygame.quit()

    print("--- 조이스틱 노드 정상 종료 ---")


if __name__ == '__main__':
    main()