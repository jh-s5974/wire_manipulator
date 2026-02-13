#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from msg_interfaces.msg import Observation
from msg_interfaces.msg import Action
from msg_interfaces.msg import Test
from sensor_msgs.msg import JointState
import threading # 스레드 잠금을 위해 import

class JointStateTranslator(Node):
    def __init__(self):
        super().__init__('obstorviz_node')
        
        # 3. URDF에 정의된 관절 이름 목록
        self.joint_names = [
            "hip_yaw_left",         
            "hip_yaw_right",        
            "hip_roll_left",        
            "hip_roll_right",       
            "hip_pitch_left",       
            "hip_pitch_right",      
            "knee_left",            
            "knee_right",           
            "ankle_pitch_left",     
            "ankle_pitch_right",    
            "ankle_roll_left",      
            "ankle_roll_right",     
        ]
        
        # 최신 관절 위치를 저장할 변수 (0.0으로 초기화)
        self.latest_joint_pos = [0.0] * len(self.joint_names)
        # 데이터 접근을 보호하기 위한 잠금 (Lock)
        self.lock = threading.Lock()

        # 1. mujoco_obs 토픽 구독
        self.subscription = self.create_subscription(
            # Observation,
            Test,
            # 'mujoco_obs',
            # 'action',
            'test',
            self.obs_callback, # 구독 콜백 지정
            10
        )
        
        # 2. /joint_states 토픽 발행
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # 10Hz (0.1초) 주기로 발행 타이머 생성
        timer_period = 0.1  # 1.0 / 10Hz = 0.1초
        self.timer = self.create_timer(timer_period, self.timer_callback) # 타이머 콜백 지정

    def obs_callback(self, msg):
        # 스레드 충돌을 방지하기 위해 lock 사용
        with self.lock:
            # self.latest_joint_pos = list(msg.action)
            # self.latest_joint_pos = list(msg.joint_pos)
            self.latest_joint_pos = list(msg.test)

    # 10Hz 타이머가 호출할 함수
    def timer_callback(self):
        joint_state_msg = JointState()
        
        # A. 헤더 설정
        joint_state_msg.header.stamp = self.get_clock().now().to_msg() 
        
        # B. 관절 이름 및 위치 설정
        joint_state_msg.name = self.joint_names
        
        # 스레드 충돌을 방지하기 위해 lock 사용
        with self.lock:
            joint_state_msg.position = self.latest_joint_pos
        
        # C. JointState 메시지 발행
        self.publisher.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)  # 1. rclpy 초기화
    
    obstorviz_node = JointStateTranslator()  # 2. 노드 클래스 인스턴스 생성
    
    try:
        rclpy.spin(obstorviz_node)  # 3. 노드 실행 (콜백 처리를 위해 대기)
    except KeyboardInterrupt:
        pass  # Ctrl+C로 종료 시
    finally:
        # 4. 노드 소멸 및 rclpy 종료
        obstorviz_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()