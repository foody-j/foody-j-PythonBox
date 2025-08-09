# 파일명: joint_limit_tester.py (목표 위치 기억 및 속도 조절 기능 추가 최종 버전)

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
import sys
import termios
import tty
import threading
import copy
import math # 라디안 <-> 각도 변환을 위해 추가

# ==============================================================================
# 1. 전역 상수 정의
# ==============================================================================
CONTROLLER_NAME = "my_robot_arm_controller"
JOINT_NAMES = [
    "link1_1_joint", "link2_1_joint", "link3_1_joint", 
    "link4_1_joint", "link5_1_joint", "link6_1_joint"
]
INCREMENT_RAD = 0.05 # 키보드 제어 시 증가/감소량 (라디안)

key_to_joint_map = {
    'q': (0, 1), 'a': (0, -1), 'w': (1, 1), 's': (1, -1),
    'e': (2, 1), 'd': (2, -1), 'r': (3, 1), 'f': (3, -1),
    't': (4, 1), 'g': (4, -1), 'y': (5, 1), 'h': (5, -1),
}
msg = """
---------------------------------------------------
      조인트 리밋 테스트 (명령어 입력 방식)
---------------------------------------------------
- 목표 위치로 바로 이동: goto <조인트 번호> <각도>
  (예: goto 1 30 -> 1번 조인트를 30도로 이동)

- 키보드로 미세 조정: manual
  ('manual' 입력 후 'q','a','w','s'... 키로 제어)
  (미세 조정 모드에서 나오려면 Enter 키를 누르세요)

- 이동 시간(속도) 조절: speed <초>
  (예: speed 3.5 -> 다음 목표까지 3.5초 동안 이동)

- 종료: exit
---------------------------------------------------
"""

def get_key_manual_mode(settings):
    """키보드 미세 조정 모드에서 사용할 키 입력 함수"""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# ==============================================================================
# 2. 메인 노드 클래스 정의
# ==============================================================================
class JointLimitTester(Node):
    def __init__(self):
        super().__init__('joint_limit_tester')
        
        self.publisher_ = self.create_publisher(
            JointTrajectory, f"/{CONTROLLER_NAME}/joint_trajectory", 10)
        
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.actual_positions_rad = [0.0] * len(JOINT_NAMES)
        self.goal_positions_rad = [0.0] * len(JOINT_NAMES)

        self.joint_state_received = False
        self.lock = threading.Lock()
        self.time_to_reach = Duration(sec=1, nanosec=0)
        
        self.get_logger().info("조인트 리밋 테스트 노드가 시작되었습니다.")
        self.get_logger().info("'/joint_states' 토픽을 기다리는 중...")

    def joint_state_callback(self, msg):
        with self.lock:
            if not self.joint_state_received:
                self.get_logger().info("첫 조인트 상태 수신 완료! 현재 위치로 동기화합니다.")
                temp_positions = [0.0] * len(JOINT_NAMES)
                for i, name in enumerate(JOINT_NAMES):
                    try:
                        idx = msg.name.index(name)
                        temp_positions[i] = msg.position[idx]
                    except ValueError:
                        pass
                self.actual_positions_rad = temp_positions
                self.goal_positions_rad = copy.deepcopy(self.actual_positions_rad) 
                self.joint_state_received = True
            else:
                for i, name in enumerate(JOINT_NAMES):
                    try:
                        idx = msg.name.index(name)
                        self.actual_positions_rad[i] = msg.position[idx]
                    except ValueError:
                        pass

    def send_goal_command(self):
        """현재 self.goal_positions_rad에 저장된 목표 위치를 로봇에 전송합니다."""
        if not self.joint_state_received:
            self.get_logger().warn("아직 로봇의 실제 상태를 모릅니다. 명령을 보낼 수 없습니다.")
            return

        with self.lock:
            positions_to_send = copy.deepcopy(self.goal_positions_rad)
            time_to_send = copy.deepcopy(self.time_to_reach)
        
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = positions_to_send
        point.time_from_start = time_to_send
        trajectory_msg.points.append(point)
        
        self.publisher_.publish(trajectory_msg)
        
        goal_pos_str = ", ".join([f"{pos:.3f}" for pos in positions_to_send])
        time_str = f"{time_to_send.sec}.{time_to_send.nanosec // 1000000:03d}"
        self.get_logger().info(f"명령 전송 (rad): [{goal_pos_str}] (도달 시간: {time_str}s)")

    def update_goal_by_key(self, joint_index, direction):
        """키보드 입력에 따라 목표 위치를 업데이트합니다."""
        with self.lock:
            self.goal_positions_rad[joint_index] += direction * INCREMENT_RAD
        self.send_goal_command()

    def update_goal_by_value(self, joint_index, angle_deg):
        """숫자 입력에 따라 목표 위치를 업데이트합니다."""
        with self.lock:
            self.goal_positions_rad[joint_index] = math.radians(angle_deg)
        self.send_goal_command()

    def set_speed(self, time_sec):
        """목표 도달 시간을 설정합니다."""
        with self.lock:
            seconds = int(time_sec)
            nanoseconds = int((time_sec - seconds) * 1e9)
            self.time_to_reach = Duration(sec=seconds, nanosec=nanoseconds)
        print(f"이동 시간이 {time_sec}초로 설정되었습니다.")

    def print_status(self):
        """현재 로봇의 실제 위치와 목표 위치를 각도(degree)로 변환하여 출력합니다."""
        if not self.joint_state_received:
            return
        
        with self.lock:
            actual_deg = [math.degrees(pos) for pos in self.actual_positions_rad]
            goal_deg = [math.degrees(pos) for pos in self.goal_positions_rad]

        header = "| Joint | " + " | ".join([f"  J{i+1}   " for i in range(len(JOINT_NAMES))]) + " |"
        sep = "|-------|" + "--------|"*len(JOINT_NAMES)
        actual_str = "| Actual| " + " | ".join([f"{pos:6.1f}" for pos in actual_deg]) + " |"
        goal_str = "| Goal  | " + " | ".join([f"{pos:6.1f}" for pos in goal_deg]) + " |"
        
        print("\n--- 현재 로봇 상태 (단위: 도) ---")
        print(header)
        print(sep)
        print(actual_str)
        print(goal_str)
        print("------------------------------------")


# ==============================================================================
# 3. 메인 실행 함수
# ==============================================================================
def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    tester_node = JointLimitTester()

    thread = threading.Thread(target=rclpy.spin, args=(tester_node,), daemon=True)
    thread.start()

    while not tester_node.joint_state_received:
        rclpy.spin_once(tester_node, timeout_sec=0.1)

    print(msg)
    
    try:
        while rclpy.ok():
            tester_node.print_status()
            command = input("명령을 입력하세요 > ").strip().lower()
            parts = command.split()

            if not parts:
                continue

            if parts[0] == 'exit':
                break
            
            elif parts[0] == 'goto':
                try:
                    if len(parts) != 3:
                        raise ValueError("형식: 'goto <번호> <각도>'")
                    joint_idx = int(parts[1]) - 1
                    angle_deg = float(parts[2])
                    if not (0 <= joint_idx < len(JOINT_NAMES)):
                        raise ValueError("조인트 번호는 1에서 6 사이여야 합니다.")
                    tester_node.update_goal_by_value(joint_idx, angle_deg)
                except (ValueError, IndexError) as e:
                    print(f"입력 오류: {e}")
            
            elif parts[0] == 'speed':
                try:
                    if len(parts) != 2:
                        raise ValueError("형식: 'speed <초>'")
                    time_sec = float(parts[1])
                    if time_sec <= 0:
                        raise ValueError("시간은 0보다 커야 합니다.")
                    tester_node.set_speed(time_sec)
                except (ValueError, IndexError) as e:
                    print(f"입력 오류: {e}")

            elif parts[0] == 'manual':
                print("\n--- 키보드 미세 조정 모드 ---")
                print("('q','a','w','s'...) 키로 제어하세요. (나가려면 Enter)")
                while True:
                    key = get_key_manual_mode(settings)
                    if key in key_to_joint_map:
                        joint_idx, direction = key_to_joint_map[key]
                        tester_node.update_goal_by_key(joint_idx, direction)
                        tester_node.print_status()
                    elif key == '\r': # Enter 키
                        print("--- 명령어 입력 모드로 복귀 ---")
                        break
                    elif key == '\x03': # CTRL-C
                        raise KeyboardInterrupt
            else:
                print("알 수 없는 명령입니다. 'goto', 'manual', 'speed', 'exit' 중 하나를 입력하세요.")

    except (KeyboardInterrupt, EOFError):
        print("\n프로그램을 종료합니다.")

    finally:
        tester_node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
