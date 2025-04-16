import numpy as np
from spatialmath import SE3
import roboticstoolbox as rtb
import time
import rclpy
import os
import tempfile
import subprocess
import math
import sys
import rclpy
from rclpy.qos import QoSProfile
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


"""
카테시안 제어 라이브러리 구현
1. 역기구학 기반 접근 방식: 카테시안 공간의 목표 포즈를 관절 공간으로 변환하여 제어
2. 다양한 경로 유형 지원: 원형 경로, 직선 경로 등 여러 형태의 경로를 생성하고 따를 수 있다
3. ROS2 통합: ROS2의 메시지 시스템과 함께 작동하도록 설계
4. 실시간 경로 추가: 실행 중에도 새로운 경로 포인트를 추가할 수 있다.
"""

class CartesianController(Node):
    """
    카테시안 공간에서 로봇 제어를 위한 컨트롤러 클래스
    """
    def __init__(self, robot_model, joint_names, control_freq=100):
        """
        초기화 함수
        
        Args:
            robot_model: 로봇 모델 (roboticstoolbox 모델)
            joint_names: 로봇 관절 이름 리스트
            control_freq: 제어 주파수 (Hz)
        """
        super().__init__('cartesian_controller')
        
        # 로봇 모델과 관절 설정
        self.robot = robot_model
        self.joint_names = joint_names
        self.control_freq = control_freq
        self.control_period = 1.0 / control_freq
        
        # 현재 관절 상태
        self.current_joint_positions = np.zeros(self.robot.n)
        # self.current_joint_velocities = np.zeros(self.robot.n) 
        
        # 관절 상태 구독
        self.joint_state_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10)
        
        # 궤적 명령 발행자
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10)
        
        # 카테시안 경로 구독
        self.cartesian_path_sub = self.create_subscription(
            PoseStamped, 
            '/cartesian_path', 
            self.cartesian_path_callback, 
            10)
        
        # 타이머 생성
        self.timer = self.create_timer(self.control_period, self.control_loop)
        
        # 실행 중인 경로 저장
        self.target_cartesian_poses = []
        self.target_times = []
        self.executing_path = False
        self.path_start_time = 0.0
        
        self.get_logger().info('카테시안 컨트롤러가 초기화되었습니다.')
    
    def joint_state_callback(self, msg):
        """
        관절 상태 수신 콜백
        """
        # 관절 순서에 맞게 데이터 매핑
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joint_positions[i] = msg.position[idx]
                # self.current_joint_velocities[i] = msg.velocity[idx] if len(msg.velocity) > 0 else 0.0
    
    def cartesian_path_callback(self, msg):
        """
        카테시안 경로 포인트 수신 콜백
        """
        # 수신된 포즈를 경로에 추가
        pose = msg.pose
        
        # SE3 변환 행렬로 변환
        T = SE3(
            np.array([
                pose.position.x,
                pose.position.y,
                pose.position.z
            ])
        )
        
        # 현재 실행중인 경로가 없다면 새 경로 시작
        if not self.executing_path:
            self.target_cartesian_poses = [T]
            self.target_times = [0.0]  # 첫 포인트는 즉시 실행
            self.executing_path = True
            self.path_start_time = self.get_clock().now().seconds_nanoseconds()[0]
            self.get_logger().info('새로운 카테시안 경로 시작')
        else:
            # 이전 포인트와의 거리 계산하여 시간 설정
            prev_T = self.target_cartesian_poses[-1]
            distance = np.linalg.norm(T.t - prev_T.t)
            prev_time = self.target_times[-1]
            new_time = prev_time + max(0.5, distance * 2.0)  # 거리에 따른 시간 할당
            
            self.target_cartesian_poses.append(T)
            self.target_times.append(new_time)
            self.get_logger().info(f'카테시안 경로 포인트 추가 (시간: {new_time:.2f}s)')
    
    def generate_joint_trajectory(self, target_cartesian_poses, target_times):
        """
        카테시안 경로에서 관절 궤적 생성 (개선된 버전)
        """
        joint_waypoints = []
        success_flags = []
        
        # 초기 관절 각도
        q_current = self.current_joint_positions.copy()
        print(f"[DEBUG] 초기 관절 값: {q_current}")
        
        # 각 카테시안 포즈에 대한 역기구학 계산
        for i, T_target in enumerate(target_cartesian_poses):
            print(f"[DEBUG] 목표 포즈 {i}: {T_target}")
            
            # 작업 공간 확인
            position = T_target.t
            if not (0.0 <= position[0] <= 1.0 and -0.5 <= position[1] <= 0.5 and 0.0 <= position[2] <= 1.0):
                print(f"[WARN] 목표 포즈 {i}가 작업 공간을 벗어났습니다: {position}")
                success_flags.append(False)
                joint_waypoints.append(q_current)
                continue
            
            # 여러 초기값으로 IK 시도
            best_q = None
            best_cost = float('inf')
            
            # 현재 관절 각도에서 시작하는 IK 시도
            sol = self.robot.ik_LM(T_target, q0=q_current)
            
            if sol[1]:  # 성공 여부
                # 비용 함수: 현재 관절 각도와의 차이(변화량)
                if i > 0:
                    cost = np.sum(np.abs(sol[0] - joint_waypoints[-1]))
                else:
                    cost = np.sum(np.abs(sol[0] - q_current))
                    
                best_q = sol[0]
                best_cost = cost
            
            # 추가 초기값으로 시도 (더 나은 해를 찾기 위해)
            for _ in range(5):  # 5번 더 시도
                # 약간 변형된 초기값
                rand_offset = np.random.uniform(-0.2, 0.2, size=len(q_current))
                q_start = q_current + rand_offset
                
                # 관절 제한 내에 있도록 보정
                q_start = np.clip(q_start, self.robot.qlim[0], self.robot.qlim[1])
                
                alt_sol = self.robot.ik_LM(T_target, q0=q_start)
                
                if alt_sol[1]:  # 성공 여부
                    # 비용 계산 - 이전 웨이포인트와의 차이
                    if i > 0:
                        cost = np.sum(np.abs(alt_sol[0] - joint_waypoints[-1]))
                    else:
                        cost = np.sum(np.abs(alt_sol[0] - q_current))
                    
                    # 더 나은 해를 찾았으면 업데이트
                    if cost < best_cost:
                        best_q = alt_sol[0]
                        best_cost = cost
                        print(f"[DEBUG] 더 나은 IK 해를 찾았습니다. 비용: {cost}")
            
            if best_q is not None:
                joint_waypoints.append(best_q)
                success_flags.append(True)
                q_current = best_q  # 다음 역기구학 계산을 위한 초기값 업데이트
                print(f"[DEBUG] 성공: 관절 값 {i}: {best_q}, 비용: {best_cost}")
            else:
                self.get_logger().warn(f"[WARN] 역기구학 해를 찾을 수 없습니다. 목표 포즈 {i}: {T_target}")
                joint_waypoints.append(q_current)  # 이전 값 유지
                success_flags.append(False)
        
        return joint_waypoints, success_flags
    
    
    def publish_joint_trajectory(self, joint_waypoints, times):
        """
        관절 궤적 메시지 발행
        """
        if len(joint_waypoints) == 0:
            return
        
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        for i, waypoint in enumerate(joint_waypoints):
            point = JointTrajectoryPoint()
            point.positions = waypoint.tolist()
            
            # 속도 및 가속도 계산 (간단한 방법, 고급 기법으로 대체 가능)
            if i < len(joint_waypoints) - 1:
                next_waypoint = joint_waypoints[i+1]
                dt = times[i+1] - times[i]
                velocities = (next_waypoint - waypoint) / dt if dt > 0 else np.zeros_like(waypoint)
                point.velocities = velocities.tolist()
            else:
                point.velocities = [0.0] * len(waypoint)
            
            point.accelerations = [0.0] * len(waypoint)
            
            # 시간 설정
            time_sec = int(times[i])
            time_nanosec = int((times[i] - time_sec) * 1e9)
            point.time_from_start = Duration(sec=time_sec, nanosec=time_nanosec)
            
            msg.points.append(point)
        
        self.trajectory_pub.publish(msg)
        self.get_logger().info(f'관절 궤적 메시지 발행 ({len(joint_waypoints)} 포인트)')
    def control_loop(self):
        """
        주 제어 루프 (단순화된 버전)
        """
        if not self.executing_path or len(self.target_cartesian_poses) == 0:
            return
        
        # 이미 경로가 발행되었는지 확인하는 플래그 추가
        if not hasattr(self, 'path_published') or not self.path_published:
            # 관절 궤적 생성 및 발행 (한 번만 실행)
            joint_waypoints, success_flags = self.generate_joint_trajectory(
                self.target_cartesian_poses, 
                self.target_times
            )
            
            if all(success_flags):
                # 경로 발행
                self.publish_joint_trajectory(joint_waypoints, self.target_times)
                self.path_published = True  # 경로가 발행되었음을 표시
                self.get_logger().info('카테시안 경로 발행 완료')
            else:
                self.get_logger().error('일부 웨이포인트에 대한 역기구학 해를 찾을 수 없습니다.')
        
        # 남은 코드는 단순히 경로 완료 여부만 확인
        current_time = self.get_clock().now().seconds_nanoseconds()[0] - self.path_start_time
        if current_time > self.target_times[-1] + 1.0:  # 마지막 웨이포인트 이후 1초 대기
            self.executing_path = False
            self.target_cartesian_poses = []
            self.target_times = []
            self.path_published = False
            self.get_logger().info('카테시안 경로 실행 완료')

    
    def follow_cartesian_path(self, poses, duration=5.0):
        """
        카테시안 경로 추종 시작
        
        Args:
            poses: 카테시안 포즈 목록 (SE3 또는 변환 행렬)
            duration: 전체 경로 실행 시간 (초)
        """
        if len(poses) == 0:
            self.get_logger().warn('빈 경로는 실행할 수 없습니다.')
            return False
        
        # 시간 간격 계산
        times = np.linspace(0, duration, len(poses))
        
        # 경로 설정
        self.target_cartesian_poses = poses
        self.target_times = times.tolist()
        self.executing_path = True
        self.path_start_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        self.get_logger().info(f'카테시안 경로 실행 시작 ({len(poses)} 포인트, {duration:.2f}초)')
        return True
    
    def follow_circular_path(self, center, radius, axis='xy', duration=10.0, num_points=20):
        """
        원형 경로 추종
        
        Args:
            center: 원의 중심 [x, y, z]
            radius: 원의 반지름
            axis: 원이 그려질 평면 ('xy', 'yz', 'xz')
            duration: 경로 실행 시간 (초)
            num_points: 경로 포인트 수
        """
        poses = []
        current_pose = self.robot.fkine(self.current_joint_positions)
        # current_pose.R과 current_pose.t를 디버깅 출력
        print(f"current_pose.R: {current_pose.R}")
        print(f"current_pose.t: {current_pose.t}")
        
        for i in range(num_points + 1):
            theta = 2.0 * np.pi * i / num_points
            
            if axis == 'xy':
                x = center[0] + radius * np.cos(theta)
                y = center[1] + radius * np.sin(theta)
                z = center[2]
            elif axis == 'yz':
                x = center[0]
                y = center[1] + radius * np.cos(theta)
                z = center[2] + radius * np.sin(theta)
            elif axis == 'xz':
                x = center[0] + radius * np.cos(theta)
                y = center[1]
                z = center[2] + radius * np.sin(theta)
            else:
                raise ValueError(f"지원하지 않는 축: {axis}")
            
            # 현재 회전은 유지하면서 위치만 변경
            # SE3 객체 생성
            try:
                T = SE3.Rt(current_pose.R, [x, y, z])  # SE3 생성자 수정
                poses.append(T)
            except ValueError as e:
                print(f"SE3 생성 오류: {e}")
                return False
        
        return self.follow_cartesian_path(poses, duration)
    
    def follow_linear_path(self, start_point, end_point, duration=5.0, num_points=10):
        """
        직선 경로 추종 - 관절 공간 보간 방식
        """
        # 현재 포즈와 목표 포즈
        current_pose = self.robot.fkine(self.current_joint_positions)
        target_pose = SE3.Rt(current_pose.R, end_point)
        
        # 현재 관절 각도
        q_current = self.current_joint_positions.copy()
        print(f"현재 관절 각도: {q_current}")
        
        # 목표 포즈에 대한 IK 해 계산
        sol = self.robot.ik_LM(target_pose, q0=q_current)
        if not sol[1]:
            self.get_logger().error('목표 포인트에 대한 IK 해를 찾을 수 없습니다.')
            return False
        
        q_target = sol[0]
        print(f"목표 관절 각도: {q_target}")
        
        # 관절 공간에서 직접 보간
        joint_waypoints = []
        for i in range(num_points):
            alpha = i / (num_points - 1)
            q_interp = q_current * (1.0 - alpha) + q_target * alpha
            joint_waypoints.append(q_interp)
        
        # 시간 간격 계산
        times = np.linspace(0, duration, num_points)
        
        # 관절 경로 직접 발행
        self.publish_joint_trajectory(joint_waypoints, times.tolist())
        
        # 발행 완료 플래그 설정
        self.target_cartesian_poses = []  # 비움 (직접 관절 경로를 사용하므로)
        self.target_times = times.tolist()
        self.executing_path = True
        self.path_start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.path_published = True  # 경로가 이미 발행됨을 표시
        
        self.get_logger().info(f'관절 보간 직선 경로 실행 시작 ({num_points} 포인트, {duration:.2f}초)')
        return True
    

def check_point_in_workspace(workspace_points, target_point, tolerance=0.01):
    """
    주어진 점이 작업 공간 내에 있는지 확인합니다.
    
    Args:
        workspace_points: 작업 공간의 모든 점 리스트
        target_point: 확인할 점 [x, y, z]
        tolerance: 허용 오차
        
    Returns:
        bool: 점이 작업 공간 내에 있으면 True, 없으면 False
    """
    target_point = np.array(target_point)
    
    # 작업 공간의 모든 점과 타겟 점 사이의 거리를 계산
    distances = np.sqrt(np.sum((workspace_points - target_point)**2, axis=1))
    
    # 가장 가까운 점의 거리가 허용 오차보다 작은지 확인
    if np.min(distances) <= tolerance:
        print(f"점 {target_point}은 작업 공간 내에 있습니다!")
        return True
    else:
        print(f"점 {target_point}은 작업 공간 내에 없습니다. 가장 가까운 점과의 거리: {np.min(distances)}")
        closest_idx = np.argmin(distances)
        print(f"가장 가까운 점: {workspace_points[closest_idx]}")
        return False
    

def calculate_workspace(robot, samples=5):
    """
    로봇의 작업 공간을 계산합니다.
    Args:
        robot: 로봇 모델 (roboticstoolbox)
        samples: 각 관절의 샘플링 개수
    Returns:
        workspace_points: 작업 공간의 모든 점 리스트
    """
    joint_limits = robot.qlim  # 관절 제한
    joint_ranges = [np.linspace(joint_limits[0, i], joint_limits[1, i], samples) for i in range(robot.n)]
    workspace_points = []

    # 모든 관절 조합에 대해 순방향 기구학 계산
    for joint_angles in np.array(np.meshgrid(*joint_ranges)).T.reshape(-1, robot.n):
        pose = robot.fkine(joint_angles)  # 순방향 기구학 계산
        workspace_points.append(pose.t)  # 앤드 이펙터 위치 추가

    return np.array(workspace_points)

def visualize_workspace_with_target(workspace_points, target_point):
    """
    작업 공간과 타겟 위치를 함께 시각화합니다.
    
    Args:
        workspace_points: 작업 공간의 모든 점 리스트
        target_point: 타겟 위치 [x, y, z]
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 작업 공간 점들 시각화
    ax.scatter(workspace_points[:, 0], workspace_points[:, 1], workspace_points[:, 2], 
               s=1, c='blue', alpha=0.5, label='Workspace')
    
    # 타겟 위치 시각화 (크게 표시)
    ax.scatter([target_point[0]], [target_point[1]], [target_point[2]], 
               s=100, c='red', marker='o', label='Target (0,0,0.4)')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Robot Workspace with Target Point')
    ax.legend()
    
    # 다양한 각도에서 볼 수 있도록 회전 기능 추가
    plt.show()



# 사용 예시
def main():
    rclpy.init()
    
    # 1. XACRO 파일을 URDF로 변환
    urdf_path = "ros2_ws/src/sfbot_can/description/urdf/test.description.urdf.xacro"
    full_xacro_path = os.path.join(os.getcwd(), urdf_path)

    # 임시 URDF 파일 생성
    temp_urdf = tempfile.NamedTemporaryFile(suffix='.urdf', delete=False).name
    print(f"임시 URDF 파일 경로: {temp_urdf}")

    # ROS2 명령으로 XACRO를 URDF로 변환
    try:
        subprocess.run(
            ["ros2", "run", "xacro", "xacro", full_xacro_path, "-o", temp_urdf],
            check=True
        )
        print("XACRO를 URDF로 성공적으로 변환했습니다.")
    except Exception as e:
        print(f"XACRO 변환 오류: {e}")
        return

    # 로봇 모델 로드
    try:
        robot = rtb.Robot.URDF(temp_urdf)  # 올바른 메서드 사용
        print("로봇 모델 로드 완료")
        print(f"로봇 이름: {robot.name}")
        print(f"관절 수: {robot.n}")
    except Exception as e:
        print(f"로봇 모델 로드 오류: {e}")
        return
    joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

    robot.q = [0, 0, 0, 0, 0, 0]  # 초기 관절 값
    pose = robot.fkine(robot.q)
    print(f"초기 관절 값: {robot.q}, 계산된 포즈: {pose}")

    # 작업 공간 계산
    workspace_points = calculate_workspace(robot, samples=5)

    # 타겟 위치 확인
    target_point = [0, 0, 0.4]
    check_point_in_workspace(workspace_points, target_point)

    # 시각화
    visualize_workspace_with_target(workspace_points, target_point)
    # 컨트롤러 초기화
    controller = CartesianController(robot, joint_names)
    time.sleep(3)  # 초기화 대기
    """
    # 특정 포인트로 이동
    target_point = [0.1, 0.2, 0.2] # 예: [x, y, z]
    current_pose = robot.fkine(controller.current_joint_positions)  # 현재 포즈 가져오기
    print(f"현재 포즈: {current_pose}")

    try:
        # 목표 포즈 생성 (현재 회전 유지)
        target_pose = SE3.Rt(current_pose.R, target_point)
        print(f"목표 포즈: {target_pose}")

        # 경로 생성 및 실행
        print(f"전달된 poses: {[target_pose]}")
        controller.follow_cartesian_path([target_pose], duration=5.0)
        print(f"self.target_cartesian_poses: {controller.target_cartesian_poses}")
    except Exception as e:
        print(f"목표 포즈로 이동 중 오류 발생: {e}")
    """
    # 현재 포즈 가져오기
    current_pose = robot.fkine(controller.current_joint_positions)
    # 직선 경로 생성
    start_point = [0.0, 0.0, 0.4]
    end_point = [0.05, 0.05, 0.02]
    num_points = 10  # 포인트 개수

    # 직선 경로 포인트 생성
    linear_poses = []
    for i in range(num_points):
        alpha = i / (num_points - 1)
        x = start_point[0] * (1 - alpha) + end_point[0] * alpha
        y = start_point[1] * (1 - alpha) + end_point[1] * alpha
        z = start_point[2] * (1 - alpha) + end_point[2] * alpha
        pose = SE3.Rt(current_pose.R, [x, y, z])  # 현재 회전 유지
        linear_poses.append(pose)

    # 컨트롤러로 전달
    controller.follow_linear_path(start_point, end_point, duration=5.0, num_points=10)
    # 경로 실행이 완료될 때까지 대기
    time.sleep(7.0)  # 5초(duration) + 약간의 여유 시간
    # 2. 끝점에서 시작점으로 돌아오기
    controller.follow_linear_path(end_point, start_point, duration=5.0, num_points=10)

    rclpy.spin(controller)
    
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()