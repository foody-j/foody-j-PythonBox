#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from builtin_interfaces.msg import Duration
import json
import numpy as np

class SimpleCartesianController(Node):
    """
    간단한 카테시안 컨트롤러 - XYZ 명령을 받아서 관절 공간으로 변환
    """
    def __init__(self, robot_model, joint_names):
        super().__init__('simple_cartesian_controller')
        
        self.robot = robot_model
        self.joint_names = joint_names
        self.current_joint_positions = np.zeros(self.robot.n)
        
        # IK 통계
        self.ik_stats = {'total': 0, 'success': 0, 'failed': 0}
        
        # 구독자들
        self.joint_state_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10)
        
        # XYZ 명령 구독 (단일 포인트)
        self.xyz_command_sub = self.create_subscription(
            PointStamped,
            '/cartesian_command',
            self.xyz_command_callback,
            10)
        
        # JSON 형태의 복합 명령 구독
        self.path_command_sub = self.create_subscription(
            String,
            '/cartesian_path_command',
            self.path_command_callback,
            10)
        
        # 발행자
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10)
        
        self.status_pub = self.create_publisher(
            String,
            '/cartesian_controller_status',
            10)
        
        self.get_logger().info('🚀 개선된 카테시안 컨트롤러가 초기화되었습니다.')
        self.get_logger().info('💡 개선된 IK 알고리즘이 적용되었습니다.')
        self.get_logger().info('사용법:')
        self.get_logger().info('  단일 포인트: ros2 topic pub /cartesian_command geometry_msgs/PointStamped ...')
        self.get_logger().info('  복합 명령: ros2 topic pub /cartesian_path_command std_msgs/String ...')
    
    def joint_state_callback(self, msg):
        """관절 상태 업데이트 - 실제 순서에 맞게 매핑"""
        # 첫 번째 콜백에서 실제 순서 확인 및 매핑 테이블 생성
        if not hasattr(self, '_joint_mapping'):
            self.get_logger().info(f"실제 /joint_states에서 받은 관절 이름들: {msg.name}")
            self.get_logger().info(f"컨트롤러에서 사용하는 관절 이름들: {self.joint_names}")
            
            # 매핑 테이블 생성: 컨트롤러 관절 순서 -> 실제 메시지 인덱스
            self._joint_mapping = []
            matched_joints = []
            
            for controller_name in self.joint_names:
                if controller_name in msg.name:
                    msg_index = msg.name.index(controller_name)
                    self._joint_mapping.append(msg_index)
                    matched_joints.append(controller_name)
                else:
                    self._joint_mapping.append(-1)  # 매칭되지 않음
            
            self.get_logger().info(f"매칭되는 관절들: {matched_joints}")
            self.get_logger().info(f"관절 매핑 인덱스: {self._joint_mapping}")
            
            if len(matched_joints) != len(self.joint_names):
                self.get_logger().error(f"일부 관절이 매칭되지 않았습니다: {len(matched_joints)}/{len(self.joint_names)}")
        
        # 실제 순서에 맞게 관절 상태 업데이트
        updated_joints = 0
        for i, msg_index in enumerate(self._joint_mapping):
            if msg_index >= 0 and msg_index < len(msg.position):
                self.current_joint_positions[i] = msg.position[msg_index]
                updated_joints += 1
        
        # 주기적으로 상태 확인 (30초마다)
        if not hasattr(self, '_last_status_time'):
            self._last_status_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if current_time - self._last_status_time > 30.0:
            self.get_logger().info(f"관절 상태: {updated_joints}/{len(self.joint_names)} 관절 업데이트")
            self.get_logger().info(f"IK 통계: {self.ik_stats['success']}/{self.ik_stats['total']} 성공 ({self.ik_stats['success']/max(1,self.ik_stats['total'])*100:.1f}%)")
            self._last_status_time = current_time
    
    def xyz_command_callback(self, msg):
        """단일 XYZ 포인트 명령 처리"""
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
        
        self.get_logger().info(f'🎯 XYZ 명령 수신: [{x:.3f}, {y:.3f}, {z:.3f}]')
        
        success = self.move_to_xyz(x, y, z, duration=3.0)
        
        status_msg = String()
        if success:
            status_msg.data = f"SUCCESS: Moved to [{x:.3f}, {y:.3f}, {z:.3f}]"
        else:
            status_msg.data = f"FAILED: Could not reach [{x:.3f}, {y:.3f}, {z:.3f}]"
        
        self.status_pub.publish(status_msg)
    
    def path_command_callback(self, msg):
        """JSON 형태의 복합 명령 처리"""
        try:
            command = json.loads(msg.data)
            command_type = command.get('type', 'unknown')
            
            if command_type == 'linear_path':
                start = command['start']
                end = command['end']
                duration = command.get('duration', 5.0)
                points = command.get('points', 10)
                
                success = self.follow_linear_path(start, end, duration, points)
                
            elif command_type == 'circular_path':
                center = command['center']
                radius = command['radius']
                axis = command.get('axis', 'xy')
                duration = command.get('duration', 10.0)
                points = command.get('points', 20)
                
                success = self.follow_circular_path(center, radius, axis, duration, points)
                
            elif command_type == 'waypoints':
                waypoints = command['waypoints']
                duration = command.get('duration', 5.0)
                
                success = self.follow_waypoints(waypoints, duration)
                
            else:
                self.get_logger().error(f'알 수 없는 명령 타입: {command_type}')
                success = False
            
            status_msg = String()
            status_msg.data = f"{'SUCCESS' if success else 'FAILED'}: {command_type}"
            self.status_pub.publish(status_msg)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON 파싱 오류: {e}')
        except Exception as e:
            self.get_logger().error(f'명령 처리 오류: {e}')
    
    def solve_ik_improved(self, target_pose, current_q=None):
        """
        🔥 개선된 IK - 더 많은 초기값과 스마트한 시도
        """
        self.ik_stats['total'] += 1
        best_solution = None
        best_error = float('inf')
        success_count = 0
        
        # 시도할 초기값들
        initial_guesses = []
        
        # 1. 현재 위치 (최우선)
        if current_q is not None:
            initial_guesses.append(current_q.copy())
            self.get_logger().info(f"✅ 현재 관절 위치를 초기값으로 사용")
        
        # 2. 홈 포지션
        initial_guesses.append(np.zeros(self.robot.n))
        
        # 3. 일반적인 작업 포즈들
        if self.robot.n >= 6:  # 6축 이상인 경우
            common_poses = [
                np.array([0, -np.pi/4, np.pi/2, 0, np.pi/4, 0]),      # 전방 작업
                np.array([np.pi/2, -np.pi/4, np.pi/2, 0, np.pi/4, 0]), # 측면 작업
                np.array([-np.pi/2, -np.pi/4, np.pi/2, 0, np.pi/4, 0]), # 반대편 측면
                np.array([np.pi, -np.pi/4, np.pi/2, 0, np.pi/4, 0]),   # 후방 작업
                np.array([0, 0, np.pi/2, 0, 0, 0]),                   # 수직 상향
                np.array([0, -np.pi/2, 0, 0, 0, 0]),                  # 수평 전방
                np.array([0, -np.pi/6, np.pi/3, 0, np.pi/6, 0]),      # 약간 높은 작업
            ]
            for pose in common_poses:
                if len(pose) == self.robot.n:
                    initial_guesses.append(pose)
        
        # 4. 목표 위치 기반 스마트 추정
        target_pos = target_pose.t
        for _ in range(12):  # 12개 스마트 시도
            # 베이스 회전각을 목표 위치 기반으로 설정
            theta1 = np.arctan2(target_pos[1], target_pos[0])
            
            # 여러 변형 시도
            variations = [0, np.pi/6, -np.pi/6, np.pi/4, -np.pi/4, np.pi/3, -np.pi/3, np.pi/2]
            variation = variations[_ % len(variations)]
            theta1 += variation
            
            # 나머지는 일반적인 범위에서 랜덤
            smart_q = np.random.uniform(-np.pi/2, np.pi/2, self.robot.n)
            smart_q[0] = theta1  # 첫 번째 관절은 목표 기반
            
            # 관절 제한 적용
            smart_q = np.clip(smart_q, -np.pi, np.pi)
            initial_guesses.append(smart_q)
        
        # 5. 완전 랜덤 (백업용)
        for _ in range(8):
            random_q = np.random.uniform(-np.pi, np.pi, self.robot.n)
            initial_guesses.append(random_q)
        
        self.get_logger().info(f"🔍 총 {len(initial_guesses)}개 초기값으로 IK 시도")
        
        # 6. 각 초기값으로 IK 시도
        for i, q0 in enumerate(initial_guesses):
            try:
                # LM 방법 시도
                sol = self.robot.ik_LM(target_pose, q0=q0)
                if sol[1]:  # 성공
                    q_solution = sol[0]
                    
                    # 관절 제한 확인
                    if self._check_joint_limits(q_solution):
                        # 포즈 오차 계산
                        error = self._calculate_pose_error(q_solution, target_pose)
                        
                        if error < best_error:
                            best_error = error
                            best_solution = q_solution
                            success_count += 1
                            
                            self.get_logger().info(f"✨ IK 해 #{success_count}: 오차 {error:.4f}m (시도 {i+1}, LM)")
                            
                            # 충분히 좋은 해면 조기 종료
                            if error < 0.001:  # 1mm 이하
                                self.get_logger().info("🎯 충분히 정확한 해 발견, 조기 종료")
                                break
                
                # NR 방법도 시도 (다양성 확보)
                if i % 3 == 0:  # 3번에 1번씩만 시도 (속도 고려)
                    sol_nr = self.robot.ik_NR(target_pose, q0=q0)
                    if sol_nr[1]:
                        q_solution_nr = sol_nr[0]
                        if self._check_joint_limits(q_solution_nr):
                            error_nr = self._calculate_pose_error(q_solution_nr, target_pose)
                            
                            if error_nr < best_error:
                                best_error = error_nr
                                best_solution = q_solution_nr
                                success_count += 1
                                self.get_logger().info(f"✨ IK 해 #{success_count}: 오차 {error_nr:.4f}m (시도 {i+1}, NR)")
            
            except Exception as e:
                # 조용히 다음 시도로
                continue
        
        # 결과 반환
        if best_solution is not None:
            self.ik_stats['success'] += 1
            success_rate = self.ik_stats['success'] / self.ik_stats['total'] * 100
            self.get_logger().info(f"🎉 IK 성공! 최종 오차: {best_error:.4f}m")
            self.get_logger().info(f"📊 총 {success_count}개 해 발견, 누적 성공률: {success_rate:.1f}%")
            return best_solution, True
        else:
            self.ik_stats['failed'] += 1
            self.get_logger().error(f"❌ IK 실패: {len(initial_guesses)}개 초기값 모두 시도")
            self.get_logger().error(f"목표 위치: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
            return np.zeros(self.robot.n), False
    
    def _check_joint_limits(self, q):
        """관절 제한 확인 (유연한 버전)"""
        try:
            # 약간 넉넉한 관절 제한 (라디안)
            for i, angle in enumerate(q):
                if abs(angle) > np.pi * 1.2:  # ±216도까지 허용 (약간 넉넉하게)
                    return False
            return True
        except:
            return True
    
    def _calculate_pose_error(self, q, target_pose):
        """포즈 오차 계산"""
        try:
            current_pose = self.robot.fkine(q)
            position_error = np.linalg.norm(current_pose.t - target_pose.t)
            return position_error
        except:
            return float('inf')
    
    def move_to_xyz(self, x, y, z, duration=3.0):
        """단일 XYZ 포인트로 이동 - 개선된 IK 사용"""
        try:
            # 현재 포즈 확인
            current_pose = self.robot.fkine(self.current_joint_positions)
            self.get_logger().info(f"현재 엔드 이펙터 위치: {current_pose.t}")
            self.get_logger().info(f"목표 위치: [{x}, {y}, {z}]")
            
            from spatialmath import SE3
            target_pose = SE3.Rt(current_pose.R, [x, y, z])
            
            # 🔥 개선된 IK 사용
            best_sol, success = self.solve_ik_improved(target_pose, self.current_joint_positions)
            
            if not success:
                self.get_logger().error(f'IK 해를 찾을 수 없습니다: [{x}, {y}, {z}]')
                distance = np.linalg.norm(np.array([x, y, z]) - current_pose.t)
                self.get_logger().error(f'목표까지의 거리: {distance:.3f}m')
                return False
            
            # 궤적 생성 및 발행
            waypoints = [self.current_joint_positions, best_sol]
            times = [0.0, duration]
            
            self.get_logger().info(f"🚀 최종 IK 해: {best_sol}")
            self.publish_joint_trajectory(waypoints, times)
            return True
            
        except Exception as e:
            self.get_logger().error(f'XYZ 이동 오류: {e}')
            return False
    
    def follow_linear_path(self, start, end, duration=5.0, num_points=10):
        """직선 경로 추종 - 개선된 IK 사용"""
        try:
            # 관절 공간 보간 방식 사용
            current_pose = self.robot.fkine(self.current_joint_positions)
            from spatialmath import SE3
            
            target_pose = SE3.Rt(current_pose.R, end)
            best_sol, success = self.solve_ik_improved(target_pose, self.current_joint_positions)
            
            if not success:
                self.get_logger().error('목표 포인트에 대한 IK 해를 찾을 수 없습니다.')
                return False
            
            # 관절 공간에서 보간
            q_start = self.current_joint_positions.copy()
            q_end = best_sol
            
            waypoints = []
            times = []
            
            for i in range(num_points):
                alpha = i / (num_points - 1)
                q_interp = q_start * (1.0 - alpha) + q_end * alpha
                waypoints.append(q_interp)
                times.append(alpha * duration)
            
            self.publish_joint_trajectory(waypoints, times)
            return True
            
        except Exception as e:
            self.get_logger().error(f'직선 경로 오류: {e}')
            return False
    
    def follow_circular_path(self, center, radius, axis='xy', duration=10.0, num_points=20):
        """원형 경로 추종 - 개선된 IK 사용"""
        try:
            current_pose = self.robot.fkine(self.current_joint_positions)
            from spatialmath import SE3
            
            self.get_logger().info(f"🔄 원형 경로 시작 - 중심: {center}, 반지름: {radius}, 축: {axis}")
            self.get_logger().info(f"현재 엔드이펙터 위치: {current_pose.t}")
            
            waypoints = []
            times = []
            failed_points = 0
            
            # 이전 성공한 관절 위치 (IK 연속성을 위해)
            previous_q = self.current_joint_positions.copy()
            
            for i in range(num_points + 1):
                theta = 2.0 * np.pi * i / num_points
                
                # 원형 경로 계산
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
                    self.get_logger().error(f"지원하지 않는 축: {axis}")
                    return False
                
                target_position = [x, y, z]
                
                # 목표 포즈 생성
                target_pose = SE3.Rt(current_pose.R, target_position)
                
                # 🔥 개선된 IK 사용 (이전 위치를 초기값으로)
                ik_solution, ik_success = self.solve_ik_improved(target_pose, previous_q)
                
                if ik_success:
                    waypoints.append(ik_solution)
                    times.append(i * duration / num_points)
                    previous_q = ik_solution
                    
                    # FK로 실제 위치 확인
                    actual_pose = self.robot.fkine(ik_solution)
                    actual_pos = actual_pose.t
                    error = np.linalg.norm(np.array(target_position) - actual_pos)
                    self.get_logger().info(f"✅ 포인트 {i}: 위치 오차 {error:.4f}m")
                else:
                    self.get_logger().warn(f"⚠️ 포인트 {i}: IK 실패 - 위치 [{x:.3f}, {y:.3f}, {z:.3f}]")
                    failed_points += 1
                    
                    # 실패한 포인트가 너무 많으면 중단
                    if failed_points > num_points * 0.3:  # 30% 이상 실패
                        self.get_logger().error(f"원형 경로 중단: {failed_points}/{i+1} 포인트 실패")
                        return False
            
            success_rate = len(waypoints) / (num_points + 1) * 100
            self.get_logger().info(f"🎯 원형 경로 계산 완료: {len(waypoints)}/{num_points+1} 포인트 성공 ({success_rate:.1f}%)")
            
            if len(waypoints) >= 3:  # 최소 3개 포인트는 있어야 함
                self.publish_joint_trajectory_smooth(waypoints, times)
                return True
            else:
                self.get_logger().error(f"원형 경로 실패: 충분한 포인트를 생성할 수 없음 ({len(waypoints)} 포인트)")
                return False
                
        except Exception as e:
            self.get_logger().error(f'원형 경로 오류: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False
    
    def check_joint_limits(self, joint_positions):
        """관절 제한 확인"""
        try:
            # 일반적인 6축 로봇 관절 제한 (라디안)
            joint_limits = [
                (-np.pi, np.pi),      # Joint 1
                (-np.pi/2, np.pi/2),  # Joint 2
                (-np.pi, np.pi),      # Joint 3
                (-np.pi, np.pi),      # Joint 4
                (-np.pi/2, np.pi/2),  # Joint 5
                (-np.pi, np.pi),      # Joint 6
            ]
            
            for i, (pos, (min_limit, max_limit)) in enumerate(zip(joint_positions, joint_limits)):
                if pos < min_limit or pos > max_limit:
                    self.get_logger().warn(f"관절 {i+1} 제한 위반: {pos:.3f} (범위: {min_limit:.3f}~{max_limit:.3f})")
                    return False
            return True
        except Exception:
            return True  # 제한 확인 실패시 통과
        
    def publish_joint_trajectory_smooth(self, waypoints, times):
        """관절 궤적 발행 - 부드러운 속도 프로파일"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for i, (waypoint, time_val) in enumerate(zip(waypoints, times)):
            point = JointTrajectoryPoint()
            point.positions = waypoint.tolist()
            
            # 개선된 속도 계산
            if i == 0:
                # 첫 번째 포인트: 현재 위치에서 시작
                if len(waypoints) > 1:
                    dt = times[1] - times[0]
                    if dt > 0:
                        velocities = (waypoints[1] - waypoint) / dt * 0.5  # 부드럽게 시작
                        point.velocities = velocities.tolist()
                    else:
                        point.velocities = [0.0] * len(waypoint)
                else:
                    point.velocities = [0.0] * len(waypoint)
                    
            elif i == len(waypoints) - 1:
                # 마지막 포인트: 속도 0으로 끝
                point.velocities = [0.0] * len(waypoint)
                
            else:
                # 중간 포인트: 중앙 차분으로 속도 계산
                dt_prev = times[i] - times[i-1]
                dt_next = times[i+1] - times[i]
                
                if dt_prev > 0 and dt_next > 0:
                    vel_prev = (waypoint - waypoints[i-1]) / dt_prev
                    vel_next = (waypoints[i+1] - waypoint) / dt_next
                    velocities = (vel_prev + vel_next) / 2.0  # 평균 속도
                    point.velocities = velocities.tolist()
                else:
                    point.velocities = [0.0] * len(waypoint)
            
            # 가속도는 0으로 설정 (컨트롤러가 계산)
            point.accelerations = [0.0] * len(waypoint)
            
            # 시간 설정
            time_sec = int(time_val)
            time_nanosec = int((time_val - time_sec) * 1e9)
            point.time_from_start = Duration(sec=time_sec, nanosec=time_nanosec)
            
            msg.points.append(point)
        
        self.trajectory_pub.publish(msg)
        self.get_logger().info(f'🚀 부드러운 관절 궤적 발행: {len(waypoints)} 포인트')

    def follow_waypoints(self, waypoints, duration=5.0):
        """여러 웨이포인트 순차 실행 - 개선된 IK 사용"""
        try:
            joint_waypoints = []
            times = []
            
            current_pose = self.robot.fkine(self.current_joint_positions)
            from spatialmath import SE3
            
            q_current = self.current_joint_positions.copy()
            
            for i, point in enumerate(waypoints):
                pose = SE3.Rt(current_pose.R, point)
                sol, success = self.solve_ik_improved(pose, q_current)
                
                if not success:
                    self.get_logger().warn(f'웨이포인트 {i}에 대한 IK 해를 찾을 수 없습니다.')
                    continue
                
                joint_waypoints.append(sol)
                times.append(i * duration / len(waypoints))
                q_current = sol
            
            if len(joint_waypoints) > 0:
                self.publish_joint_trajectory(joint_waypoints, times)
                return True
            else:
                return False
                
        except Exception as e:
            self.get_logger().error(f'웨이포인트 경로 오류: {e}')
            return False
    
    def publish_joint_trajectory(self, waypoints, times):
        """관절 궤적 발행"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        for i, (waypoint, time_val) in enumerate(zip(waypoints, times)):
            point = JointTrajectoryPoint()
            point.positions = waypoint.tolist()
            
            # 속도 계산
            if i < len(waypoints) - 1:
                dt = times[i+1] - times[i]
                if dt > 0:
                    velocities = (waypoints[i+1] - waypoint) / dt
                    point.velocities = velocities.tolist()
                else:
                    point.velocities = [0.0] * len(waypoint)
            else:
                point.velocities = [0.0] * len(waypoint)
            
            point.accelerations = [0.0] * len(waypoint)
            
            # 시간 설정
            time_sec = int(time_val)
            time_nanosec = int((time_val - time_sec) * 1e9)
            point.time_from_start = Duration(sec=time_sec, nanosec=time_nanosec)
            
            msg.points.append(point)
        
        self.trajectory_pub.publish(msg)
        self.get_logger().info(f'🚀 관절 궤적 발행: {len(waypoints)} 포인트')


def load_robot_from_param():
    """파라미터나 환경변수에서 로봇 모델 로드"""
    import os
    import tempfile
    import subprocess
    import roboticstoolbox as rtb
    
    # URDF 파일 경로 찾기
    possible_paths = [
        "src/sfbot_can/description/urdf/my_robot.description.urdf.xacro",
        "src/sfbot_can/urdf/my_robot.description.urdf.xacro",
        "install/sfbot_can/share/sfbot_can/urdf/my_robot.description.urdf.xacro",
        "install/sfbot_can/share/sfbot_can/description/urdf/my_robot.description.urdf.xacro"
    ]
    
    xacro_path = None
    for path in possible_paths:
        full_path = os.path.join(os.getcwd(), path)
        if os.path.exists(full_path):
            xacro_path = full_path
            break
    
    if xacro_path is None:
        raise FileNotFoundError("XACRO 파일을 찾을 수 없습니다.")
    
    # 임시 URDF 파일 생성
    temp_urdf = tempfile.NamedTemporaryFile(suffix='.urdf', delete=False).name
    
    try:
        subprocess.run(["xacro", xacro_path, "-o", temp_urdf], check=True)
    except:
        subprocess.run(["ros2", "run", "xacro", "xacro", xacro_path, "-o", temp_urdf], check=True)
    
    # 로봇 모델 로드
    robot = rtb.Robot.URDF(temp_urdf)
    
    # 관절 이름 추출 - 실제 joint 이름을 찾기
    joint_names = []
    
    # 디버깅 정보 출력
    print("로봇 링크 정보:")
    for i, link in enumerate(robot.links):
        print(f"  Link {i}: {link.name}, isjoint: {link.isjoint}, jtype: {getattr(link, 'jtype', 'None')}")
        if link.isjoint:
            # joint 이름이 있다면 사용, 없다면 링크 이름에서 joint 이름 생성
            if hasattr(link, 'joint_name') and link.joint_name:
                joint_names.append(link.joint_name)
            else:
                # 링크 이름에서 joint 이름 추정
                joint_name = link.name.replace('_link', '_joint').replace('link', 'joint')
                if not joint_name.endswith('_joint'):
                    joint_name = joint_name + '_joint'
                joint_names.append(joint_name)
    
    # 만약 joint 이름을 찾지 못했다면, 일반적인 패턴 사용
    if not joint_names:
        print("Warning: joint 이름을 추출할 수 없습니다. 기본 패턴을 사용합니다.")
        joint_names = [f'joint_{i+1}' for i in range(robot.n)]
    
    print(f"추출된 관절 이름들: {joint_names}")
    
    # ROS2 control에서 실제 사용하는 joint 이름과 매칭해야 할 수도 있음
    # ros2_control에서 정의된 실제 관절 이름 사용
    actual_joint_names = ['link1_1_joint', 'link2_1_joint', 'link3_1_joint', 
                         'link4_1_joint', 'link5_1_joint', 'link6_1_joint']
    
    # 로봇 모델의 관절 수와 일치하는지 확인
    if len(actual_joint_names) != robot.n:
        print(f"Warning: 관절 수 불일치 - 로봇 모델: {robot.n}, 설정된 관절: {len(actual_joint_names)}")
        # 로봇 모델에 맞게 조정
        if robot.n < len(actual_joint_names):
            actual_joint_names = actual_joint_names[:robot.n]
        else:
            # 부족한 관절은 기본 패턴으로 추가
            for i in range(len(actual_joint_names), robot.n):
                actual_joint_names.append(f'joint_{i+1}')
    
    print(f"최종 사용할 관절 이름들: {actual_joint_names}")
    
    return robot, actual_joint_names, temp_urdf


def main():
    rclpy.init()
    
    try:
        # 로봇 모델 로드
        robot, joint_names, temp_urdf = load_robot_from_param()
        print(f"🤖 로봇 로드 완료: {robot.name}, 관절 수: {robot.n}")
        print(f"🔧 관절 이름들: {joint_names}")
        print("🚀 개선된 IK 알고리즘이 적용되었습니다!")
        
        # 컨트롤러 시작
        controller = SimpleCartesianController(robot, joint_names)
        
        try:
            rclpy.spin(controller)
        except KeyboardInterrupt:
            print("🛑 종료 중...")
        finally:
            controller.destroy_node()
            import os
            os.unlink(temp_urdf)  # 임시 파일 정리
            
    except Exception as e:
        print(f"❌ 오류: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()