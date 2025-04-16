# foody-j-PythonBox


# CartesianController: ROS2 기반 카테시안 제어 라이브러리

`hi.py`은 ROS2 환경에서 6축 로봇팔의 **카테시안 경로 제어**를 직접 구현한 Python 라이브러리입니다. MoveIt 등 고수준 프레임워크 없이, **직접 카테시안 경로 생성, 역기구학 변환, 관절 궤적 생성 및 ROS2 퍼블리시**까지 전체 파이프라인을 제공합니다.

---

## 주요 특징

- **카테시안 경로 기반 제어**  
  - 엔드이펙터의 목표 위치/자세(SE3 포즈)로부터 관절 공간(Joint Space) 궤적을 자동 생성합니다.
  - 원형, 직선 등 다양한 경로 타입 지원

- **역기구학 기반 경로 생성**  
  - 각 경로 포인트마다 여러 초기값을 활용한 반복 역기구학(ik_LM)으로 최적의 관절각을 탐색
  - 관절 변화량 최소화, 작업공간 내 유효성 검증 등 내장

- **ROS2 통합**  
  - `/joint_states`, `/cartesian_path` 등 ROS2 메시지 구독/발행
  - `JointTrajectory` 메시지로 ROS2 컨트롤러와 직접 연동

- **실시간 경로 추가 및 제어**  
  - 실행 중에도 새로운 카테시안 경로 포인트를 동적으로 추가 가능
  - 경로 완료 자동 감지 및 상태 관리

- **작업공간 계산 및 시각화**  
  - 관절 제한 기반 작업공간 샘플링 및 3D 시각화 함수 제공
  - 타겟 포인트의 도달 가능성 검증 기능 포함

---

## 주요 클래스 및 함수

| 이름                       | 설명                                                         |
|---------------------------|-------------------------------------------------------------|
| `CartesianController`     | 카테시안 경로를 받아 관절 궤적을 생성·발행하는 ROS2 노드 클래스      |
| `generate_joint_trajectory` | 카테시안 포즈 리스트에서 관절 궤적 및 성공 여부 플래그 생성         |
| `publish_joint_trajectory`  | 관절 궤적(JointTrajectory) 메시지를 ROS2로 발행                  |
| `follow_cartesian_path`     | 임의의 카테시안 경로(포즈 리스트) 추종                           |
| `follow_circular_path`      | 원형 경로 생성 및 추종                                         |
| `follow_linear_path`        | 직선 경로 생성 및 추종 (관절 공간 보간)                         |
| `calculate_workspace`       | 관절 제한 기반 작업공간 샘플링                                 |
| `visualize_workspace_with_target` | 작업공간과 타겟 포인트 3D 시각화                        |
| `check_point_in_workspace`  | 타겟 포인트의 도달 가능성(작업공간 포함 여부) 검증               |

---

## 사용 예시

```python
# 1. XACRO → URDF 변환 및 로봇 모델 로드
robot = rtb.Robot.URDF("your_robot.urdf")
joint_names = ['joint_1', 'joint_2', ..., 'joint_6']

# 2. 컨트롤러 생성
controller = CartesianController(robot, joint_names)

# 3. 직선 경로 추종
start = [0.0, 0.0, 0.4]
end   = [0.05, 0.05, 0.02]
controller.follow_linear_path(start, end, duration=5.0, num_points=10)

# 4. 작업공간 계산 및 시각화
workspace_points = calculate_workspace(robot, samples=5)
visualize_workspace_with_target(workspace_points, [0, 0, 0.4])
```

---

## 활용 및 확장

- MoveIt 등 고수준 프레임워크를 사용하지 않고, **로봇 제어의 원리와 세부 동작을 직접 구현·실험**하고 싶은 경우에 적합합니다.
- 연구/실험 목적의 커스텀 경로 생성, 실시간 제어, 특수한 역기구학 알고리즘 적용 등에도 쉽게 확장 가능합니다.
- ROS2 기반 6축 로봇팔의 카테시안 제어 파이프라인을 처음부터 끝까지 직접 경험하고 싶을 때 참고용으로 활용할 수 있습니다.

---

## 참고

- Python 3, ROS2, roboticstoolbox, spatialmath, matplotlib 등 필요
- 자세한 사용법 및 예시는 코드 내 주석과 `main()` 함수 참고

---

**문의/기여**  
코드 개선, 버그 리포트, 제안 등은 언제든 환영합니다!