import numpy as np
import trimesh
import math
from scipy.spatial.transform import Rotation as R

class RobotToolpathGenerator:
    def __init__(self, layer_height=0.2, nozzle_diameter=0.4):
        self.layer_height = layer_height
        self.nozzle_diameter = nozzle_diameter
        self.toolpath = []
        self.current_position = {'X': 0, 'Y': 0, 'Z': 0, 'A': 0, 'B': 0, 'C': 0}
        self.extrusion_multiplier = 1.0
        self.print_speed = 60  # mm/s
        self.travel_speed = 120  # mm/s
        
    def load_stl(self, stl_file_path):
        """STL 파일 로드"""
        print(f"Loading STL file: {stl_file_path}")
        self.mesh = trimesh.load(stl_file_path) # trimesh 객체로 변환
        print(f"Mesh loaded: {len(self.mesh.faces)} faces, {len(self.mesh.vertices)} vertices") # 메시 정보 출력 (면, 정점)
        
        # 메시의 바운딩 박스 분석
        self.bounds = self.mesh.bounds # 메시의 바운딩 박스 (최소, 최대)
        self.min_bound = self.bounds[0] # 최소 바운딩 박스
        self.max_bound = self.bounds[1] # 최대 바운딩 박스
        self.dimensions = self.max_bound - self.min_bound # 모델의 가로 세로 높이 계산
        
        print(f"Model dimensions: {self.dimensions}")
        print(f"Model bounds: {self.bounds}")
        
        return self.mesh # 로드된 메시 객체 반환
        
    def slice_mesh(self):
        """메시를 수평 레이어로 슬라이싱"""
        print("Slicing mesh into layers...")
        
        z_min = self.min_bound[2]
        z_max = self.max_bound[2]
        
        self.layers = []
        current_z = z_min + self.layer_height / 2  # 첫 레이어 시작
        
        while current_z <= z_max:
            # Z 평면으로 메시 슬라이싱하여 윤곽선 가져오기
            slice_plane = trimesh.intersections.mesh_plane(
                self.mesh, 
                plane_normal=[0, 0, 1], 
                plane_origin=[0, 0, current_z]
            )
            
            if slice_plane is not None and len(slice_plane) > 0:
                # 모든 윤곽선을 폐곡선으로 구성
                paths = []
                for path in slice_plane:
                    if len(path) > 2:  # 최소 3개 점 필요
                        # 시작점과 끝점이 같은지 확인 (폐곡선)
                        if not np.array_equal(path[0], path[-1]):
                            path = np.vstack([path, path[0]])  # 폐곡선으로 만들기
                        paths.append(path)
                
                if paths:
                    self.layers.append({'z': current_z, 'paths': paths})
                    print(f"Layer at z={current_z:.2f}mm: {len(paths)} paths")
            
            current_z += self.layer_height
        
        print(f"Total layers: {len(self.layers)}")
        return self.layers
    
    def generate_infill(self, layer, density=0.2, pattern='grid'):
        """레이어에 인필 생성"""
        # 단순한 그리드 패턴 인필 생성 
        # (실제 구현에서는 더 복잡한 알고리즘 필요)
        
        infill_spacing = self.nozzle_diameter / density
        x_min, y_min = self.min_bound[0], self.min_bound[1]
        x_max, y_max = self.max_bound[0], self.max_bound[1]
        
        infill_lines = []
        
        # 수직 라인 (Y 방향)
        x = x_min
        while x <= x_max:
            line = np.array([[x, y_min, layer['z']], [x, y_max, layer['z']]])
            infill_lines.append(line)
            x += infill_spacing
            
        # 수평 라인 (X 방향)
        y = y_min
        while y <= y_max:
            line = np.array([[x_min, y, layer['z']], [x_max, y, layer['z']]])
            infill_lines.append(line)
            y += infill_spacing
        
        return infill_lines
    
    def calculate_orientation(self, point, next_point):
        """툴 방향 계산 (6축 로봇용)"""
        # 기본 방향: 노즐이 아래를 향하도록 (Z축 방향)
        default_orientation = {'A': 0, 'B': 0, 'C': 0}
        
        # 더 복잡한 방향 계산 (선택 사항)
        if next_point is not None:
            # 현재 점에서 다음 점으로 이동 방향 계산
            direction = next_point[:3] - point[:3]
            if np.linalg.norm(direction) > 0:
                direction = direction / np.linalg.norm(direction)
                
                # 방향 벡터를 회전 각도로 변환
                # 이 부분은 로봇의 좌표계와 회전 규약에 따라 조정 필요
                try:
                    # Z축을 방향 벡터에 정렬하기 위한 회전 계산
                    z_axis = np.array([0, 0, 1])
                    rotation_axis = np.cross(z_axis, direction)
                    
                    if np.linalg.norm(rotation_axis) > 1e-6:  # 평행하지 않은 경우
                        rotation_angle = np.arccos(np.dot(z_axis, direction))
                        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
                        
                        rot = R.from_rotvec(rotation_axis * rotation_angle)
                        euler = rot.as_euler('xyz', degrees=True)
                        
                        default_orientation['A'] = euler[0]
                        default_orientation['B'] = euler[1]
                        default_orientation['C'] = euler[2]
                except:
                    pass  # 오류 발생 시 기본 방향 사용
        
        return default_orientation
    
    def generate_robot_commands(self):
        """최종 로봇 명령어 생성"""
        print("Generating robot commands...")
        
        robot_commands = []
        
        # 홈 위치로 이동
        home_cmd = {
            'command': 'HOME',
            'position': {'X': 0, 'Y': 0, 'Z': 50, 'A': 0, 'B': 0, 'C': 0},
            'speed': self.travel_speed
        }
        robot_commands.append(home_cmd)
        
        # 각 레이어 처리
        for layer_idx, layer in enumerate(self.layers):
            print(f"Processing layer {layer_idx+1}/{len(self.layers)}...")
            
            # 레이어 시작 높이로 이동
            layer_z = layer['z']
            layer_start_cmd = {
                'command': 'MOVE',
                'position': {'X': self.current_position['X'], 
                             'Y': self.current_position['Y'], 
                             'Z': layer_z + 5,  # 안전 높이
                             'A': 0, 'B': 0, 'C': 0},
                'speed': self.travel_speed
            }
            robot_commands.append(layer_start_cmd)
            self.current_position = layer_start_cmd['position']
            
            # 외곽선 경로 처리
            for path_idx, path in enumerate(layer['paths']):
                print(f"  Processing path {path_idx+1}/{len(layer['paths'])}...")
                
                # 경로 시작점으로 이동
                start_point = path[0]
                start_cmd = {
                    'command': 'MOVE',
                    'position': {'X': start_point[0], 
                                 'Y': start_point[1], 
                                 'Z': layer_z,
                                 'A': 0, 'B': 0, 'C': 0},
                    'speed': self.travel_speed
                }
                robot_commands.append(start_cmd)
                self.current_position = start_cmd['position']
                
                # 압출 시작
                extrude_start_cmd = {
                    'command': 'EXTRUDE_START'
                }
                robot_commands.append(extrude_start_cmd)
                
                # 경로 따라 이동
                for i in range(1, len(path)):
                    point = path[i]
                    next_point = path[i+1] if i+1 < len(path) else None
                    
                    # 6축 방향 계산
                    orientation = self.calculate_orientation(path[i-1], point)
                    
                    move_cmd = {
                        'command': 'EXTRUDE_MOVE',
                        'position': {'X': point[0], 
                                     'Y': point[1], 
                                     'Z': layer_z,
                                     'A': orientation['A'],
                                     'B': orientation['B'],
                                     'C': orientation['C']},
                        'speed': self.print_speed
                    }
                    robot_commands.append(move_cmd)
                    self.current_position = move_cmd['position']
                
                # 압출 종료
                extrude_end_cmd = {
                    'command': 'EXTRUDE_END'
                }
                robot_commands.append(extrude_end_cmd)
            
            # 인필 생성 및 처리 (옵션)
            infill_lines = self.generate_infill(layer)
            if infill_lines:
                print(f"  Processing {len(infill_lines)} infill lines...")
                
                for line in infill_lines:
                    # 선 시작점으로 이동
                    start_point = line[0]
                    start_cmd = {
                        'command': 'MOVE',
                        'position': {'X': start_point[0], 
                                     'Y': start_point[1], 
                                     'Z': start_point[2],
                                     'A': 0, 'B': 0, 'C': 0},
                        'speed': self.travel_speed
                    }
                    robot_commands.append(start_cmd)
                    self.current_position = start_cmd['position']
                    
                    # 압출 시작
                    robot_commands.append({'command': 'EXTRUDE_START'})
                    
                    # 선 끝점으로 이동
                    end_point = line[1]
                    end_cmd = {
                        'command': 'EXTRUDE_MOVE',
                        'position': {'X': end_point[0], 
                                     'Y': end_point[1], 
                                     'Z': end_point[2],
                                     'A': 0, 'B': 0, 'C': 0},
                        'speed': self.print_speed
                    }
                    robot_commands.append(end_cmd)
                    self.current_position = end_cmd['position']
                    
                    # 압출 종료
                    robot_commands.append({'command': 'EXTRUDE_END'})
        
        # 프린팅 종료 - 안전 위치로 이동
        end_cmd = {
            'command': 'MOVE',
            'position': {'X': 0, 'Y': 0, 'Z': 50, 'A': 0, 'B': 0, 'C': 0},
            'speed': self.travel_speed
        }
        robot_commands.append(end_cmd)
        
        print(f"Generated {len(robot_commands)} robot commands")
        return robot_commands
    
    def export_robot_commands(self, output_file):
        """로봇 명령어를 파일로 저장"""
        print(f"Exporting robot commands to: {output_file}")
        
        commands = self.generate_robot_commands()
        
        with open(output_file, 'w') as f:
            for i, cmd in enumerate(commands):
                f.write(f"{i+1}: {cmd}\n")
        
        print(f"Successfully exported {len(commands)} commands")
        return True

    def process_stl(self, stl_file, output_file):
        """STL 파일 전체 처리 파이프라인"""
        print(f"Processing STL file: {stl_file}")
        
        # 1. STL 로드
        self.load_stl(stl_file)
        
        # 2. 슬라이싱
        self.slice_mesh()
        
        # 3. 로봇 명령어 생성 및 내보내기
        self.export_robot_commands(output_file)
        
        print("Processing complete!")
        return True

# 사용 예시
if __name__ == "__main__":
    generator = RobotToolpathGenerator(layer_height=0.2, nozzle_diameter=0.4)
    generator.process_stl("cube.stl", "robot_commands.txt")