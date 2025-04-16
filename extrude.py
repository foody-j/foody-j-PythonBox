import numpy as np
import trimesh
import math
from scipy.spatial.transform import Rotation as R

class RobotToolpathGenerator:
    def __init__(self, layer_height=0.2, nozzle_diameter=0.4, filament_diameter=1.75):
        self.layer_height = layer_height
        self.nozzle_diameter = nozzle_diameter
        self.filament_diameter = filament_diameter
        self.toolpath = []
        self.current_position = {'X': 0, 'Y': 0, 'Z': 0, 'A': 0, 'B': 0, 'C': 0}
        self.current_e = 0  # 현재 압출기 위치
        self.extrusion_multiplier = 1.0
        self.print_speed = 60  # mm/s
        self.travel_speed = 120  # mm/s
        self.retraction_distance = 2.0  # mm
        self.retraction_speed = 40  # mm/s
        
    def load_stl(self, stl_file_path):
        """STL 파일 로드"""
        print(f"Loading STL file: {stl_file_path}")
        self.mesh = trimesh.load(stl_file_path)
        print(f"Mesh loaded: {len(self.mesh.faces)} faces, {len(self.mesh.vertices)} vertices")
        
        # 메시의 바운딩 박스 분석
        self.bounds = self.mesh.bounds
        self.min_bound = self.bounds[0]
        self.max_bound = self.bounds[1]
        self.dimensions = self.max_bound - self.min_bound
        
        print(f"Model dimensions: {self.dimensions}")
        print(f"Model bounds: {self.bounds}")
        
        return self.mesh
        
    def slice_mesh(self):
        """메시를 수평 레이어로 슬라이싱"""
        # 이전 코드와 동일
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
    
    def calculate_extrusion_amount(self, start_point, end_point):
        """두 점 사이의 이동에 필요한 필라멘트 압출량 계산"""
        # 두 점 사이의 거리 계산
        distance = np.linalg.norm(np.array(end_point) - np.array(start_point))
        
        # 압출할 필라멘트의 부피 계산
        # V = π × (노즐 지름/2)² × 이동 거리 × layer_height / 노즐 지름
        volume = (math.pi * (self.nozzle_diameter/2)**2 * distance * 
                 self.layer_height / self.nozzle_diameter)
        
        # 필라멘트 직경에 따른 필라멘트 길이 계산
        # L = V / (π × (필라멘트 지름/2)²)
        filament_length = volume / (math.pi * (self.filament_diameter/2)**2)
        
        # 압출 배율 적용
        filament_length *= self.extrusion_multiplier
        
        return filament_length
    
    def generate_robot_commands(self):
        """최종 로봇 명령어 생성 (압출기 제어 포함)"""
        print("Generating robot commands with extrusion control...")
        
        robot_commands = []
        
        # 초기화 명령어
        init_cmd = {
            'command': 'INIT',
            'comment': '초기화 - 온도 설정 및 시스템 준비'
        }
        robot_commands.append(init_cmd)
        
        # 압출기 온도 설정
        temp_cmd = {
            'command': 'SET_TEMPERATURE',
            'nozzle': 200,  # 노즐 온도
            'bed': 60       # 베드 온도
        }
        robot_commands.append(temp_cmd)
        
        # 온도 대기
        wait_cmd = {
            'command': 'WAIT_TEMPERATURE'
        }
        robot_commands.append(wait_cmd)
        
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
            self.current_position = layer_start_cmd['position'].copy()
            
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
                self.current_position = start_cmd['position'].copy()
                
                # 압출 시작 전 프라이밍 (필라멘트 준비)
                prime_cmd = {
                    'command': 'PRIME_EXTRUDER',
                    'amount': 0.5,  # 0.5mm 필라멘트 프라이밍
                    'speed': self.retraction_speed
                }
                robot_commands.append(prime_cmd)
                self.current_e += 0.5  # 압출기 위치 업데이트
                
                # 경로 따라 이동하며 압출
                for i in range(1, len(path)):
                    point = path[i]
                    prev_point = path[i-1]
                    
                    # 압출량 계산
                    extrusion = self.calculate_extrusion_amount(prev_point, point)
                    
                    # 6축 방향 계산
                    next_point = path[i+1] if i+1 < len(path) else None
                    orientation = self.calculate_orientation(prev_point, point, next_point)
                    
                    move_cmd = {
                        'command': 'EXTRUDE_MOVE',
                        'position': {'X': point[0], 
                                     'Y': point[1], 
                                     'Z': layer_z,
                                     'A': orientation['A'],
                                     'B': orientation['B'],
                                     'C': orientation['C']},
                        'extrusion': {
                            'amount': extrusion,
                            'absolute_e': self.current_e + extrusion
                        },
                        'speed': self.print_speed
                    }
                    robot_commands.append(move_cmd)
                    self.current_position = move_cmd['position'].copy()
                    self.current_e += extrusion  # 압출기 위치 업데이트
                
                # 압출 종료 (리트랙션)
                retract_cmd = {
                    'command': 'RETRACT',
                    'amount': self.retraction_distance,
                    'speed': self.retraction_speed
                }
                robot_commands.append(retract_cmd)
                self.current_e -= self.retraction_distance  # 압출기 위치 업데이트
            
            # 인필 처리 (생략)...
        
        # 프린팅 종료 - 청소 및 안전 위치로 이동
        end_cmds = [
            {
                'command': 'RETRACT',
                'amount': 5,  # 5mm 추가 리트랙션
                'speed': self.retraction_speed
            },
            {
                'command': 'MOVE',
                'position': {'X': 0, 'Y': 0, 'Z': 50, 'A': 0, 'B': 0, 'C': 0},
                'speed': self.travel_speed
            },
            {
                'command': 'SET_TEMPERATURE',
                'nozzle': 0,  # 노즐 온도 끄기
                'bed': 0       # 베드 온도 끄기
            },
            {
                'command': 'DISABLE_MOTORS'
            }
        ]
        robot_commands.extend(end_cmds)
        
        print(f"Generated {len(robot_commands)} robot commands")
        return robot_commands
    
    def calculate_orientation(self, current_point, next_point, next_next_point=None):
        """툴 방향 계산 (6축 로봇용)"""
        # 기본 방향: 노즐이 아래를 향하도록 (Z축 방향)
        default_orientation = {'A': 0, 'B': 0, 'C': 0}
        
        # 현재 점에서 다음 점으로 이동 방향 계산
        if next_point is not None:
            direction = np.array(next_point) - np.array(current_point)
            if np.linalg.norm(direction) > 0:
                direction = direction / np.linalg.norm(direction)
                
                # 방향 벡터를 회전 각도로 변환
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
    
    def convert_to_robot_script(self, commands, robot_type='generic'):
        """로봇 명령어를 특정 로봇 스크립트로 변환"""
        script_lines = []
        
        if robot_type == 'generic':
            # 간단한 텍스트 기반 스크립트
            for i, cmd in enumerate(commands):
                line = f"{i+1}: {cmd['command']}"
                
                if 'position' in cmd:
                    pos = cmd['position']
                    line += f" X:{pos['X']:.3f} Y:{pos['Y']:.3f} Z:{pos['Z']:.3f}"
                    line += f" A:{pos['A']:.3f} B:{pos['B']:.3f} C:{pos['C']:.3f}"
                
                if 'extrusion' in cmd:
                    ext = cmd['extrusion']
                    line += f" E:{ext['amount']:.4f} E_ABS:{ext['absolute_e']:.4f}"
                
                if 'speed' in cmd:
                    line += f" F:{cmd['speed']}"
                
                script_lines.append(line)
        
        elif robot_type == 'kuka':
            # KUKA KRL 스크립트 형식
            script_lines.append("DEF print_job()")
            script_lines.append("  ; 3D printing job generated by RobotToolpathGenerator")
            
            for cmd in commands:
                if cmd['command'] == 'MOVE' or cmd['command'] == 'EXTRUDE_MOVE':
                    pos = cmd['position']
                    
                    # KUKA PTP (Point-to-Point) 또는 LIN (Linear) 움직임
                    move_type = "PTP" if cmd['command'] == 'MOVE' else "LIN"
                    speed = cmd.get('speed', 100)
                    
                    # KUKA 좌표 형식으로 변환
                    kuka_pos = f"{{X {pos['X']:.3f},Y {pos['Y']:.3f},Z {pos['Z']:.3f},"
                    kuka_pos += f"A {pos['A']:.3f},B {pos['B']:.3f},C {pos['C']:.3f}}}"
                    
                    line = f"  {move_type} {kuka_pos} VEL={speed/10:.1f}%"
                    
                    if cmd['command'] == 'EXTRUDE_MOVE' and 'extrusion' in cmd:
                        # KUKA 디지털 출력 신호로 압출기 제어
                        line += f" ; Extrude {cmd['extrusion']['amount']:.4f}mm"
                        script_lines.append(line)
                        script_lines.append(f"  $OUT[1]=TRUE ; Enable extruder")
                    else:
                        script_lines.append(line)
                        if cmd['command'] == 'MOVE':
                            script_lines.append(f"  $OUT[1]=FALSE ; Disable extruder")
                
                elif cmd['command'] == 'RETRACT':
                    script_lines.append(f"  $OUT[1]=FALSE ; Disable extruder")
                    script_lines.append(f"  ; Retract {cmd['amount']}mm")
                
                elif cmd['command'] == 'SET_TEMPERATURE':
                    script_lines.append(f"  ; Set nozzle temp to {cmd['nozzle']}°C, bed to {cmd['bed']}°C")
                    # 온도 제어 명령은 로봇 특정 방식으로 구현
                
                elif cmd['command'] == 'INIT':
                    script_lines.append("  ; Initialize printer")
                
                elif cmd['command'] == 'HOME':
                    script_lines.append("  ; Move to home position")
                    pos = cmd['position']
                    kuka_pos = f"{{X {pos['X']:.3f},Y {pos['Y']:.3f},Z {pos['Z']:.3f},"
                    kuka_pos += f"A {pos['A']:.3f},B {pos['B']:.3f},C {pos['C']:.3f}}}"
                    script_lines.append(f"  PTP {kuka_pos} VEL=50%")
            
            script_lines.append("END ; End of program")
        
        return script_lines
    
    def export_robot_script(self, commands, output_file, robot_type='generic'):
        """로봇 스크립트를 파일로 저장"""
        print(f"Exporting {robot_type} script to: {output_file}")
        
        script_lines = self.convert_to_robot_script(commands, robot_type)
        
        with open(output_file, 'w') as f:
            for line in script_lines:
                f.write(line + "\n")
        
        print(f"Successfully exported script with {len(script_lines)} lines")
        return True

    def process_stl(self, stl_file, output_file, robot_type='generic'):
        """STL 파일 전체 처리 파이프라인"""
        print(f"Processing STL file: {stl_file}")
        
        # 1. STL 로드
        self.load_stl(stl_file)
        
        # 2. 슬라이싱
        self.slice_mesh()
        
        # 3. 로봇 명령어 생성
        commands = self.generate_robot_commands()
        
        # 4. 로봇 스크립트로 변환 및 내보내기
        self.export_robot_script(commands, output_file, robot_type)
        
        print("Processing complete!")
        return True

# 사용 예시
if __name__ == "__main__":
    generator = RobotToolpathGenerator(
        layer_height=0.2, 
        nozzle_diameter=0.4,
        filament_diameter=1.75
    )
    
    # STL 처리 및 로봇 스크립트 생성
    generator.process_stl("cube.stl", "robot_script.txt", robot_type='generic')
    
    # KUKA 로봇용 스크립트 생성 (옵션)
    # generator.process_stl("cube.stl", "kuka_script.src", robot_type='kuka')