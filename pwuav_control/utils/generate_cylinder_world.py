import random
import os

# ================= 사용자 설정 =================
OUTPUT_FILE = "cylinders.sdf"
MAP_SIZE = 50.0        # 100m x 100m 영역
NUM_CYLINDERS = 100      # 생성할 실린더 개수
CYLINDER_HEIGHT = 5.0   # 높이 고정 5m
MIN_RADIUS = 0.2        # 최소 반지름
MAX_RADIUS = 1.0        # 최대 반지름
SAFE_ZONE = 2.0         # (0,0) 드론 스폰 지점 보호 반경 (m)
# ============================================

# 사용자가 제공한 기본 맵 템플릿 (PX4 호환 설정 포함)
BASE_TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="default">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>1 1</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>500 500</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
        <inertial>
          <pose>0 0 0 0 -0 0</pose>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
        <enable_wind>false</enable_wind>
      </link>
      <pose>0 0 0 0 -0 0</pose>
      <self_collide>false</self_collide>
    </model>
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation>
        <range>2000</range>
        <linear>0</linear>
        <constant>1</constant>
        <quadratic>0</quadratic>
      </attenuation>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg> 8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
"""

def generate_cylinders_sdf():
    models_sdf = ""
    
    for i in range(NUM_CYLINDERS):
        # 좌표 생성 및 Safe Zone 검사
        while True:
            x = random.uniform(-MAP_SIZE/2, MAP_SIZE/2)
            y = random.uniform(-MAP_SIZE/2, MAP_SIZE/2)
            # (0,0) 기준 Safe Zone 밖인지 확인
            if (x**2 + y**2) > SAFE_ZONE**2:
                break
        
        radius = random.uniform(MIN_RADIUS, MAX_RADIUS)
        # Gazebo origin은 도형의 중심이므로 z = 높이/2
        z_pos = CYLINDER_HEIGHT / 2.0

        # 실린더 모델 SDF (PX4 default 스타일과 매칭)
        models_sdf += f"""
    <model name='cylinder_{i}'>
      <static>true</static>
      <pose>{x:.2f} {y:.2f} {z_pos:.2f} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <cylinder>
              <radius>{radius:.2f}</radius>
              <length>{CYLINDER_HEIGHT}</length>
            </cylinder>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <cylinder>
              <radius>{radius:.2f}</radius>
              <length>{CYLINDER_HEIGHT}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""
    return models_sdf

if __name__ == "__main__":
    # 랜덤 실린더 생성
    cylinders_part = generate_cylinders_sdf()
    
    # 템플릿 닫는 태그 추가
    final_sdf = BASE_TEMPLATE + cylinders_part + "\n  </world>\n</sdf>"
    
    with open(OUTPUT_FILE, "w") as f:
        f.write(final_sdf)
    
    print(f" '{OUTPUT_FILE}' 생성 완료!(실린더 개수: {NUM_CYLINDERS})")