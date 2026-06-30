## 2025 SEA:ME Hackathon
![포스터(기한연장)](https://github.com/user-attachments/assets/6112b1fe-1118-40ec-882d-eec1de128dbc)

> During this hackathon, your mission is to develop a autonomous driving system based on PiRacer Pro. We'll provide you a Raspberry Pi 4 board.


![2025 해커톤 시스템 아키텍쳐](https://github.com/user-attachments/assets/4ab8e098-ef6f-4bd5-8b8e-cdb42a2befdd)

***
# Tutorial for Hackaton

## Index
- [Hardware Setup](docs/hardware_setup.md)
- [Raspberry Pi OS Setup (Ubuntu 22.04)](docs/raspi-os.md)
- [ROS2-humble Installation](docs/ros2-humble_installation.md)
- [Sensor Setup](docs/sensors.md)

# Scale Car Camera Node

ROS2 기반 자율주행 스케일카 카메라 처리 패키지입니다.  
카메라 영상을 받아 차선, 정지선, 어린이보호구역을 감지하고, 감지 결과를 기반으로 조향값과 속도값을 생성합니다.

## Features

- 카메라 이미지 퍼블리시
- 차선 인식 및 차선 중심/각도 계산
- 정지선 감지
- 어린이보호구역 감지
- 차선/정지선/어린이보호구역 정보를 기반으로 throttle, steer 명령 생성

## Package Structure

```bash
camera_node/
├── publisher/
│   └── camera_publisher.py
│
│   • USB 카메라 또는 웹캠으로부터 실시간 영상을 획득
│   • OpenCV 프레임을 ROS2 Image 메시지로 변환(cv_bridge)
│   • /camera/image_raw 토픽으로 영상 퍼블리시
│   • 다른 인식 노드들이 사용할 입력 영상을 제공
│
├── subscriber/
│   ├── lane_detection_subscriber.py
│   │
│   │   • /camera/image_raw 영상을 구독
│   │   • ROI 설정 및 전처리(Grayscale, Blur, Canny)
│   │   • Hough Transform을 이용한 차선 검출
│   │   • 차선 중심 위치와 진행 방향(조향각) 계산
│   │   • 계산된 차선 정보를 /lane_info 토픽으로 퍼블리시
│   │
│   ├── stop_line_subscriber.py
│   │
│   │   • /camera/image_raw 영상을 구독
│   │   • 영상에서 흰색 수평선을 탐지하여 정지선 여부 판단
│   │   • 정지선이 감지되면 Bool 형태의 결과를 생성
│   │   • /stop_line_detected 토픽으로 감지 결과 퍼블리시
│   │
│   └── child_zone_subscriber.py
│
│       • /camera/image_raw 영상을 구독
│       • HSV 색공간에서 빨간색 영역을 검출
│       • 어린이보호구역(적색 노면 표시) 여부 판단
│       • 감지 결과를 Bool 형태로 생성
│       • /child_zone_detected 토픽으로 퍼블리시


------------------------------------------------------




