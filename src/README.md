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
src/
├── camera_node/
│   ├── publisher/
│   │   └── camera_publisher.py
│   ├── subscriber/
│   │   ├── lane_detection_subscriber.py
│   │   ├── stop_line_subscriber.py
│   │   └── child_zone_subscriber.py
│   ├── image2motor/
│   │   └── lane_motor_bridge.py
│   ├── package.xml
│   └── setup.py
├── motor_controller/
├── joystick_control/
└── integration/


# 2025 SEA:ME Hackathon
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


