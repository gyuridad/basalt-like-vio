# Basalt-like VIO

Basalt-like VIO는 Basalt VIO 원본 구조를 참고하여 ROS2 / Isaac Sim 환경에서 Visual-Inertial Odometry 기반 SLAM을 직접 구현하는 프로젝트입니다.

현재 단계에서는 VIO의 핵심 구성 요소 중 하나인 **IMU preintegration** 모듈을 우선 구현하고, Isaac Sim에서 수집한 IMU / odom 로그를 이용해 적분 결과를 검증하고 있습니다.

## Project Goal

이 프로젝트의 목표는 Basalt-style VIO 구조를 참고하여 다음 구성 요소를 단계적으로 구현하는 것입니다.

- IMU preintegration
- VIO state propagation
- Visual residual
- IMU factor
- Optimization backend
- Frontend tracking
- Full visual-inertial SLAM pipeline

현재 구현은 전체 SLAM 완성 전 단계로, 카메라 프레임 사이의 motion prediction을 담당하는 IMU propagation 구조를 중심으로 개발되어 있습니다.

## Environment

테스트 환경은 다음과 같습니다.

- Simulator: Isaac Sim
- Robot: Nova Carter mobile robot
- Scene: `warehouse_with_forklifts`
- Middleware: ROS2 Humble
- Main topics:
  - `/chassis/imu`
  - `/chassis/odom`

## Current Implementation

### IMU Preintegration

IMU preintegration 모듈은 ROS2 / Isaac Sim 환경에서 수신한 IMU 데이터를 기반으로 카메라 프레임 사이의 상대 운동량을 누적 적분합니다.

현재 구현된 주요 값은 다음과 같습니다.

- `delta_rotation`: 시작 IMU frame 기준 상대 회전 변화량
- `delta_velocity`: 시작 IMU frame 기준 속도 변화량
- `delta_position`: 시작 IMU frame 기준 위치 변화량
- `delta_time`: preintegration 시간 구간
- `linearization_bias`: 적분 당시 사용한 gyro / accel bias 기준값
- `covariance`: preintegration 불확실성
- `jacobian`: bias 변화에 따른 delta 보정용 Jacobian

### Test Results

- 직진 후 정지 테스트 

  - Speed change: 0.605 -> 0.000 m/s
  
  - odom delta velocity norm: 0.605 m/s
  - IMU delta velocity norm : 0.491 m/s
  - error norm              : 0.114 m/s
  
  - 감속 방향은 odom과 IMU 적분 결과가 일치했습니다.
  - 다만 IMU 적분값이 감속 크기를 약간 작게 추정했습니다.

- 직진 테스트

  - Speed change: 0.000 -> 0.605 m/s
  
  - odom delta velocity norm: 0.605 m/s
  - IMU delta velocity norm : 0.587 m/s
  - error norm              : 0.018 m/s
  
  - 직진 가속 구간에서는 IMU 적분 결과가 odom 기준 속도 변화와 매우 잘 일치했습니다.

- 회전 테스트

  - odom yaw change: -90.403 deg
  - IMU yaw change : -90.714 deg
  - yaw error      : 0.31 deg
  
  - gyro 기반 회전 적분이 odom 기준 회전 변화와 잘 정합됨을 확인했습니다.

## Meaning of This Implementation

IMU preintegration은 VIO / SLAM에서 카메라 프레임 사이의 빠른 motion prediction을 담당하는 핵심 모듈입니다.

특히 Basalt 계열의 VIO 구조에서는 IMU 적분 결과가 다음 요소에 직접적으로 영향을 줍니다.

- frontend tracking 안정성
- backend optimization 초기값
- IMU factor residual 구성
- 고속 motion 대응
- visual-inertial state propagation

현재 구현은 전체 SLAM pipeline 중 IMU propagation과 검증 기반을 구성한 단계이며,
이후 visual residual과 optimization backend를 연결해 full Basalt-like VIO 구조로 확장할 예정입니다.

