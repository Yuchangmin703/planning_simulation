깃허브에 코드를 올리신 것을 축하드립니다! 팀원들이 이 리포지토리를 클론(Clone)받았을 때, 단 몇 줄의 명령어만으로 사용자님과 동일한 시각화 화면을 띄울 수 있도록 README.md 파일을 전문적으로 구성해 드립니다.

아래 내용을 그대로 복사해서 깃허브의 README.md 파일에 붙여넣으세요.
🚀 F1TENTH Planning Simulation Kit

이 패키지는 실제 카메라나 라이다 센서 없이도 판단/계획(Planning) 알고리즘을 실시간으로 검증할 수 있는 가상 시뮬레이션 환경입니다. 맵 상에서 차량의 이동을 TF로 처리하며, 장애물 회피 및 경로 추종 성능을 시각적으로 확인할 수 있습니다.
🛠 설치 및 빌드 환경

    OS: Ubuntu 22.04 LTS

    ROS 2: Humble

    의존성 패키지: rclcpp, rclpy, geometry_msgs, nav_msgs, visualization_msgs, tf2_ros

1. 워크스페이스 준비 및 클론
Bash

mkdir -p ~/planning_ws/src
cd ~/planning_ws/src
git clone https://github.com/yuchanmin703/planning_simulation.git

2. 빌드
Bash

cd ~/planning_ws
colcon build --symlink-install
source install/setup.zsh

🚦 실행 방법

모든 노드를 정상적으로 확인하려면 총 3개의 터미널이 필요합니다.
터미널 1: 가상 인지 시뮬레이터 실행

트랙(Map), 장애물, 차량의 현재 위치(TF)를 발행합니다.
Bash

source ~/planning_ws/install/setup.zsh
ros2 run planning_integration perception_sim.py

터미널 2: 플래닝 노드 실행

장애물을 인지하여 회피 경로를 생성하고 주행 기록을 관리합니다.
Bash

source ~/planning_ws/install/setup.zsh
ros2 run planning_integration planning_node

터미널 3: RViz2 시각화

설정 파일(.rviz)을 로드하여 즉시 최적화된 화면을 봅니다.
Bash

rviz2 -d src/planning_simulation/planning_integration/planning.rviz

📊 시각화 가이드 (What to see)

RViz2 화면에서 다음 요소들을 통해 알고리즘을 검증하세요:

    ⚪ 흰색 두 줄 (World Track): 맵에 고정된 전체 주행 경로입니다.

    🟢 녹색 실선 (Smooth Path): 플래닝 알고리즘이 계산한 미래의 목표 궤적입니다. 장애물 감지 시 부드럽게 옆 선으로 경로를 변경합니다.

    🟡 노란색 점선 (History Path): 최근 1초 동안 차량이 실제로 지나온 궤적입니다. 녹색 선과 얼마나 일치하는지 확인하여 추종 성능을 판단합니다.

    🔴 붉은 점/화살표 (Obstacle): 맵 위에 고정된 장애물 위치입니다.

📡 주요 토픽 맵 (Data Flow)
구분	토픽명	타입	역할
Input	/perception/tracked_objects	PoseArray	장애물 위치 정보 수신
State	TF (map -> base_link)	TF	차량의 현재 전역 좌표 확인
Output	/planning/smooth_path	Path	제어기(Control)로 전달할 최종 목표 경로
Debug	/planning/history_path	Path	최근 1초 주행 기록 시각화
💡 개발자 참고 사항

    Path Smoothing: 차선 변경 시 불연속성을 방지하기 위해 시그모이드(Sigmoid) 보간법이 적용되어 있습니다.

    Coordinate System: 모든 시뮬레이션은 전역 좌표계인 map 프레임을 기준으로 동작하며, 차량 중심 좌표계는 base_link입니다.
