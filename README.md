# rbpodo_ros2

> :warning: **IMPORTANT WARNING**: This software is under active development. DO NOT USE in production to avoid potential instability.


## Installation

### Prerequisites

- Install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- Install [rbpodo](https://github.com/RainbowRobotics/rbpodo)
  ```bash
  sudo apt install -y build-essential cmake git
  git clone https://github.com/RainbowRobotics/rbpodo.git
  mkdir -p rbpodo/build
  cd rbpodo/build
  cmake -DCMAKE_BUILD_TYPE=Release ..
  make
  sudo make install
  ```
- Install ROS 2 package dependencies
  ```bash
  sudo apt install -y \
    ros-humble-ament-cmake \
    ros-humble-joint-state-publisher \
    ros-humble-moveit \
    ros-humble-pluginlib \
    ros-humble-robot-state-publisher \
    ros-humble-ros2-controllers \
    ros-humble-ros2-control \
    ros-humble-rviz2 \
    ros-humble-urdf-launch \
    ros-humble-xacro 
  ```
- Set up environment
  ```bash
  source /opt/ros/humble/setup.bash
  ```

### Build From Source

1. Create a ROS 2 workspace
   ```bash
   mkdir -p ~/rbpodo_ros2_ws/src
   ```
2. Clone repo and build ``rbpodo_ros2`` packages:
   ```bash
   cd ~/rbpodo_ros2_ws
   git clone https://github.com/RainbowRobotics/rbpodo_ros2.git src/rbpodo_ros2
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.sh
   ```

## How to Use

```bash
source ~/rbpodo_ros2_ws/install/setup.bash
ros2 launch rbpodo_bringup rbpodo.launch.py model_id:=rb3_730es_u use_rviz:=true//Joint publisher, discription만 실행함.
ros2 launch rb3_730es_u_moveit_config moveit.launch.py //Moveit까지 실행하여 모션플랜이 가능한 소스
```

## moveit.launch.py 파일 설명
이 프로젝트는 ROS2에서 MoveIt을 사용하여 로봇을 제어하고 RViz로 시각화하는 데 필요한 런치 파일을 설정하는 코드입니다. </br>주요 기능은 로봇 상태를 퍼블리시하고, 경로 계획을 수행하며, RViz에서 로봇을 시각화하는 것입니다. </br>이 파일은 ROS2 환경에서 로봇을 제어하고 테스트할 때 사용할 수 있습니다.

## 주요 구성 요소
### 1. generate_launch_description() 함수
이 함수는 런치 파일의 주요 엔트리 포인트입니다. </br>로봇의 설정 파일과 RViz 설정 파일을 받는 인수를 선언하고, 전체 런치 파일을 정의하는 LaunchDescription 객체를 반환합니다.</br>

#### DeclareLaunchArgument: rviz_config라는 이름의 인수를 선언하고, 기본값을 moveit.rviz로 설정합니다.</br> 이 인수는 RViz에서 사용할 설정 파일을 지정하는 데 사용됩니다.
#### OpaqueFunction: launch_setup 함수 호출을 통해 로봇과 관련된 노드를 설정하고 실행합니다.
### 2. launch_setup() 함수
이 함수는 실제로 런치 파일에서 실행될 노드를 설정합니다. 각 노드는 ROS2 시스템에서 특정 기능을 수행하는 역할을 합니다.

#### 설정되는 주요 노드:
#### MoveIt 관련 노드:
#### run_move_group_node: MoveIt의 move_group 노드를 실행하여 모션 계획을 수행합니다.
#### rviz_node: RViz를 실행하여 로봇과 계획된 경로를 시각화합니다.
#### robot_state_publisher: 로봇의 상태를 퍼블리시하는 노드입니다. URDF 파일을 사용하여 로봇의 관절 상태를 전달합니다.
#### static_tf: 정적 변환을 퍼블리시하는 노드로, 로봇 프레임과 월드 프레임 간의 변환을 설정합니다.</br>

#### 컨트롤러 및 하드웨어 관련 노드:
#### ros2_control_node: ROS2의 ros2_control을 사용하여 하드웨어와 상호작용합니다. 이 노드는 실제 로봇과의 인터페이스를 담당합니다.
#### joint_state_broadcaster_spawner: joint_state_broadcaster를 실행하여 로봇의 관절 상태를 퍼블리시합니다.
#### arm_controller_spawner: joint_trajectory_controller를 실행하여 로봇 팔의 모션을 제어합니다.
### 3. MoveIt 설정
MoveIt을 위한 설정은 MoveItConfigsBuilder를 통해 생성됩니다. </br>이 설정은 로봇 설명 파일(URDF), 경로 계획 파이프라인, 트레젝토리 실행 파일 등을 포함하며, 이를 통해 MoveIt의 설정을 커스터마이즈합니다. </br>이 설정은 moveit_config 변수로 반환되어 각 노드에 전달됩니다.

### 4. RViz 설정
rviz_config 인수는 RViz의 설정 파일을 지정합니다.</br> 기본값은 moveit.rviz이며, 이는 RViz에서 로봇을 시각화하는 데 사용됩니다.</br> RViz를 실행할 때 이 파일을 로드하여 로봇 모델, 경로 계획, 그리고 기타 시각화 요소들을 화면에 표시합니다.

### 5. 경로 설정
파일 경로는 FindPackageShare와 PathJoinSubstitution을 사용하여 설정됩니다.</br> 이를 통해 설정 파일이나 URDF 파일 등 로봇에 필요한 파일들을 올바른 경로에서 자동으로 찾을 수 있습니다.

### 6. 노드 실행
모든 노드는 Node 객체로 정의되며, 각 노드는 ROS2 시스템에서 개별적인 역할을 수행합니다.</br> 이 노드들은 LaunchDescription에 추가되어 순차적으로 실행됩니다.

### 사용 방법
이 런치 파일은 ROS2 환경에서 roslaunch 명령을 통해 실행할 수 있습니다. RViz 설정을 변경하려면, rviz_config 인수를 사용하여 원하는 설정 파일을 지정할 수 있습니다.

## 컴퓨터 ip 변경방법(ubuntu)

1. 유선 연결됨 → 유선 네트워크 설정
![image](https://github.com/user-attachments/assets/2f7aed2c-1842-4236-8d0f-df8928d3d8d2)

2. '유선'의 톱니바퀴 클릭 → IPv4
![image](https://github.com/user-attachments/assets/1bb18e70-f40b-4877-8636-7b1701a6b034)

3. 자동(DHCP)에 체크 되어 있는 걸 다음 사진처럼 변경 → 적용 누르고 재 시동
![image](https://github.com/user-attachments/assets/acd72eaa-8ef7-43c9-84f1-3bf669d6ad3b)

주소 - 10.0.2.xx (xx는 7과 1을 제외한 숫자를 입력, 7은 로봇ip주소이며 1은 게이트웨이임)</br>
서브넷마스크 -255.255.255.0</br>
게이트웨이 - 10.0.2.1</br>
네임서버 - 8.8.8.8</br>
4-1. 다시 톱니바퀴 들어가서 확인해 보면 내가 설정한 IP주소로 바뀌어 있는 것 확인 가능
4-2. 다음 명령어 입력 후 inet 주소가 설정한 고정IP로 잘 바뀌어 있는지 확인
