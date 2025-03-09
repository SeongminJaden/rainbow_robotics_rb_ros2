import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

# LaunchDescription 생성 함수
def generate_launch_description():
    # Launch 파일에서 사용할 인수 선언
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",  # RViz 설정 파일을 받을 인수 이름
            default_value="moveit.rviz",  # 기본값 설정
            description="RViz 구성 파일",  # 인수에 대한 설명
        )
    )

    # LaunchDescription 반환, OpaqueFunction을 사용하여 launch_setup 함수 실행
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context, *args, **kwargs):
    # 로봇 설정을 위한 매핑 값 설정
    mappings = {
        "robot_ip": "10.0.2.7",  # 로봇 IP 주소
        "cb_simulation": "true",  # 시뮬레이션을 사용할지 여부
        "use_fake_hardware": "false",  # 가상 하드웨어 사용 여부
        "fake_sensor_commands": "false",  # 가짜 센서 명령 사용 여부
    }

    # MoveIt 설정을 위한 MoveItConfigsBuilder 생성
    moveit_config = (
        MoveItConfigsBuilder("rb3_730es_u")  # 로봇 이름
        .robot_description(file_path="config/rb3_730es_u.urdf.xacro", mappings=mappings)  # URDF 파일과 매핑 설정
        .trajectory_execution(file_path="config/moveit_controllers.yaml")  # MoveIt 컨트롤러 설정 파일
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )  # 로봇 설명과 시맨틱 정보를 퍼블리시
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )  # 사용할 경로 계획 파이프라인 설정
        .to_moveit_configs()  # MoveIt 설정 객체로 변환
    )

    # move_group 노드 실행 (모션 계획 처리)
    run_move_group_node = Node(
        package="moveit_ros_move_group",  # MoveIt 패키지
        executable="move_group",  # 모션 계획을 담당하는 노드
        output="screen",  # 출력을 화면에 출력
        parameters=[moveit_config.to_dict()],  # MoveIt 설정을 파라미터로 전달
    )

    # RViz 설정 파일 경로 설정
    rviz_base = LaunchConfiguration("rviz_config")  # RViz 설정 파일 이름을 launch 인수로부터 받음
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("rb3_730es_u_moveit_config"), "config", rviz_base]  # RViz 설정 파일 경로
    )

    # RViz 노드 실행
    rviz_node = Node(
        package="rviz2",  # RViz2 패키지
        executable="rviz2",  # RViz2 실행 파일
        name="rviz2",  # 노드 이름
        output="log",  # 로그 출력
        arguments=["-d", rviz_config],  # 설정 파일을 인수로 전달
        parameters=[
            moveit_config.robot_description,  # 로봇 설명 파라미터
            moveit_config.robot_description_semantic,  # 로봇 시맨틱 설명
            moveit_config.robot_description_kinematics,  # 로봇 운동학 설명
            moveit_config.planning_pipelines,  # 경로 계획 파이프라인
            moveit_config.joint_limits,  # 관절 제한
        ],  # RViz에 필요한 추가 파라미터 전달
    )

    # 정적 변환을 방송하는 TF 노드 실행
    static_tf = Node(
        package="tf2_ros",  # TF2 패키지
        executable="static_transform_publisher",  # 정적 변환을 퍼블리시하는 노드
        name="static_transform_publisher",  # 노드 이름
        output="log",  # 로그 출력
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "link0"],  # 변환 값
    )

    # 로봇 상태 퍼블리셔 실행
    robot_state_publisher = Node(
        package="robot_state_publisher",  # 로봇 상태 퍼블리셔 패키지
        executable="robot_state_publisher",  # 로봇 상태 퍼블리셔 실행 파일
        name="robot_state_publisher",  # 노드 이름
        output="both",  # 로그와 화면에 출력
        parameters=[moveit_config.robot_description],  # 로봇 설명을 파라미터로 전달
    )

    # ros2_control을 사용하여 가상 하드웨어 노드 실행
    ros2_controllers_path = os.path.join(
        get_package_share_directory("rbpodo_bringup"),
        "config",
        "controllers.yaml",  # 컨트롤러 설정 파일 경로
    )
    ros2_control_node = Node(
        package="controller_manager",  # 컨트롤러 매니저 패키지
        executable="ros2_control_node",  # ros2_control 실행 파일
        parameters=[moveit_config.robot_description, ros2_controllers_path],  # 로봇 설명과 컨트롤러 설정을 파라미터로 전달
        output="both",  # 로그와 화면에 출력
    )

    # joint_state_broadcaster 스포너 노드 실행
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",  # 컨트롤러 매니저 패키지
        executable="spawner",  # 컨트롤러 스포너 실행 파일
        arguments=[
            "joint_state_broadcaster",  # 컨트롤러 이름
            "--controller-manager-timeout",
            "300",  # 타임아웃 설정
            "--controller-manager",
            "/controller_manager",  # 컨트롤러 매니저 경로
        ],
    )

    # arm_controller 스포너 노드 실행
    arm_controller_spawner = Node(
        package="controller_manager",  # 컨트롤러 매니저 패키지
        executable="spawner",  # 컨트롤러 스포너 실행 파일
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],  # 팔 관절 경로 추적 컨트롤러 스포너
    )

    # 실행할 노드 목록
    nodes_to_start = [
        rviz_node,
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ]

    return nodes_to_start  # 실행할 노드들을 반환
