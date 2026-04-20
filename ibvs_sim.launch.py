import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ur5_ibvs_pkg  = get_package_share_directory('ur5_ibvs')
    ur_desc_pkg   = get_package_share_directory('ur_description')

    # Full absolute paths — no ambiguity
    xacro_file       = os.path.join(ur5_ibvs_pkg, 'urdf', 'ur5_with_camera.urdf.xacro')
    controllers_yaml = os.path.join(ur5_ibvs_pkg, 'config', 'controllers.yaml')

    # Expand xacro to URDF string
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' sim_gazebo:=true',
        ' ur_type:=ur5',
        ' name:=ur',
        ' tf_prefix:=',
        ' safety_limits:=false',
        ' use_fake_hardware:=false',
    ])

    robot_description = {'robot_description': robot_description_content}

    # 1. Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 2. Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',
             '-s', 'libgazebo_ros_init.so'],
        output='screen',
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}
    )

    # 3. Spawn UR5 into Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ur',
            '-topic', 'robot_description',
        ],
        output='screen'
    )

    # 4. Controller manager — load our controllers yaml directly
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controllers_yaml
        ],
        output='screen'
    )

    # 5. Spawn controllers (after delay to let CM start)
    spawn_jsb = TimerAction(period=5.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'run', 'controller_manager', 'spawner',
                 'joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        )
    ])

    spawn_jtc = TimerAction(period=6.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'run', 'controller_manager', 'spawner',
                 'joint_trajectory_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        )
    ])

    spawn_fvc = TimerAction(period=7.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'run', 'controller_manager', 'spawner',
                 'forward_velocity_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        )
    ])

    # 6. Spawn blue ball after everything loads
    spawn_ball = TimerAction(period=10.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-file', os.path.join(ur5_ibvs_pkg, 'models', 'blue_ball', 'model.sdf'),
                 '-entity', 'blue_ball',
                 '-x', '0.5', '-y', '0.0', '-z', '0.5'],
            output='screen'
        )
    ])

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        controller_manager,
        spawn_jsb,
        spawn_jtc,
        spawn_fvc,
        spawn_ball,
    ])