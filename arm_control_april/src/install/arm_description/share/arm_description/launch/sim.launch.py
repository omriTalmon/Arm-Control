# arm_description/launch/sim.launch.py
#
# Starts Ignition Gazebo Fortress with an empty world, spawns the rover
# described in arm_description/urdf/my_rover.urdf, and bridges key
# topics between Gazebo and ROS 2.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    RegisterEventHandler, SetEnvironmentVariable
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution, LaunchConfiguration
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # ────────────────── CLI arguments
    gui          = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rvizconfig   = LaunchConfiguration('rvizconfig')

    # ────────────────── package paths
    desc_share = FindPackageShare('arm_description').find('arm_description')

    default_model_path = os.path.join(desc_share, 'urdf',  'my_rover.urdf')
    default_rviz_path  = os.path.join(desc_share, 'rviz',  'simulation.rviz')
    default_world_path = os.path.join(desc_share, 'world', 'empty.sdf')   # make one if missing

    # ────────────────── robot description
    with open(default_model_path, 'r') as f:
        robot_description = {'robot_description': f.read()}

    # ────────────────── core nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen')

    # Add controller manager with proper parameters
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{
            'robot_description': robot_description['robot_description'],
            'use_sim_time': use_sim_time,
            'update_rate': 100
        }],
        output='screen')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rvizconfig],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # Optional joint_state_publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        condition=UnlessCondition(gui),          # only if gui:=False
        output='screen')

    # ────────────────── spawn entity in Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'my_rover',
            '-z',     '0.3',
            '-allow_renaming', 'true'],
        output='screen')

    # ros2_control (optional)
    load_js_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen')

    load_ackermann_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'ackermann_steering_controller'],
        output='screen')

    spawn_then_js = RegisterEventHandler(
        OnProcessExit(target_action=gz_spawn_entity,
                      on_exit=[load_js_broadcaster]))
    js_then_ack   = RegisterEventHandler(
        OnProcessExit(target_action=load_js_broadcaster,
                      on_exit=[load_ackermann_controller]))

    # ────────────────── bridges
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/model/my_rover/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/model/my_rover/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose',
            '/model/my_rover/cmd_vel@geometry_msgs/msg/Twist[ignition.msgs.Twist'],
        output='screen')


    set_pkg_path = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', desc_share)
    set_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', desc_share)
    set_pkg_path_old = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', desc_share)  

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': 'empty.sdf -r',
            'gz_version': 'fortress'  # Specify Gazebo version
        }.items())

    # ────────────────── resource path for meshes / textures
    set_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', desc_share)
    
    # Additional resource paths
    set_ign_resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', desc_share)
    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', desc_share)
    set_gz_sim_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', desc_share)
    set_ros_package_path = SetEnvironmentVariable(
        'ROS_PACKAGE_PATH', desc_share)
    set_ament_prefix_path = SetEnvironmentVariable(
        'AMENT_PREFIX_PATH', desc_share)

    # ────────────────── assemble LD
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='False',
                              description='Start joint_state_publisher GUI'),
        DeclareLaunchArgument('use_sim_time', default_value='True',
                              description='Use simulation clock'),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz_path,
                              description='RViz2 config'),

        set_resource_path,
        set_ign_resource_path,
        set_gazebo_model_path,
        set_gz_sim_resource_path,
        set_ros_package_path,
        set_ament_prefix_path,
        gazebo_launch,
        robot_state_publisher,
        controller_manager,
        joint_state_publisher_gui,
        gz_spawn_entity,
        bridge,
        rviz,
        spawn_then_js,
        js_then_ack,
        ])