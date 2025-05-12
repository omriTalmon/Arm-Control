# arm_description/launch/arm_description.launch.py
#
# Wrapper: launches sim.launch.py (also in arm_description/launch)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ─────────────── user-settable arguments
    gui          = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rvizconfig   = LaunchConfiguration('rvizconfig')

    # Folder that contains this package
    desc_share = FindPackageShare('arm_description').find('arm_description')

    # Include the main bring-up launch file
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([desc_share, 'launch', 'sim.launch.py'])
        ),
        launch_arguments={
            'gui': gui,
            'use_sim_time': use_sim_time,
            'rvizconfig': rvizconfig
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='False',
                              description='Start joint_state_publisher GUI'),
        DeclareLaunchArgument('use_sim_time', default_value='True',
                              description='Use simulation clock from /clock'),
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=PathJoinSubstitution(
                [desc_share, 'rviz', 'simulation.rviz']),
            description='RViz2 configuration file'),

        # Kick off the simulation
        sim_launch
        ])