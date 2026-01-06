import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'poc1_landing_world'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Path to the world file
    world_file = os.path.join(pkg_share, 'worlds', 'landing_mission.world')
    
    # 2. Path to your models
    models_path = os.path.join(pkg_share, 'models')

    # 3. Path to PX4 models (for the x500 drone)
    # Adjust this path if your PX4 is not in the home directory
    px4_src_path = os.path.expanduser('~/PX4-Autopilot') 
    px4_models_path = os.path.join(px4_src_path, 'Tools', 'simulation', 'gz', 'models')

    # 4. Setup Resource Path (New Gazebo uses GZ_SIM_RESOURCE_PATH)
    # We combine your custom models AND the PX4 models
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        resource_path = os.environ['GZ_SIM_RESOURCE_PATH'] + ':' + models_path + ':' + px4_models_path
    else:
        resource_path = models_path + ':' + px4_models_path

    return LaunchDescription([
        # Set the path so Gazebo finds your Brick/Tower and the Drone
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path),

        # Start New Gazebo (gz sim)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            # -r runs the simulation immediately
            launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        ),

        # Bridge: Connects Gazebo Clock to ROS 2 (Optional but good for sync)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),
        
        # NOTE: We do NOT spawn the drone here. 
        # PX4 will spawn the drone automatically when we run the binary in the next step.
    ])