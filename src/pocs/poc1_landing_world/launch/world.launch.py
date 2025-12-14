import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_name = 'poc1_landing_world'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Path to the world file
    world_file = os.path.join(pkg_share, 'worlds', 'landing_mission.world')

    # 2. Path to the models directory (for Gazebo to find AprilTags)
    models_path = os.path.join(pkg_share, 'models')

    # 3. Append to GAZEBO_MODEL_PATH
    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path = os.environ['GAZEBO_MODEL_PATH'] + ':' + models_path
    else:
        model_path = models_path

    return LaunchDescription([
        # Set the environment variable so Gazebo finds our tags
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),

        # Start Gazebo Server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        ),

        # Start Gazebo Client (GUI)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
            ),
        ),
    ])