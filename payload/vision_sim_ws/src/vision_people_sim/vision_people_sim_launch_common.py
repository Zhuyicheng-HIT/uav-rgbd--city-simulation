import json, os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node

def make_launch(world_file, world_name, agents, rate_hz=20.0):
    pkg = get_package_share_directory('vision_people_sim')
    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models') + ':/home/zyc/ardupilot_gazebo/models'),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', '/home/zyc/ardupilot_gazebo/build'),
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-v', '3', '--render-engine-gui', 'ogre', os.path.join(pkg, 'worlds', world_file)], output='screen'),
        TimerAction(period=3.0, actions=[Node(package='vision_people_sim', executable='people_motion', name='people_motion', output='screen', parameters=[{'world_name': world_name, 'agents_json': json.dumps(agents), 'rate_hz': rate_hz}])]),
    ])
