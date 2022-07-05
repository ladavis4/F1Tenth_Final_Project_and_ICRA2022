from http.server import executable
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()
    bringup = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('f1tenth_stack'), 'launch'),
         '/bringup_launch.py'])
      )
    pf = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('particle_filter'), 'launch'),
         '/localize_launch.py'])
      )
    # # Simulator Launch
    # config = os.path.join(
    #     get_package_share_directory('f1tenth_gym_ros'),
    #     'config',
    #     'sim.yaml'
    #     )
    # config_dict = yaml.safe_load(open(config, 'r'))
    # has_opp = config_dict['bridge']['ros__parameters']['num_agent'] > 1
    # teleop = config_dict['bridge']['ros__parameters']['kb_teleop']

    # bridge_node = Node(
    #     package='f1tenth_gym_ros',
    #     executable='gym_bridge',
    #     name='bridge',
    #     parameters=[config]
    # )
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz',
    #     arguments=['-d', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'gym_bridge.rviz')]
    # )
    # map_server_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     parameters=[{'yaml_filename': config_dict['bridge']['ros__parameters']['map_path'] + '.yaml'},
    #                 {'topic': 'map'},
    #                 {'frame_id': 'map'},
    #                 {'output': 'screen'},
    #                 {'use_sim_time': True}]
    # )
    # nav_lifecycle_node = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_localization',
    #     output='screen',
    #     parameters=[{'use_sim_time': True},
    #                 {'autostart': True},
    #                 {'node_names': ['map_server']}]
    # )
    # ego_robot_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='ego_robot_state_publisher',
    #     parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'ego_racecar.xacro')])}],
    #     remappings=[('/robot_description', 'ego_robot_description')]
    # )
    # opp_robot_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='opp_robot_state_publisher',
    #     parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'opp_racecar.xacro')])}],
    #     remappings=[('/robot_description', 'opp_robot_description')]
    # )

    # Pure pursuit
    pure_pursuit_node = Node(
        package = 'pure_pursuit_pkg',
        executable = 'pure_pursuit',
        name = 'pure_pursuit_node'
    )

    # Obstacle detection
    obs_detect_node = Node(
        package = 'obs_detect_pkg',
        executable = 'obs_detect_node',
        name = 'obstacle_detection_node'
    )

    # Obstacle avoidance
    gap_follow_node = Node(
        package = 'gap_follow',
        executable = 'reactive_node',
        name = 'obstacle_avoidance_node'
    )

    # finalize
    # ld.add_action(rviz_node)
    # ld.add_action(bridge_node)
    # ld.add_action(nav_lifecycle_node)
    # ld.add_action(map_server_node)
    # ld.add_action(ego_robot_publisher)
    # if has_opp:
    #     ld.add_action(opp_robot_publisher)
    ld.add_action(bringup)
    ld.add_action(pf)
    ld.add_action(pure_pursuit_node)
    ld.add_action(obs_detect_node)
    ld.add_action(gap_follow_node)



    return ld