import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
  # Launch arguments.
  start_rviz = LaunchConfiguration('start_rviz')
  start_rviz_arg = DeclareLaunchArgument(
    'start_rviz',
    default_value="True"
  )
  start_gazebo = LaunchConfiguration('start_gazebo')
  start_gazebo_arg = DeclareLaunchArgument(
    'start_gazebo',
    default_value='True'
  )  
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='True'
  )
  world = LaunchConfiguration('world')
  world_file_arg = DeclareLaunchArgument(
    'world',
    default_value=[
      get_package_share_directory('barista_robot_description'), 
      '/worlds/empty.world'
    ],
  )

  # robot_state_publisher
  robot_description_path = os.path.join(
    get_package_share_directory("barista_robot_description"),
    "urdf",
    "barista_robot_model.urdf"
  )
  robot_description = Command(['xacro ', robot_description_path])
  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    emulate_tty=True,
    parameters=[{
      'use_sim_time': use_sim_time, 
      'robot_description': robot_description}
    ],
    output="screen"
  )

  # rviz
  rviz_config_path = os.path.join(
    get_package_share_directory("barista_robot_description"),
    'rviz',
    'urdf_vis.rviz'
  )
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    output='screen',
    name='rviz_node',
    parameters=[{'use_sim_time': use_sim_time}],
    arguments=['-d', rviz_config_path],
    condition=IfCondition(start_rviz)
  )

  # Gazebo
  gazebo_launch_args = {
    'verbose': 'false',
    'pause': 'false',
    'world': world
  }
  gazebo_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('gazebo_ros'), 
        'launch'
      ), 
      '/gazebo.launch.py'
    ]),
    launch_arguments=gazebo_launch_args.items(),
    condition=IfCondition(start_gazebo)
  )
  spawn_robot_node = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
      '-entity',
      'barista_robot',
      '-x',
      '0.0',
      '-y',
      '0.0',
      '-z',
      '0.5',
      '-topic',
      '/robot_description'
    ],
    condition=IfCondition(start_gazebo)
  )

  return LaunchDescription([
    start_rviz_arg,
    start_gazebo_arg,
    use_sim_time_arg,
    world_file_arg,
    robot_state_publisher_node,
    rviz_node,
    gazebo_node,
    spawn_robot_node
 ])
