import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
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
  
  robot_name_1 = "rick"
  robot_name_2 = "morty"

  # robot_state_publisher
  robot_description_path = os.path.join(
    get_package_share_directory("barista_robot_description"),
    "xacro",
    "barista_robot_model.xacro"
  )
  robot_1_description = Command([
    'xacro ', robot_description_path, 
    ' robot_name:=', robot_name_1, 
    ' color:=', "blue"
  ])
  robot_1_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    namespace=robot_name_1,
    emulate_tty=True,
    parameters=[{
      'use_sim_time': use_sim_time, 
      'robot_description': robot_1_description}
    ],
    output="screen"
  )

  robot_2_description = Command([
    'xacro ', robot_description_path, 
    ' robot_name:=', robot_name_2, 
    ' color:=', "red"
  ])
  robot_2_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    namespace=robot_name_2,
    emulate_tty=True,
    parameters=[{
      'use_sim_time': use_sim_time, 
      'robot_description': robot_2_description}
    ],
    output="screen"
  )  

  # static world->odom transform publishers
  robot_1_static_tf_publisher_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    namespace=robot_name_1,
    name='static_transform_publisher',
    output='screen',
    emulate_tty=True,
    arguments=['0', '0', '0', '0', '0', '0', 'world', robot_name_1 + '_odom']
  )
  robot_2_static_tf_publisher_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    namespace=robot_name_2,
    name='static_transform_publisher',
    output='screen',
    emulate_tty=True,
    arguments=['0.0', '0.0', '0', '0', '0', '0', 'world', robot_name_2 + '_odom']
  )

  # rviz
  rviz_config_path = os.path.join(
    get_package_share_directory("barista_robot_description"),
    'rviz',
    'urdf_vis_two_robots.rviz'
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
  gazebo_models_path = os.path.join("barista_robot_description", 'meshes')
  install_dir = get_package_prefix("barista_robot_description")
  if 'GAZEBO_MODEL_PATH' in os.environ:
    os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + \
      ':' + install_dir + '/share' + ':' + gazebo_models_path
  else:
    os.environ['GAZEBO_MODEL_PATH'] = install_dir + \
      "/share" + ':' + gazebo_models_path
    
  if 'GAZEBO_PLUGIN_PATH' in os.environ:
    os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + \
      ':' + install_dir + '/lib'
  else:
    os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

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
  spawn_robot_1_node = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
      '-entity',
      robot_name_1,
      '-x',
      '0.0',
      '-y',
      '0.0',
      '-z',
      '0.5',
      '-topic',
      robot_name_1 + '/robot_description'
    ],
    condition=IfCondition(start_gazebo)
  )
  spawn_robot_2_node = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
      '-entity',
      robot_name_2,
      '-x',
      '1.0',
      '-y',
      '1.0',
      '-z',
      '0.5',
      '-topic',
      robot_name_2+'/robot_description'
    ],
    condition=IfCondition(start_gazebo)
  )

  return LaunchDescription([
    start_rviz_arg,
    start_gazebo_arg,
    use_sim_time_arg,
    world_file_arg,
    robot_1_state_publisher_node,
    robot_2_state_publisher_node,
    robot_1_static_tf_publisher_node,
    robot_2_static_tf_publisher_node,
    rviz_node,
    gazebo_node,
    spawn_robot_1_node,
    spawn_robot_2_node
 ])
