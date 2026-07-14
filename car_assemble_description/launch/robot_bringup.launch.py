"""Pi 端 headless 整合啟動檔（給樹莓派跑,不開任何 GUI）。

一次拉起:
  1. robot_state_publisher —— 用真正的 URDF 發布車體/輪子/lidar_link 的 TF 與 /robot_description
  2. joint_state_publisher —— headless(非 GUI)版,把四顆 continuous 輪子 joint 補 0,讓輪子 TF 存在
  3. sllidar_node + base_link->laser_frame 靜態 TF —— 沿用 my_robot_lidar/lidar_start.launch.py
  4. 底盤:use_fake_odom=true → 發假的 odom->base_link 靜態 TF;
        use_fake_odom=false → 改跑 ominibot_driver(收 /cmd_vel、發 /odom + 真 odom->base_link TF)
  5. slam_toolbox(async)—— 沿用 my_robot_lidar 的 mapper 參數

RViz 一律不在這裡開;請在另一台 Ubuntu PC 上用相同 ROS_DOMAIN_ID 連過來看
(見 car_assemble_description/rviz/view_robot.rviz 與 repo 內的雙機連線說明)。

用法:
  ros2 launch car_assemble_description robot_bringup.launch.py
  ros2 launch car_assemble_description robot_bringup.launch.py use_slam:=false      # 只出光達+模型,不建圖
  ros2 launch car_assemble_description robot_bringup.launch.py use_fake_odom:=false # 接上真正底盤(跑 ominibot_driver)
  ros2 launch car_assemble_description robot_bringup.launch.py use_fake_odom:=false ominibot_port:=/dev/ttyUSB1  # udev 還沒裝時
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('car_assemble_description')
    lidar_share = get_package_share_directory('my_robot_lidar')

    urdf_path = os.path.join(desc_share, 'urdf', 'CAR_ASSEMBLE_URDF.urdf')
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    slam_config = os.path.join(lidar_share, 'config', 'mapper_params_online_async.yaml')

    use_slam = LaunchConfiguration('use_slam')
    use_fake_odom = LaunchConfiguration('use_fake_odom')
    ominibot_port = LaunchConfiguration('ominibot_port')
    vx_sign = LaunchConfiguration('vx_sign')
    vy_sign = LaunchConfiguration('vy_sign')
    wz_sign = LaunchConfiguration('wz_sign')

    return LaunchDescription([
        DeclareLaunchArgument('use_slam', default_value='true',
                              description='是否啟動 slam_toolbox 建圖'),
        DeclareLaunchArgument('use_fake_odom', default_value='true',
                              description='true=發假 odom 靜態 TF;false=改跑 ominibot_driver 真底盤'),
        DeclareLaunchArgument('ominibot_port', default_value='/dev/ominibot',
                              description='OminiBotHV 底盤板序列埠(udev 未裝時可設 /dev/ttyUSB1)'),
        DeclareLaunchArgument('vx_sign', default_value='1.0',
                              description='前後反了就設 -1.0(x 前進為正)'),
        DeclareLaunchArgument('vy_sign', default_value='1.0',
                              description='左右平移反了就設 -1.0(y 左移為正)'),
        DeclareLaunchArgument('wz_sign', default_value='1.0',
                              description='旋轉方向反了就設 -1.0(z 逆時針為正)'),

        # 1. 車體模型 TF + /robot_description(PC 端 RViz 的 RobotModel 會訂閱這個 topic)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # 2. headless joint_state_publisher(非 GUI),讓四顆輪子 joint 有 TF
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        # 3. 光達驅動 + base_link->laser_frame 靜態 TF(沿用你現有的檔案)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lidar_share, 'launch', 'lidar_start.launch.py')
            )
        ),

        # 4a. use_fake_odom=true:暫時的假里程計 odom->base_link 靜態 TF。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fake_odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            condition=IfCondition(use_fake_odom),
            output='screen',
        ),

        # 4b. use_fake_odom=false:真的 OminiBotHV 底盤驅動,收 /cmd_vel、發 /odom + odom->base_link TF。
        Node(
            package='ominibot_driver',
            executable='ominibot_driver_node',
            name='ominibot_driver',
            output='screen',
            parameters=[{
                'port': ominibot_port,
                'linear_x_sign': ParameterValue(vx_sign, value_type=float),
                'linear_y_sign': ParameterValue(vy_sign, value_type=float),
                'angular_z_sign': ParameterValue(wz_sign, value_type=float),
            }],
            condition=UnlessCondition(use_fake_odom),
        ),

        # 5. SLAM Toolbox(async),沿用 my_robot_lidar 的參數
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config, {'use_sim_time': False}],
            condition=IfCondition(use_slam),
        ),
    ])
