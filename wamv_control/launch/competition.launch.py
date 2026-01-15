import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Ruta a tu paquete y al de VRX
    pkg_wamv_control = get_package_share_directory('wamv_control')
    pkg_vrx_gz = get_package_share_directory('vrx_gz')

    # Ruta a tu configuración de EKF
    ekf_config = os.path.join(pkg_wamv_control, 'config', 'ekf.yaml')

    # 1. Incluimos el launch completo de VRX (con bridges y todo)
    vrx_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_vrx_gz, 'launch', 'competition.launch.py')
        ),
        launch_arguments={
            'world': 'sydney_regatta',
            'headless': 'False',
            # Puedes cambiar otros argumentos si quieres (paused, etc.)
        }.items()
    )

    # 2. Nodo navsat_transform_node (convierte GPS a odometría local)
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('gps/fix', '/wamv/sensors/gps/gps/fix'),
            ('imu', '/wamv/sensors/imu/imu/data'),
            ('odometry/gps', '/odometry/gps')
        ]
    )

    # 3. Nodo EKF principal (fusión final)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/odometry/filtered')
        ]
    )

    return LaunchDescription([
        vrx_launch,
        navsat_node,
        ekf_node
    ])