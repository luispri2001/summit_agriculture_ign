import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions


def generate_launch_description():
    # Obtener los nombres de los paquetes
    pkg_path = get_package_share_directory("summit_cornfield")

    package_name = "summit_xl_description"  # Nombre del paquete con el robot
    desc_package_name = get_package_share_directory(package_name)

    set_ign_gazebo_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=os.getenv("IGN_GAZEBO_RESOURCE_PATH", "") + ":" + os.path.join(pkg_path, "models")
    )


    # Archivo Xacro
    xacro_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        "robots",
        "summit_xl_std.urdf.xacro",
    ])

    # Parámetro para usar tiempo de simulación
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Generar la descripción del robot usando xacro
    robot_description_config = Command([
        "ros2 ", "run ", "xacro ", "xacro ", xacro_file,
        " sim_mode:=", use_sim_time
    ])

    # Crear el nodo robot_state_publisher
    params = {
        "robot_description": launch_ros.descriptions.ParameterValue(robot_description_config, value_type=str),
        "use_sim_time": use_sim_time,
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    # Obtener el mundo a lanzar
    world = LaunchConfiguration("world")
    world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_path, "worlds", "empty.sdf"),
        description="Ignition Gazebo world",
    )

    # Definir si se lanza la GUI
    launch_gui = LaunchConfiguration("launch_gui")
    launch_gui_cmd = DeclareLaunchArgument(
        "launch_gui",
        default_value="True",
        description="Whether to launch Ignition GUI",
    )

    # Definir si se pausa la simulación al inicio
    pause_sim = LaunchConfiguration("pause_sim")
    pause_sim_cmd = DeclareLaunchArgument(
        "pause_sim",
        default_value="False",
        description="Whether to pause simulation at start",
    )
    
    # Iniciar Ignition Gazebo
    # Comando para iniciar Ignition Gazebo con el mundo especificado
    start_ignition_cmd = ExecuteProcess(
        cmd=["ign", "gazebo", world, "-r"],
        output="both",
        shell=True,
    )

    # Spawnear el robot en Ignition Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "summit_xl",
            "-topic", "/robot_description",
            "-x", "-11",
            "-y", "-11",
            "-z", "1.0",
        ],
        output="screen",
    )

    summit_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("summit_localization"),
                "launch",
                "localization.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": "True",
        }.items(),
    )

    summit_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("summit_navigation"),
                "launch",
                "bringup.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            # Comando de velocidad (ROS 2 -> Ignition)
            "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            
            # Odometría (Ignition -> ROS 2)
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            
            # TF (Ignition -> ROS 2)
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            
            # Reloj (Ignition -> ROS 2)
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            
            # Estados de las articulaciones (Ignition -> ROS 2)
            "/world/default/model/summit_xl/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
            
            # Lidar (Ignition -> ROS 2)
            "/world/default/model/summit_xl/linkbase_footprint/sensorfront_laser_sensor/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            
            # Puntos del Lidar (Ignition -> ROS 2)
            "/world/default/model/summit_xl/linkbase_footprint/sensorfront_laser_sensor/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            
            # IMU (Ignition -> ROS 2)
            "/world/default/model/summit_xl/linkbase_footprint/sensorimu/sensor/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            
            # Cámara (Ignition -> ROS 2)
            "/world/default/model/summit_xl/linkbase_footprint/sensorfront_camera_left_sensor/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/world/default/model/summit_xl/linkbase_footprint/sensorfront_camera_left_sensor/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            
            # GPS (Ignition -> ROS 2)
            "/gps/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat",
        ],
        remappings=[
            # Remappings de topics entre Gazebo y ROS 2
            ("/world/default/model/summit_xl/joint_state", "/joint_states"),  # Joints
            ("/world/default/model/summit_xl/odometry", "/odom"),  # Odometría
            ("/world/default/model/summit_xl/linkbase_footprint/sensorfront_laser_sensor/scan", "/scan"),  # Lidar
            ("/world/default/model/summit_xl/linkbase_footprint/sensorimu/sensor/imu", "/imu"),  # IMU
            ("/world/default/model/summit_xl/linkbase_footprint/sensorfront_camera_left_sensor/image", "/camera/rgb/image_raw"),  # Cámara
            ("/world/default/model/summit_xl/linkbase_footprint/sensorfront_camera_left_sensor/camera_info", "/camera/rgb/camera_info"),  # Cámara info
            ("/world/default/model/summit_xl/linkbase_footprint/sensorfront_laser_sensor/scan/points", "/scan/points"),  # Lidar puntos
            ("/world/default/model/summit_xl/linkbase_footprint/sensorfront_camera_sensor/depth_image/points", "/camera/depth/points"),  # Profundidad (Cámara)
        ],
        output="screen",
    )


    # Retornar la descripción del Launch
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use sim time if true"),
        set_ign_gazebo_resource_path,
        robot_state_publisher_node,
        start_ignition_cmd,
        spawn_entity,
        launch_gui_cmd,
        pause_sim_cmd,
        world_cmd,
        bridge,
        joint_state_publisher_node,
        summit_localization_cmd,
        summit_navigation_cmd,  # Ensure this declaration is in the launch description
    ])
