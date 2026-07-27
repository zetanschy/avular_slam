#!/usr/bin/env python3
"""
Clase 5 - SLAM: simulacion base con el Avular Origin One.

Usamos el robot de la clase 3 (`origin_one_description`) porque ya trae los
tres sensores que necesitamos:

    LiDAR 2D    360 rayos, 30 m          -> /scan            -> slam_toolbox
    LiDAR 3D    Ouster OS1-32, 1024x32   -> /robot/lidar/points -> GLIM
    RealSense   D435 RGB-D 640x480       -> /robot/camera/... -> RTAB-Map
    IMU                                  -> /imu
    Traccion skid-steer (gira sobre si mismo)

Este launch NO toca el paquete de Avular. Lo que hace es procesar su xacro y
retocar el URDF resultante en memoria antes de spawnearlo (ver
`preparar_urdf`), porque hay tres cosas que la simulacion de Avular no trae
pensadas para SLAM:

  1. La IMU publica a 25 Hz. GLIM quiere >= 100 Hz para integrar bien.
  2. La camara va a 6 Hz. Para SLAM visual es muy poco.
  3. No hay verdad de campo con la que medir el error.

Uso:
    ros2 launch avular_slam simulation.launch.py
    ros2 launch avular_slam simulation.launch.py headless:=true
    ros2 launch avular_slam simulation.launch.py rviz_config:=slam_2d.rviz
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable,
                            SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PACKAGE = "avular_slam"

# ---------------------------------------------------------------------------
# Trozos de URDF que le inyectamos al robot de Avular
# ---------------------------------------------------------------------------

# Frame optico de la camara (REP-103: Z adelante, X derecha, Y abajo).
# Gazebo entrega las imagenes en esta convencion, pero el URDF de Avular las
# etiqueta con "camera_link", que es X-adelante. Si se deja asi, RTAB-Map
# monta el mapa girado 90 grados. Creamos el frame optico y le decimos al
# sensor que publique con ese.
FRAME_OPTICO = """
  <link name="camera_optical_link"/>
  <joint name="camera_optical_joint" type="fixed">
    <origin xyz="0 0 0" rpy="-1.5707963 0 -1.5707963"/>
    <parent link="camera_link"/>
    <child link="camera_optical_link"/>
  </joint>
"""

# El LiDAR 3D publica sus nubes con frame_id "os_lidar" (el nombre que usa el
# driver del Ouster de verdad), pero en el URDF de Avular ese link NO EXISTE:
# solo esta "lidar_link_gazebo". Resultado: RViz se queja de un frame
# desconocido y la nube no se puede dibujar. Lo creamos, coincidiendo con el
# link del sensor.
FRAME_LIDAR = """
  <link name="os_lidar"/>
  <joint name="os_lidar_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="lidar_link_gazebo"/>
    <child link="os_lidar"/>
  </joint>
"""

# Odometria de VERDAD. Gazebo sabe exactamente donde esta el robot;
# publicamos esa pose como una odometria mas para poder medir el error de la
# odometria de ruedas y el del SLAM (ver la tarea). En un robot real esto no
# existe: haria falta un Vicon o un GPS RTK.
ODOMETRIA_VERDAD = """
  <gazebo>
    <plugin filename="ignition-gazebo-odometry-publisher-system"
            name="ignition::gazebo::systems::OdometryPublisher">
      <odom_frame>mundo</odom_frame>
      <robot_base_frame>base_verdad</robot_base_frame>
      <odom_topic>odom_verdad</odom_topic>
      <odom_publish_frequency>30</odom_publish_frequency>
      <dimensions>3</dimensions>
    </plugin>
  </gazebo>
"""


def preparar_urdf(context, *args, **kwargs):
    """Procesa el xacro de Avular y le aplica nuestros retoques."""
    descripcion = get_package_share_directory("origin_one_description")
    xacro_file = os.path.join(descripcion, "urdf", "origin_one.urdf.xacro")

    urdf = xacro.process_file(
        xacro_file,
        mappings={"drive_configuration": "skid_steer_drive"},
    ).toxml()

    imu_rate = LaunchConfiguration("imu_rate").perform(context)
    camera_rate = LaunchConfiguration("camera_rate").perform(context)

    cambios = [
        # 1. IMU: 25 Hz -> imu_rate. GLIM preintegra la IMU entre scans; a
        #    25 Hz la integracion es demasiado gruesa y la odometria se va.
        ("<update_rate>25.0</update_rate>",
         f"<update_rate>{imu_rate}</update_rate>"),
        # 2. Camara: 6 Hz -> camera_rate. Con 6 Hz la odometria visual pierde
        #    el seguimiento en cuanto el robot gira un poco rapido.
        ("<update_rate>6</update_rate>",
         f"<update_rate>{camera_rate}</update_rate>"),
        # 3. Las imagenes con el frame optico, no con camera_link.
        ("<gz_frame_id>camera_link</gz_frame_id>",
         "<gz_frame_id>camera_optical_link</gz_frame_id>"),
    ]
    for viejo, nuevo in cambios:
        if viejo not in urdf:
            print(f"[avular_slam] AVISO: no encuentro '{viejo}' en el URDF de "
                  f"Avular. Habran cambiado su paquete; revisad "
                  f"simulation.launch.py")
        urdf = urdf.replace(viejo, nuevo, 1)

    urdf = urdf.replace(
        "</robot>", FRAME_OPTICO + FRAME_LIDAR + ODOMETRIA_VERDAD + "</robot>")

    return [SetLaunchConfiguration("robot_urdf", urdf)]


def generate_launch_description():
    pkg_share = FindPackageShare(package=PACKAGE)
    pkg_share_dir = get_package_share_directory(PACKAGE)
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")

    world_path = PathJoinSubstitution(
        [pkg_share, "worlds", LaunchConfiguration("world")])
    bridge_config = PathJoinSubstitution(
        [pkg_share, "config", "bridge_slam.yaml"])
    rviz_config = PathJoinSubstitution(
        [pkg_share, "rviz", LaunchConfiguration("rviz_config")])

    args = [
        DeclareLaunchArgument(
            "world", default_value="slam_world.sdf",
            description="Mundo SDF de worlds/. El de por defecto es un "
                        "circuito cerrado con texturas, hecho para SLAM."),
        DeclareLaunchArgument("robot_x", default_value="-9.0"),
        DeclareLaunchArgument("robot_y", default_value="-6.0"),
        DeclareLaunchArgument("robot_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "imu_rate", default_value="200",
            description="Hz de la IMU (Avular trae 25, GLIM quiere >=100)"),
        DeclareLaunchArgument(
            "camera_rate", default_value="15",
            description="Hz de la camara RGB-D (Avular trae 6)"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config", default_value="sensores.rviz",
            description="sensores | slam_2d | slam_3d | visual_slam"),
        DeclareLaunchArgument(
            "odom_tf", default_value="true",
            description="Publicar la TF odom->base_link de Gazebo. false "
                        "SOLO para la demo de odometria visual (rtabmap-vo)."),
        DeclareLaunchArgument("headless", default_value="false"),
    ]

    # Gazebo busca las texturas del mundo como
    # model://avular_slam/media/textures/... -> hay que meter en el path el
    # PADRE del share/ del paquete. Sin esto no da ningun error: las paredes
    # salen blancas y el SLAM visual no encuentra features.
    # Y las mallas del robot van como
    # model://origin_one_description/urdf/meshes/... -> tambien hace falta el
    # padre del share de ESE paquete, o el robot sale invisible (solo salen
    # errores "Mesh manager can't find mesh named...").
    descripcion_dir = get_package_share_directory("origin_one_description")
    resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[os.path.dirname(pkg_share_dir), ":", pkg_share_dir, ":",
               os.path.dirname(descripcion_dir), ":",
               os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")])

    gz_args = PythonExpression([
        "'-r -s --headless-rendering -v1 ' if '",
        LaunchConfiguration("headless"), "' == 'true' else '-r -v1 '"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": [gz_args, world_path],
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        # El Origin One publica los estados de las articulaciones en
        # /robot/joint_states, pero robot_state_publisher escucha en
        # /joint_states a secas. Sin este remapeo no le llegan y no puede
        # calcular las TF de las cuatro ruedas (que son joints continuous):
        # en RViz el RobotModel sale en rojo con "No transform from
        # [left_front_wheel]". No afecta al SLAM -los sensores cuelgan de
        # links fijos- pero queda feo y despista.
        remappings=[("/joint_states", "/robot/joint_states")],
        parameters=[{
            "use_sim_time": True,
            # value_type=str obligatorio: si no, launch intenta leer el URDF
            # como YAML y falla en cuanto hay un ": " dentro.
            "robot_description": ParameterValue(
                LaunchConfiguration("robot_urdf"), value_type=str),
        }],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        output="screen",
        arguments=[
            "-name", "origin_one",
            "-string", LaunchConfiguration("robot_urdf"),
            "-x", LaunchConfiguration("robot_x"),
            "-y", LaunchConfiguration("robot_y"),
            "-z", "0.25",
            "-Y", LaunchConfiguration("robot_yaw"),
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[{
            "config_file": bridge_config,
            "use_sim_time": True,
        }],
    )

    # La TF odom->base_link la publica el plugin DiffDrive de Avular en el
    # topic de Gazebo /robot/tf. Va en un nodo aparte para poder apagarla
    # (odom_tf:=false) cuando quien estima la odometria es RTAB-Map: dos
    # nodos publicando el mismo padre->hijo rompen el arbol TF.
    bridge_tf = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge_tf",
        output="screen",
        arguments=["/robot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V"],
        remappings=[("/robot/tf", "/tf")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("odom_tf")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(args + [
        resource_path,
        OpaqueFunction(function=preparar_urdf),
        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
        bridge_tf,
        rviz,
    ])
