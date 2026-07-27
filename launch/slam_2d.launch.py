#!/usr/bin/env python3
"""
Clase 5 - DEMO 1: SLAM 2D con slam_toolbox.

Se lanza ENCIMA de simulation.launch.py, en otra terminal:

    # terminal 1
    ros2 launch avular_slam simulation.launch.py rviz_config:=slam_2d.rviz
    # terminal 2
    ros2 launch avular_slam slam_2d.launch.py
    # terminal 3
    ros2 run avular_slam recorrido_bucle

Para guardar el mapa cuando este completo:

    ros2 run nav2_map_server map_saver_cli -f ~/mapa_clase5

Demo recomendada en clase (es la que mejor se ve):
    1. Lanzarlo con do_loop_closing:=false -> al completar la vuelta el
       pasillo no cierra: se ve la deriva acumulada de la odometria.
    2. Repetir con el valor por defecto (true) -> al pasar por el punto de
       partida el grafo se re-optimiza y el mapa "salta" a su sitio.

    ros2 launch avular_slam slam_2d.launch.py do_loop_closing:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PACKAGE = "avular_slam"


def generate_launch_description():
    pkg_share = FindPackageShare(package=PACKAGE)
    params = PathJoinSubstitution([pkg_share, "config", "slam_toolbox_2d.yaml"])

    args = [
        DeclareLaunchArgument(
            "do_loop_closing", default_value="true",
            description="false = se ve la deriva sin corregir (buena demo)"),
        DeclareLaunchArgument(
            "resolution", default_value="0.05",
            description="Tamano de celda del mapa en metros"),
        DeclareLaunchArgument(
            "minimum_travel_distance", default_value="0.3",
            description="Cada cuantos metros se anade un nodo al grafo"),
    ]

    # async = procesa el ultimo scan disponible y descarta los atrasados.
    # sync = los procesa todos en orden. En simulacion a RTF 1.0 los dos
    # valen; en un robot real siempre async.
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            params,
            {
                "use_sim_time": True,
                # value_type explicito: si no, llegan como string y
                # slam_toolbox rechaza el parametro por tipo incorrecto.
                "do_loop_closing": ParameterValue(
                    LaunchConfiguration("do_loop_closing"), value_type=bool),
                "resolution": ParameterValue(
                    LaunchConfiguration("resolution"), value_type=float),
                "minimum_travel_distance": ParameterValue(
                    LaunchConfiguration("minimum_travel_distance"),
                    value_type=float),
            },
        ],
    )

    return LaunchDescription(args + [slam])
