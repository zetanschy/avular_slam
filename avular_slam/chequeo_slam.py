#!/usr/bin/env python3
"""
Chequeo previo: ¿esta todo lo que necesita el SLAM?

Antes de pelearse con GLIM o RTAB-Map, ejecutad esto. Mide 8 segundos y
dice que topic falta, cual va a una frecuencia rara y que TF no existe.
El 90% de los "no me funciona el SLAM" de la practica se resuelven aqui.

    ros2 run avular_slam chequeo_slam
"""

import time

import rclpy
from geometry_msgs.msg import TransformStamped  # noqa: F401  (documenta el tipo)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener

VERDE = "\033[92m"
ROJO = "\033[91m"
AMARILLO = "\033[93m"
FIN = "\033[0m"

# topic, tipo, Hz esperados, para que sirve
TOPICS = [
    ("/clock", Clock, None, "tiempo de simulacion"),
    ("/robot/odom", Odometry, 50, "odometria de ruedas"),
    ("/odom_verdad", Odometry, 30, "verdad de campo (para medir el error)"),
    ("/scan", LaserScan, 10, "SLAM 2D (slam_toolbox)"),
    ("/robot/lidar/points", PointCloud2, 10, "SLAM 3D (GLIM / KISS-ICP)"),
    ("/imu", Imu, 200, "SLAM 3D (GLIM la necesita rapida)"),
    ("/robot/camera/color/image_raw", Image, 15, "SLAM visual (RTAB-Map)"),
    ("/robot/camera/depth/image_raw", Image, 15, "SLAM visual (RTAB-Map)"),
    ("/robot/camera/color/camera_info", CameraInfo, 15, "SLAM visual"),
]

TFS = [
    ("odom", "base_link", "Gazebo (plugin DiffDrive del Origin One)"),
    ("base_link", "scan_link", "robot_state_publisher (LiDAR 2D)"),
    ("base_link", "os_lidar", "robot_state_publisher (LiDAR 3D)"),
    ("base_link", "camera_optical_link", "lo inyecta simulation.launch.py"),
    ("base_link", "imu_link", "robot_state_publisher"),
]

DURACION = 8.0


class Chequeo(Node):

    def __init__(self):
        super().__init__("chequeo_slam")
        self.set_parameters([
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.n = {t: 0 for t, _, _, _ in TOPICS}
        for topic, typ, _, _ in TOPICS:
            self.create_subscription(
                typ, topic,
                (lambda k: lambda _m: self.n.__setitem__(k, self.n[k] + 1))(topic),
                qos_profile_sensor_data)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)


def main():
    rclpy.init()
    node = Chequeo()
    print(f"\nEscuchando {DURACION:.0f} s...\n")
    t0 = time.time()
    while time.time() - t0 < DURACION:
        rclpy.spin_once(node, timeout_sec=0.1)
    dt = time.time() - t0

    fallos = 0
    print(f"{'TOPIC':<30}{'Hz':>8}   {'ESTADO':<10} PARA QUE")
    print("-" * 88)
    for topic, _, esperado, uso in TOPICS:
        hz = node.n[topic] / dt
        if node.n[topic] == 0:
            estado, color = "FALTA", ROJO
            fallos += 1
        elif esperado and hz < 0.5 * esperado:
            estado, color = "LENTO", AMARILLO
        else:
            estado, color = "OK", VERDE
        print(f"{topic:<30}{hz:>8.1f}   {color}{estado:<10}{FIN} {uso}")

    print(f"\n{'TRANSFORMADA':<40}{'ESTADO':<10} QUIEN LA PUBLICA")
    print("-" * 88)
    for padre, hijo, quien in TFS:
        try:
            node.buf.lookup_transform(padre, hijo, rclpy.time.Time())
            estado, color = "OK", VERDE
        except Exception:
            estado, color = "FALTA", ROJO
            fallos += 1
        print(f"{padre + ' -> ' + hijo:<40}{color}{estado:<10}{FIN} {quien}")

    print()
    if fallos:
        print(f"{ROJO}{fallos} problema(s).{FIN} Lo mas habitual:")
        print("  - /scan o /imu a 0 Hz -> no son parte del puente de Avular;")
        print("                           tienen que estar en bridge_slam.yaml")
        print("  - odom->base_link     -> falta el puente /robot/tf -> /tf")
        print("                           (o lo habeis lanzado con odom_tf:=false)")
        print("  - camera_optical_link -> el URDF sin parchear no lo tiene:")
        print("                           lo anade simulation.launch.py")
    else:
        print(f"{VERDE}Todo correcto. Podeis lanzar cualquiera de los tres SLAM.{FIN}")
    print()

    node.destroy_node()
    rclpy.shutdown()
    return 1 if fallos else 0


if __name__ == "__main__":
    main()
