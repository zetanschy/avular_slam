#!/usr/bin/env python3
"""
Da vueltas al circuito automaticamente, para no depender del teleop
mientras se explica el SLAM en clase.

El recorrido es el anillo alrededor del bloque central de slam_world.sdf.
La gracia esta en dar VARIAS vueltas: en la primera se construye el mapa,
y al volver al punto de partida es cuando se ve el cierre de bucle
corrigiendo la deriva acumulada.

    ros2 run avular_slam recorrido_bucle
    ros2 run avular_slam recorrido_bucle --ros-args -p vueltas:=3 -p velocidad:=1.2

Es un seguidor de waypoints con control proporcional sobre el error de
rumbo. No es un planificador: no esquiva nada. Lleva un freno de emergencia
por LiDAR que avisa si se acerca demasiado a algo, porque un choque estropea
la demo de una forma muy dificil de diagnosticar (ver el README).
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# Esquinas del anillo EN COORDENADAS DEL MUNDO (ver generar_mundo_slam.py):
#   pared exterior en x = +-12, y = +-9
#   bloque central  en x = +-6,  y = +-3
# -> el carril va por el centro del pasillo: x = +-9.0, y = +-6.0
ESQUINAS = [(9.0, -6.0), (9.0, 6.0), (-9.0, 6.0), (-9.0, -6.0)]

# El Origin One es skid-steer: puede girar sobre si mismo. Aun asi
# redondeamos las esquinas, porque girar en el sitio patina mucho y es justo
# lo que mas ensucia la odometria (y de paso el scan matching).
RADIO_ESQUINA = 2.0
PASO = 1.0

# DE DONDE SACA EL ROBOT SU POSE PARA SEGUIR LA RUTA
# ---------------------------------------------------
# Por defecto usamos /odom_verdad, o sea la pose REAL que nos da Gazebo.
# Puede sonar a trampa, asi que conviene tenerlo claro:
#
#   - Este nodo NO es un sistema de navegacion. Es una ayuda de clase para
#     que el robot de vueltas solo mientras se explica el SLAM.
#   - Los algoritmos de SLAM siguen viendo unicamente sensores y la
#     odometria de ruedas. No tocamos nada de lo que se esta demostrando.
#   - Con la odometria de ruedas del Origin One no se puede: es skid-steer,
#     patina de lado en cada giro y acumula ~3 m de error por vuelta.
#     Medido: tras vuelta y media, /robot/odom decia (-1.7, 4.7) y el robot
#     estaba de verdad en (2.0, 7.7). Siguiendo esa pose se sale del pasillo
#     y choca. Y encima cada choque estropea aun mas la odometria.
#
# Esa deriva no es un defecto del ejercicio: es EL motivo de que exista el
# SLAM, y es justo lo que se ve corregir cuando cierra el bucle.
#
# Con fuente:="odom" se usa la odometria de ruedas (y se ve el problema).
FUENTE_POR_DEFECTO = "verdad"          # "verdad" | "odom"
TOPIC_VERDAD = "/odom_verdad"          # coordenadas del MUNDO
TOPIC_ODOM = "/robot/odom"             # nace en el punto de spawn

# OJO, ESTO SE CONFUNDE SIEMPRE: /robot/odom NO esta en coordenadas del
# mundo. El frame odom nace donde aparece el robot y ahi vale (0,0,0). Si se
# le pasan al controlador los waypoints del mundo tal cual, el robot se va
# derecho contra la pared. Por eso, SOLO en modo "odom", restamos el spawn.
ORIGEN_POR_DEFECTO = (-9.0, -6.0, 0.0)   # == robot_x/robot_y/robot_yaw del launch


def generar_carril(esquinas, radio, paso):
    """Rectangulo con las esquinas redondeadas, muestreado cada `paso`."""
    pts = []
    n = len(esquinas)
    for i in range(n):
        p = esquinas[i]
        ant = esquinas[(i - 1) % n]
        sig = esquinas[(i + 1) % n]

        # puntos de entrada y salida del arco de esta esquina
        def hacia(a, b, d):
            vx, vy = b[0] - a[0], b[1] - a[1]
            L = math.hypot(vx, vy)
            return (a[0] + vx / L * d, a[1] + vy / L * d)

        entrada = hacia(p, ant, radio)
        salida = hacia(p, sig, radio)

        # tramo recto desde la salida de la esquina anterior hasta la entrada
        prev_salida = hacia(ant, p, radio)
        largo = math.hypot(entrada[0] - prev_salida[0], entrada[1] - prev_salida[1])
        for k in range(1, max(1, int(largo / paso)) + 1):
            t = k / max(1, int(largo / paso))
            pts.append((prev_salida[0] + (entrada[0] - prev_salida[0]) * t,
                        prev_salida[1] + (entrada[1] - prev_salida[1]) * t))

        # arco de 90 grados (cuadrante de circunferencia) por la esquina
        centro = (entrada[0] + salida[0] - p[0], entrada[1] + salida[1] - p[1])
        a0 = math.atan2(entrada[1] - centro[1], entrada[0] - centro[0])
        a1 = math.atan2(salida[1] - centro[1], salida[0] - centro[0])
        d = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
        for k in range(1, 7):
            a = a0 + d * k / 6.0
            pts.append((centro[0] + radio * math.cos(a),
                        centro[1] + radio * math.sin(a)))
    return pts


class RecorridoBucle(Node):

    def __init__(self):
        super().__init__("recorrido_bucle")

        self.declare_parameter("velocidad", 0.8)      # m/s
        self.declare_parameter("k_rumbo", 1.6)        # ganancia proporcional
        self.declare_parameter("tolerancia", 0.7)     # m para dar el wp por hecho
        self.declare_parameter("vueltas", 2)          # 0 = infinitas
        self.declare_parameter("max_giro", 1.2)       # rad/s
        self.declare_parameter("freno_lidar", 0.9)    # m; 0 = desactivar
        self.declare_parameter("fuente", FUENTE_POR_DEFECTO)  # verdad | odom
        # por encima de este error de rumbo, gira parado (rad)
        self.declare_parameter("umbral_giro", 0.35)
        # Pose de spawn, para pasar los waypoints de mundo a odom
        self.declare_parameter("origen_x", ORIGEN_POR_DEFECTO[0])
        self.declare_parameter("origen_y", ORIGEN_POR_DEFECTO[1])
        self.declare_parameter("origen_yaw", ORIGEN_POR_DEFECTO[2])

        self.v = self.get_parameter("velocidad").value
        self.k = self.get_parameter("k_rumbo").value
        self.tol = self.get_parameter("tolerancia").value
        self.vueltas = self.get_parameter("vueltas").value
        self.max_giro = self.get_parameter("max_giro").value
        self.freno = self.get_parameter("freno_lidar").value
        self.umbral_giro = self.get_parameter("umbral_giro").value

        self.fuente = self.get_parameter("fuente").value
        if self.fuente not in ("verdad", "odom"):
            raise ValueError("fuente tiene que ser 'verdad' u 'odom'")

        # en modo "verdad" los waypoints ya estan en coordenadas del mundo
        if self.fuente == "verdad":
            ox = oy = oyaw = 0.0
        else:
            ox = self.get_parameter("origen_x").value
            oy = self.get_parameter("origen_y").value
            oyaw = self.get_parameter("origen_yaw").value
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        self.waypoints = [
            (c * (wx - ox) - s * (wy - oy), s * (wx - ox) + c * (wy - oy))
            for wx, wy in generar_carril(ESQUINAS, RADIO_ESQUINA, PASO)
        ]

        self.idx = 0
        self.vuelta = 0
        self.pose = None
        self.dist_frontal = float("inf")
        self.lado_libre = 1.0             # +1 izquierda, -1 derecha
        self.retrocediendo_hasta = None   # ns; None = marcha normal

        self.pub = self.create_publisher(Twist, "/robot/cmd_vel", 10)
        topic = TOPIC_VERDAD if self.fuente == "verdad" else TOPIC_ODOM
        self.create_subscription(Odometry, topic, self.on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan,
                                 qos_profile_sensor_data)
        self.create_timer(0.05, self.control)

        self.get_logger().info(
            f"Anillo con {len(self.waypoints)} waypoints, "
            f"{self.vueltas or 'infinitas'} vuelta(s) a {self.v} m/s")
        self.get_logger().info(
            f"Siguiendo la ruta con {topic} "
            f"({'pose real de Gazebo' if self.fuente == 'verdad' else 'odometria de ruedas, va a derivar'})")

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def on_scan(self, msg):
        """Distancia libre delante, y hacia que lado conviene escapar."""
        if not msg.ranges:
            return
        n = len(msg.ranges)
        centro = int((0.0 - msg.angle_min) / msg.angle_increment) % n
        ancho = int(n * (0.44 / (2 * math.pi)))          # +-25 grados

        def libre(desde, hasta):
            vals = [msg.ranges[(centro + k) % n] for k in range(desde, hasta)]
            vals = [v for v in vals if msg.range_min < v < msg.range_max]
            return min(vals) if vals else float("inf")

        self.dist_frontal = libre(-ancho, ancho + 1)
        # comparamos los dos costados para saber por donde salir
        izq = libre(ancho, 3 * ancho)
        der = libre(-3 * ancho, -ancho)
        self.lado_libre = 1.0 if izq > der else -1.0

    def parar(self):
        self.pub.publish(Twist())

    def control(self):
        if self.pose is None:
            return

        x, y, yaw = self.pose

        # avanzar tantos waypoints como haga falta (si vamos cortos de
        # trayectoria no queremos volver hacia atras a por uno que ya pasamos)
        for _ in range(len(self.waypoints)):
            gx, gy = self.waypoints[self.idx]
            if math.hypot(gx - x, gy - y) >= self.tol:
                break
            self.idx = (self.idx + 1) % len(self.waypoints)
            if self.idx == 0:
                self.vuelta += 1
                self.get_logger().info(
                    f"*** VUELTA {self.vuelta} COMPLETA -> aqui es donde el "
                    f"SLAM deberia cerrar el bucle ***")
                if self.vueltas and self.vuelta >= self.vueltas:
                    self.get_logger().info("Recorrido terminado. Parando.")
                    self.parar()
                    rclpy.shutdown()
                    return

        gx, gy = self.waypoints[self.idx]
        err = math.atan2(gy - y, gx - x) - yaw
        err = math.atan2(math.sin(err), math.cos(err))

        ahora = self.get_clock().now().nanoseconds
        cmd = Twist()

        # Anticolision: mejor retroceder y avisar que empotrarse en silencio.
        # Un choque hace patinar las ruedas, la odometria cuenta metros que el
        # robot no ha recorrido y el mapa sale doblado sin que se sepa por que.
        if (self.freno and self.dist_frontal < self.freno
                and self.retrocediendo_hasta is None):
            self.retrocediendo_hasta = ahora + int(2.5e9)
            self.get_logger().warn(
                f"Obstaculo a {self.dist_frontal:.2f} m: marcha atras. "
                f"Si esto pasa en mitad del pasillo, el robot se ha desviado "
                f"del carril.")

        if self.retrocediendo_hasta is not None:
            if ahora < self.retrocediendo_hasta:
                # Marcha atras girando hacia el lado MAS DESPEJADO. Girar
                # hacia el waypoint no sirve: si el obstaculo esta justo en
                # medio, el robot vuelve a chocar una y otra vez.
                cmd.linear.x = -0.35
                cmd.angular.z = -self.lado_libre * self.max_giro * 0.8
                self.pub.publish(cmd)
                return
            self.retrocediendo_hasta = None

        # Control para skid-steer: si el rumbo esta muy desviado, GIRA SOBRE
        # SI MISMO y solo despues avanza. Con un seguidor "avanza y corrige a
        # la vez" el Origin One se pasa de largo en las curvas (patina de
        # lado) y termina rozando la pared exterior. Girar parado es mas lento
        # pero no falla, y ademas se parece a como se mueve de verdad un robot
        # de almacen.
        cmd.linear.x = (0.0 if abs(err) > self.umbral_giro
                        else self.v * math.cos(err))
        cmd.angular.z = max(-self.max_giro, min(self.max_giro, self.k * err))
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = RecorridoBucle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.parar()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
