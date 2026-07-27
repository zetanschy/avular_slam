# avular_slam — Clase 5

Tres algoritmos de SLAM sobre el **Avular Origin One** (el robot de la clase 3),
en un mundo con circuito cerrado para poder mostrar el cierre de bucle.

| Demo | Sensor | Algoritmo | Cómo se ejecuta |
|---|---|---|---|
| SLAM 2D | LiDAR 2D (`/scan`) | `slam_toolbox` | apt, nativo |
| SLAM 3D | Ouster OS1-32 + IMU | **GLIM** | Docker |
| SLAM 3D sin backend | Ouster OS1-32 | **KISS-ICP** | Docker (se compila) |
| SLAM visual | RealSense D435 RGB-D | **RTAB-Map** | Docker |

Probado en **ROS 2 Humble + Gazebo Fortress** (`ign gazebo` 6.16).

Este paquete **no modifica nada** del paquete de Avular: usa su URDF tal cual y
le aplica los ajustes que necesita el SLAM en memoria, al lanzar (ver §6).

---

## 1. Requisitos

El robot viene de los repos de Avular, los mismos que usan en la clase 3:

```
~/ros2_ws/src/avular_origin_description     (origin_one_description)
~/ros2_ws/src/avular_origin_simulation      (origin_one_gazebo)
```

Además:

```bash
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-map-server
docker pull koide3/glim_ros2:humble          # 2.4 GB
docker pull introlab3it/rtabmap_ros:humble   # 4.0 GB
```

Instalación del paquete:

```bash
cp -r avular_slam ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select avular_slam --symlink-install
source install/setup.bash
```

---

## 2. Simulación base

```bash
ros2 launch avular_slam simulation.launch.py
```

Revisen **siempre** esto antes de pelearse con un SLAM:

```bash
ros2 run avular_slam chequeo_slam
```

Sale una tabla con las 9 fuentes de datos y las 5 transformadas que hacen
falta. Si algo aparece en rojo, el mismo programa dice qué revisar.

Para que el robot dé vueltas solo mientras ustedes explican:

```bash
ros2 run avular_slam recorrido_bucle --ros-args -p vueltas:=2
```

Argumentos del launch:

| Argumento | Por defecto | Para qué |
|---|---|---|
| `world` | slam_world.sdf | El circuito con texturas |
| `imu_rate` | 200 | Hz de la IMU (Avular trae 25) |
| `camera_rate` | 15 | Hz de la cámara (Avular trae 6) |
| `headless` | false | Sin ventana de Gazebo |
| `rviz_config` | sensores.rviz | `slam_2d` \| `slam_3d` \| `visual_slam` |
| `odom_tf` | true | Publicar odom→base_link (false para odometría visual) |

---

## 3. Demo 1 — SLAM 2D (slam_toolbox)

```bash
# terminal 1
ros2 launch avular_slam simulation.launch.py rviz_config:=slam_2d.rviz
# terminal 2
ros2 launch avular_slam slam_2d.launch.py
# terminal 3
ros2 run avular_slam recorrido_bucle
```

Guardar el mapa:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/mapa_clase5 --ros-args -p use_sim_time:=true
```

**La demo que mejor se ve** es correrlo dos veces, con y sin cierre de bucle:

```bash
ros2 launch avular_slam slam_2d.launch.py do_loop_closing:=false   # el mapa NO cierra
ros2 launch avular_slam slam_2d.launch.py                          # el mapa SÍ cierra
```

---

## 4. Demo 2 — SLAM 3D (GLIM)

```bash
xhost +local:docker                       # una vez por sesión
cd ~/ros2_ws/src/avular_slam/docker
./glim/preparar_config.sh                 # una vez: extrae la config de la imagen
docker compose up glim
```

```bash
# OJO: odom_tf:=false. GLIM publica el árbol completo map → odom → base_link,
# o sea que REEMPLAZA a la odometría de ruedas. Si Gazebo publica también
# odom → base_link, ese frame queda con dos padres y el árbol TF se rompe.
ros2 launch avular_slam simulation.launch.py rviz_config:=slam_3d.rviz odom_tf:=false
```

Con GPU NVIDIA:

```bash
./glim/preparar_config.sh koide3/glim_ros2:humble_cuda12.2
docker compose up glim-gpu
```

Comparación con odometría LiDAR pura (sin IMU, sin grafo, sin cierre de bucle):

```bash
docker compose build kiss-icp      # unos minutos la primera vez
docker compose up kiss-icp
```

Publica su trayectoria en `/kiss/odometry`. No publica TF a propósito
(`publish_odom_tf:=false`): solo queremos su trayectoria para ponerla al lado
de la de GLIM. Al arrancar avisa algo que vale la pena leer en voz alta:

```
Field 't', 'timestamp', 'time_stamp', or 'time' does not exist.
Disabling scan deskewing
```

La nube de Gazebo no trae tiempo por punto, así que no hay forma de corregir
la distorsión de movimiento. Un LiDAR real sí lo trae: en simulación el mapa
3D sale **mejor** de lo que saldría en el robot de verdad.

GLIM publica sus resultados bajo `/glim_ros/...` (nube acumulada en
`/glim_ros/aligned_points`, trayectoria en `/glim_ros/odom`) y el árbol TF
`map → odom → base_link`. En RViz, *Fixed Frame* = `map`.

Un aviso que **va a aparecer** en el log de GLIM y que no es una falla:

```
IMU prediction is not good. Possibly T_lidar_imu is not accurate...
```

Es el chequeo interno de GLIM reclamando por la IMU simulada. Sus propios
números dicen que la IMU sí ayuda (error de rotación 0.022° con IMU contra
0.472° sin ella). Si la extrínseca estuviera realmente mal, esos números se
invertirían: es justo el experimento del punto 3.3 de la tarea.

### La extrínseca LiDAR-IMU de este robot

Está en `docker/glim/overrides.json`, calculada desde el URDF de Avular:

```
T_lidar_imu = [-0.3986, 0.0749, -0.1580,  0.0, 0.0, 0.9240, 0.3823]
                 x       y        z        qx   qy   qz      qw
```

La IMU está 40 cm por detrás del LiDAR, 16 cm por debajo, y **rotada 135° en
yaw**. Esa rotación es la que más daño hace si se pone mal: GLIM interpreta
los giros al revés y diverge apenas el robot rota. Es un ejercicio de la
tarea: pónganla en identidad y vean cómo se rompe.

---

## 5. Demo 3 — SLAM visual (RTAB-Map)

```bash
cd ~/ros2_ws/src/avular_slam/docker
docker compose up rtabmap
```

```bash
ros2 launch avular_slam simulation.launch.py rviz_config:=visual_slam.rviz
```

Esta variante usa la odometría de las ruedas y RTAB-Map se encarga del mapa y
del cierre de bucle visual.

### El filtro que rechaza los cierres buenos

Los servicios de `docker-compose.yml` pasan `--RGBD/OptimizeMaxError 0`. Sin
esa opción, RTAB-Map **detecta** los cierres de bucle y después los **rechaza**:

```
Loop closure 100->3 rejected!
... maximum graph error ratio of 12.27 ... "RGBD/OptimizeMaxError" is 3.0
```

Por defecto descarta un cierre si al optimizar el grafo el error supera 3
veces la desviación esperada. Es un filtro anti falsos positivos muy sensato,
pero la odometría del skid-steer deriva tanto que la corrección necesaria lo
dispara y se pierden también los cierres correctos. Vale la pena mostrarlo
primero **sin** la opción: un SLAM que "no funciona" y que en realidad está
haciendo lo correcto con datos malos.

Para que la odometría la calcule **la cámara**:

```bash
ros2 launch avular_slam simulation.launch.py odom_tf:=false
docker compose up rtabmap-vo
```

---

## 6. Qué le hacemos al robot de Avular (y por qué)

`simulation.launch.py` procesa el xacro de `origin_one_description` y **ajusta
el URDF en memoria** antes de hacer el spawn. No se toca ningún archivo de
Avular. Son cinco cambios, todos visibles en `preparar_urdf()`:

| Cambio | Motivo |
|---|---|
| IMU 25 Hz → 200 Hz | GLIM preintegra la IMU entre scans; a 25 Hz la integración queda demasiado gruesa |
| Cámara 6 Hz → 15 Hz | a 6 Hz la odometría visual pierde el seguimiento apenas el robot gira |
| `gz_frame_id` de la cámara → `camera_optical_link` (+ el link) | Gazebo entrega las imágenes en convención óptica (Z adelante) pero el URDF las etiqueta con `camera_link` (X adelante): RTAB-Map armaría el mapa rotado 90° |
| Se agrega el link `os_lidar` | el LiDAR 3D publica con `frame_id: os_lidar`, pero **ese link no existe** en el URDF de Avular: RViz no puede dibujar la nube |
| Se inyecta `OdometryPublisher` → `/odom_verdad` | verdad de campo para medir el error del SLAM (en un robot real haría falta un Vicon) |

Si algún día Avular cambia su URDF, el launch avisa por consola
(`AVISO: no encuentro ... en el URDF de Avular`) en vez de fallar en silencio.

---

## 7. Sensores

| Topic ROS | Tipo | Hz | Frame | Notas |
|---|---|---|---|---|
| `/scan` | LaserScan | 10 | `scan_link` | 360 rayos, 30 m. **No** viene en el puente de Avular |
| `/robot/lidar/points` | PointCloud2 | 10 | `os_lidar` | Ouster OS1-32, 1024×32, campos `x y z intensity ring` |
| `/imu` | Imu | 200 | `imu_link` | **No** viene en el puente de Avular |
| `/robot/camera/color/image_raw` | Image | 15 | `camera_optical_link` | 640×480 |
| `/robot/camera/depth/image_raw` | Image | 15 | `camera_optical_link` | 32FC1 |
| `/robot/camera/color/camera_info` | CameraInfo | 15 | | fx=554.25 cx=320.5 — **correcto** |
| `/robot/odom` | Odometry | 40 | `odom`→`base_link` | odometría de ruedas |
| `/odom_verdad` | Odometry | 30 | `mundo`→`base_verdad` | verdad de campo |

> Sobre el `camera_info`: acá está bien **porque el URDF de Avular declara
> `<lens><intrinsics>` de forma explícita**. Si no se declaran, Gazebo Fortress
> publica los intrínsecos por defecto (320×240, 60°) aunque la imagen salga en
> 640×480, y el SLAM visual arma el mapa con la escala mal sin dar ningún
> error. Vale la pena mostrarlo en clase como contraejemplo.

---

## 8. Tres trampas silenciosas

Ninguna da error. Todas hacen que el SLAM "salga mal" sin decir por qué.

**1. Texturas que no cargan.** En Fortress, una ruta relativa
(`media/textures/x.png`) en `<albedo_map>` no resuelve y Gazebo no avisa: las
paredes salen blancas, la cámara no ve esquinas y el SLAM visual parece roto.
Por eso el mundo usa `model://avular_slam/...` y el launch agrega el padre del
`share/` a `IGN_GAZEBO_RESOURCE_PATH`. Lo mismo con las mallas del robot: sin
el `share/` de `origin_one_description` en esa variable, el Origin One sale
invisible.

**2. Un mapa torcido casi nunca es culpa del algoritmo.** Si el robot choca y
patina, la odometría cuenta metros que no recorrió y el SLAM hace lo que
puede. Antes de tocar parámetros:

```bash
ign model -m origin_one -p            # dónde está de verdad
ros2 topic echo /robot/odom --once    # dónde cree que está
ros2 topic echo /odom_verdad --once   # la verdad, ya en ROS
```

El generador del mundo verifica solo que ningún obstáculo invada el carril del
robot, y `recorrido_bucle` frena y **avisa por consola** si se acerca demasiado
a algo, en vez de chocar en silencio.

**3. Dos nodos publicando la misma TF.** `odom → base_link` la publica Gazebo.
Si además la publica RTAB-Map (odometría visual), el árbol TF se rompe: un
frame no puede tener dos padres. De ahí el argumento `odom_tf`.

---

## 9. Estructura

```
avular_slam/
├── launch/
│   ├── simulation.launch.py     robot Avular + mundo + puentes + ajustes
│   └── slam_2d.launch.py        slam_toolbox
├── config/
│   ├── bridge_slam.yaml         puente Gazebo <-> ROS 2 (agrega /scan e /imu)
│   └── slam_toolbox_2d.yaml
├── worlds/slam_world.sdf        GENERADO, no editar a mano
├── media/textures/              GENERADAS
├── scripts/generar_mundo_slam.py
├── rviz/                        sensores | slam_2d | slam_3d | visual_slam
├── avular_slam/
│   ├── recorrido_bucle.py       da vueltas al circuito solo
│   └── chequeo_slam.py          diagnóstico previo
└── docker/
    ├── docker-compose.yml       glim | glim-gpu | rtabmap | rtabmap-vo | kiss-icp
    ├── glim/preparar_config.sh + overrides.json
    └── kiss_icp/Dockerfile
```
