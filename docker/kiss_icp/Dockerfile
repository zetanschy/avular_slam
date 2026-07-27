# ---------------------------------------------------------------------------
# KISS-ICP para ROS 2 Humble.
#
# No hay paquete apt ni imagen oficial, asi que lo compilamos. Son unos
# minutos la primera vez; despues queda en cache.
#
#   docker compose build kiss-icp
#
# Por que lo metemos en la clase: KISS-ICP es SOLO odometria LiDAR por ICP.
# Ni IMU, ni grafo de poses, ni cierre de bucle. Poner su trayectoria al
# lado de la de GLIM sobre la MISMA nube de puntos es la forma mas clara de
# ver que aporta el backend de un SLAM de verdad.
# ---------------------------------------------------------------------------
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        build-essential \
        libeigen3-dev \
        libtbb-dev \
        python3-pip \
        python3-colcon-common-extensions \
        ros-humble-tf2-ros \
        ros-humble-nav-msgs \
        ros-humble-sensor-msgs \
        ros-humble-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

# CMake nuevo. El de Ubuntu 22.04 es el 3.22, y Sophus (que KISS-ICP se
# descarga solo con FetchContent) pide 3.24 o superior:
#   CMake Error ... cmake_minimum_required: CMake 3.24 or higher is required.
# Se instala por pip porque queda en /usr/local/bin, que va antes en el PATH.
RUN pip3 install --no-cache-dir "cmake>=3.28,<4" && cmake --version

WORKDIR /ws/src
# Fijamos la version: si el repo cambia de API, la clase del ano que viene
# seguiria compilando.
ARG KISS_ICP_REF=main
RUN git clone --recursive https://github.com/PRBonn/kiss-icp.git \
    && cd kiss-icp && git checkout ${KISS_ICP_REF}

WORKDIR /ws
RUN source /opt/ros/humble/setup.bash \
    && colcon build --packages-select kiss_icp \
       --cmake-args -DCMAKE_BUILD_TYPE=Release

# El entrypoint de la imagen ros: ya hace source de /opt/ros/humble.
# Anadimos nuestro workspace.
RUN echo 'source /ws/install/setup.bash' >> /ros_entrypoint.sh.tmp \
    && head -n -1 /ros_entrypoint.sh > /tmp/e \
    && cat /tmp/e /ros_entrypoint.sh.tmp > /ros_entrypoint.sh \
    && echo 'exec "$@"' >> /ros_entrypoint.sh \
    && chmod +x /ros_entrypoint.sh
