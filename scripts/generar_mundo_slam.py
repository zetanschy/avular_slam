#!/usr/bin/env python3
"""
Genera el mundo de la Clase 5 (SLAM): worlds/slam_world.sdf + sus texturas.

Por que un mundo nuevo y no el warehouse de clase 2:

  1. LOOP CLOSURE. El mundo es un anillo cerrado alrededor de un bloque
     central: el robot sale, da la vuelta y VUELVE al mismo sitio. Sin un
     bucle no hay cierre de bucle que ensenar, solo odometria.
  2. TEXTURA. Las paredes lisas de Gazebo no tienen esquinas -> ORB no
     detecta nada -> el SLAM visual "no funciona" y parece un bug nuestro.
     Aqui el suelo y las paredes van texturizados y hay posters distintos
     en cada tramo (tambien sirven de "place recognition" para RTAB-Map).
  3. ESTRUCTURA 3D. Columnas y cajas de distintas alturas para que el
     LiDAR 3D vea algo mas que un plano (GLIM necesita geometria en Z,
     si no el mapa se desliza verticalmente).
  4. El plugin Imu del mundo: sin el, el sensor IMU del URDF no publica.

Uso:
    python3 scripts/generar_mundo_slam.py
"""

import math
import os
import random

from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEX_DIR = os.path.join(HERE, "media", "textures")
WORLD = os.path.join(HERE, "worlds", "slam_world.sdf")

# ------------------------------------------------------------------ arena
ARENA_X = 24.0          # largo total
ARENA_Y = 18.0          # ancho total
WALL_H = 2.5
WALL_T = 0.2
# Bloque central. Los pasillos salen de (ARENA - BLOCK)/2 y tienen que ser
# ANCHOS: la odometria de un skid-steer deriva del orden de 3 m por vuelta
# (patina de lado en cada giro, es inherente a la traccion), asi que el robot
# no va por donde cree. Con pasillos de 6 m aguanta dos vueltas sin rozar.
BLOCK_X = 12.0          # -> pasillo de (24-12)/2 = 6 m
BLOCK_Y = 6.0           # -> pasillo de (18-6)/2  = 6 m
TILE = 3.0              # lado de las baldosas del suelo

N_FLOOR_TEX = 6
N_WALL_TEX = 3
N_POSTER = 10


# ====================================================================
#  TEXTURAS
# ====================================================================
def _noise(draw, size, rng, base, spread, step):
    """Manchas de alta frecuencia: es lo que hace que ORB/GFTT encuentren
    esquinas. Sin esto la pared es un color plano y no hay features."""
    for y in range(0, size, step):
        for x in range(0, size, step):
            c = tuple(
                max(0, min(255, b + rng.randint(-spread, spread))) for b in base
            )
            draw.rectangle([x, y, x + step, y + step], fill=c)


def textura_suelo(idx, size=512):
    rng = random.Random(1000 + idx)
    base = [(120, 118, 112), (105, 110, 118), (128, 120, 105),
            (110, 108, 104), (118, 124, 118), (125, 115, 118)][idx % 6]
    img = Image.new("RGB", (size, size), base)
    d = ImageDraw.Draw(img)
    _noise(d, size, rng, base, 22, 8)
    # junta de la baldosa: borde oscuro -> linea recta que el LiDAR no ve
    # pero la camara si (util para explicar que cada sensor "ve" distinto)
    d.rectangle([0, 0, size - 1, size - 1], outline=(60, 58, 55), width=10)
    for _ in range(40):
        x, y = rng.randint(0, size), rng.randint(0, size)
        r = rng.randint(4, 18)
        c = tuple(max(0, b - rng.randint(20, 60)) for b in base)
        d.ellipse([x - r, y - r, x + r, y + r], fill=c)
    return img


def textura_pared(idx, size=512):
    rng = random.Random(2000 + idx)
    base = [(196, 190, 178), (178, 186, 192), (188, 182, 190)][idx % 3]
    img = Image.new("RGB", (size, size), base)
    d = ImageDraw.Draw(img)
    _noise(d, size, rng, base, 14, 16)
    # ladrillos
    bh = size // 16
    for row in range(16):
        off = (row % 2) * (size // 16)
        y0 = row * bh
        d.line([0, y0, size, y0], fill=(150, 145, 138), width=3)
        for col in range(9):
            x = col * (size // 8) + off
            d.line([x, y0, x, y0 + bh], fill=(150, 145, 138), width=3)
    return img


def textura_poster(idx, size=512):
    """Cada poster es unico: es la "huella" visual que permite a RTAB-Map
    reconocer que ya estuvo aqui cuando cierra el bucle."""
    rng = random.Random(3000 + idx)
    bg = (rng.randint(95, 165), rng.randint(95, 165), rng.randint(95, 165))
    img = Image.new("RGB", (size, size), bg)
    d = ImageDraw.Draw(img)
    for _ in range(rng.randint(18, 30)):
        c = (rng.randint(20, 255), rng.randint(20, 255), rng.randint(20, 255))
        kind = rng.choice(["rect", "ellipse", "tri", "line"])
        x0, y0 = rng.randint(0, size - 60), rng.randint(0, size - 60)
        w, h = rng.randint(40, 200), rng.randint(40, 200)
        if kind == "rect":
            d.rectangle([x0, y0, x0 + w, y0 + h], fill=c)
        elif kind == "ellipse":
            d.ellipse([x0, y0, x0 + w, y0 + h], fill=c)
        elif kind == "tri":
            d.polygon([(x0, y0 + h), (x0 + w // 2, y0), (x0 + w, y0 + h)], fill=c)
        else:
            d.line([x0, y0, x0 + w, y0 + h], fill=c, width=rng.randint(4, 14))
    # numero grande del poster (referencia visual para la clase)
    d.rectangle([size // 2 - 90, size // 2 - 90, size // 2 + 90, size // 2 + 90],
                fill=(250, 250, 250))
    d.text((size // 2 - 12, size // 2 - 12), str(idx), fill=(10, 10, 10))
    d.rectangle([0, 0, size - 1, size - 1], outline=(250, 250, 250), width=12)
    return img


def generar_texturas():
    os.makedirs(TEX_DIR, exist_ok=True)
    for i in range(N_FLOOR_TEX):
        textura_suelo(i).save(os.path.join(TEX_DIR, f"suelo_{i:02d}.png"))
    for i in range(N_WALL_TEX):
        textura_pared(i).save(os.path.join(TEX_DIR, f"pared_{i:02d}.png"))
    for i in range(N_POSTER):
        textura_poster(i).save(os.path.join(TEX_DIR, f"poster_{i:02d}.png"))
    print(f"  texturas -> {TEX_DIR}")


# ====================================================================
#  SDF
# ====================================================================
def material(tex=None, rgba="0.8 0.8 0.8 1"):
    if tex is None:
        return f"""          <material>
            <ambient>{rgba}</ambient>
            <diffuse>{rgba}</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>"""
    # OJO con la ruta de la textura. En Gazebo Fortress:
    #   media/textures/x.png      -> NO resuelve (silenciosamente, sin error:
    #   ../media/textures/x.png      la pared sale blanca y el SLAM visual
    #                                "no encuentra features")
    #   model://<paquete>/...     -> SI resuelve, buscando <paquete> dentro
    #                                de IGN_GAZEBO_RESOURCE_PATH.
    # Por eso simulation.launch.py mete el padre del share/ en esa variable.
    return f"""          <material>
            <diffuse>1.0 1.0 1.0</diffuse>
            <specular>0.15 0.15 0.15</specular>
            <pbr>
              <metal>
                <albedo_map>model://avular_slam/media/textures/{tex}</albedo_map>
                <metalness>0.0</metalness>
                <roughness>0.9</roughness>
              </metal>
            </pbr>
          </material>"""


def caja(name, x, y, z, sx, sy, sz, yaw=0.0, tex=None, rgba="0.8 0.8 0.8 1",
         collision=True):
    col = f"""        <collision name="col">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
""" if collision else ""
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
      <link name="link">
{col}        <visual name="vis">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
{material(tex, rgba)}
        </visual>
      </link>
    </model>
"""


def cilindro(name, x, y, z, r, h, tex=None, rgba="0.7 0.7 0.75 1"):
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
        </collision>
        <visual name="vis">
          <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
{material(tex, rgba)}
        </visual>
      </link>
    </model>
"""


def generar_mundo():
    rng = random.Random(7)
    p = []

    # ---------------------------------------------------------- suelo
    # Colision: un unico plano. Visual: baldosas texturizadas (una textura
    # estirada sobre 24x18 m se ve borrosa y no da features).
    p.append("""    <model name="suelo_colision">
      <static>true</static>
      <link name="link">
        <collision name="col">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
        </collision>
      </link>
    </model>
""")
    nx = int(ARENA_X / TILE)
    ny = int(ARENA_Y / TILE)
    for i in range(nx):
        for j in range(ny):
            x = -ARENA_X / 2 + TILE * (i + 0.5)
            y = -ARENA_Y / 2 + TILE * (j + 0.5)
            p.append(caja(f"baldosa_{i}_{j}", x, y, 0.01, TILE, TILE, 0.02,
                          yaw=rng.choice([0.0, math.pi / 2]),
                          tex=f"suelo_{rng.randrange(N_FLOOR_TEX):02d}.png",
                          collision=False))

    # ------------------------------------------------- paredes exteriores
    hx, hy = ARENA_X / 2, ARENA_Y / 2
    muros = [
        ("muro_norte", 0, hy, ARENA_X, WALL_T),
        ("muro_sur", 0, -hy, ARENA_X, WALL_T),
        ("muro_este", hx, 0, WALL_T, ARENA_Y),
        ("muro_oeste", -hx, 0, WALL_T, ARENA_Y),
    ]
    for k, (name, x, y, sx, sy) in enumerate(muros):
        p.append(caja(name, x, y, WALL_H / 2, sx, sy, WALL_H,
                      tex=f"pared_{k % N_WALL_TEX:02d}.png"))

    # --------------------------------------------------- bloque central
    # Es lo que convierte el mundo en un ANILLO -> el robot puede volver
    # al punto de partida y disparar el cierre de bucle.
    bx, by = BLOCK_X / 2, BLOCK_Y / 2
    bloque = [
        ("bloque_norte", 0, by, BLOCK_X, WALL_T),
        ("bloque_sur", 0, -by, BLOCK_X, WALL_T),
        ("bloque_este", bx, 0, WALL_T, BLOCK_Y),
        ("bloque_oeste", -bx, 0, WALL_T, BLOCK_Y),
    ]
    for k, (name, x, y, sx, sy) in enumerate(bloque):
        p.append(caja(name, x, y, WALL_H / 2, sx, sy, WALL_H,
                      tex=f"pared_{(k + 1) % N_WALL_TEX:02d}.png"))

    # ------------------------------------------------------------ posters
    # Uno cada pocos metros, a la altura de la camara (1.2 m).
    posters = []
    for i, x in enumerate([-5.0, 0.0, 5.0]):                     # bloque N/S
        posters.append((x, by - WALL_T / 2 - 0.03, 0.0))
        posters.append((x, -by + WALL_T / 2 + 0.03, math.pi))
    for i, y in enumerate([-2.0, 2.0]):                          # bloque E/O
        posters.append((bx - WALL_T / 2 - 0.03, y, -math.pi / 2))
        posters.append((-bx + WALL_T / 2 + 0.03, y, math.pi / 2))
    for i, (x, y, yaw) in enumerate(posters):
        p.append(caja(f"poster_{i:02d}", x, y, 1.2, 0.02, 1.4, 1.0, yaw=yaw,
                      tex=f"poster_{i % N_POSTER:02d}.png", collision=False))

    # posters en la pared exterior (para que la camara vea algo tambien
    # cuando mira "hacia afuera" en las curvas)
    ext = [(-8.0, -hy + WALL_T / 2 + 0.03, 0.0),
           (8.0, -hy + WALL_T / 2 + 0.03, 0.0),
           (-8.0, hy - WALL_T / 2 - 0.03, math.pi),
           (8.0, hy - WALL_T / 2 - 0.03, math.pi),
           (hx - WALL_T / 2 - 0.03, 0.0, -math.pi / 2),
           (-hx + WALL_T / 2 + 0.03, 0.0, math.pi / 2)]
    for i, (x, y, yaw) in enumerate(ext):
        p.append(caja(f"poster_ext_{i:02d}", x, y, 1.3, 0.02, 1.6, 1.1, yaw=yaw,
                      tex=f"poster_{(i + 4) % N_POSTER:02d}.png", collision=False))

    # ------------------------------------------- estructura 3D (columnas)
    # Alturas distintas: sin variacion en Z, el mapa 3D "resbala".
    #
    # CUIDADO AL MOVER ESTO: el carril por donde circula el robot es el
    # rectangulo x = +-9.5, y = +-6.5 (ver recorrido_bucle.py). Todo lo que
    # se ponga a menos de ~1.2 m de ese rectangulo se lo lleva por delante.
    # La primera version de este mundo tenia las columnas en (+-10, +-6.5),
    # o sea justo encima del carril: el robot chocaba a los 3 segundos y
    # patinaba, la odometria contaba 14 m mientras el robot llevaba 2.7 m
    # recorridos, y el mapa salia torcido. Muy buen ejemplo de que un mapa
    # feo casi nunca es culpa del algoritmo de SLAM.
    #
    # Zonas libres: entre el carril y la pared exterior (|y| ~ 8.2) y entre
    # el carril y el bloque central (|y| ~ 4.8 / |x| ~ 7.8).
    columnas = [(-11.2, -8.3, 0.30, 2.5), (11.2, -8.3, 0.28, 1.8),
                (-11.2, 8.3, 0.30, 2.2), (11.2, 8.3, 0.28, 2.5),
                (0.0, -8.4, 0.25, 1.2), (0.0, 8.4, 0.25, 1.6),
                (-3.5, -3.6, 0.22, 2.0), (3.5, 3.6, 0.22, 1.4)]
    obstaculos = []
    for i, (x, y, r, h) in enumerate(columnas):
        obstaculos.append((f"columna_{i}", x, y))
        p.append(cilindro(f"columna_{i}", x, y, h / 2, r, h,
                          tex=f"pared_{i % N_WALL_TEX:02d}.png"))

    # cajas apiladas: obstaculos que el LiDAR 2D ve a 0.35 m de altura
    # y el LiDAR 3D ve completos
    # Mismo cuidado que con las columnas: todas fuera del carril.
    cajas = [(-6.0, -8.5, 0.8, 0.9, 0.9), (6.5, -8.4, 0.5, 1.2, 0.6),
             (-6.5, 8.5, 0.6, 0.8, 1.4), (6.0, 8.4, 0.7, 0.7, 0.7),
             (-11.5, 0.0, 0.6, 0.6, 1.8), (11.5, 2.0, 0.5, 0.9, 1.0),
             (11.5, -2.5, 0.6, 0.8, 0.5), (-6.6, 1.5, 0.5, 0.7, 1.1),
             (6.6, -1.5, 0.5, 0.7, 0.8)]
    for i, (x, y, sx, sy, sz) in enumerate(cajas):
        obstaculos.append((f"caja_{i}", x, y))
        p.append(caja(f"caja_{i}", x, y, sz / 2, sx, sy, sz,
                      yaw=rng.uniform(0, 1.5),
                      tex=f"poster_{(i + 2) % N_POSTER:02d}.png"))

    comprobar_carril(obstaculos)

    cuerpo = "".join(p)

    sdf = f"""<?xml version="1.0"?>
<!-- ============================================================
     GENERADO por scripts/generar_mundo_slam.py - no editar a mano
     Clase 5 - SLAM. Arena {ARENA_X:.0f} x {ARENA_Y:.0f} m con bloque
     central: circuito cerrado para demostrar cierre de bucle.
     ============================================================ -->
<sdf version="1.8">
  <world name="slam_world">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="libignition-gazebo-physics-system.so"
            name="ignition::gazebo::systems::Physics"/>
    <plugin filename="libignition-gazebo-user-commands-system.so"
            name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so"
            name="ignition::gazebo::systems::SceneBroadcaster"/>
    <!-- OJO: aqui NO van los plugins Sensors ni Imu.
         El robot Origin One los trae en su propio URDF (a nivel de modelo),
         y cargar dos sistemas Sensors en el mismo mundo crea dos escenas de
         render. Si algun dia usais este mundo con OTRO robot que no los
         traiga, hay que anadirlos aqui o los sensores no publican nada:

           <plugin filename="libignition-gazebo-sensors-system.so"
                   name="ignition::gazebo::systems::Sensors">
             <render_engine>ogre2</render_engine>
           </plugin>
           <plugin filename="libignition-gazebo-imu-system.so"
                   name="ignition::gazebo::systems::Imu"/>
    -->

    <gravity>0 0 -9.8</gravity>
    <magnetic_field>5.565e-06 2.289e-05 -4.239e-05</magnetic_field>
    <atmosphere type="adiabatic"/>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.75 0.80 0.88 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sol">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
      <attenuation>
        <range>100</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.4 0.3 -0.9</direction>
    </light>

    <light type="point" name="lampara_1">
      <pose>-7 0 2.4 0 0 0</pose>
      <diffuse>0.6 0.6 0.55 1</diffuse>
      <attenuation><range>18</range><constant>0.4</constant><linear>0.05</linear></attenuation>
    </light>
    <light type="point" name="lampara_2">
      <pose>7 0 2.4 0 0 0</pose>
      <diffuse>0.6 0.6 0.55 1</diffuse>
      <attenuation><range>18</range><constant>0.4</constant><linear>0.05</linear></attenuation>
    </light>

{cuerpo}
  </world>
</sdf>
"""
    os.makedirs(os.path.dirname(WORLD), exist_ok=True)
    with open(WORLD, "w") as f:
        f.write(sdf)
    print(f"  mundo    -> {WORLD}")


def comprobar_carril(obstaculos, margen=2.0):
    """Ningun obstaculo puede estar encima del carril del robot.

    El carril es el rectangulo x = +-9.0, y = +-6.0 que recorre
    recorrido_bucle.py. Esta comprobacion existe porque la primera version
    del mundo tenia columnas justo en el carril y el robot chocaba a los
    tres segundos.
    """
    lx, ly = 9.0, 6.0

    def distancia(x, y):
        d = []
        for cx in (-lx, lx):
            d.append(abs(x - cx) if -ly <= y <= ly
                     else math.hypot(x - cx, abs(y) - ly))
        for cy in (-ly, ly):
            d.append(abs(y - cy) if -lx <= x <= lx
                     else math.hypot(abs(x) - lx, y - cy))
        return min(d)

    malos = [(n, x, y, round(distancia(x, y), 2))
             for n, x, y in obstaculos if distancia(x, y) < margen]
    if malos:
        raise SystemExit(
            f"ERROR: {len(malos)} obstaculo(s) invaden el carril del robot "
            f"(margen {margen} m):\n  " +
            "\n  ".join(f"{n} en ({x}, {y}), a {d} m" for n, x, y, d in malos))
    print(f"  carril   -> libre ({len(obstaculos)} obstaculos comprobados)")


if __name__ == "__main__":
    print("Generando mundo de la clase de SLAM...")
    generar_texturas()
    generar_mundo()
    print("Listo. Recuerden hacer colcon build para copiarlo al share/.")
