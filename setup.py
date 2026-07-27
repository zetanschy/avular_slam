from setuptools import setup
from glob import glob
import os

package_name = 'avular_slam'


def ficheros(patron):
    """glob() que descarta directorios.

    Hace falta porque data_files solo admite ficheros: si se le cuela una
    carpeta, colcon build revienta con

        error: can't copy 'docker/glim/dump': doesn't exist or not a
        regular file

    Y carpetas ahi dentro aparecen solas: docker/glim/config/ la crea
    preparar_config.sh, y docker/glim/dump/ y docker/rtabmap/db/ las crea
    Docker al montar los volumenes. O sea que el paquete compilaba hasta que
    alguien ejecutaba GLIM una vez.
    """
    return [f for f in glob(patron) if os.path.isfile(f)]

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ficheros('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), ficheros('worlds/*.sdf')),
        (os.path.join('share', package_name, 'rviz'), ficheros('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), ficheros('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), ficheros('scripts/*.py')),
        # Las texturas TIENEN que llegar al share: el mundo las busca como
        # model://avular_slam/media/textures/... via IGN_GAZEBO_RESOURCE_PATH.
        (os.path.join('share', package_name, 'media', 'textures'),
            ficheros('media/textures/*.png')),
        (os.path.join('share', package_name, 'docker'), ficheros('docker/*.yml')),
        (os.path.join('share', package_name, 'docker', 'glim'),
            ficheros('docker/glim/*')),
        (os.path.join('share', package_name, 'docker', 'kiss_icp'),
            ficheros('docker/kiss_icp/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='Clase 5 - SLAM 2D, 3D y visual sobre el Avular Origin One',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recorrido_bucle = avular_slam.recorrido_bucle:main',
            'chequeo_slam = avular_slam.chequeo_slam:main',
        ],
    },
)
