import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mini_cheetah_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.dae'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wang',
    maintainer_email='1664486711@qq.com',
    description='Simplified Mini Cheetah Quadruped Robot RViz Demo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sine_joint_publisher = mini_cheetah_demo.sine_joint_publisher:main',
            'ik_gait_controller = mini_cheetah_demo.ik_gait_controller:main',
            'teleop_keyboard = mini_cheetah_demo.teleop_keyboard:main',
            'rl_gait_controller = mini_cheetah_demo.rl_gait_controller:main',
        ],
    },
)
