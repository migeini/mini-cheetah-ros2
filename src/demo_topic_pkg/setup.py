import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'demo_topic_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wang',
    maintainer_email='1664486711@qq.com',
    description='ROS 2 topic pub/sub tutorial package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = demo_topic_pkg.demo_publisher:main',
            'subscriber_node = demo_topic_pkg.demo_subscriber:main',
        ],
    },
)
