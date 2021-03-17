from setuptools import setup
import os
from glob import glob
package_name = 'tc3_ros_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('tc3_ros_package/*.crt')),
        (os.path.join('share', package_name), glob('mcx_tracking_cam_pb2.py')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rodion',
    maintainer_email='rodion_anisimov@mail.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tc3_node = tc3_ros_package.tc3_node:main',
            'mcx = tc3_ros_package.mcx_tracking_cam_pb2'
        ],
    },
)
