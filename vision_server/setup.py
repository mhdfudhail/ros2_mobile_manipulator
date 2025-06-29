from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fudhail',
    maintainer_email='mhmdfudhail@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "object_locator_server=vision_server.color_object_locator:main",
            "get_pose_server=vision_server.getpose_server:main",
            "get_pose_tf_server=vision_server.getpose_tf_server:main",
            "get_pose_client=vision_server.getpose_client:main",
            "test_server=vision_server.test_server:main",
            "object_tf=vision_server.object_tf:main",
        ],
    },
)
