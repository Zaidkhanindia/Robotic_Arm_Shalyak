from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robotic_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/meshes', glob('meshes/*.stl')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zaid-khan',
    maintainer_email='zaidkhan.dewas@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "jointpublisher = robotic_arm.joint_pub_node:main",
            "gripperpub = robotic_arm.pickplace_node:main"
        ],
    },
)
