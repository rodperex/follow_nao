from setuptools import find_packages, setup

package_name = 'follow_nao'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/2d_follow_nao.launch.py']),
        ('share/' + package_name + '/launch', ['launch/3d_follow_nao.launch.py']),
        ('share/' + package_name + '/launch', ['launch/track.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roi',
    maintainer_email='rodrigo.perez@urjc.es',
    description='VFF controller with attractive and repulsive vectors',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_control_3d = follow_nao.motion_control_3d:main',
            'motion_control_2d = follow_nao.motion_control:main',
            'yolo_to_standard = follow_nao.yolo_to_standard:main',
            'entity_tracker = follow_nao.entity_tracker:main',
        ],
    },
)
