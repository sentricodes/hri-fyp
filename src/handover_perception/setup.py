from setuptools import find_packages, setup

package_name = 'handover_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='conall',
    maintainer_email='conallodowd@gmail.com',
    description='Handover hand nodes and perception via MediaPipe',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mediapipe_hand_node = handover_perception.mediapipe_hand_node:main',
            'handover_zone_node = handover_perception.handover_zone_node:main',
            'palm_transform_node = handover_perception.palm_transform_node:main',
            'approach_pose_node = handover_perception.approach_pose_node:main',
        ],
    },
)