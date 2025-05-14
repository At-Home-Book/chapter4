from setuptools import find_packages, setup

package_name = 'yolov8_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jeffrey Too Chuan TAN',
    maintainer_email='at-home-book@googlegroups.com',
    description='YOLOv8 applications with ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection = yolov8_ros2.object_detection:main',
            'object_segmentation = yolov8_ros2.object_segmentation:main',
        ],
    },
)
