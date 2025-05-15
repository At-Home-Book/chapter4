from setuptools import find_packages, setup

package_name = 'vlm_ros2'

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
    maintainer='Jeffrey Tan',
    maintainer_email='at-home-book@googlegroups.com',
    description='Vision-Language Model applications with ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_recognition = vlm_ros2.image_recognition:main',
            'object_detection = vlm_ros2.object_detection:main',
        ],
    },
)
