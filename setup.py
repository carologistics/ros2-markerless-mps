from setuptools import setup
from glob import glob
import os

package_name = 'ros2_markerless_mps'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'model'), glob(os.path.join('model', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danielhonies',
    maintainer_email='daniel.honies@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_sniffer = ros2_markerless_mps.tf_sniffer:main',
            'inference_test = ros2_markerless_mps.inference_test:main',
            'image_saver = ros2_markerless_mps.image_saver:main',
            'map_navigator = ros2_markerless_mps.map_navigator:main',
        ],
    },
)
