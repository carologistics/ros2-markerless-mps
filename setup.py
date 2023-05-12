from setuptools import setup

package_name = 'hthesis3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'lib/{package_name}', ["hthesis3/pyrealsense2.cpython-311-x86_64-linux-gnu.so"]),
        (f'lib/{package_name}', ["hthesis3/librealsense2.so"])
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
            'tf_sniffer = hthesis3.tf_sniffer:main',
            'inference_test = hthesis3.inference_test:main'
        ],
    },
)
