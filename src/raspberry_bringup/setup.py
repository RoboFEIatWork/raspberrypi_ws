from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'raspberry_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raspberrypi',
    maintainer_email='raspberrypi@todo.todo',
    description='Bringup package for Caramelo robot on Raspberry Pi (robot description publication, control, PCA9685 motor interface).',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pca9685_interface = raspberry_bringup.pac9685_controller:main',
        ],
    },
)
