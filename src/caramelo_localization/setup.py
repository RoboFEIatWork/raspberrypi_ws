from setuptools import find_packages, setup
from glob import glob

package_name = 'caramelo_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    # Optional launch and config folders (installed if present)
    ('share/' + package_name + '/launch', glob('launch/*.py')),
    ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linux24-04',
    maintainer_email='victoroliveiraayres@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kalman_filter = robot_localizacao.kalman_filter:main',
        ],
    },
)
