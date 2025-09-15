from setuptools import find_packages, setup

package_name = 'raspberry_exemples'

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
    maintainer='raspberrypi',
    maintainer_email='raspberrypi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_pub = raspberry_exemples.publisher_hello:main',
            'pca9685_test = raspberry_exemples.pca9685_node_simple:main',
            'encoder_test = raspberry_exemples.encoder_node_simples:main'
        ],
    },
)
