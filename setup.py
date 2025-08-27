from setuptools import find_packages, setup

package_name = 'power_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/power_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='winton',
    maintainer_email='davidcai918@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
         'console_scripts': [
            'power = power_nodes.power:main',
            'serial_bridge = power_nodes.serial_bridge:main',
        ],
    },
)
