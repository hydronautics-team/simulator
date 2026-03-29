from setuptools import find_packages, setup

package_name = 'simulator_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'resource/bbox_attrs.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hydronautics Team',
    maintainer_email='team@hydronautics.local',
    description='Python nodes for simulator perception/control adapters',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manual_thruster_bridge_node = simulator_perception.manual_thruster_bridge_node:main',
            'simulator_perception_node = simulator_perception.simulator_perception_node:main',
        ],
    },
)
