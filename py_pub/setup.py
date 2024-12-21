from setuptools import find_packages, setup

package_name = 'py_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['package.xml']),
        ('resource', ['resource/bbox_attrs.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='isla',
    maintainer_email='isla@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = py_pub.publisher_member_function:main',
                'converter = py_pub.converter:main'
        ],
},
)
