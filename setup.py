from setuptools import setup

package_name = 'bag_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pierce Nichols',
    maintainer_email='pierce@ladonrobotics.com',
    description='Package for turning on and off bag logging based on a mavros state message',
    license='Copyright Ladon Robotics (c) 2023. All Rights Reserved',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_control_node = bag_control.bag_control_node:main',
        ],
    },
)
