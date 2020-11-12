from setuptools import find_packages
from setuptools import setup

package_name = 'ros2controlcli'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Victor Lopez',
    author_email='victor.lopez@pal-robotics.com',
    maintainer='Victor Lopez',
    maintainer_email='victor.lopez@pal-robotics.com',
    url='https://github.com/ros-controls/ros2_control',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='ROS2 Control command  interface.',
    long_description="""\
ROS2 Control command  interface.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'control = ros2controlcli.command.control:ControlCommand',
        ],
        'ros2controlcli.verb': [
            'list = ros2controlcli.verb.list:ListVerb',
            'list_interfaces = ros2controlcli.verb.list_interfaces:ListInterfacesVerb',
            'list_types = ros2controlcli.verb.list_types:ListTypesVerb',
            'load = ros2controlcli.verb.load:LoadVerb',
            'reload_libraries = ros2controlcli.verb.reload_libraries:ReloadLibrariesVerb',
            'switch = ros2controlcli.verb.switch:SwitchVerb',
            'unload = ros2controlcli.verb.unload:UnloadVerb',
        ],
    }
)
