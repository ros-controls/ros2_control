from glob import glob

from setuptools import setup

package_name = 'rqt_controller_manager'

setup(
    name=package_name,
    version='2.36.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/resource", glob("resource/*.*")),
        ("share/" + package_name, ["plugin.xml"]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bence Magyar',
    maintainer_email='bence.magyar.robotics@gmail.com',
    description='Graphical frontend for interacting with the controller manager.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rqt_controller_manager = \
                rqt_controller_manager.main:main",
        ],
    },
)
