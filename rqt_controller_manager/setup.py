from glob import glob

from setuptools import setup

package_name = 'rqt_controller_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/resource", glob("resource/*.*")),
        ("share/" + package_name, ["plugin.xml"]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ijnek',
    maintainer_email='kenjibrameld@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rqt_controller_manager = \
                rqt_controller_manager.main:main",
        ],
    },
)
