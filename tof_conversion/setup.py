import os
import glob

from setuptools import find_packages, setup

package_name = 'tof_conversion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Make the launch files available from source to install location
        (
            os.path.join("share", package_name),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='astanea',
    maintainer_email='Adrian.Stanea@analog.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "subscriberNode = tof_conversion.ConversionNode:main",
        ],
    },
)
