from setuptools import find_packages, setup

package_name = 'simple_commander'

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
    maintainer='analog',
    maintainer_email='laurentiu.popa@analog.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo_run = simple_commander.demo_run:main',
            'waypoint_follower = simple_commander.waypoint_follower:main',
            'elevator_server = simple_commander.elevator_server:main',
        ],
    },
)