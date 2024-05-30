import os
from glob import glob
from setuptools import setup

package_name = 'my_controller'
launch_name = 'kalBotBringup_launch.py'
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
    maintainer='wisnu',
    maintainer_email='kusumawisnu1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odomTf = my_controller.odomTf:main',
            'publisherTest = my_controller.publisherTest:main',
            'driver = my_controller.driver:main',
            'sonar = my_controller.sonar:main'
        ],
    },
    #  data_files=[
    #     # ... Other data files
    #     # Include all launch files.
    #     (os.path.join('share', launch_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    # ]
)
