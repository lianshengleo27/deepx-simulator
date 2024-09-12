import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'excavator_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'boom_cmd = excavator_controller.boom_cmd:main',
            'arm_cmd = excavator_controller.arm_cmd:main',
            'bucket_cmd = excavator_controller.bucket_cmd:main',
            'track_cmd = excavator_controller.track_cmd:main',
            'combined_cmd = excavator_controller.combined_cmd:main',
            'reset_cmd = excavator_controller.reset_cmd:main',
        ],
    },
)
