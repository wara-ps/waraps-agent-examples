from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'simulated_agent_l1_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/params.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='olivia',
    maintainer_email='olivia.enroth@combitech.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
