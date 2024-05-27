from setuptools import find_packages, setup

package_name = 'simulated_agent_l1'

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
    maintainer='olivia',
    maintainer_email='olivia.enroth@combitech.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "position_node = simulated_agent_l1.position_node:main",
            "mqtt_publish_node = simulated_agent_l1.mqtt_publish_node:main"
        ],
    },
)
