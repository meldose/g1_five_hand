from setuptools import find_packages, setup

package_name = 'humanoid_hand_controller'

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
    maintainer='jeremy.ong',
    maintainer_email='jeremy.ong@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_node=humanoid_hand_controller.hand:main',
            'multi_sensor_node=humanoid_hand_controller.multi_sensor:main',
        ],
    },
)
