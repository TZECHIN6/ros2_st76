from setuptools import find_packages, setup

package_name = 'ros2_st76'

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
    maintainer='apas',
    maintainer_email='terencelingtat.fung@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'st76_node = ros2_st76.st76_node:main',
            'vehicle_speed_rpt_publisher_node = ros2_st76.vehicle_speed_rpt_publisher:main'
        ],
    },
)
