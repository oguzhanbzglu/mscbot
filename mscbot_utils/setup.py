from setuptools import find_packages, setup

package_name = 'mscbot_utils'

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
    maintainer='oguzhan',
    maintainer_email='oguzhanbzglu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory = mscbot_utils.trajectory:main',
            'path_publisher = mscbot_utils.path_publisher:main',
            'waypoint_tracking = mscbot_utils.waypoint_tracking:main'

        ],
    },
)
