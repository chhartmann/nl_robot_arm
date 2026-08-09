from setuptools import find_packages, setup

package_name = 'nlra_motion_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src', exclude=['test']),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/motion_planner.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='christian',
    maintainer_email='christian@example.com',
    description='MoveIt 2 motion planner wrapper for NL Robot Arm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_planner = nlra_motion_planner.motion_planner:main',
        ],
    },
)
