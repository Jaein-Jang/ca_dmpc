from glob import glob
from setuptools import setup

package_name = 'ca_dmpc_experiments'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laptop-j',
    maintainer_email='laptop-j@localhost',
    description='Experiment logging and state collection infrastructure for CA-DMPC studies.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_state_collector = ca_dmpc_experiments.robot_state_collector:main',
            'metric_logger = ca_dmpc_experiments.metric_logger:main',
            'path_follower = ca_dmpc_experiments.path_follower:main',
            'trajectory_broadcaster = ca_dmpc_experiments.trajectory_broadcaster:main',
            'neighbor_trajectory_buffer = ca_dmpc_experiments.neighbor_trajectory_buffer:main',
            'occupancy_grid_builder = ca_dmpc_experiments.occupancy_grid_builder:main',
            'path_frame_adapter = ca_dmpc_experiments.path_frame_adapter:main',
            'rule_based_controller = ca_dmpc_experiments.rule_based_controller:main',
            'fixed_path_publisher = ca_dmpc_experiments.fixed_path_publisher:main',
            'standard_mpc_controller = ca_dmpc_experiments.standard_mpc_controller:main',
            'ca_dmpc_controller = ca_dmpc_experiments.ca_dmpc_controller:main',
        ],
    },
)
