from setuptools import setup, find_packages

package_name = 'pose_constraints_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['package.xml']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/test_solver.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Visualize geometric constraints in RViz',
    entry_points={
        'console_scripts': [
            'geometric_constraints_visualizer = pose_constraints_planner.geometric_constraints_visualizer:main'
        ],
    },
)
