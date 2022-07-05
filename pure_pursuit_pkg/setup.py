from setuptools import setup
from glob import glob
import os


package_name = 'pure_pursuit_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Archit Hardikar',
    maintainer_email='architnh@seas.upenn.edu',
    description='f1tenth pure pursuit node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit = pure_pursuit_pkg.pure_pursuit_node:main',
            'pure_pursuit_opp = pure_pursuit_pkg.pure_pursuit_node_opp:main',

        ],
    },
)
