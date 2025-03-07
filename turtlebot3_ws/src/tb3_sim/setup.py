from setuptools import setup, find_packages
import os
import glob

package_name = 'tb3_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            [f'resource/{package_name}'] if os.path.exists(f'resource/{package_name}') else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob.glob('launch/*.launch.*')),
        (os.path.join('share', package_name, 'maps/'), glob.glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Katie Forster',
    maintainer_email='katie.e.forster@student.uts.edu.au',
    description='TurtleBot3 simulation package for navigation and mapping.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amcl_init_pose_publisher = tb3_sim.set_init_amcl_pose:main', 
        ],
    },
)
