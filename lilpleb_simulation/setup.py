from setuptools import find_packages, setup
import os
import sys
from glob import glob

package_name = 'lilpleb_simulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # resource
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch and rviz files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dmitrii Gusev',
    maintainer_email='dmitrii@gusev.tech',
    description='Lilpleb robot simulation package',
    license='GNU Public License 3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },

)
