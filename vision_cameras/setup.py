from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'vision_cameras'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch files
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'),
             glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=[
        'setuptools',
        'opencv-contrib-python',
        'PyYAML'
    ],
    zip_safe=True,
    maintainer='Dmitrii Gusev',
    maintainer_email='dmitrii@gusev.tech',
    description='Basic camera package that can be reused',
    license='GNU Public License 3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration = vision_cameras.calibration:main'
        ],
    },
)
