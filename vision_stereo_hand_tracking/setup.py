from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'vision_stereo_hand_tracking'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # include camera configuration
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.dat'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dmitrii Gusev',
    maintainer_email='dmitrii@gusev.tech',
    description='CV package to enable 3d hand reconstruction using mediapipe',
    license='GNU Public License 3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_stereo_hand_tracking = vision_stereo_hand_tracking.main:main'
        ],
    },
)
