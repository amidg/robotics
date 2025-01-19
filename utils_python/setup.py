from setuptools import find_packages, setup

package_name = 'utils_python'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'PyYAML'
    ],
    zip_safe=True,
    maintainer='Dmitrii Gusev',
    maintainer_email='dmitrii@gusev.tech',
    description='Various Python utility functions',
    license='GNU Public License 3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
