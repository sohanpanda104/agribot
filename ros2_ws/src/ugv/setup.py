import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ugv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.config.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suraj',
    maintainer_email='38291736+suraj1102@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'combine_descriptions_node = ugv.combine_descriptions_node:main',
        ],
    },
)
