from setuptools import setup
import os
from glob import glob

package_name = 'uran_move'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.plugins'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='URAN Dev',
    maintainer_email='dev@example.com',
    description='URAN-move package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uran_move_node = uran_move.uran_move_node:main',
        ],
    },
)
