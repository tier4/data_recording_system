from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'c2_readout_delay_setter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'scripts'),
         glob(os.path.join('scripts', '*.sh'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manatohirabayashi',
    maintainer_email='manato.hirabayashi@tier4.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'c2_readout_delay_setter = ' + package_name + '.c2_readout_delay_setter:main',
        ],
    },
)
