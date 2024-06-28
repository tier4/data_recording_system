from setuptools import find_packages, setup

package_name = 'simple_frontend'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'simple_frontend = ' + package_name + '.simple_frontend:main',
            'switch_monitor = ' + package_name + '.switch_monitor:main',
        ],
    },
)
