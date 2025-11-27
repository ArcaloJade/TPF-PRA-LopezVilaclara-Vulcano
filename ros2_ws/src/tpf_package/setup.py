from glob import glob
from setuptools import find_packages, setup

package_name = 'tpf_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mateolopezv',
    maintainer_email='mateolv2003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_node = tpf_package.slam_node:main',
            'explorer_node = tpf_package.explorer_node:main',
            'map_saver_node = tpf_package.map_saver_node:main',
        ],
    },
)
