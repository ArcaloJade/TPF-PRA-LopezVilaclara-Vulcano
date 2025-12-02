from setuptools import setup

package_name = 'tpf_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/navigation_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/navigation_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mateolopezv',
    maintainer_email='mateolv2003@gmail.com',
    description='Navegación autónoma para el TP Final (Parte B).',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization_node = tpf_navigation.localization_node:main',
            'planner_node = tpf_navigation.planner_node:main',
            'controller_node = tpf_navigation.controller_node:main',
            'map_loader_node = tpf_navigation.map_loader_node:main',
        ],
    },
)
