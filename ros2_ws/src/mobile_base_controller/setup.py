from setuptools import find_packages, setup

package_name = 'mobile_base_controller'

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
    maintainer='lamm-robot',
    maintainer_email='lamm-robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_driver = mobile_base_controller.motor_driver:main',
            'simple_tracker = mobile_base_controller.simple_tracker:main',
            'alchemist_core = mobile_base_controller.alchemist_core:main',
            'controller = mobile_base_controller.controller:main',
            'rpm_only_controller = mobile_base_controller.rpm_only_controller:main',
        ],
    },
)
