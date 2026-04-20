from setuptools import find_packages, setup

package_name = 'teleop_keyboard'

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
    maintainer='jetson',
    maintainer_email='jetson@example.com',
    description='Keyboard teleop node for publishing cmd_vel with arrow keys.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_keyboard = teleop_keyboard.teleop_keyboard:main'
        ],
    },
)
