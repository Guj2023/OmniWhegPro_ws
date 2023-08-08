from setuptools import find_packages, setup

package_name = 'uart_sender'

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
    maintainer='Guj',
    maintainer_email='guj2023@outlook.jp',
    description='This is to send command to the robot via uart.',
    license='No license, use it whatever you like',
    tests_require=['pytest','pyserial','pytime'],
    entry_points={
        'console_scripts': [
            'uart_sender_node = uart_sender.uart_sender_node:main'
        ],
    },
)
