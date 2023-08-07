from setuptools import find_packages, setup

package_name = 'uart_converter'

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
    description='This package is to convert uart message to ros2 topics, using in OmniWhegPro Project',
    license='No license, use it whatever you like',
    tests_require=['pytest','pyserial','pytime'],
    entry_points={
        'console_scripts': [
            'uart_converter_node = uart_converter.uart_converter_node:main'
        ],
    },
)
