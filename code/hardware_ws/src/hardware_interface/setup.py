from setuptools import find_packages, setup

package_name = 'hardware_interface'

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
    maintainer='pim',
    maintainer_email='p.h@concepts.nl',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hardware_node = hardware_interface.hardware_node:main',
            'hardware_nodeV2 = hardware_interface.hardware_nodeV2:main'
        ],
    },
)
