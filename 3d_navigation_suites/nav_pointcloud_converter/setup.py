from setuptools import find_packages, setup

package_name = 'nav_pointcloud_converter'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='wheeltec@example.com',
    description='Navigation point cloud converter with robot self-filtering',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_converter_node = nav_pointcloud_converter.nav_converter_node:main',
        ],
    },
)
