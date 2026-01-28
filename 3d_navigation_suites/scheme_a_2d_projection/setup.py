from setuptools import setup
import os
from glob import glob

package_name = 'scheme_a_2d_projection'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装启动文件
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        # 安装配置文件
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.rviz')),
        # 创建地图目录
        (os.path.join('share', package_name, 'maps'),
         []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='wheeltec@example.com',
    description='方案A：八叉树投影为2D的导航方案',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 可以在这里添加可执行脚本
        ],
    },
)