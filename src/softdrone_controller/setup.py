# -*- coding: utf-8 -*-
from setuptools import setup
import os
from glob import glob

package_name = 'softdrone_controller'

setup(
    name=package_name,
    version='0.0.1',
    # 核心修复：必须明确包含子包，Python 才能识别 softdrone_controller.config
    packages=[
        package_name,
        package_name + '.config'
    ],
    data_files=[
        # 必须的资源索引文件
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        
        # 安装 package.xml
        ('share/' + package_name, ['package.xml']),
        
        # 安装 config 目录下的所有文件 (yaml/py) 到 share 目录下
        (os.path.join('share', package_name, 'config'), 
         glob('softdrone_controller/config/*.py') + glob('softdrone_controller/config/*.yaml')),
        
        # 如果你有 launch 文件，取消下面这行的注释并创建目录
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    
    maintainer='博洋',
    maintainer_email='ssybh2@nottingham.edu.cn',
    description='软体无人机控制器包 - 包含主飞控、位置控制器、路径指令生成节点',
    license='Apache-2.0',
    
    tests_require=['pytest'],
    extras_require={
        'test': ['pytest'],
    },
    
    # 定义可执行节点入口
    entry_points={
        'console_scripts': [
            'drone_controller_sim = softdrone_controller.drone_controller_sim:main',
            'position_controller = softdrone_controller.position_controller:main',
            'position_cmd = softdrone_controller.position_cmd:main',
            'keyboard_teleop = softdrone_controller.keyboard_teleop:main'
        ],
    },
)
