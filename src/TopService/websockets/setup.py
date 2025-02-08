from setuptools import setup, find_packages
import os

# 获取当前工作目录
current_dir = os.path.abspath(os.path.dirname(__file__))

setup(
    name='websockets',
    version='0.0.0',
    description='A ROS package for demonstrating Python setup installation.',
    author='denk',
    author_email='ldq@qq.com',
    url='https://github.com/username/my_ros_package',
    packages=find_packages(),
    install_requires=[
        'rospy',  # 如果你的包依赖于 rospy 或其他 ROS 包
        # 添加其他依赖项，如 numpy, matplotlib 等
    ],
    package_data={
        'websockets': [
            'launch/*',   # 复制所有 launch 文件
            # 'scripts/*',  # 复制配置文件
            # 添加其他需要复制的文件或目录
        ]
    },
    entry_points={
        'console_scripts': [
            'talker = websockets.talker:main',
            'listener = websockets.listener:main',
            # 添加其他可执行脚本的入口点
        ],
    },
)