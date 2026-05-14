from setuptools import find_packages, setup

package_name = 'eddy_current_sensor_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],   # 添加 numpy 依赖
    zip_safe=True,
    maintainer='r',
    maintainer_email='r@todo.todo',
    description='Eddy current sensor ROS2 node',   # 建议修改描述
    license='Apache License 2.0',                  # 建议修改许可证
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'sensor_node = eddy_current_sensor_pkg.sensor_node:main',   # 添加入口点
        ],
    },
)