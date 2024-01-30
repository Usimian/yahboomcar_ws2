from setuptools import setup
import os
from glob import glob

package_name = 'yahboomcar_description_x1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share',package_name,'urdf'),glob(os.path.join('urdf','*.*'))),
        (os.path.join('share',package_name,'meshes/mecanum'),glob(os.path.join('meshes/mecanum','*.*'))),
        (os.path.join('share',package_name,'meshes/sensor'),glob(os.path.join('meshes/sensor','*.*'))),
        (os.path.join('share',package_name,'meshes'),glob(os.path.join('meshes','*.*'))),
        (os.path.join('share',package_name,'rviz'),glob(os.path.join('rviz','*.rviz*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='1461190907@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
