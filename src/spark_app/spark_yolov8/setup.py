from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'spark_yolov8'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name,'rviz'), glob(os.path.join('rviz','*.rviz'))),

	(os.path.join('share', package_name,'launch'), glob(os.path.join('launch','*launch.py'))),
    (os.path.join('lib', package_name,'model'), glob(os.path.join('model','*pt'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nxrobo',
    maintainer_email='10360882+huo-haijie@user.noreply.gitee.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'camera_predict = spark_yolov8.camera_predict:main',
             'camera_pose = spark_yolov8.camera_pose:main',
             'camera_predict_tf = spark_yolov8.camera_predict_tf:main',
             'id_service = spark_yolov8.id_service:main',
             'camera_object = spark_yolov8.camera_object:main',

        ],
    },
)
