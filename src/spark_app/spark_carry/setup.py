from glob import glob
from setuptools import setup,find_packages
import os

package_name = 'spark_carry'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('lib', package_name), glob('scripts/*.sh')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spark',
    maintainer_email='litian.zhuang@nxrobo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['spark_carry'],
    entry_points={
        'console_scripts': [
            'cali_cam = spark_carry.cali_cam:main',
            'cali_pos = spark_carry.cali_pos:main',
            'hsv_detection = spark_carry.hsv_detection:main',
            's_carry_object = spark_carry.s_carry_object:main',

        ],
    },
)
