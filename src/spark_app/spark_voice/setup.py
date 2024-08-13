from glob import glob
from setuptools import setup,find_packages
import os

package_name = 'spark_voice'
submodules = "spark_voice/lib"

data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name), glob('launch/*.launch.py')),

]


def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:

        for (path, dirs, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)

                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],

    data_files=package_files(data_files, ['spark_voice/lib/pocketsphinx-data/']),

    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    #     (os.path.join('share', package_name), glob('launch/*.launch.py')),

    # ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spark',
    maintainer_email='litian.zhuang@nxrobo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['spark_voice'],
    entry_points={
        'console_scripts': [
            'local_asr = spark_voice.local_asr:main',
            'voice_nav = spark_voice.voice_nav:main',
            'baidu_asr = spark_voice.baidu_asr:main',
            'ekho_tts = spark_voice.ekho_tts:main',
        ],
    },
)
