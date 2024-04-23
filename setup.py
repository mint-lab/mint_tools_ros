from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'mint_tools_ros2'
SHARE_DIR = os.path.join("share", package_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join(SHARE_DIR, "launch"), glob(os.path.join("launch", "*.launch.py"))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunglok',
    maintainer_email='sunglok@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_player = mint_tools_ros2.video_player:main',
            'compress_img = mint_tools_ros2.compress_img:main',
            'decompress_img = mint_tools_ros2.decompress_img:main',
        ],
    },
)
