from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'br2_vff_avoidance_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all files in the launch directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eth',
    maintainer_email='mdether0056@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avoidance_vff_main = br2_vff_avoidance_py.avoidance_vff_main:main',
            'avoidance_new = br2_vff_avoidance_py.avoidance_new:main',
            'goal_navigator = br2_vff_avoidance_py.goal_navigator:main'
        
        ],
    },
)
