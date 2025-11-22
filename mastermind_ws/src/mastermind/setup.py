from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'mastermind'

def files_under(pattern):
    return [p for p in glob(pattern, recursive=True) if os.path.isfile(p)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/world',  glob('world/*')), 
        
        ('share' + package_name + 'launch', glob('launch/*.rviz')),

        ('share/' + package_name + '/models', files_under('models/*')),
        ('share/' + package_name + '/models/realsense_d435', files_under('models/realsense_d435/*')),
        ('share/' + package_name + '/models/realsense_d435/meshes', files_under('models/realsense_d435/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cyyang',
    maintainer_email='cyyang@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
