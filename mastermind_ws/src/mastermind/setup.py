import os
from setuptools import setup, find_packages
from glob import glob

package_name = 'mastermind'

def generate_data_files(share_path, dir_names):
    """
    Finds all files in directories and PRESERVES the folder structure.
    """
    data_files = []
    for dir_name in dir_names:
        for root, dirs, files in os.walk(dir_name):
            # install_dir will look like: share/mastermind/models/cube
            install_dir = os.path.join(share_path, root)
            # source_files will look like: models/cube/model.config
            source_files = [os.path.join(root, f) for f in files]
            data_files.append((install_dir, source_files))
    return data_files

# Generate paths for 'models' and 'world' folders automatically
# This ensures models/cube/model.sdf stays inside a 'cube' folder
custom_paths = generate_data_files('share/' + package_name, ['models', 'world'])

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/launch', glob('launch/*.rviz')),
    ] + custom_paths, # Add the preserved folder structures here
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cyyang',
    maintainer_email='cyyang@todo.todo',
    description='Mastermind Robot Simulation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_and_place = mastermind.src.robot_controller.pick_and_place:main',
        ],
    },
)