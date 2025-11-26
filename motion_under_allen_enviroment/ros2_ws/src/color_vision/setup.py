from setuptools import setup

package_name = 'color_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Junxian Zhu',
    maintainer_email='you@example.com',
    description='Color detection node for lab06 overhead camera.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'block_color_scanner = color_vision.block_color_scanner:main',
        ],
    },
)

