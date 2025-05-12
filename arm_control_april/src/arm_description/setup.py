from setuptools import setup
from glob import glob
import os

package_name = 'arm_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],          # or use find_packages() if you have submodules
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launch files
        ('share/' + package_name + '/launch', glob('launch/*.py')),

        # URDF / meshes
        ('share/' + package_name + '/urdf',  glob('urdf/*.urdf')),
        ('share/' + package_name + '/meshes', glob('meshes//*', recursive=True)),

        # optional: rviz, world files
        ('share/' + package_name + '/rviz',  glob('rviz/*.rviz')),
        ('share/' + package_name + '/world', glob('world/empty.sdf')),
        ('share/' + package_name, ['model.config']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omrit',
    maintainer_email='omrit@todo.todo',
    description='URDF, meshes and launch files for the arm / rover simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
    
)