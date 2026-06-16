from setuptools import find_packages, setup
import os 
from glob import glob  #ACW: Added for using new URDF with camera

package_name = 'autonomous_tb3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/turtlebot3_burger.urdf', 'urdf/turtlebot3_burger_cam.urdf']),       #ACW: Added for using new URDF with camera
#        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),          #ACW: Added for using new URDF with camera
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'meshes', 'bases'), glob('meshes/bases/*')),  #ACW: Added for using new URDF with camera
        (os.path.join('share', package_name, 'meshes', 'wheels'), glob('meshes/wheels/*')),#ACW: Added for using new URDF with camera
        (os.path.join('share', package_name, 'meshes', 'sensors'), glob('meshes/sensors/*')),#ACW: Added for using new URDF with camera
        (os.path.join('share',package_name,'config') , glob('config/*')),
        (os.path.join('share', 'autonomous_tb3', 'world/maze'), glob('world/maze/*')),
	(os.path.join('share', 'autonomous_tb3', 'world/small_maze'), glob('world/small_maze/*')),
        
            ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='srazavi',
    maintainer_email='srazavi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explorer_node = autonomous_tb3.explorer_node:main',
            'apriltag_overlay = autonomous_tb3.apriltag_overlay:main',
            'apriltag_text_marker = autonomous_tb3.apriltag_text_marker:main',
            'camera_viewer = autonomous_tb3.camera_viewer:main',
            'occupancy_grid_pub = autonomous_tb3.occupancy_grid_pub:main' ,
            'sdf_spawner = autonomous_tb3.spawn_entity:main' ,
            'maze_solver= autonomous_tb3.maze_solver:main' 
        ],
    },
)
