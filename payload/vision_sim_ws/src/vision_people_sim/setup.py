from glob import glob
from setuptools import find_packages, setup
package_name = 'vision_people_sim'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    py_modules=['vision_people_sim_launch_common'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/models/textured_person', ['models/textured_person/model.config', 'models/textured_person/model.sdf']),
        ('share/' + package_name + '/models/textured_person/materials/textures', glob('models/textured_person/materials/textures/*')),
        ('share/' + package_name + '/models/textured_vehicle', ['models/textured_vehicle/model.config', 'models/textured_vehicle/model.sdf']),
        ('share/' + package_name + '/models/textured_vehicle/materials/textures', glob('models/textured_vehicle/materials/textures/*')),
        ('share/' + package_name + '/models/apm_iris_follow_static', ['models/apm_iris_follow_static/model.config', 'models/apm_iris_follow_static/model.sdf']),
        ('share/' + package_name + '/models/apm_iris_follow_static/meshes', glob('models/apm_iris_follow_static/meshes/*')),
        ('share/' + package_name + '/models/iris_apm_rgbd', ['models/iris_apm_rgbd/model.config', 'models/iris_apm_rgbd/model.sdf']),
        ('share/' + package_name + '/models/d435i_downward_rgbd', ['models/d435i_downward_rgbd/model.config', 'models/d435i_downward_rgbd/model.sdf']),
        ('share/' + package_name + '/models/d435i_downward_sensor_only', ['models/d435i_downward_sensor_only/model.config', 'models/d435i_downward_sensor_only/model.sdf']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='ld666', maintainer_email='ld666@example.com',
    description='Gazebo Sim visual-only pedestrian motion worlds controlled by ROS2.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['people_motion = vision_people_sim.people_motion:main',
            'rgbd_camera_follow = vision_people_sim.rgbd_camera_follow:main',
            'd435i_topic_adapter = vision_people_sim.d435i_topic_adapter:main',
            'gz_rgbd_bridge = vision_people_sim.gz_rgbd_bridge:main',
            'guided_flight = vision_people_sim.guided_flight:main']},
)
