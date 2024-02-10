from setuptools import find_packages, setup

package_name = 'webots_sim'
data_files = []
data_files.append(
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch',
                  ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds',
                  ['worlds/complete_apartment.wbt']))
data_files.append(('share/' + package_name + '/worlds',
                  ['worlds/city.wbt']))
data_files.append(
    ('share/' + package_name + '/resource', ['resource/robot1.urdf']))
data_files.append(
    ('share/' + package_name + '/resource', ['resource/robot2.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user.name@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_driver = webots_sim.robot_driver:main',
        ],
    },
)
