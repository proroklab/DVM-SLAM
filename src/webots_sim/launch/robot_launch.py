import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('webots_sim')
    robot1_description_path = os.path.join(
        package_dir, 'resource', 'robot1.urdf')
    robot2_description_path = os.path.join(
        package_dir, 'resource', 'robot2.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'city.wbt'),
        simulation_server_ip="10.211.55.2",
    )

    robot1_driver = WebotsController(
        robot_name='robot1',
        parameters=[
            {'robot_description': robot1_description_path},
        ],
        ip_address="10.211.55.2"
    )

    robot2_driver = WebotsController(
        robot_name='robot2',
        parameters=[
            {'robot_description': robot2_description_path},
        ],
        ip_address="10.211.55.2"
    )

    return LaunchDescription([
        webots,
        robot1_driver,
        robot2_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())],
            )
        )
    ])
