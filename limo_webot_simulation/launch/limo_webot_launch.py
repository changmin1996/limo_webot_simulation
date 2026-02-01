import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('limo_webot_simulation')
    robot_description_path = os.path.join(package_dir, 'resource', 'limo_webot.urdf')
    
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'webot/limo_simulation/worlds', 'limo_track.wbt')
    )

    my_robot_driver = WebotsController(
        robot_name='limo',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])