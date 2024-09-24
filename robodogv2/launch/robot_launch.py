import os
import pathlib
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix

def generate_launch_description():
    world = LaunchConfiguration('world')
    package_dir = get_package_share_directory("my_package")
    urdf_path = os.path.join(package_dir, 'resource', 'my_urdf_file.urdf')
    robot_description = pathlib.Path(urdf_path).read_text()

    # Define your URDF robots here
    # The name of an URDF robot has to match the WEBOTS_ROBOT_NAME of the driver node
    # You can specify the URDF file to use with "urdf_path"
    spawn_URDF_robot = URDFSpawner(
        name='myRobot',
        urdf_path=urdf_path,
        translation='0 0 1',
        rotation='0 0 1 -1.5708',
    )

    # Driver nodes
    # When having multiple robot it is enough to specify the `additional_env` argument.
    # The `WEBOTS_CONTROLLER_URL` has to match the robot name in the world file.
    # You can check for more information at:
    # https://cyberbotics.com/doc/guide/running-extern-robot-controllers#single-simulation-and-multiple-extern-robot-controllers
    webots_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': 'myRobot'},
        parameters=[
            {'robot_description': robot_description},
        ],
    )

    # The WebotsLauncher is a Webots custom action that allows you to start a Webots simulation instance.
    # It searches for the Webots installation in the path specified by the `WEBOTS_HOME` environment variable and default installation paths.
    # The accepted arguments are:
    # - `world` (str): Path to the world to launch.
    # - `gui` (bool): Whether to display GUI or not.
    # - `mode` (str): Can be `pause`, `realtime`, or `fast`.
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world])
    )

    # The Ros2Supervisor is a Webots custom node that communicates with a Supervisor robot in the simulation.
    # The accepted arguments from launch.actions.ExecuteProcess are:
    # - `output` (string): Output configuration for process output logging. Default is 'screen'.
    # - `respawn` (bool): Relaunch the process that abnormally died.  Default is 'True'.
    ros2_supervisor = Ros2SupervisorLauncher()

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='robotic_arms.wbt',
            description='Choose one of the world files from `/webots_ros2_universal_robot/worlds` directory'
        ),
        # Starts Webots
        webots,

        # Starts the Ros2Szpervisor node
        ros2_supervisor,

        # Request the spawn of your URDF robot
        spawn_URDF_robot,

        # Launch the driver node once the URDF robot is spawned
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessIO(
                target_action=spawn_URDF_robot,
                on_stdout=lambda event: get_webots_driver_node(event, webots_driver),
            )
        ),

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])