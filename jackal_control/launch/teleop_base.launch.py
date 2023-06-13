from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Launch Arguments
    is_sim = LaunchConfiguration('is_sim', default=False)

    is_sim_arg = DeclareLaunchArgument('is_sim', default_value=is_sim)

    filepath_config_twist_mux = PathJoinSubstitution(
        [FindPackageShare('jackal_control'), 'config', 'twist_mux.yaml']
    )

    filepath_config_interactive_markers = PathJoinSubstitution(
        [FindPackageShare('jackal_control'), 'config',
         'teleop_interactive_markers.yaml']
    )

    node_interactive_marker_twist_server = Node(
        package='interactive_marker_twist_server',
        executable='marker_server',
        name='twist_server_node',
        remappings={('cmd_vel', 'twist_marker_server/cmd_vel')},
        parameters=[filepath_config_interactive_markers],
        output='screen',
    )

    node_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings={
            ('/cmd_vel_out', '/jackal_velocity_controller/cmd_vel_unstamped')},
        parameters=[filepath_config_twist_mux],
        condition=UnlessCondition(is_sim)
    )

    ld = LaunchDescription()
    ld.add_action(node_interactive_marker_twist_server)
    ld.add_action(is_sim_arg)
    ld.add_action(node_twist_mux)
    return ld
