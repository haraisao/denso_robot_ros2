#
#
import os

from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import OrSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node, SetParameter

from ament_index_python.packages import get_package_share_directory

# --------- Gazebo Nodes (only if 'sim:=true') ---------
set_param_use_sim_time = SetParameter(
        name='use_sim_time', value=True,
        condition=IfCondition(OrSubstitution(sim,
                PythonExpression(["'", ros2_control_hardware_type, "'=='ign_sim'"]))))

# Sets Paths for ignition#
env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.environ['LD_LIBRARY_PATH'],
        'IGN_GAZEBO_RESOURCE_PATH': os.path.dirname
        (get_package_share_directory('denso_robot_descriptions')) + ':' +
        os.path.dirname(
            get_package_share_directory
                (str(LaunchConfiguration('description_package').perform(context))))
}

ign_gazebo = ExecuteProcess(
        condition=IfCondition(OrSubstitution(sim,
            PythonExpression(["'", ros2_control_hardware_type, "'=='ign_sim'"]))),
        cmd=['ign gazebo -r', 'empty.sdf'],
        output='screen',
        additional_env=env,
        shell=True
)

ignition_spawn_entity_node = Node(
    condition=IfCondition(OrSubstitution(sim,
        PythonExpression(["'", ros2_control_hardware_type, "'=='ign_sim'"]))),
    package='ros_gz_sim',
    executable='create',
    output='screen',
    arguments=['-topic', '/robot_description',
                '-name', robot_name,
                '-allow_renaming', 'true'],
)

bridge = Node(
    condition=IfCondition(OrSubstitution(sim,
        PythonExpression(["'", ros2_control_hardware_type, "'=='ign_sim'"]))),
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
    output='screen'
)