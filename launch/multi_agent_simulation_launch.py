from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from time import gmtime, strftime
import os
import yaml

def generate_launch_description():
    package_path = str(os.path.realpath(get_package_share_directory('apf')))
    mission_file_name = package_path + "/mission/mission_multi_agent_2.yaml"
    # mission_file_name = package_path + "/mission/mission_multi_agent_30.yaml"
    with open(mission_file_name, encoding='UTF-8') as f:
        mission = yaml.load(f, Loader=yaml.FullLoader)
    agents = mission['agents']
    number_of_agent = len(agents)

    ld = LaunchDescription()
    for agent_id in range(0, number_of_agent):
        namespace="cf" + str(agents[agent_id]["crazyflie_id"])

        traj_planner_node = Node(
            package="apf",
            namespace=namespace,
            executable="agent_node",
            output="screen",
            name="agent",
            parameters=[
                {"agent_id": agent_id,
                 "mission_file_name": mission_file_name}
            ]
        )
        ld.add_action(traj_planner_node)

    rviz_path = package_path + '/config/rviz_config_sim.rviz'
    rviz_node = Node(
        package='rviz2',
        namespace='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path],
    )
    ld.add_action(rviz_node)

    return ld
