#!/usr/bin/python3

import yaml
import xml.etree.ElementTree as ET
from xml.dom import minidom

def load_vehicle_positions(file_path):
  
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    
    positions = data['initial_shape_positions']
    vehicle_number = data['vehicle_number']
    
    position_list = []
    for i in range(1, vehicle_number + 1):
        pos_key = f'vehicle_pos_{i}'
        x = positions[pos_key]['x']
        y = positions[pos_key]['y']
        position_list.append([x, y])
    
    return position_list, vehicle_number

def generate_launch(input_file = "../config/frm_shape.yaml", output_file = "../launch/multi_robots_simulation_custom.launch"):  

    positions, robot_number = load_vehicle_positions(input_file)

    launch = ET.Element("launch")

	# start gazebo
    ET.SubElement(launch, "arg", {"name":"gz_gui", "default":"false"})
	
    gzb_include = ET.SubElement(launch, "include",{"file" : "$(find gazebo_ros)/launch/empty_world.launch"})

    # ET.SubElement(gzb_include, "arg", {"name" : "world_name", "value" : "$(find multi_turtlebot3_simulation)/worlds/corrider.world"})
    ET.SubElement(gzb_include, "arg", {"name" : "paused", "value" : "false"})
    ET.SubElement(gzb_include, "arg", {"name" : "use_sim_time", "value" : "true"})
    ET.SubElement(gzb_include, "arg", {"name" : "headless", "value" : "false"})
    ET.SubElement(gzb_include, "arg", {"name" : "gui", "value" : "$(arg gz_gui)"})
    ET.SubElement(gzb_include, "arg", {"name" : "debug", "value" : "false"})
	
    # ET.SubElement(gzb_include, "arg", {"name" : "world_name", "value" : "${find multi_turtlebot3_simulation)/worlds/corrider.world"})

	# start rviz
    rviz_node = ET.SubElement(launch, "node", {"name" : "rviz", "pkg"  : "rviz", "type" : "rviz", "args" : "-d $(find multi_turtlebot3_simulation)/launch/multi_robot.rviz", "required" : "true"})

    for i in range(robot_number):

		# start gazebo robots
        include = ET.SubElement(launch, "include")
        include.set("file", "$(find multi_turtlebot3_simulation)/launch/one_robot.xml")

        ET.SubElement(include, "arg", {"name" : "tb3_name", "value" : f"tb_{i}"})

        ET.SubElement(include, "arg", {"name" : "robot_x_pos", "value" : f"{positions[i][0]:.1f}"})
        ET.SubElement(include, "arg", {"name" : "robot_y_pos", "value" : f"{positions[i][1]:.1f}"})
        ET.SubElement(include, "arg", {"name" : "robot_z_pos", "value" : "0.0"})
        ET.SubElement(include, "arg", {"name" : "robot_yaw", "value" : "0.0"})
	
    rough_xml = ET.tostring(launch, 'utf-8')

    parsed = minidom.parseString(rough_xml)
    pretty_xml = parsed.toprettyxml(indent="    ", encoding="utf-8")

    with open(output_file, "wb") as f:
        f.write(pretty_xml)	
        


# 测试案例
if __name__ == "__main__":
    generate_launch()

