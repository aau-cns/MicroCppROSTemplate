#!/usr/bin/python3

# Copyright (C) 2021 Alessandro Fornasier
#
# Permission is hereby granted, free of charge, to any person
# obtaining a copy of this software and associated documentation
# files (the "Software"), to deal in the Software without
# restriction, including without limitation the rights to use,
# copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following
# conditions:
# 
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
# OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
# WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
# OTHER DEALINGS IN THE SOFTWARE.
#
# You can contact the author at alessandro.fornasier@aau.at

# ----------------------------------------------------------------------
#
#  - Python version required: 3.5+
#
# ----------------------------------------------------------------------

#
# Import libs
#

import os
import sys
import argparse
import datetime
import itertools
import subprocess
import io
import yaml
import re

from string import Template as tmpl

#
# Functions
#

def create_folder_structure(package_path) :

	# Create folder structure
	os.makedirs(package_path, exist_ok=True)
	os.makedirs(package_path + "/src", exist_ok=True)
	os.makedirs(package_path + "/cfg", exist_ok=True)
	os.makedirs(package_path + "/srv", exist_ok=True)
	os.makedirs(package_path + "/act", exist_ok=True)
	os.makedirs(package_path + "/msg", exist_ok=True)
	os.makedirs(package_path + "/include", exist_ok=True)
	os.makedirs(package_path + "/include/utils", exist_ok=True)
	os.makedirs(package_path + "/launch", exist_ok=True)

#
# Create file from template file
#

def create_from_template(template_filepath, d, filepath) :

	# Open template file as string
	with open(template_filepath, "r") as fh:
		template = fh.read()
	
	# Substitute
	file_content = tmpl(template).safe_substitute(d)
	
	# Open, write and close file
	fh = open(filepath, "w")
	fh.write(file_content)
	fh.close()

#
# Main
#

if __name__ == "__main__" :

	###################
	# Parse arguments #
	###################

	# Define input arguments
	parser = argparse.ArgumentParser(description="Automatic setup of ROS (1) node/wrapper")
	parser.add_argument('-t', '--type', help='type of node/wrapper. It could be either \"simple\" for a simple single source file node or \"oop\" for a object-oriented based node', default="not specified", required=True)
	parser.add_argument('-w', '--workspace', help='Path of the workspace to be created, it can be an already existing workspace', default='~/cws')
	parser.add_argument('-p', '--package', help='Name of the ROS package', default="not specified", required=True)
	parser.add_argument('-n', '--node', help='Name of the ROS node', default="not specified", required=True)
	parser.add_argument('-st', '--subscribe_topics', help='Array of topics (topic_1 topic_2 ... topic_N) to subscribe on', nargs='+')
	parser.add_argument('-sm', '--subscribe_messages', help='Array of message types for each specified subscribe_topic (message_1 message_2 ... message_N). Messages has to be specified with their full name, e.g., sensor_msgs/Imu', nargs='+')	
	parser.add_argument('-pt', '--publish_topics', help='Array of topics (topic_1 topic_2 ... topic_N) to publish on', nargs='+')
	parser.add_argument('-pm', '--publish_messages', help='Array of message types for each specified publish_topic (message_1 message_2 ... message_N). Messages has to be specified with their full name, e.g., sensor_msgs/Imu', nargs='+')
	
	# Parse input arguments
	args = parser.parse_args()

	##################
	# Init variables #
	##################

	# Declare variables
	script_path = os.path.abspath(os.path.dirname(sys.argv[0]))
	node_type = str(args.type)
	workspace = str(args.workspace)
	package = str(args.package)
	node = str(args.node)
	subscribe_topics = []
	publish_topics = []
	subscribe_messages = []
	publish_messages = []

	# Built string with topics and messages for visualization
	if args.subscribe_topics != None :
		for topic in args.subscribe_topics :
			subscribe_topics.append(str(topic))
	if args.subscribe_messages != None :
		for message in args.subscribe_messages :
			subscribe_messages.append(str(message))
	if args.publish_topics != None :
		for topic in args.publish_topics :
			publish_topics.append(str(topic))
	if args.publish_messages != None :
		for message in args.publish_messages :
			publish_messages.append(str(message))

	##################
	# Load from file #
	##################
	
	# Load info
	with open(script_path + "/resources/info.yaml", "r") as fh:
		yaml_dict = yaml.load(fh, yaml.SafeLoader)
		
	# Load header
	with open(script_path + "/resources/HEADER", "r") as fh:
		header_template = fh.read()
	
	#################
	# Safety checks #
	#################

	# Check length of topic and messages
	if (len(subscribe_topics) != len(subscribe_messages)) and (len(publish_topics) != len(publish_messages)) :
		print("\n-------------------------------------------------------------------------")
		print(" Unmatching number of topics and messages.")
		print(" Please, check that for each specified topic there is a specified message")
		print("-------------------------------------------------------------------------\n")
		exit(0)

	# Check type
	if (node_type != "simple") and (node_type != "oop") :
		print("\n-------------------------------------------------------------------------")
		print(" Incorrect -t (--type) argument. Please, use either \"simple\" or \"oop\"")
		print("-------------------------------------------------------------------------\n")
		exit(0)

	# Check messages
	message_check = True
	for message in publish_messages :
		message_check = message_check and ('/' in message)
	for message in subscribe_messages :
		message_check = message_check and ('/' in message)
	if message_check != True :
		print("\n-----------------------------------------------------------------------")
		print(" Incorrect messages.")
		print(" Please, make sure the messages are defined as <msg_family>/<msg_type>")
		print("-----------------------------------------------------------------------\n")
		exit(0)
		
	###############
	# Print infos #
	###############

	# Visualize input arguments
	print("\n----------------------------------------------------")
	print(" Micro C++ - ROS1 Node/Wrapper Template")
	print(f" - Type: {node_type}")
	print(f" - Workspace: {workspace}")
	print(f" - Package: {package}")
	print(f" - Node: {node}")
	print(f" - Subscribe topics: {subscribe_topics}")
	print(f" - Subscribe Messages: {subscribe_messages}")
	print(f" - Publish topics: {publish_topics}")
	print(f" - Publish Messages: {publish_messages}")
	print("----------------------------------------------------\n")
	input("If everything is correct, press Enter to continue...")

	
	################
	# Dictionaries #
	################

	# Define dictionary
	general_dict = {
		**yaml_dict,
		"YEAR":str(datetime.datetime.now().year),
		"PACKAGE_NAME":package,
		"NODE_NAME":node,
		"EXECUTABLES":f"src/{node}_node.cpp",
		"NODE_INCLUDES":"#include <ros/ros.h>\n#include <Eigen/Eigen>",
		"HEADER_INCLUDES":"#include <ros/ros.h>\n#include <Eigen/Eigen>",
		"SOURCE_INCLUDES":f"\n#include \"utils/colors.hpp\"\n#include \"{node}.hpp>\"",
	}
	
	general_dict["HEADER"] = tmpl(header_template).safe_substitute(general_dict)
	
	# Define includes
	additional_includes = str()
	for message in subscribe_messages :
		additional_includes = additional_includes + f"\n#include <{message}.h>"
	if any("sensor_msgs/Image" in s for s in subscribe_messages) :
		additional_includes = additional_includes + "\n#include <opencv2/opencv.hpp>\n#include <cv_bridge/cv_bridge.h>"
	additional_includes = additional_includes + "\n#include \"utils/colors.hpp\"\n"
	
	# Check if subscribed topics exist
	if subscribe_topics != None and subscribe_messages != None :

		# Define subscribers and callbacks for each subscribed topic
		callbacks_def = str()
		callbacks_impl = str()
		subscribers_def = str()
		subscribers_impl = str()
		subscribers_full = str()
		subscribers_echo = str()
		subscribers_echo_ros = str()
		for (topic, message) in zip(subscribe_topics, subscribe_messages) :
			msg = message.split("/")
			callbacks_def += f"void callback_{msg[1].lower()}(const {msg[0]}::{msg[1]}::ConstPtr& msg);\n"
			callbacks_impl += f"void callback_{msg[1].lower()}(const {msg[0]}::{msg[1]}::ConstPtr& msg) {{\n\n  // Write your code here ...\n\n}}\n\n"
			subscribers_def += f"ros::Subscriber sub_{msg[1].lower()};\n"
			subscribers_impl += f"sub_{msg[1].lower()} = nh.subscribe(\"{topic}\", 1, callback_{msg[1].lower()});\n"
			subscribers_full += f"ros::Subscriber sub_{msg[1].lower()} = nh.subscribe(\"{topic}\", 1, callback_{msg[1].lower()});\n"
			subscribers_echo += f"std::cout \"Subscribing: \" << sub_{msg[1].lower()}.getTopic().c_str()) << std::endl;\n"
			subscribers_echo_ros += f"ROS_INFO(\"Subscribing: %s\", sub_{msg[1].lower()}.getTopic().c_str());\n"
	
	# Check if published topics exist
	if publish_topics != None and publish_messages != None :
	
		# Define publisher and message variables
		publishers_def = str()
		publishers_impl = str()
		publishers_full = str()
		publishers_echo = str()
		publishers_echo_ros = str()
		ros_messages = str()
		publish = str()
		for (topic, message) in zip(publish_topics, publish_messages) :
			publishers_def += f"ros::Publisher pub{topic.replace('/','_')};\n"
			publishers_impl += f"pub{topic.replace('/','_')} = nh.advertise<{message.replace('/','::')}>(\"{topic}\", 1);\n"
			publishers_full += f"ros::Publisher pub{topic.replace('/','_')} = nh.advertise<{message.replace('/','::')}>(\"{topic}\", 1);\n"
			publishers_echo += f"std::cout \"Publishing: \" << pub{topic.replace('/','_')}.getTopic().c_str()) << std::endl;\n"
			publishers_echo_ros += f"ROS_INFO(\"Publishing: %s\", pub{topic.replace('/','_')}.getTopic().c_str());\n"
			ros_messages += f"{message.replace('/','::')} msg{topic.replace('/','_')};\n"
			publish += f"pub{topic.replace('/','_')}.publish(msg{topic.replace('/','_')});\n"
	
	if node_type == "simple" :
		general_dict["EXECUTABLES"] = f"src/{node}.cpp"
		general_dict["NODE_INCLUDES"] = general_dict["NODE_INCLUDES"] + additional_includes
		general_dict["PUBLISHERS"] = publishers_full
		general_dict["SUBSCRIBERS"] = subscribers_full
		general_dict["PUBLISHERS_ECHO"] = publishers_echo_ros
		general_dict["SUBSCRIBERS_ECHO"] = subscribers_echo_ros
	elif node_type == "oop" :
		general_dict["EXECUTABLES"] = f"src/{node}.cpp src/{node}_node.cpp"
		general_dict["NODE_INCLUDES"] = general_dict["NODE_INCLUDES"] + f"\n#include \"utils/colors.hpp\"\n#include \"{node}.hpp>\"\n"
		general_dict["HEADER_INCLUDES"] = general_dict["HEADER_INCLUDES"] + additional_includes
		general_dict["PUBLISHERS_DEF"] = publishers_def
		general_dict["SUBSCRIBERS_DEF"] = subscribers_def
		general_dict["PUBLISHERS_IMPL"] = publishers_impl
		general_dict["SUBSCRIBERS_IMPL"] = subscribers_impl
		general_dict["PUBLISHERS_ECHO"] = publishers_echo
		general_dict["SUBSCRIBERS_ECHO"] = subscribers_echo
		general_dict["CLASS_NAME"] = ''.join(word.title() for word in re.sub('[^A-Za-z0-9]+', '_', node.lower()).split('_'))
	general_dict["MESSAGES"] = ros_messages
	general_dict["PUBLISH"] = publish
	general_dict["CALLBACKS_DEFINITION"] = callbacks_def
	general_dict["CALLBACKS_IMPLEMENTATION"] = callbacks_impl
	general_dict["LICENSE_SW_NAME"] = node
	
	##############################
	# Create structure and files #
	##############################

	# package_path tilde expansion (it makes it understand ~)
	package_path = workspace + "/src/" + package
	package_path = os.path.expanduser(package_path)

	# Create folder structure
	create_folder_structure(package_path)

	# Init workspace
	subprocess.run("catkin init", cwd=os.path.expanduser(workspace), shell=True, stdout=subprocess.DEVNULL)

	# Create and write LICENSE
	license_template_path = script_path + "/resources/LICENSE"
	license_path = package_path + "/LICENSE"
	create_from_template(license_template_path, general_dict, license_path)
	
	# Create and write package.xml
	package_xml_template_path = script_path + "/templates/ros/package.xml.tmpl"
	package_xml_path = package_path + "/package.xml"
	create_from_template(package_xml_template_path, general_dict, package_xml_path)

	# Create and write CMakeList.txt
	cmakelist_template_path = script_path + "/templates/cmake/CMakeLists.txt.tmpl"
	cmakelist_path = package_path + "/CMakeLists.txt"
	create_from_template(cmakelist_template_path, general_dict, cmakelist_path)

	# Create and write common node files
	header_template_path = script_path + "/templates/cpp/headers/colors.hpp.tmpl"
	header_path = package_path + "/include/utils/colors.hpp"
	create_from_template(header_template_path, general_dict, header_path)
	
	# Create and write specific node files ({node}.cpp {node}.hpp {node}_node.cpp)
	if node_type == "simple" :
		node_template_path = script_path + "/templates/cpp/sources/simple_node.cpp.tmpl"
		node_path = package_path + f"/src/{node}.cpp"
		create_from_template(node_template_path, general_dict, node_path)
	elif node_type == "oop" :
		node_template_path = script_path + "/templates/cpp/sources/oop_node.cpp.tmpl"
		node_path = package_path + f"/src/{node}_node.cpp"
		create_from_template(node_template_path, general_dict, node_path)
		header_template_path = script_path + "/templates/cpp/headers/oop_header.hpp.tmpl"
		header_path = package_path + f"/include/{node}.hpp"
		create_from_template(header_template_path, general_dict, header_path)
		source_template_path = script_path + "/templates/cpp/sources/oop_source.cpp.tmpl"
		source_path = package_path + f"/src/{node}.cpp"
		create_from_template(source_template_path, general_dict, source_path)
	
	# Source and build the workspace	
	#subprocess.run("catkin build", cwd=os.path.expanduser(workspace), shell=True)#, stdout=subprocess.DEVNULL)
