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
#  - Python version required: 3.2+
#  - Packages required: argparse
#
# ----------------------------------------------------------------------


import os
import argparse
import datetime
import itertools

#
# Global variables
#

USER = os.getenv('USER')
TIME = datetime.datetime.now()

#
# Functions
#

def create_write_file(path, filename, content) :
	
	# Open, write and close file
	fh = open(path + "/" + filename, "w")
	fh.write(content)
	fh.close()

def create_folder_structure(package_path) :

	# Create folder structure
	os.makedirs(package_path, exist_ok=True)
	os.makedirs(package_path + "/src", exist_ok=True)
	os.makedirs(package_path + "/cfg", exist_ok=True)
	os.makedirs(package_path + "/srv", exist_ok=True)
	os.makedirs(package_path + "/act", exist_ok=True)
	os.makedirs(package_path + "/msg", exist_ok=True)
	os.makedirs(package_path + "/include", exist_ok=True)
	os.makedirs(package_path + "/launch", exist_ok=True)

def write_package_xml(package, package_path) :

	# Define file content
	content = f"""<?xml version=\"1.0\"?>
<package format=\"2\">
  <name>{USER}</name>
  <version>0.0.1</version>
  <description>The {package} package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <maintainer email=\"{USER}@email.com\">{USER}</maintainer>

  <!-- One license tag required, multiple allowed, one license per tag -->
  <license>LICENSE</license>

  <!-- Url tags are optional, but multiple are allowed, one per tag -->
  <!-- <url type=\"website\">http://wiki.ros.org/${package}</url> -->
  <!-- <url type=\"repository\">http://github.com/${package}</url> -->

  <!-- Author tags are optional, multiple are allowed, one per tag -->
  <author email=\"{USER}@email.com\">{USER}</author>

  <!-- The *depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!-- Use build_export_depend for packages you need in order to build against this package: -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!-- Use exec_depend for packages you need at runtime: -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!-- Use doc_depend for packages you need only for building documentation: -->
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>dynamic_reconfigure</depend>
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_Depend>

  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->
  </export>

</package>"""

	# Create and write file
	create_write_file(package_path, "package.xml", content)

def write_CMakeLists(package, package_path, node) :

	# Define file content
	content = f"""cmake_minimum_required(VERSION 3.0.2)
project({package})

## Try to compile with newer versions of C++
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS \"${{CMAKE_CXX_FLAGS}} -std=c++17\")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS \"${{CMAKE_CXX_FLAGS}} -std=c++11\")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS \"${{CMAKE_CXX_FLAGS}} -std=c++0x\")
else()
    message(WARNING \"The compiler ${{CMAKE_CXX_COMPILER}} has no C++0x support. Please use a different C++ compiler.\")
endif()

## enforcing cleaner code by adding -Wall and -Werror, the compiler will not ignore warnings anymore
add_definitions(-Wall -Werror)

## Display compiler flags
message(STATUS \"CMAKE_CXX_FLAGES: ${{CMAKE_CXX_FLAGS}}\")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure message_generation std_msgs nav_msgs sensor_msgs geometry_msgs)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

## Display packages version
message(STATUS \"EIGEN VERSION: \" ${{EIGEN3_VERSION}})
message(STATUS \"BOOST VERSION: \" ${{Boost_VERSION}})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
#  FILES Message1.msg Message2.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
#  FILES Service1.srv Service2.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY act
#  FILES Action1.action Action2.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs nav_msgs sensor_msgs geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
#  cfg/DynReconf1.cfg cfg/DynReconf2.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs nav_msgs sensor_msgs geometry_msgs
#  LIBRARIES library
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${{catkin_INCLUDE_DIRS}} ${{Boost_INCLUDE_DIR}} ${{EIGEN3_INCLUDE_DIR}}
)

## List third party libraries used by all binaries
list(
  APPEND thirdparty_libraries ${{catkin_LIBRARIES}} ${{Boost_LIBRARIES}} ${{EIGEN3_LIBRARIES}}
)

## List custom libraries inside lib folder
# list(
#   APPEND custom_libraries_sources lib/library_name/src/library_source.cpp
# )

## Declare a C++ library
# add_library(custom_libraries ${{custom_libraries_sources}})

## Specify libraries to link a library or executable target against
# target_link_libraries(custom_libraries ${{thirdparty_libraries}})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${{PROJECT_NAME}} ${{${{PROJECT_NAME}}_EXPORTED_TARGETS}} ${{catkin_EXPORTED_TARGETS}})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${{PROJECT_NAME}}_{node} src/{node}.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. \"rosrun someones_pkg node\" instead of \"rosrun someones_pkg someones_pkg_node\"
set_target_properties(${{PROJECT_NAME}}_{node} PROPERTIES OUTPUT_NAME {node} PREFIX)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${{PROJECT_NAME}}_{node} ${{${{PROJECT_NAME}}_EXPORTED_TARGETS}} ${{catkin_EXPORTED_TARGETS}})

## Specify libraries to link a library or executable target against
target_link_libraries(${{PROJECT_NAME}}_{node} ${{thirdparty_libraries}})
# target_link_libraries(${{PROJECT_NAME}}_{node} custom_libraries)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION \${{CATKIN_PACKAGE_BIN_DESTINATION}}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS \${{PROJECT_NAME}}_node
#   RUNTIME DESTINATION \${{CATKIN_PACKAGE_BIN_DESTINATION}}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS \${{PROJECT_NAME}}
#   ARCHIVE DESTINATION \${{CATKIN_PACKAGE_LIB_DESTINATION}}
#   LIBRARY DESTINATION \${{CATKIN_PACKAGE_LIB_DESTINATION}}
#   RUNTIME DESTINATION \${{CATKIN_GLOBAL_BIN_DESTINATION}}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/\${{PROJECT_NAME}}/
#   DESTINATION \${{CATKIN_PACKAGE_INCLUDE_DESTINATION}}
#   FILES_MATCHING PATTERN \"*.h\"
#   PATTERN \".svn\" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION \${{CATKIN_PACKAGE_SHARE_DESTINATION}}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(\${{PROJECT_NAME}}-test test/test_tmp_project.cpp)
# if(TARGET \${{PROJECT_NAME}}-test)
#   target_link_libraries(\${{PROJECT_NAME}}-test \${{PROJECT_NAME}})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)"""

	# Create and write file
	create_write_file(package_path, "CMakeLists.txt", content)

def write_node_cpp(package_path, node, subscribe_topics, publish_topics, subscribe_messages, publish_messages) :
	
	# Define license header
	license_header = f"""/// Copyright (C) {TIME.year} {USER}.
///
/// You can contact the author at {USER}@email.com\n\n"""


	# Check if some messages are specified exist
	if publish_messages != None and subscribe_messages != None :
	
		# Build a single include_messages array without duplicates by jsut
		# merging the different lists and converting to a set and then back to list
		include_messages = list(set(subscribe_messages + publish_messages))
		
		# Define include section
		includes = "#include <ros/ros.h>\n"
		for message in include_messages :
			includes += f"#include <{message}.h>\n"

	# Check if subscribed topics exist
	if subscribe_topics != None and subscribe_messages != None :
		assert len(subscribe_topics) == len(subscribe_messages)

		# Define callbacks for each subscribed topic
		callbacks = "\n// Callback functions\n" 
		for (topic, message) in zip(subscribe_topics, subscribe_messages) :
			msg = message.split("/")
			callbacks += f"void callback_{msg[1]}(const {msg[0]}::msg[1]::ConstPtr& msg);\n"

	# Define file content	
	content = license_header + includes + callbacks + f"""


void callback_imu(const sensor_msgs::Imu::ConstPtr& msg);

// Main function
int main(int argc, char** argv) {{

  // Launch ros node
  ros::init(argc, argv, \"{node}\");
  ros::NodeHandle nh(\"~\");

  // Get parameters from launchfile (the topic where imu is going to be publish)
  std::string topic_imu;

  // Check existance of parameter
  if(!nh.getParam(\"topic_imu\", topic_imu)) {{
    std::cout << std::endl;
    ROS_ERROR(\"No name_of_param defined\");
    std::exit(EXIT_FAILURE);
  }}

  // Create info publisher
  ros::Publisher pub = nh.advertise<std_msgs::String>(\"/info\", 1);

  // Print topics where we are publishing on
  std::cout << std::endl;
  ROS_INFO(\"Publishing: %s\", pub_params.getTopic().c_str());
  std::cout << std::endl;

  // Create IMU subscriber
  ros::Subscriber sub_imu = nh.subscribe(topic_imu, 999, callback_imu);

  // Ros messages
  std_msgs::String info_msg;

  // Write info message
  info_msg = \"Subscribing to: \" + topic_imu;

  // Publish
  pub.publish(info_msg);

  // ROS Spin
  ros::spin();

  // Done!
  std::cout << std::endl;
  ROS_INFO(\"Done!\");
  return EXIT_SUCCESS;

}}

void callback_inertial(const sensor_msgs::Imu::ConstPtr& msg) {{

  // Define local data structure
  double timestamp;
  Eigen::Vector3d wm;
  Eigen::Vector3d am;

  // Parse message to defined data structure
  timestamp = msg->header.stamp.toSec();
  wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
  am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

  // Write your code here ...

}} """
	# Create and write file
	create_write_file(package_path + "/src", f"{node}.cpp", content)
	

#
# Main
#

if __name__ == "__main__":

	# Parse input arguments
	parser = argparse.ArgumentParser(description="Automatic setup of ROS (1) Wrapper")
	parser.add_argument('-w', '--workspace', help='Path of the workspace to be created', default="~/cws")
	parser.add_argument('-p', '--package', help='Name of the ROS package', default="not specified", required=True)
	parser.add_argument('-n', '--node', help='Name of the ROS node', default="not specified", required=True)
	parser.add_argument('-st', '--subscribe_topics', help='Array of topics (topic_1 topic_2 ... topic_N) to subscribe on', nargs='+')
	parser.add_argument('-sm', '--subscribe_messages', help='Array of message types for each specified subscribe_topic (message_1 message_2 ... message_N). Messages has to be specified with their full name, e.g., sensor_msgs/Imu', nargs='+')	
	parser.add_argument('-pt', '--publish_topics', help='Array of topics (topic_1 topic_2 ... topic_N) to publish on', nargs='+')
	parser.add_argument('-pm', '--publish_messages', help='Array of message types for each specified publish_topic (message_1 message_2 ... message_N). Messages has to be specified with their full name, e.g., sensor_msgs/Imu', nargs='+')
	args = parser.parse_args()

	# Declare variables
	workspace = args.workspace
	package = args.package
	node = args.node
	subscribe_topics = []
	publish_topics = []
	subscribe_messages = []
	publish_messages = []

	# Built string with topics and messages for visualization
	if args.subscribe_topics != None :
		for topic in args.subscribe_topics :
			subscribe_topics.append(topic)
	if args.subscribe_messages != None :
		for message in args.subscribe_messages :
			subscribe_messages.append(message)
	if args.publish_topics != None :
		for topic in args.publish_topics :
			publish_topics.append(topic)
	if args.publish_messages != None :
		for message in args.publish_messages :
			publish_messages.append(message)

	# Visualize input arguments
	print("\n----------------------------------------")
	print(" Micro C++ - ROS1 Wrapper Template")
	print(f" - Workspace: {workspace}")
	print(f" - Package: {package}")
	print(f" - Node: {node}")
	print(f" - Subscribe topics: {subscribe_topics}")
	print(f" - Subscribe Messages: {subscribe_messages}")
	print(f" - Publish topics: {publish_topics}")
	print(f" - Publish Messages: {publish_messages}")
	print("----------------------------------------\n")

	# Create folder structure
	package_path = workspace + "/src/" + package

	# package_path tilde expansion (it makes it understand ~)
	package_path = os.path.expanduser(package_path)

	# Create folder structure
	create_folder_structure(package_path)

	# Create and write package.xml
	write_package_xml(package, package_path)

	# Create and write CMakeList.txt
	write_CMakeLists(package, package_path, node)

	# Create and write node.cpp
	write_node_cpp(package_path, node, subscribe_topics, publish_topics, subscribe_messages, publish_messages)
