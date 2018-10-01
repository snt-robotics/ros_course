# Packages
## Creating packages
```bash
	$ catkin_create_pkg your_pkg std_msgs rospy roscpp #Dependencies
	$ catkin_make
```
## XML dependencies
```xml
	<build_depend> 	Packages needed to build your package
	<build_export_depend>	Pkg. needed to build libraries against this pkg
	<run_depend> <exec_depend>	Needed to run
	<test_depend>	For testing
	<buildtool_depend>	Essential for building anything.
	<doc_depend>	To build documentation
```
## CMakeLists
```bash
cmake_minimum_required(VERSION 2.8.3) #Cmake version
project(package_name) #Can be referenced later as ${PROJECT_NAME}
find_package(catkin REQUIRED COMPONENTS # Packages needed to build the project
	 message_generation
	 )
catkin_python_setup( )		#Enables python module support
catkin_package(
   INCLUDE_DIRS include		#The exported include paths for the package
   LIBRARIES ${PROJECT_NAME}	#The exported libraries from the project
   CATKIN_DEPENDS roscpp 		#Other catkin projects that this project depends on
   DEPENDS eigen opencv #Non-catkin CMake projects that this project depends on.
	 )
include_directories(
   ${catkin_INCLUDE_DIRS}
	 /other/paths
   )
add_executable(my_node
	 src/my_node_program.cpp
	 src/some_file.cpp
	 src/another_file.cpp
	 )
add_library(my_library src/my_library.cpp)
target_link_libraries(my_node, my_library, <lib2>, ... <libN>)
```
