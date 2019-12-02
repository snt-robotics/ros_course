# Workspaces

## Create a workspace

-	Source the main ros configuration (Access to ROS commands)
	```bash
	$ source /opt/ros/your_ros_distro/setup.bash
	```
- Create your workspace folder and initialize it
	```bash
	$ mkdir -p ~/your_ws/src
	$ cd ~/your_ws/src
	$ catkin_init_workspace
	```
- Build the workspace (empty)
	```bash
	$ cd ~/catkin_ws/
	$ catkin_make
	```
- Source your workspace configuration
	```bash
	$ source devel/setup.bash
	```
- Check that everything points to your workspace
	```bash
	$ printenv | grep ROS
	```

## Clean workspace
```bash
	sudo rm -r build install devel
```

## Change between workspaces
```bash
	$ source /opt/ros/your_ros_distro/setup.bash
	$ source your_ws/devel/setup.bash
	$ printenv | grep ROS #Check the environment
```
## Structure of a workspace

-	Source Space (src):	Source code.
-	Build Space (build):	Where cmake is invoked.
-	Development space (devel):	Where built targets are placed prior to being	installed (executables)
-	Install space
-	Result space
