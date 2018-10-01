# Action

## Describing own action message
-	Create the archive
	```bash
	$ roscd your_package
	$ mkdir action && cd action
	$ touch YourAction.action
	```

-	Fill it with data
	```
  # Goal
  geometry_msgs/Pose desired_position
  ---
  # Result
  int32 done
  ---
  # Feedback
  geometry_msgs/Pose current_position
  ```

-	Edit package.xml
	```xml
	<run_depend>message_generation</run_depend>
	```

-	Edit CMakeLists.txt
	```c
  find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
  )
  add_action_files(
    DIRECTORY action
    FILES YourAction.action
  )
  generate_messages(
    DEPENDENCIES actionlib_msgs std_msgs
  )
  catkin_package(
  CATKIN_DEPENDS actionlib_msgs
  )
	```
------

------

### Command line utilities (rosmsg)
```bash
$ rosmsg -h       #Help about all rosmsg utilities
$ rosmsg list     #List all messages
$ rosmsg show     #Show message description
```

## Defining the server (C++)
```c++
#include <your_package/YourActionAction.h>
#include <actionlib/server/simple_action_server.h>
typedef actionlib::SimpleActionServer<your_package::YourActionAction> Server;
void execute(const your_package::YourActionGoalConstPtr& goal, Server* as){
  // Do things to reach the goal
  as->setSucceeded();
}
int main(int argc, char** argv){
  ros::init(argc, argv, "actionserver_node");
  ros::NodeHandle n;
  Server server(n, "server_name", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
```
## Defining a Client (C++)
```c++
#include <your_package/YourActionAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<your_package::YourActionAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "actionclient_node");
  Client client("client_name", true); // true -> don't need ros::spin()
  client.waitForServer();
  your_package::YourActionGoal goal;
  // Fill in goal here
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay!");
  return 0;
}
```

## Defining the server (python)
```python
import actionlib
from your_package.msg import YourActionAction
def execute(self, goal):
  # Do things to reach the goal
  server.set_succeeded()

rospy.init_node('server_node')
server = actionlib.SimpleActionServer('server_name', YourActionAction, self.execute, False)
server.start()
rospy.spin()
```

## Defining a Client (python)
```python
import rospy
import actionlib
from your_package.msg import YourActionAction, YourActionGoal
rospy.init_node('your_client_node')
client = actionlib.SimpleActionClient('client_name', YourActionAction)
client.wait_for_server()
goal = YourActionGoal()
# Fill in the goal here
client.send_goal(goal)
client.wait_for_result(rospy.Duration.from_sec(5.0))
```

### Command line utilities (rostopic)
```bash
$ rostopic -h		#Help about all rostopic utilities
$ rostopic list
$ rostopic info /TopicName
$ rostopic echo	/TopicName  #Print messages published on /Topic
$ rostopic pub 	/TopicName package/Msg [TAB][TAB] #publish package/Msg to /TopicName
```
