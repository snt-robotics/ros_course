# Publisher and Subscriber

## Topics
Named buses over which nodes exchange messages.
#### Command line utilities (rostopic)
```bash
$	rostopic -h		#Help about all rostopic utilities
$ rostopic list
$ rostopic info /TopicName
$ rostopic echo	/TopicName  #Print messages published on /Topic
$ rostopic pub 	/TopicName package/Msg [TAB][TAB] #publish package/Msg to /TopicName
```
#### Command line utilities (rqt_graph)
```bash
$ rosrun rqt_graph rqt_graph #Graph of topics, publishers, subscribers
```
#### Command line utilities (rqt_plot)
```bash
$ rosrun rqt_plot rqt_plot #Plot topics values
```
## Messages
#### Define own message
-	Create the archive
	```bash
	$ roscd your_package
	$ mkdir msg && cd msg
	$ touch YourMsg.msg
	```
-	Define its data
	```bash
	int64 A
	int64 B[] # An array based on vector library (C++)
	```
-	Add to package.xml
	```xml
	<build_depend>message_generation</build_depend>
 	<run_depend>message_runtime</run_depend>
	```

-	Add to CMakeLists.txt
	```c
	find_package(catkin REQUIRED COMPONENTS
 			std_msgs
 			message_generation
	)
	catkin_package(
		CATKIN_DEPENDS message_runtime
	)
	add_message_files(
		FILES
	    YourMsg.msg
	)
	generate_messages(
		 DEPENDENCIES
		 std_msgs
	)
	```
#### Command line utilities (rosmsg)
Displays information about ros message types
```bash
$ rosmsg -h				#Help about all rosmsg utilities
$ rosmsg list			#List all messages
$ rosmsg show			#Show message description
```

## Create a publisher (C++)

- As a global variable
	```c++
	#include "your_package/YourMsg.h"
	ros::Publisher your_publisher;
	int main()
	{
			ros::NodeHandle n;
			your_publisher = n.advertise<your_package::YourMsg>("TopicName",queue_size);
			//Use it
			your_package::YourMsg data;
			data = ...;
			your_publisher.pub(data);
	}
	```
## Create a subscriber (C++)

- As a global variable
	```c++
	#include "your_package/YourMsg.h"
	ros::Subscriber your_sub;
	void callback(const your_package::YourMsg::ConstPtr& msg)
	{
		//Make use of msg->data;
	}

	int main()
	{
			ros:NodeHandle n;
			sub = n.subscribe("TopicName",queue_size,callback);
			//ros::spin();
			ros::Rate loop_rate(10); //10 Hz
			while (ros::ok())
			{
				ros::spinOnce();
				loop_rate.sleep();
			}
			return 0;
	}
	```

## Create a publisher (python)

```python
from your_package.msg import YourMsg
rospy.init_node('publisher_node')
your_publisher = rospy.Publisher('TopicName', YourMsg, queue_size=...)
# Use it
data = YourMsg()
data = ...;
your_publisher.publish(data)
```
## Create a subscriber (python)

```python
from your_package.msg import YourMsg
def callback(msg):
  # Make use of msg.data
rospy.init_node('subscriber_node')
your_subscriber = rospy.Subscriber('TopicName', YourMsg, callback)
rospy.spin()
```
