# Client and Server


## Describing own service message
-	Create the archive
	```bash
	$ roscd your_package
	$ mkdir srv && cd srv
	$ touch YourService.srv
	```
-	Fill it with data
	```
	# Request
	int64 a
	int64 b
	---
	# Response
	int64 sum
	```

-	Edit package.xml
	```xml
	<build_depend>message_generation</build_depend>
	<run_depend>message_runtime</run_depend>
	```
-	Edit CMakeLists.txt
	```c
	find_package(catkin REQUIRED COMPONENTS
		std_msgs
		message_generation
	)
	add_service_files(
		FILES
		YourService.srv
	)

	add_executable(your_server_node src/your_server.cpp)
	target_link_libraries(your_server_node ${catkin_LIBRARIES})
	add_dependencies(your_server_node your_package_gencpp)

	add_executable(your_client_node src/your_client.cpp)
	target_link_libraries(your_client_node ${catkin_LIBRARIES})
	add_dependencies(your_client_node your_package_gencpp)
	```
	-----

	-----

### Command line utility (rossrv)
```bash
$ rossrv -h #Help about all rossrv utilities
$ rossrv list	# List all services
$ rossrv show	# Show service description
```

## Defining the server (C++)
```c++
#include "your_package/YourService.h"
ros::ServiceServer server;
return_value server_fnc(	your_package::YourService::Request 	&req, \
													your_package::YourService::Response &res 	)
{
	res.data = f(req.data1, req.data2);
	return whatever;
}
int main()
{
	ros::init(argc, argv, "your_server_node");
  ros::NodeHandle n;
	server = n.advertiseService("service_name", server_fnc);
	ros::spin();
}
```
## Defining a Client (C++)
```c++
#include "your_package/YourService.h"
ros::ServiceClient client;
int main()
{
		ros::init(argc, argv, "your_client_node");
		ros::NodeHandle n;
		client = n.serviceClient<your_package::YourService>("service_name");
		your_package::YourService srv;
		srv.request.a = ...;
		srv.request.b = ...;
		client.call(srv); //Call and wait for response.
		unsigned c = srv.response.sum;
		return 0;
}
```
-------

-------

## Defining the server (python)
```python
from your_package.srv import YourService, YourServiceResponse #Or * to add all
def server_fnc(req):
		res = YourServiceResponse()
		res.data = f(req.data1, req.data2);
    return whatever

rospy.init_node('your_server_node')
s = rospy.Service('service_name', YourService, server_fnc)
rospy.spin()

```
## Defining a Client (python)
```python
from your_package.srv import YourService, YourServiceRequest #Or * to add all
rospy.init_node('your_client_node') #Is not necessary but can be called
# Blocks until service_name is available
rospy.wait_for_service('service_name')
# Create a handle to call the service
service_call_fnc = rospy.ServiceProxy('service_name', YourService)
req = YourServiceRequest()
req.data1 = ...
req.data2 = ...
res = service_call_fnc(req)
```

### Command line utilities (rosservice)
```bash
$	rosservice -h		#Help about all rosservice utilities
$ rosservice list
$ rosservice info /ServiceName
$ rosservice call /ServiceName [TAB][TAB] #call /ServiceName
```
