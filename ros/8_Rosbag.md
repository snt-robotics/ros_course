# Record the data
	
	Usage: rosbag record TOPIC1 [TOPIC2 TOPIC3 ...]

	rosbag record -a --- Records every topic
	
	rosbag record -O subset /turtle1/cmd_vel /turtle1/pose
	
# Manual

	Usage: rosbag <subcommand> [options] [args]

	A bag is a file format in ROS for storing ROS message data. The rosbag command can record, 		replay and manipulate bags.

	Available subcommands:
	   check  	Determine whether a bag is playable in the current system, or if it can be migrated.
	   compress  	Compress one or more bag files.
	   decompress  	Decompress one or more bag files.
	   filter  	Filter the contents of the bag.
	   fix  	Repair the messages in a bag file so that it can be played in the current system.
	   help  
	   info  	Summarize the contents of one or more bag files.
	   play  	Play back the contents of one or more bag files in a time-synchronized fashion.
	   record  	Record a bag file with the contents of specified topics.
	   reindex  	Reindexes one or more bag files.


## Rosbag to standard csv file 

-	rosbag record TOPICS
-	Terminal:

-		rosbag play --pause bag_file.bag
-		rostopic echo -p topic data_file.csv
