#	Parameters

Store and manipulate Ros Parameter Server --> Used by nodes to store and retrieve parameters at runtime.
	-	Parameters:	int, float, bool, dictionary, list

YAML Syntax:
	1 		->	int
	1.0		->	float
	"one"		->	string
	true		->	boolean
	[1, 2, 3]	->	List of int
	{a:b, c:d}	->	Dictionary

## Terminal commands

-	rosparam list
-	rosparam dump foo.yaml
-	rosparam load foo.yaml namespace
