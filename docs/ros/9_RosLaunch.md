# RosLaunch

## Main Scheme
```xml
<launch/>
  <group/> 				<!--Top level launch tag (container)-->
		<node/>				<!--Launch a node-->
			<remap/>		<!--Declare a name remapping (in the node scope)-->
			<rosparam/>		<!--Set ROS parameters for the launch-->
			<param/>		<!--Set a parameter for the Parameter Server-->
		<remap/>
		<rosparam/>
		<param/>
		<machine/>			<!--Declare a machine to use for launching-->
		<include/>			<!--Include other roslaunch files-->
			<arg/>
		<arg/>				<!--Declare an argument-->
</launch>
```
## Definitions
- Group
  ```xml
  <group 	ns				    = "namespace"
  	      clear_params	=	"true|false" 	<!-- Delete all parameters in the groups
                                             namespace before launch.-->
  />
  ```
- Node
  ```xml
 	<node	pkg				= 	"mypackage" 		<!-- Package of a node -->
				type			= 	"nodefile.cpp"			<!-- Corresponding executable -->
				name 			=	  "nodename" <!-- Name specified in nodefile.cpp -->
				args			=	  "arg1 arg2 arg3"
				machine			=	"machine-name"
				respawn			=	"true"	<!--Restart the node automatically if it quits.-->
				respawn_delay	=	"30"  <!-- Wait 30 seconds for restarting. -->
				required		=	"true"	<!-- If node dies, kill entire roslaunch.-->
				ns				=	"foo"			<!--	Start the node in the 'foo' namespace. -->
				clear_params	=	"true|false"	<!-- Delete all parameters in the node's
                                           private namespace before launch. -->
				output			=	"log|screen"		<!-- If 'screen', stdout/stderr from the
        node will be sent to the screen. If 'log', the stdout/stderr output will
        be sent to a log file in $ROS_HOME/log, and stderr will continue to be
        sent to screen. The default is 'log'. -->
				cwd				=	"ROS_HOME|node"		<!-- If 'node', the working directory of
        the node will be set to the same directory as the node's executable. -->
				launch-prefix	=	"prefix arguments"	<!--Command/arguments to prepend to
        node's launch arguments. This is a powerful feature that enables you to
        enable gdb, valgrind, xterm, nice, or other handy tools. See Roslaunch
        Nodes in Valgrind or GDB for examples. -->
	>
 	</node>
  ```
- Machine
  ```xml
	<machine name		  = 	"manifold" <!--(Name assignation) -->
			     address 	= 	"192.168.xx.x" <!--(Network address/hostname)-->
			     user 		=  	"user_name@192.168.xx.x" <!--(ssh user name)-->
			     password 	= 	"*****" <!--(ssh password)-->
			     env-loader 	=  	"/opt/ros/indigo/env.sh"
			     default		=	"true" <!-- Be the default machine to assgn nodes-->
			     timeout 	= 	"10.0" <!--seconds to consider roslaunch as failure)-->
	 />
   ```
-  Include
   ```xml
   <include	file			=	"$(find pkg-name)/path/filename.xml"	<!--Name of file
                                                                  to include.-->
				    ns				=	"foo" <!--Import the 'foo' namespace.-->
				    clear_params	=	"true|false" <!--Delete all parameters in the
            <include>'s namespace before launch.-->
				    pass_all_args	=	"true|false" <!--If true, then all args set in the
             current context are added	to the child context that is created for
             processing the included file. You can do this instead of explicitly
             listing each argument that you want to pass down.-->
	  />
    ```
- Remap
  ```xml
 	<remap 	from="original-name"	 <!--Nodes, parameters, topics, services -->
	 				to="new-name"			 <!--SCOPES: launch, group, node-->
	/>
   ```
- Rosparam
  ```xml
  <rosparam	command		=	"load|dump|delete"
						file		=	"$(find pkg-name)/path/foo.yaml" <!-- rosparam file.-->
						param		=	"param-name" <!--Name of parameter.-->
						ns			=	"namespace"  <!--Scope the parameters to the specified
                                       namespace.-->
						subst_value	=	true|false 	<!--Allows use of substitution args in
                                          the YAML text.-->
  />
  ```
  ```xml
	<param name		=	"namespace/name"	<!--Parameter name.-->
				 value		=	"value" <!-- (optional)	Defines the value of the parameter.
         If this attribute is omitted, binfile, textfile or command must be
         specified. -->
				 type		=	"str|int|double|bool"
				 textfile	=	"$(find pkg-name)/path/file.txt"	<!--The contents of the
         file will be read and stored as a string.-->
				 binfile		=	"$(find pkg-name)/path/file"	<!--Read and stored as a
         base64-encoded XML-RPC binary object. -->
				 command		=	"$(find pkg-name)/exe '$(find pkg-name)/arg.txt'"
	/>
  ```
  ```xml
  <arg	name		  =	"name"		<!--Argument name.-->
  			value		  =	"value"		<!--Argument value.-->
  			default		=	"value"		<!--Argument default value.-->
  			doc			  =	"Description of the argument"
	/>
  ```
