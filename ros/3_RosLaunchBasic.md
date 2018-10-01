# RosLaunch Basic 
```xml
<launch>
  <include file="$(find the_package)/launch/other_launch_file.launch"/>
  <node pkg ="your_package"
        type="your_source.py"
        name="your_node_name"
        output="screen">
  </node>
</launch>
```
