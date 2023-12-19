# Create Node with Class

The official ROS tutorial invites to use inheritence of ROS node.
<!--
Like this: 

```python 

```
-->

We propose here, to explore another way.





IMPORTANT - For a better security, implement the control function independently from the scan callback and activate your control at a desired frequency by using a timer.
Stop the robot if no scan was arrived the last second and test it by disconnecting the laser.
