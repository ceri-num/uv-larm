# Autonomous Navigation

Using the saved map, it is now possible to achieve autonomous navigation i.e. the robot can compute a global trajectory to a target point and then autonomously navigate to through this trajectory while avoiding obstacles locally. 

Write a new launch file named `navigation.launch` that achieve this.
Goals can be sent trhough rviz by clicking on a specific location on the loaded map.

Documentation and packages:
- http://wiki.ros.org/navigation/Tutorials
- http://wiki.ros.org/map_server
- http://wiki.ros.org/amcl
- http://wiki.ros.org/move_base

# Final exercice (your project)

As a final exercice, you should:

1. Use a real turtlebot with a laser
2. Create a map of a maze in the robotics room 
3. Make the robot autonomously navigate into the maze using goals sent via `rviz`
