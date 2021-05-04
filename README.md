# Particle Filter
This repo implement Monte Carlo Localization using particle filters from [CSC477](http://www.cs.toronto.edu/~florian/courses/csc477_fall20/) at UofT, created by [Prof. Florian Shkurti](http://www.cs.toronto.edu/~florian/)

## Execution
prepare 3 terminal windows and run following commands separately
```
roslaunch estimation_and_vision_assignment monte_carlo_localization_v2.launch
rosbag play laser_controls_and_odometry.bag
rosrun rviz rviz    
``` 
When rviz initializes, go to `File > OpenConfig` and then load the configuration file in `estimation_and_vision_assignment/resources/comp417.rviz` which is going to start the visualization of laser scan messages, frames of reference, and particles.

![demo](demo.gif)
