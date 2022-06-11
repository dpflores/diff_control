diff_control
===

ROS package for the control and visualization of the turtlebot3 using python. You can follow these instructions either using a ubuntu 20.04 distribution with ROS noetic installed or with a free account in https://www.theconstructsim.com/, where you can emulate a ROS environment with the required resources.

## Installation

This will assume that you already have a catkin workspace (http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
Go to the source directory of the workspace
  ```
  $ roscd; cd ../src
  ```
  
Clone this repository
  ```
  $ git clone https://github.com/dpflores/diff_control
  ```
Create a  directory for the turtlebot3 packages and clone them
  ```
  $ mkdir turtlebot3
  $ cd turtlebot3
  $ git clone https://github.com/ROBOTIS-GIT/turtlebot3
  $ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs
  $ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations

  ```

Build using catkin_make
  ```
  $ cd ../../
  $ catkin_make
  ```
Now you have it installed! 

## Testing the turtlebot3 packages

In the workspace folder, launch the following:
  ```
  $ export TURTLEBOT3_MODEL=waffle
  $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
  ```
We can try a better environment like a house (look at the packages to see the other options):
  ```
  $ export TURTLEBOT3_MODEL=waffle
  $ roslaunch turtlebot3_gazebo turtlebot3_house.launch
  ``` 
While the environment is launched, we can launch in a new terminal the teleoperation control, which allows us to move the robot with the keys
  ```
  $ export TURTLEBOT3_MODEL=waffle
  $ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
  ``` 
If you want to reset the starting position, which is important when you make multiple tests, you can use a ROS service (http://wiki.ros.org/rosservice)
  ```
  $ rosservice call /gazebo/reset_simulation
  ``` 

You can look for the other examples in the turtlebot packages. Feel free to use them!
  
## Day 1: Sensors and visualization

To watch the sensor messages, the odometry of the robot, and other visualization, we need to use Rviz (http://wiki.ros.org/rviz). In another terminal run

  ```
  $ rosrun rviz rviz
  ``` 
In Rviz, you can add the robot visualization using the "Add" button at the bottom left, and selecting "robot model". In order to see the robot, with a new terminal run

  ```
  $ rosrun robot_state_publisher robot_state_publisher 
  ``` 
Then, in the "Fixed Frame" option at the upper left, select "odom" instead of "map"

Now that you can see the robot, let's see what he can see.

You can know the sensors by looking at the robot description in the .gazebo file, searching for sensors, and getting the topic name.
From the topics, we can see that we can add the following visualizations in Rviz.

<ul>
  <li>Laser scan</li>
  <li>RGB camera </li>
  <li>Depth camera</li>
  <li>Depth point cloud</li>
</ul>
When you select a visualization, make sure that you selected the appropriate topic for it. You can save your Rviz configuration to use it again.

If we want to use that topic in our code implementation, we need the type of the message, you can use the following command to know about that topic and the respective message.

  ```
  $ rostopic info /topic_name
  ``` 
Now you know how to implement a robot like the turtlebot3, look at the sensor's messages and visualize them in Rviz. On the next day, we are going to implement the orientation control of the turtlebot3 with a PID controller, a well common controller in control systems theory.

## Day 2: Orientation control

Tomorrow...