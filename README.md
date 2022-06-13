diff_control
===

ROS package for the control and visualization of the turtlebot3 using python. You can follow these instructions either using a ubuntu 20.04 distribution with ROS noetic installed or with a free account in [TheConstruct](https://www.theconstructsim.com/), where you can emulate a ROS environment with the required resources.

## Installation

This will assume that you already have a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
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
If you want to reset the starting position, which is important when you make multiple tests, you can use a [ROS service](http://wiki.ros.org/rosservice)
  ```
  $ rosservice call /gazebo/reset_simulation
  ``` 

You can look for the other examples in the turtlebot packages. Feel free to use them!
  
## Day 1: Sensors and visualization

To watch the sensor messages, the odometry of the robot, and other visualization, we need to use [Rviz](http://wiki.ros.org/rviz). In another terminal run

  ```
  $ rosrun rviz rviz
  ``` 
In Rviz, you can add the robot visualization using the `Add` button at the bottom left, and selecting `robot model`. In order to see the robot, with a new terminal run

  ```
  $ rosrun robot_state_publisher robot_state_publisher 
  ``` 
Then, in the `Fixed Frame` option at the upper left, select `odom` instead of `map`

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

Yesterday, we learned how to simulate and watch our robot, and its sensors, now it's time to control it.

First, let's update the repository to get any changes that have been made. In the `diff_control` directory, run

  ```
  $ git pull
  ``` 

If you look at the `src` directory of the package, we have three python nodes created (the .py files are additional scripts for visualization purposes).

First, launch the simulation environment

  ```
  $ export TURTLEBOT3_MODEL=waffle
  $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
  ``` 
It is important to understand how [publishers and subscribers](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) work 

The `start_node` is a simple node to command the linear and angular speeds of the turtlebot using a publisher for the `cmd_vel` topic.

  ```
  $ rosrun diff_control start_node
  ``` 
Now to start our control, we can use the `orientation_control` node. This node subscribes to the `/odom` topic to get the current pose of the robot, uses a desired position that you can set and apply a proportional controller for orientation. Also, it writes a file with necessary data to track the position and how our controller is working.

To visualize the data while the node is running, you can start the live plotter. In the `src` directory of the package, run

  ```
  $ python3 live_plotter.py
  ``` 

You will see a plot, which might have some traces due to the previous data storage, but it will be updated while you run the node. 

  ```
  $ rosrun diff_control orientation_control
  ``` 

This will control the vehicle with a simple proportional controller for the orientation and position.

You can also analyze the results after the simulation, running the plotter file in the `src` directory of the package.

  ```
  $ python3 plotter.py
  ``` 

Now, for the last node, we implemented a PID controller script using [Object Oriented Programming](https://pythonprogramming.net/object-oriented-programming-introduction-intermediate-python-tutorial/). It is important to know the theory about this linear controller to know how it works. 

 ```
  $ rosrun diff_control orientation_pid_control
  ``` 

You can change the parameters of the PID controller and even implement other control types. A PID controller works well in our case, because of the linear relation of the output and input; however, this is not the case in most situations, so you can look for [nonlinear controllers theory](https://www.youtube.com/watch?v=LpoGv3gIeG0&list=PLMFfRskH3EaqITi8mu6mKWAq2T_lrqwq9).

If you are more interested in self-driving cars, a type of autonomous vehicle, I strongly recommend you this [set of courses](https://www.coursera.org/specializations/self-driving-cars?) from Toronto University in Coursera.

Good luck!
