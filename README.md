# **Hand_Gesture_Controlled_Robot**

* ## **Introduction**

In this we are controlling the simulated(turtlebot3 in Gazebo) or real world robot with the help of hand gestures.

- ## **Tools and Hardware Used**
1. VsCode
2. Python3
3. USB WebCam
4. Ubuntu OS
* ## **Requirements and Dependencies**
 
1. Libraries-OpenCV,Numpy,math,rospy,time
2. Install Robot Operating System(ROS) http://wiki.ros.org/ROS/Installation
3. For installing Ros Control package clone this package:[https://github.com/ros-controls/ros_control.git](https://github.com/ros-controls/ros_control) (branch: noetic-devel)
4. For installing Turtlebot3 package follow the below link:
https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation




* ## **Roadmap**

1. #### Detecting the hand_gestures for giving commands:-
    First of all we detect the hand gestures by         doing    some image processiong like                 thresholding, hsv        color separation           followed by finding contours,   convexity           defects to classify different hand      gestures.
    For hand_gesture detection refer this repo: https://github.com/Deepshikhar/Hand_Gesture
2. #### Controlling the robot(turtlebot3 in Gazebo)  using hand_gestures commands:-
    After detecting the hand gestures we publish the     message (velocity or rotaion) corresponding to       particular hand gesture to turtlebot3 via topic     called `cmd_vel` 
    


- ## **Results**

    ![](https://i.imgur.com/r8zGr8v.gif)
<br>Commanding Gestures</br>

   ![]([Robot control.gif](https://github.com/Deepshikhar/Hand_gesture_controlled_robot/blob/6327b121188b23f7dd90705f1029e3e51eb9f23b/Robot%20control.gif))
<br>Final Result</br>
