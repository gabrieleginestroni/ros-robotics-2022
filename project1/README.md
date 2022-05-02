# Robotics - Project 1 2021/2022 

The first project requires to compute the odometry for an omnidirectional robot with four mecanum wheels.
<br />

## Group members:
- ###  [Gabriele Ginestroni](https://github.com/gabrieleginestroni) | 10687747
- ###  [Tommaso Capacci](https://github.com/TommasoCapacci) | 10654230

## Files
All of our code has been organized in a single package named _project1_. Inside this package we can find some subdirectories:

Directory | Description
--------|------------
bags | ros bags provided with the project specification
cfg | file used for dynamic reconfigure of the integration type parameter
csv | support files used as input of python script for parameter calibration
launch | launch file to start the implemented nodes
msg | custom messages definition files
python | script used for parameter calibration
src | source files of ros nodes
srv | odometry reset service declaration file


## Instructions
To start the project type the following code:

- ##### start roscore 
  ```shell
   > roscore
    ```


- ##### compile project in a new shell
  ```shell
   > cd your_catkin_workspace
   > catkin_make
  ```

- ##### launch project nodes
  ```shell
   > roslaunch project1 launcher.launch
  ```
  
## Project description

- ### Topics
Topic   | Message type | Publisher | Subscribers | Description
--------|------------|------|---------|---------
/cmd_vel | geometry_msgs/TwistStamped | /vel_pub | /odom_pub, /inverter | base_link linear and angular velocities
/odom | nav_msgs/Odometry | /odom_pub | | global position computed by integrating base_link velocities
/robot/pose | geometry_msgs/PoseStamped | | /pose_broadcaster | ground truth pose measured with Optitrack system
/w_rpm | project1/RpmStamped | /vel_pub | | wheels angular speed computed from RPM [rad/s]
/w_ticks | project1/RpmStamped | /vel_pub | | wheels angular speed computed from TICKS [rad/s]
/wheel_states | sensor_msgs/JointState | | /vel_pub | motor angular speed [rad/min] and current motor encoder position fo each wheel's motor
/wheels_rpm | project1/RpmStamped | /vel_pub | | wheels angular speed computed by inverse kinematics

