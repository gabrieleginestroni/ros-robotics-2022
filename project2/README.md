# Robotics - Project 2 2021/2022 

The second project requires to perform mapping and localization with an homemade omnidirectional robot with four mecanum wheels.
<br />

## Group members (with person code):
- ###  [Gabriele Ginestroni](https://github.com/gabrieleginestroni) (10687747)
- ###  [Tommaso Capacci](https://github.com/TommasoCapacci) (10654230)

## Files
All of our code has been organized in a single package named _project2_. Inside this package we can find the following subdirectories:

| Directory | Description                                                            |
|-----------|------------------------------------------------------------------------|
| bags      | ros bags provided with the project specification                       |
| config    | configuration files for gmapping and  amcl                             |
| launch    | launch files to start mapping and localization                          |
| scripts   | python ros nodes for trajectory and map editing                                                       |
| src       | ros node for publishing the odometry tf                                           |
| srv       | odometry reset service declaration file                                |
| map       | odometry reset service declaration file                                |

## Instructions
To start the project type the following code:

- ##### compile project in a new shell :
  ```shell
   > cd path/to/your/catkin_workspace
   > catkin_make
  ```

- ##### launch gmapping :
  ```shell
   > roslaunch project2 mapping.launch
  ```

- ##### launch amcl localization :
  ```shell
   > roslaunch project2 localization.launch
  ```
  
## Project description

- ### Nodes for gmapping
| Node      | Description                                                       |
|-----------|-------------------------------------------------------------------|
| /base_link_to_laser_front  | static TF broadcaster              |
| /base_link_to_laser_rear | static TF broadcaster        |
| /base_link_to_scan | static TF broadcaster |
| /laserscan_multi_merger |node from ira_laser_tools for merging scans from the two sensors  |
| /odom_tf | TF broadcaster for odometry  |
| /occupancy_grid_processing | outputs de-noised version of the map as a topic   |
| /slam_gmapping | gmapping node for mapping  |

- ### Nodes for localization
| Node      | Description                                                       |
|-----------|-------------------------------------------------------------------|
| /base_link_to_laser_front  |  static TF broadcaster            |
| /base_link_to_laser_rear |  static TF broadcaster       |
| /base_link_to_scan | static TF broadcaster  |
| /laserscan_multi_merger | node from ira_laser_tools for merging scans from the two sensors  |
| /odom_tf | TF broadcaster for odometry |
| /map_server | publishes map as a topic  |
| /path_finder | python node that prints the robot trajectory into the map image |
| /amcl | amcl node for localization   |

## Trajectory drawer service
The project provides a service that can be called to save a pgm image of the map with the trajectory the robot has followed so far 
```shell
   > rosservice call /draw_path
  ```
the map will be saved in the map folder as _map_with_path.pgm_

## TF 
![](img/tf_tree.png)
In case of gmapping the tf transform between map and odom is provided by the /slam_gmapping node

## Fine tuning
Starting from the default parameters we tuned the ones related to the sensors (YDLIDAR G4) to match the vendor datasheet.
Other relevant amcl parameters that we changed are odom_model_type (omni) and laser model type (beam).