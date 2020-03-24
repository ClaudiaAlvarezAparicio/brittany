# BRITTANY (Biometric RecognITion Through gAit aNalYsis)

Brittany is a tool which allows a Biometric recognition through gait analysis by using LIDAR sensors. The system is based on a classification neural network which allow identify 5 diferent users of the system or intruders in it.


## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

* ROS KINETIC
* Petra (People Tracking). Follow the steps in https://github.com/ClaudiaAlvarezAparicio/petra to install.


### Brittany Installation

```
$ cd ~/catkin_ws/src/  
$ git clone https://github.com/ClaudiaAlvarezAparicio/brittany.git
$ cd ..  
$ catkin_make  
```

### Execution

Execution with a rosbag:

```
$ roslaunch brittany brittany_rosbag.launch rosbag_file:=absolute_path_to_bag_file
```

Execution in real time:

```
$ roslaunch brittany brittany.launch
```