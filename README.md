# BRITTANY (Biometric RecognITion Through gAit aNalYsis)

Brittany allows a Biometric recognition through gait analysis by using LIDAR sensors. The system is based on a classification neural network which allow identify 5 diferent users in the system or intruders in it.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

* ROS KINETIC or ROS MELODIC
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

## Docker Image   
To test Brittany we have created a docker image, steps to launch it:  
```
$ docker pull claudiaalvarezaparicio/brittany:latest  
$ docker run claudiaalvarezaparicio/brittany:latest <custom, lenet, alexnet> <5, 10> <0, 1, 2>  
where:
<custom, lenet, alexnet>: the model to execute
<5, 10>: number of images to concatenate
<0, 1, 2>: number of steps between images   
```  

### Output 
The result of executing each rosbag. Examples:  

+++++++++++ Processing User identification for rosbag: /user0-kitchen-07.bag +++++++++++  
The user is: user_0  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  

+++++++++++ Processing User identification for rosbag: /user1-livingroom-window-10.bag +++++++++++  
The user is: user_1  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  

+++++++++++ Processing User identification for rosbag: /user2-kitchen-11.bag +++++++++++   
The user is: user_2  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  

+++++++++++ Processing User identification for rosbag: /user3-livingroom-window-12.bag +++++++++++  
The user is: user_3  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  

+++++++++++ Processing User identification for rosbag: /user4-livingroom-door-08.bag +++++++++++  
The user is: user_4  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  

+++++++++++ Processing User identification for rosbag: /no-user-livingroom-window-06.bag +++++++++++  
The user is: NOT IDENTIFIED IN THE SYSTEM  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
