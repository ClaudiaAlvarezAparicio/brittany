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
To test Brittany we have create two docker images, steps to launch it:  
* ROS Kinetic
```  
$ docker pull claudiaalvarezaparicio/brittany:kinetic  
$ docker run -d -p 6901:6901 -e VNC_PW=brittany --name=brittany claudiaalvarezaparicio/brittany:kinetic  
```  
* ROS Melodic
```  
$ docker pull claudiaalvarezaparicio/brittany:melodic  
$ docker run -d -p 6901:6901 -e VNC_PW=brittany --name=brittany claudiaalvarezaparicio/brittany:melodic  
```  

In the browser: http://localhost:6901/  
Password: brittany  
  
### Execute Brittany in docker  
Open terminal:   
  
* Example with a user of the system  
```  
$ roslaunch brittany brittany_rosbag.launch rosbag_file:=/home/student/rosbags_brittany/user0/user0-kitchen-07.bag  
```  
Output in terminal:  
```  
+++++++++++ Processing Users identification... +++++++++++  
The user is: user_0  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
```  
* Example with a NO user of the system:  
```  
$ roslaunch brittany brittany_rosbag.launch rosbag_file:=/home/student/rosbags_brittany/no-user/no-user-livingroom-door-02.bag  
```  
Output in terminal:  
```  
+++++++++++ Processing Users identification... +++++++++++  
The user is: user_1  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  

OR

+++++++++++ Processing Users identification... +++++++++++  
The user is: NOT IDENTIFIED IN THE SYSTEM  
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  


```  
## Stop and Remove Docker Container   
```  
$ docker stop brittany
$ docker rm brittany 
```  
