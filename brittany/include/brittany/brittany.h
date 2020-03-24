#ifndef BRITTANY_HH
#define BRITTANY_HH
#include <unistd.h>

#include <ros/ros.h>
#include "tf/message_filter.h"
#include "petra/People.h"
#include "sensor_msgs/LaserScan.h"
#include <brittany/networkPrediction.h>
#include <brittany/Person.h>
#include <opencv/cv.hpp>
#include "std_msgs/String.h"
#include<iostream>

using namespace networkprediction;
using namespace cv;
#define LENGTH_MATRIX 256

namespace brittany{

  class Brittany{

    public:
      Brittany(ros::NodeHandle nh);
      ~Brittany();
      bool rosbag;



    private:
      ros::Subscriber scan_sub, petra_sub, start_stop_sub;
      ros::Publisher restart_petra_pub, identificador_pub;
      std::vector<sensor_msgs::LaserScan> historicScan;
      NetworkPrediction* network;
      std::vector<Person*> vectorPeople;
      float range_person;
      float brittanyTimer;
      ros::Timer timer;
      bool globalStartStop;

      void petraCallback(const petra::People& petra);
      void scanCallback(const sensor_msgs::LaserScan& scan);
      void timerCallback(const ros::TimerEvent& t);
      void startStopCallback(const std_msgs::String& startStop);

      sensor_msgs::LaserScan getLaserScan(std_msgs::Header header_petra);
      int getPositionPersonVector(std::string name);
      cv::Mat classify_scan_data(sensor_msgs::LaserScan scan_info , geometry_msgs::Point point_person);
      bool isGoodValue(float num);
      geometry_msgs::Point getPointXY(float range, int index, float angleIncrement, float angleMin);
      void getPointInMatrix(geometry_msgs::Point pointLaser, int *i, int *j);
      bool is_in_range(geometry_msgs::Point point_person, geometry_msgs::Point point_laser);
      void identifyPeople();

  };
};

#endif
