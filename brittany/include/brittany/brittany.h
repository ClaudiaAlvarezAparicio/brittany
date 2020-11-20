#ifndef BRITTANY_HH
#define BRITTANY_HH
#include <unistd.h>

#include <ros/ros.h>
#include "tf/message_filter.h"
#include "petra/People.h"
#include <sensor_msgs/LaserScan.h>
#include <brittany/networkPrediction.h>
#include <brittany/Person.h>
#include <opencv/cv.hpp>
#include "std_msgs/String.h"
#include <iostream>
#include "tf/tf.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;
using namespace networkprediction;
using namespace cv;
#define LENGTH_MATRIX 256

namespace brittany{

  class Brittany{

    public:
      Brittany(ros::NodeHandle nh);
      ~Brittany();
      bool rosbag;
      void identifyPeople();
      NetworkPrediction* network;
      Person* person_to_identify;
      std::vector<Person*> vectorPeople;
      int num_images;
      int num_steps;



    private:
      ros::Subscriber start_stop_sub;
      ros::Publisher restart_petra_pub, identificador_pub;
      std::vector<sensor_msgs::LaserScan> historicScan;

      message_filters::Subscriber<petra::People> people_sub;
      message_filters::Subscriber<sensor_msgs::LaserScan>  scan_sub;
      typedef sync_policies::ApproximateTime<petra::People, sensor_msgs::LaserScan> MySyncPolicy;
      typedef Synchronizer<MySyncPolicy> Sync;
      boost::shared_ptr<Sync> sync;


      
      float range_person;
      ros::Timer timer;
      bool globalStartStop;
      std::string scan_topic, rosbag_file;
      geometry_msgs::Point last_position;

      void peopleScanCallback(const petra::PeopleConstPtr& people, const sensor_msgs::LaserScanConstPtr& scan);

      //void petraCallback(const petra::People& petra);
      //void scanCallback(const sensor_msgs::LaserScan& scan);
      void timerCallback(const ros::TimerEvent& t);
      void startStopCallback(const std_msgs::String& startStop);

      sensor_msgs::LaserScan getLaserScan(std_msgs::Header header_petra);
      int getPositionPersonVector(std::string name);
      cv::Mat classify_scan_data(const sensor_msgs::LaserScanConstPtr& scan_info , geometry_msgs::Point point_person, cv::Mat image);
      bool isGoodValue(float num);
      geometry_msgs::Point getPointXY(float range, int index, float angleIncrement, float angleMin);
      void getPointInMatrix(geometry_msgs::Point pointLaser, int *i, int *j);
      bool is_in_range(geometry_msgs::Point point_person, geometry_msgs::Point point_laser);
      

  };
};

#endif
