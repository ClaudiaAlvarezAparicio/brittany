#ifndef DATA_LABELING_BRITTANY_HH
#define DATA_LABELING_BRITTANY_HH


#include "npy.hpp"
#include <unistd.h>
#include <iostream>
#include <ros/ros.h>
#include "petra/People.h"
#include "petra/Person.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>


using namespace ros;
using namespace message_filters;

#define LENGTH_MATRIX 256

namespace data_labeling_brittany{

  class DataLabelingBrittany{

    public:
        DataLabelingBrittany(ros::NodeHandle nh);
        ~DataLabelingBrittany(){}

        int id_label_person;
        int num_images;
        int num_steps;
        
        std::string directory;
        std::string rosbag;

        std::vector<std::vector<int> > vector_matrix;

        std::vector<std::vector<int> > concatenate_vector();
        void save_npy(std::vector<std::vector<int> > vector);


    private:
        message_filters::Subscriber<petra::People> people_sub;
        message_filters::Subscriber<sensor_msgs::LaserScan>  scan_sub;
        typedef sync_policies::ApproximateTime<petra::People, sensor_msgs::LaserScan> MySyncPolicy;
        typedef Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync;

        int matrix[LENGTH_MATRIX][LENGTH_MATRIX];
        float range_person;
        
        std::string scan_topic;
    
        void peopleScanCallback(const petra::PeopleConstPtr& people, const sensor_msgs::LaserScanConstPtr& scan);
        void classify_scan_data(const sensor_msgs::LaserScanConstPtr& scan_info , geometry_msgs::Point point_person);
        geometry_msgs::Point get_point_xy(float range, int index,const sensor_msgs::LaserScanConstPtr& scan_data);
        bool is_good_value(float num);
        bool is_in_range(geometry_msgs::Point point_person, geometry_msgs::Point point_laser);
        void get_point_in_matrix(geometry_msgs::Point point_laser, int *i, int *j);
        std::vector<int> matrix_to_vector(int matriz[LENGTH_MATRIX][LENGTH_MATRIX]);
        void initialize_matrix();
        std::vector<int> vector_to_global_vector(std::vector<std::vector<int> > vector);
        std::string get_name_npy_file();
        std::vector<int> vector_labels(int number);
  };
};

#endif
