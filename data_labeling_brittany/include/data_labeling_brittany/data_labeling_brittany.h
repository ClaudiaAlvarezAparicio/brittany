#ifndef DATA_LABELING_BRITTANY_HH
#define DATA_LABELING_BRITTANY_HH

#include <ros/ros.h>
#include "tf/message_filter.h"
#include "petra/People.h"
#include "sensor_msgs/LaserScan.h"
#include "npy.hpp"

#define LENGTH_MATRIX 256

namespace data_labeling_brittany{

  class DataLabelingBrittany{

    public:
        DataLabelingBrittany(ros::NodeHandle nh);
        ~DataLabelingBrittany(){}
        std::vector<std::vector<int> > concatenate_vector();
        void save_npy(std::vector<std::vector<int> > vector);

    private:
      int contador_mensajes;
      int matrix[LENGTH_MATRIX][LENGTH_MATRIX];
      std::string rosbag;
      std::string scan_topic;
      std::string directory;
      petra::Person backup_person;

      ros::NodeHandle private_nh_;
      ros::Subscriber scan_sub, petra_sub;
      std::vector<sensor_msgs::LaserScan> historicScan;
      std::vector<std::vector<int> > vector_matrix;
      float range_person;

      void petraCallback(const petra::People& petra);

      void scanCallback(const sensor_msgs::LaserScan& scan);
      sensor_msgs::LaserScan getLaserScan(std_msgs::Header header_petra);

      void initialize_matrix();
      void classify_scan_data(sensor_msgs::LaserScan scan_info , geometry_msgs::Point point_person);
      bool is_good_value(float num);
      geometry_msgs::Point get_point_xy(float range, int index, sensor_msgs::LaserScan scan_data);
      bool is_in_range(geometry_msgs::Point point_person, geometry_msgs::Point point_laser);
      void get_point_in_matrix(geometry_msgs::Point point_laser, int *i, int *j);
      std::vector<int> matrix_to_vector(int matriz[LENGTH_MATRIX][LENGTH_MATRIX]);
      std::vector<int> vector_to_global_vector(std::vector<std::vector<int> > vector);
      std::string get_name_npy_file();
  };
};

#endif
