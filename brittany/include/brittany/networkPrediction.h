#ifndef NETWORKPREDICTION_HH
#define NETWORKPREDICTION_HH

#include "tensorflow/core/public/session.h"
#include "tensorflow/core/platform/env.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv/cv.hpp>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>

using namespace tensorflow;
using namespace ros;
using namespace cv;

#define LENGTH_MATRIX 256


namespace networkprediction{
  class NetworkPrediction{
  public:
    NetworkPrediction(std::string networkModel);
    ~NetworkPrediction();
    std::vector<float> prediction(cv::Mat image);

  private:
    Session* session;
    tensorflow::SessionOptions options;
    Status status;
    GraphDef graph_def;
    std::string model;

    void openConnection();
    void closeConnection();
  };
};
#endif
