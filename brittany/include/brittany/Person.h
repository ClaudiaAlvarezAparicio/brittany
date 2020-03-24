#ifndef BIOMETRIC_LABELING_HH
#define BIOMETRIC_LABELING_HH
class Person{
  public:
    std::string id;
    std::string authentication_id;
    std::vector< cv::Mat > vectorImages;
    float accuracy;
};

#endif
