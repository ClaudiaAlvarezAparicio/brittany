#include <data_labeling_brittany/data_labeling_brittany.h>

using namespace data_labeling_brittany;

//using namespace cv;
using namespace std;
using namespace message_filters;

int main(int argc, char** argv){

  ros::init(argc, argv, "data_labeling_brittany");
  ros::NodeHandle nh("~");
  DataLabelingBrittany* bl = new DataLabelingBrittany(nh);

  ros::spin();
  // The execution stops

  // concatenate vectors
  std::vector<std::vector<int> > vector_save = bl->concatenate_vector();
  // save npy file
  bl->save_npy(vector_save);

  return 0;
}

DataLabelingBrittany::DataLabelingBrittany(ros::NodeHandle nh):range_person(0.50){
  // Save params
  nh.getParam("rosbag_file", this->rosbag);
  nh.getParam("npy_directory", this->directory);
  nh.getParam("scan_topic", this->scan_topic);
  

  // Publishers and subscribers
  petra_sub = nh.subscribe("/people", 1, &DataLabelingBrittany::petraCallback, this);
  scan_sub = nh.subscribe(this->scan_topic, 1, &DataLabelingBrittany::scanCallback, this);
  // Matrix initialization
  this->initialize_matrix();
}

void DataLabelingBrittany::petraCallback(const petra::People& petra){
    // Get lidar scan
    sensor_msgs::LaserScan scan = this->getLaserScan(petra.header);

    petra::Person person;
    // Matrix initialization
    this->initialize_matrix();

    for (int i = 0; i < petra.people.size(); i++){
      this->classify_scan_data(scan, petra.people[i].position_person);
    }
}


void DataLabelingBrittany::scanCallback(const sensor_msgs::LaserScan& scan){
  this->historicScan.push_back(scan);
}

/*
 * Method that returns the lidar scan with the same  timestamp than petra
 */
sensor_msgs::LaserScan DataLabelingBrittany::getLaserScan(std_msgs::Header header_petra){
  sensor_msgs::LaserScan scan;
  int position = 0;
  for (int i = 0; i< this->historicScan.size(); i++){
    if(this->historicScan[i].header.stamp == header_petra.stamp){
      scan = historicScan[i];
      position = i;
    }
  }
  // Remove the previous scans
  this->historicScan.erase(this->historicScan.begin(), this->historicScan.begin()+position);
  // return scan msgs
  return scan;
}


/*
 *  Method that save in the matrix_raw, the points of the lidar where was a person. Saved as 1's 
 */
void DataLabelingBrittany::classify_scan_data(sensor_msgs::LaserScan scan_info , geometry_msgs::Point point_person){
    std::vector<float> vector_scan = scan_info.ranges;
    geometry_msgs::Point point_laser;
    bool in_range = false;
    int j, k, count_points_label = 0;

    for (int i= 0; i < vector_scan.size(); i++){
       j = k = 0;
      // If the range is good
       if (this->is_good_value(vector_scan[i]) == true){
         // Calculate the xy point from the angle/range
         point_laser = this->get_point_xy(vector_scan[i],i, scan_info);

         // Calculate if the point is in range of the person
         in_range = this-> is_in_range(point_person, point_laser);

         // Calculate the position of the point in the matrix
         this->get_point_in_matrix(point_laser,&j,&k);
         // If the point is in range it is write in matrix
         if(j>=0 and j<LENGTH_MATRIX and k>=0 and k<LENGTH_MATRIX and in_range == true){
           matrix[j][k] = 1;
         }
       }
    }
    this->vector_matrix.push_back(this->matrix_to_vector(matrix));

}


/*
 * Method that transform the matrix recived to a vector and return it
 */
std::vector<int> DataLabelingBrittany::matrix_to_vector(int matriz[LENGTH_MATRIX][LENGTH_MATRIX]){
  std::vector<int> vector_out;

  for (int i = 0; i < LENGTH_MATRIX; i++){
    for (int j = 0; j < LENGTH_MATRIX;j++){
      vector_out.push_back(matriz[i][j]);
    }
  }
  return vector_out;
}

/*
 * Method that calculates the point xy of the laser in a ij position in the matrix
 */
void DataLabelingBrittany::get_point_in_matrix(geometry_msgs::Point point_laser, int *i, int *j){
  int half_matrix = LENGTH_MATRIX / 2;
  // transform the values to cm and divided it by 2 to do the transformation to matrix
  int point_x = floor((point_laser.x * 100)/2);
  int point_y = floor((point_laser.y * 100)/2);

  *i = point_x;
  *j = half_matrix - point_y;
}

/*
 * Method that calculate if the point of the laser is in range of the person
 * Returns true if it is in range if not it return false
 */
bool DataLabelingBrittany::is_in_range(geometry_msgs::Point point_person, geometry_msgs::Point point_laser){
  float distancia_euclidea = 0;

  // Calculate de euclidean distance between point of person and the point of laser
  distancia_euclidea = sqrt(pow(point_laser.x - point_person.x , 2) +pow(point_laser.y - point_person.y , 2));

  if (distancia_euclidea < this->range_person){
    return true;
  }else{
    return false;
  }
}

/*
 * Method that transform the range provided by the laser to polar coordinates
 * Return a Point object with the point in cartesian coordinates (x,y)
 */
geometry_msgs::Point DataLabelingBrittany::get_point_xy(float range, int index,sensor_msgs::LaserScan scan_data){

  geometry_msgs::Point point_xy;
  float polar_d = range;
  float polar_angle = 0;
  float polar_angle_radians = 0;
  float cartesian_x = 0;
  float cartesian_y = 0;
  float alfa_radians = 0;

  // alfa is the complementary angle in the polar coordinates
  alfa_radians = (scan_data.angle_increment * (index + 1)) + scan_data.angle_min;

  // Calculate the point (x,y)
  cartesian_x = polar_d * cos(alfa_radians);
  cartesian_y = polar_d * sin(alfa_radians);

  // Add the values to the object to return it
  point_xy.x = cartesian_x;
  point_xy.y = cartesian_y;

  return point_xy;
}

/*
 * Method that determines if the value tha recives is not equal to inf, nan, -inf or -nan, and if it is in range
 * Return true if is a good value and false if is not
 */
bool DataLabelingBrittany::is_good_value(float num){

  bool good_value = true;
  if (isnan(num) || isnan(-num)){
    good_value = false;
  }

  if (isinf(num) || isinf(-num)){
    good_value = false;
  }

  // If the value is greater than 5.12m, we ignore it
  if (num > 5.12){
    good_value = false;
  }

  return good_value;
}

void DataLabelingBrittany::initialize_matrix(){
  for (int i = 0; i < LENGTH_MATRIX; i++){
    for (int j = 0; j < LENGTH_MATRIX; j++){
      matrix[i][j] = 0;
    }
  }
}


// CREATION OF IMAGES AND SAVE NPY FILES //
std::vector<std::vector<int> > DataLabelingBrittany::concatenate_vector(){
  std::vector<std::vector<int> > vector_out;

  // concat_5_1
  //for (int i = 0; i < this->vector_matrix.size()-5; i++){
  //  std::vector<int> aux = this->vector_matrix[i];
  //  for (int j = i; j < i + 5; j++){
  //    for (int k = 0; k < this->vector_matrix[j].size(); k++){
  //      if(aux[k]+this->vector_matrix[j][k] == 0){
  //         aux[k]=0;
  //      }else{
  //         aux[k]=1;
  //      }
  //    }
  //  }
  //  vector_out.push_back(aux);
  //}

  // concat_5_2
  //for (int i = 0; i < this->vector_matrix.size()-10; i++){
  //  std::vector<int> aux = this->vector_matrix[i];
  //  for (int j = i; j < i + 10; (j = j + 2)){
  //    for (int k = 0; k < this->vector_matrix[j].size(); k++){
  //      if(aux[k]+this->vector_matrix[j][k] == 0){
  //         aux[k]=0;
  //      }else{
  //         aux[k]=1;
  //      }
  //    }
  //  }
  //  vector_out.push_back(aux);
  //}

  // concat_5_3
  //for (int i = 0; i < this->vector_matrix.size()-15; i++){
  //  std::vector<int> aux = this->vector_matrix[i];
  //  for (int j = i; j < i + 15; (j = j + 3)){
  //    for (int k = 0; k < this->vector_matrix[j].size(); k++){
  //      if(aux[k]+this->vector_matrix[j][k] == 0){
  //         aux[k]=0;
  //      }else{
  //         aux[k]=1;
  //      }
  //    }
  //  }
  //  vector_out.push_back(aux);
  //}

  // concat_10_1
  //for (int i = 0; i < this->vector_matrix.size()-10; i++){
  //  std::vector<int> aux = this->vector_matrix[i];
  //  for (int j = i; j < i + 10; j++){
  //    for (int k = 0; k < this->vector_matrix[j].size(); k++){
  //      if(aux[k]+this->vector_matrix[j][k] == 0){
  //         aux[k]=0;
  //      }else{
  //         aux[k]=1;
  //      }
  //    }
  //  }
  //  vector_out.push_back(aux);
  //}


  // concat_10_2
  //for (int i = 0; i < this->vector_matrix.size()-20; i++){
  //  std::vector<int> aux = this->vector_matrix[i];
  //  for (int j = i; j < i + 20; (j = j + 2)){
  //    for (int k = 0; k < this->vector_matrix[j].size(); k++){
  //      if(aux[k]+this->vector_matrix[j][k] == 0){
  //         aux[k]=0;
  //      }else{
  //         aux[k]=1;
  //      }
  //    }
  //  }
  //  vector_out.push_back(aux);
  //}

  // concat_10_3
  for (int i = 0; i < this->vector_matrix.size()-30; i++){
    std::vector<int> aux = this->vector_matrix[i];
    for (int j = i; j < i + 30; (j = j + 3)){
      for (int k = 0; k < this->vector_matrix[j].size(); k++){
        if(aux[k]+this->vector_matrix[j][k] == 0){
           aux[k]=0;
        }else{
           aux[k]=1;
        }
      }
    }
    vector_out.push_back(aux);
  }

  std::cout << "Number of images: "<< vector_out.size() << '\n';

  return vector_out;
}

void DataLabelingBrittany::save_npy(std::vector<std::vector<int> > vector){
  const long unsigned longs [] = {vector.size(), 256, 256, 1};

  std::vector<int> global_vector = this->vector_to_global_vector(vector);
  npy::SaveArrayAsNumpy(this->directory+this->get_name_npy_file(), false, 4, longs, global_vector);
}

std::string DataLabelingBrittany::get_name_npy_file(){
  std::string archivo = this->rosbag;
  size_t found = archivo.find_last_of("/");
  archivo.replace(0,found,"");
  found = archivo.find_last_of(".");
  archivo.replace(found,4,".npy");
  return archivo;
}

/*
 * Method that transform the vectors recived to a unique vector and return it
 */
std::vector<int> DataLabelingBrittany::vector_to_global_vector(std::vector<std::vector<int> > vector){
  std::vector<int> global_vector;

  for (int i = 0; i < vector.size(); i++){
    for (int j = 0; j < vector[i].size();j++){
      global_vector.push_back(vector[i][j]);
    }
  }
  return global_vector;
}
