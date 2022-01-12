#include <data_labeling_brittany/data_labeling_brittany.h>

using namespace data_labeling_brittany;

using namespace std;

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
  bl->~DataLabelingBrittany();

  return 0;
}

DataLabelingBrittany::DataLabelingBrittany(ros::NodeHandle nh):range_person(0.50){
  // Save params
  nh.getParam("rosbag_file", rosbag);
  nh.getParam("npy_directory", directory);
  nh.getParam("scan_topic", this->scan_topic);
  nh.getParam("id_label_person", id_label_person);
  nh.getParam("num_img_concat", num_images);
  nh.getParam("num_steps_between_concat", num_steps);
  this->num_steps = num_steps + 1;

  // Sincronizamos la recogida de datos del LIDAR y de PeTra
  people_sub.subscribe(nh, "/people", 1);
  scan_sub.subscribe(nh, this->scan_topic, 1);
  
  sync.reset(new Sync(MySyncPolicy(10), people_sub, scan_sub));      
  sync->registerCallback(boost::bind(&DataLabelingBrittany::peopleScanCallback, this, _1, _2));
}

void DataLabelingBrittany::peopleScanCallback(const petra::PeopleConstPtr& people, const sensor_msgs::LaserScanConstPtr& scan){

    // Matrix initialization
    this->initialize_matrix();
    bool find = false;

    for (int i = 0; i < people->people.size(); i++){
      this->classify_scan_data(scan, people->people[i].position_person); 
      find = true;  
    }    

    if (find == true){
      vector_matrix.push_back(this->matrix_to_vector(matrix));  
    }
}

/*
 *  Method that save in the matrix_raw, the points of the lidar where was a person. Saved as 1's 
 */
void DataLabelingBrittany::classify_scan_data(const sensor_msgs::LaserScanConstPtr& scan_info , geometry_msgs::Point point_person){
  std::vector<float> vector_scan = scan_info->ranges;
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
geometry_msgs::Point DataLabelingBrittany::get_point_xy(float range, int index, const sensor_msgs::LaserScanConstPtr& scan_data){

  geometry_msgs::Point point_xy;
  float polar_d = range;
  float polar_angle = 0;
  float polar_angle_radians = 0;
  float cartesian_x = 0;
  float cartesian_y = 0;
  float alfa_radians = 0;

  // alfa is the complementary angle in the polar coordinates
  alfa_radians = (scan_data->angle_increment * (index + 1)) + scan_data->angle_min;

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

  int aux_value = num_images * num_steps;

  for (int i = 0; i < vector_matrix.size() - aux_value; i++){
    std::vector<int> aux = vector_matrix[i];
    for (int j = i; j < (i + aux_value) ; (j = j + num_steps)){
      for (int k = 0; k < vector_matrix[j].size(); k++){
        if(aux[k]+vector_matrix[j][k] == 0){
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

    // Imagenes
  const long unsigned raw [] = {vector.size(), 256, 256};
  std::vector<int> global_vector_raw = vector_to_global_vector(vector);
  npy::SaveArrayAsNumpy(directory+"/raw/"+get_name_npy_file(), false, 3, raw, global_vector_raw);

  // Labels
  const long unsigned label [] = {vector.size()};
  std::vector<int> global_vector_label = vector_labels(vector.size());
  npy::SaveArrayAsNumpy(directory+"/label/"+get_name_npy_file(), false, 1, label, global_vector_label);
}

/*
 * Method that builds a vector with the id received to label data
 */
std::vector<int> DataLabelingBrittany::vector_labels(int number){
  std::vector<int> label_vector;

  for (int i = 0; i < number ; i++){
    label_vector.push_back(id_label_person);
  }

  return label_vector;
}

std::string DataLabelingBrittany::get_name_npy_file(){
  std::string archivo = rosbag;
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
