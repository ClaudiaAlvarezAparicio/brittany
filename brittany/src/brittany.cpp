#include <brittany/brittany.h>

using namespace brittany;

using namespace std;

int main(int argc, char** argv){

  ros::init(argc, argv, "brittany");
  ros::NodeHandle nh("~");
  Brittany* bl = new Brittany(nh);

  ros::spin();

  // If the execution depends of a rosbag, at final return data
  if (bl->rosbag == true){
    bl->identifyPeople();
  }

  bl->~Brittany();
  return 0;
}


Brittany::Brittany(ros::NodeHandle nh):range_person(0.50){
  std::string networkModel;
  // Save params
  nh.getParam("rosbag", this->rosbag);
  nh.getParam("scan_topic", this->scan_topic);
  nh.getParam("rosbag_file", this->rosbag_file);
  nh.getParam("num_img_concat", num_images);
  nh.getParam("networkModel", networkModel);
  nh.getParam("num_steps_between_concat", num_steps);
  this->num_steps = num_steps + 1;

  // Publishers and subscribers
  identificador_pub = nh.advertise<std_msgs::String>("/brittany",1);
  people_sub.subscribe(nh, "/people", 1);
  scan_sub.subscribe(nh, this->scan_topic, 1);
  // LIDAR and PeTra
  sync.reset(new Sync(MySyncPolicy(10), people_sub, scan_sub));      
  sync->registerCallback(boost::bind(&Brittany::peopleScanCallback, this, _1, _2));

  // Object to predict images
  this->network = new NetworkPrediction(networkModel);

  // Object with data of the person to identify
  person_to_identify = new Person();

  if(this->rosbag == true){
    this->globalStartStop=true;
  }else{
    start_stop_sub = nh.subscribe("/start_stop_brittany", 1, &Brittany::startStopCallback, this);
    restart_petra_pub = nh.advertise<std_msgs::String>("/restart", 1);
    this->globalStartStop=false;
  }

}

Brittany::~Brittany(){
  delete this->network;
  this->vectorPeople.clear();
  this->historicScan.clear();
}

/*
 * Method used if data is processed in real time. If the received text is "start", 
 * people vector is cleaned and a message is sent to PeTra to start the people counter 
 * from zero and start collecting data again
 */
void Brittany::startStopCallback(const std_msgs::String& startStop){
  if(startStop.data == "start"){
    this->vectorPeople.clear();
    this->historicScan.clear();
    restart_petra_pub.publish(startStop);
    this->globalStartStop=true;
  }else if(startStop.data == "stop"){
    this->globalStartStop=false;
    this->identifyPeople();
  }
}

void Brittany::peopleScanCallback(const petra::PeopleConstPtr& people, const sensor_msgs::LaserScanConstPtr& scan){
  if(this->globalStartStop == true){
    cv::Mat image = cv::Mat::zeros(cv::Size(LENGTH_MATRIX,LENGTH_MATRIX), CV_32FC1);
    bool find = false;

    this->person_to_identify->id="person_0";
    this->person_to_identify->accuracy=0;

    for (int i = 0; i < people->people.size(); i++){
      image = this->classify_scan_data(scan, people->people[i].position_person, image); 
      find = true;  
    }    

    if (find == true){
      this->person_to_identify->vectorImages.push_back(image); 
    }
  }
}


/*
 * Method that save in the matrix_raw, with 1s the points of the laser that compose a person
 */
cv::Mat Brittany::classify_scan_data(const sensor_msgs::LaserScanConstPtr& scan_info , geometry_msgs::Point point_person, cv::Mat image){
    //cv::Mat image = cv::Mat::zeros(cv::Size(LENGTH_MATRIX,LENGTH_MATRIX), CV_32FC1);
    std::vector<float> vector_scan = scan_info->ranges;
    geometry_msgs::Point point_laser;
    bool in_range = false;
    int j, k = 0;
    for (int i= 0; i < vector_scan.size(); i++){
      if (this->isGoodValue(vector_scan[i]) == true){
            // Calculate the xy point from the angle/range
            point_laser = this->getPointXY(vector_scan[i],i, scan_info->angle_increment, scan_info->angle_min);

            // Calculate if the point is in range of the person
            in_range = this-> is_in_range(point_person, point_laser);

            this->getPointInMatrix(point_laser,&j,&k);
            if(j>=0 and j<LENGTH_MATRIX and k>=0 and k<LENGTH_MATRIX and in_range == true){
                  image.at<float>(j,k) = 1.0;
            }
      }
    }
    return image;
}

/*
 * Method that calculate if the point of the laser is in range of the person
 * Returns true if it is in range if not it return false
 */
bool Brittany::is_in_range(geometry_msgs::Point point_person, geometry_msgs::Point point_laser){
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
 * Method that determines if the value that recives is not equal to inf, nan, -inf or -nan, and if it is in range
 * Return true if is a good value and false if is not
 */
bool Brittany::isGoodValue(float num){
  bool goodValue = true;

  if (std::isnan(num) || std::isnan(-num)){
    goodValue = false;
  } else if (std::isinf(num) || std::isinf(-num)){
    goodValue = false;
  } else if (num > 5.12){
    goodValue = false;
  }

  return goodValue;
}

/*
 * Method that transform the range provided by the laser to polar coordinates
 * Return a Point object with the point in cartesian coordinates (x,y)
 */
geometry_msgs::Point Brittany::getPointXY(float range, int index, float angleIncrement, float angleMin){

  geometry_msgs::Point pointXY;
  float polarD = range;
  float polarAngle = 0;
  float polarAngleRadians = 0;
  float cartesianX = 0;
  float cartesianY = 0;
  float alfaRadians = 0;

  // alfa is the complementary angle in the polar coordinates
  alfaRadians = (angleIncrement * (index + 1)) + angleMin;

  // Calculate the point (x,y)
  cartesianX = polarD * cos(alfaRadians);
  cartesianY = polarD * sin(alfaRadians);

  // Add the values to the object to return it
  pointXY.x = cartesianX;
  pointXY.y = cartesianY;

  return pointXY;
}

/*
 * Method that calculates the point xy of the laser in a ij position in the matrix
 */
void Brittany::getPointInMatrix(geometry_msgs::Point pointLaser, int *i, int *j){
  int halfMatrix = LENGTH_MATRIX / 2;
  // transform the values to cm and divided it by 2 to do the transformation to matrix
  int pointX = floor((pointLaser.x * 100)/2);
  int pointY = floor((pointLaser.y * 100)/2);

  *i = pointX;
  *j = halfMatrix - pointY;
}

/*
 * 
 */
void Brittany::identifyPeople(){
  // To print the name of the rosbag file    
  if (this->rosbag == true){
    size_t found = this->rosbag_file.find_last_of("/");
    this->rosbag_file.replace(0,found,"");
    std::cout << "+++++++++++ Processing User identification for rosbag: "<< this->rosbag_file <<  " +++++++++++" << '\n';
  }else{
    std::cout << "+++++++++++ Processing User identification... +++++++++++" << '\n';
  }
  // Create images with the configuration
  std::vector< cv::Mat > images_person;
  std::vector< std::vector<float> > predictions;

  predictions.clear(); // clear vector predictions 
  images_person = this->person_to_identify->vectorImages; // take matrix vector

  int aux_value = num_images * num_steps;

  // If there was more images than the needed
  if (images_person.size() >= aux_value){
    for(int j = 0; j < images_person.size() - aux_value;j++){
      cv::Mat aux = images_person[j]; // For each matrix
      for(int k = j; k < (j + aux_value); (k = k + num_steps)){ // The next matrix (3 hop)
        for (int l = 0; l < LENGTH_MATRIX; l++){ // For the rows of the matrix
          for (int m = 0; m < LENGTH_MATRIX; m++){
            if(aux.at<float>(l,m)+images_person[k].at<float>(l,m) == 0){
              aux.at<float>(l,m) = 0.0;
            }else{
              aux.at<float>(l,m) = 1.0;
            }
          }
        }
      }
      predictions.push_back(this->network->prediction(aux));
    }
    // To print the float values with two decimals  
    std::cout << std::setprecision(2);
    
    //cout << "Rosbag: " << this->rosbag_file << "\n";
    //cout << "Numero de imagenes: " << this->person_to_identify->vectorImages.size() << "\n";
    //cout << "Numero de mapas de ocupacion: " << predictions.size() << "\n";
     
    // The total counter vector has a length of 5, the number of people that the network could recognize 
    std::vector<int> total_counter(5, 0);
    int mayor_porcentaje = 0;
    float valor_prediccion = 0.0;

    // Through the list of predictions, the user who has a higher probability increases the counter of his position
    for (int i = 0; i < predictions.size(); i++){
      //cout << "Prediccion " << i << ": ";
      mayor_porcentaje = -1;
      valor_prediccion = 0.0;
      for(int j = 0; j < predictions[i].size();j++){
        //std::cout << predictions[i][j] << '\t';
        if(predictions[i][j] > valor_prediccion){
          mayor_porcentaje = j;
          valor_prediccion = predictions[i][j];
        }
      }
      if(mayor_porcentaje != -1){
        total_counter[mayor_porcentaje] = total_counter[mayor_porcentaje] + 1;
      }
      //  cout << "\n";
    }

    // Go through the vector of the counter to take the user who has more hits
    int posicion_persona = 0;
    valor_prediccion = 0.0;
    for (int i = 0; i< total_counter.size();i++){
      if(total_counter[i] > valor_prediccion){
        posicion_persona = i;
        valor_prediccion = total_counter[i];
      }
    }

    this->person_to_identify->authentication_id = "user_" + std::to_string(posicion_persona);

    // Percentage
    float percentaje = ( total_counter[posicion_persona] * 100 ) /  predictions.size();

    this->person_to_identify->accuracy = percentaje;
  } // End if there was more images than the needed
    


  // Publish the data in terminal and in topic "/brittany"
  std_msgs::String usuario;
  //cout << "Precision: " << this->person_to_identify->accuracy << "\n";
  if (this->person_to_identify->accuracy >= 85) {
    std::cout << "The user is: " << this->person_to_identify->authentication_id << '\n';
    usuario.data = this->person_to_identify->id + " --> " + this->person_to_identify->authentication_id;
  }else if(this->person_to_identify->accuracy != 0){
    std::cout << "The user is: NOT IDENTIFIED IN THE SYSTEM" << '\n';
    usuario.data = this->person_to_identify->id + " --> NOT IDENTIFIED IN THE SYSTEM";
  }else{
    std::cout << "The user is: NOT ENOUGH DATA" << '\n';
    usuario.data = this->person_to_identify->id + " --> NOT ENOUGH DATA";
  }

  identificador_pub.publish(usuario);

  std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << '\n';
  
  if(this->rosbag == true){
    ros::shutdown();
  }
}
