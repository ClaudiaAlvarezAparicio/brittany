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
  
  return 0;
}


Brittany::Brittany(ros::NodeHandle nh):range_person(0.50){
  // Save params
  nh.getParam("rosbag", this->rosbag);

  // Publishers and subscribers
  petra_sub = nh.subscribe("/people", 1000, &Brittany::petraCallback, this);
  scan_sub = nh.subscribe("/scan", 1000, &Brittany::scanCallback, this);
  identificador_pub = nh.advertise<std_msgs::String>("/brittany",1);

  // Object to predict images
  this->network = new NetworkPrediction();

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

void Brittany::petraCallback(const petra::People& petra){
  if(this->globalStartStop == true){
    sensor_msgs::LaserScan scan = this->getLaserScan(petra.header);

    int positionPerson;
    for (int i=0; i < petra.people.size(); i++){
      Person * person;
      //positionPerson = getPositionPersonVector(petra.people[i].name);
      // Because we only have one person in each scene
      positionPerson = getPositionPersonVector("person_0");

      // If no person with this identifier exists, is created.
      if(positionPerson == -1){
        person = new Person();
        //person->id=petra.people[i].name;
        person->id="person_0";
        vectorPeople.push_back(person);
      }else{
        person = vectorPeople[positionPerson];
      }

      cv::Mat image = this->classify_scan_data(scan, petra.people[i].position_person);
      person->vectorImages.push_back(image);
    }
  }
}

/*
 * LIDAR Callback
 */
void Brittany::scanCallback(const sensor_msgs::LaserScan& scan){
  if(this->globalStartStop == true){
    this->historicScan.push_back(scan);
  }
}

/*
 * Method that save in the matrix_raw, with 1s the points of the laser that compose a person
 */
cv::Mat Brittany::classify_scan_data(sensor_msgs::LaserScan scan_info , geometry_msgs::Point point_person){
    cv::Mat image = cv::Mat::zeros(cv::Size(LENGTH_MATRIX,LENGTH_MATRIX), CV_32FC1);
    std::vector<float> vector_scan = scan_info.ranges;
    geometry_msgs::Point point_laser;
    bool in_range = false;
    int j, k = 0;
    for (int i= 0; i < vector_scan.size(); i++){
      if (this->isGoodValue(vector_scan[i]) == true){
            // Calculate the xy point from the angle/range
            point_laser = this->getPointXY(vector_scan[i],i, scan_info.angle_increment, scan_info.angle_min);

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
 * Return the LIDAR SCAN closest in time to PeTra data
 */
sensor_msgs::LaserScan Brittany::getLaserScan(std_msgs::Header header_petra){
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
  // return scan msg
  return scan;
}

/* 
 * Return the position of the person received in the global vector of people 
 */
int Brittany::getPositionPersonVector(std::string name){
  int position = -1;
  for(int i = 0; i < vectorPeople.size(); i++){
    if(vectorPeople[i]->id == name){
      position = i;
      break;
    }
  }
  return position;
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
  std::cout << "+++++++++++ Processing Users identification... +++++++++++" << '\n';  
  // Create images with concat_10_3 configuration
  std::vector< cv::Mat > images_person;
  std::vector< std::vector<float> > predictions;

  for (int i = 0; i< this->vectorPeople.size(); i++){ // select person 
    predictions.clear(); // clear vector predictions 
    images_person = this->vectorPeople[i]->vectorImages; // take matrix vector
    for(int j = 0; j < images_person.size() - 30;j++){
      cv::Mat aux = images_person[j]; // For each matrix
      for(int k = j; k < j + 30; (k = k + 3)){ // The next matrix (3 hop)
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
      // Add the new image to the predictions vector
      if (predictions.size() < 10){
        predictions.push_back(this->network->prediction(aux));
      }else{
        break;
      }      
    }

    // The total counter vector has a length of 5, the number of people that the network could recognize 
    std::vector<int> total_counter(5, 0);
    int mayor_porcentaje = 0;
    float valor_prediccion = 0.0;


    // Through the list of predictions, the user who has a higher probability increases the counter of his position
    for (int i = 0; i < predictions.size(); i++){
      mayor_porcentaje = -1;
      valor_prediccion = 0.0;
      for(int j = 0; j < predictions[i].size();j++){
        if(predictions[i][j] > valor_prediccion){
          mayor_porcentaje = j;
          valor_prediccion = predictions[i][j];
        }
      }
      if(mayor_porcentaje != -1){
        total_counter[mayor_porcentaje] = total_counter[mayor_porcentaje] + 1;
      }
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

    this->vectorPeople[i]->authentication_id = "user_" + std::to_string(posicion_persona);

    // Percentage
    float porcentaje = ( total_counter[posicion_persona] * 100 ) /  predictions.size();

    this->vectorPeople[i]->accuracy = porcentaje;
  }


  // Publish the data in terminal and in topic "/brittany"
  for (int i = 0; i < this->vectorPeople.size(); i++){
    std_msgs::String usuario;

    if (this->vectorPeople[i]->accuracy > 70) {
      std::cout << "Person with identifier: "<< this->vectorPeople[i]->id << ". Is the user: " << this->vectorPeople[i]->authentication_id << '\n';
      usuario.data = this->vectorPeople[i]->id + " --> " + this->vectorPeople[i]->authentication_id;
      identificador_pub.publish(usuario);
    }else{
      std::cout << "Person with identifier: "<< this->vectorPeople[i]->id << ". IS NOT IDENTIFIED IN THE SYSTEM" << '\n';
      usuario.data = this->vectorPeople[i]->id + " --> NOT IDENTIFIED IN THE SYSTEM";
      identificador_pub.publish(usuario);
    }

  }
  std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << '\n';
  ros::shutdown();
}
