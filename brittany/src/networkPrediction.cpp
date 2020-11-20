#include <brittany/networkPrediction.h>

using namespace networkprediction;

NetworkPrediction::NetworkPrediction(std::string networkModel){
  // Load graph
  this->status = ReadBinaryProto(Env::Default(), ros::package::getPath("brittany") + "/model/" + networkModel, &graph_def);
  if (!this->status.ok()) {
    std::cout << this->status.ToString() << "\n";
    ros::shutdown();
  }

  // Open Connection
  this->openConnection();
}

NetworkPrediction::~NetworkPrediction(){
  this->closeConnection();
}

void NetworkPrediction::openConnection(){

  this->options = SessionOptions();
  this->options.config.mutable_gpu_options()->set_allow_growth(true);
  this->status = NewSession(this->options, &session);
  if (!this->status.ok()) {
    std::cout << this->status.ToString() << "\n";
    ros::shutdown();
  }

  // Add the graph to the session
  this->status = this->session->Create(this->graph_def);
  if (!this->status.ok()) {
    std::cout << this->status.ToString() << "\n";
    ros::shutdown();
  }
}
void NetworkPrediction::closeConnection(){
  this->session->Close();
  delete this->session;
}

std::vector<float> NetworkPrediction::prediction(cv::Mat image){
  std::vector<float> predictionsVector;

  // Create the Tensor with the input shape
  Tensor image_tensor (tensorflow::DT_FLOAT, tensorflow::TensorShape{1,LENGTH_MATRIX,LENGTH_MATRIX,1});
  // Allocate memory
  tensorflow::StringPiece tmp_data = image_tensor.tensor_data();
  memcpy(const_cast<char*>(tmp_data.data()), (image.data), LENGTH_MATRIX * LENGTH_MATRIX * sizeof(float));

  // Define input and output layers
  std::vector<std::pair<string, tensorflow::Tensor>> inputs = {{"conv2d_1_input", image_tensor}};
  std::vector<Tensor> outputs;
  this->status = this->session->Run(inputs, {"dense_2/Softmax"}, {}, &outputs);

  if (!this->status.ok()) {
    std::cout << "Running model failed: " << this->status;
  }

  auto outputValues = outputs[0].tensor<float, 2>();
  for (int i = 0; i< outputs[0].dim_size(1); i++){
    predictionsVector.push_back(outputValues(0,i));
  }

  return predictionsVector;
}
