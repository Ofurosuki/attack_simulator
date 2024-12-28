#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <random>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

class HFRSimulator : public rclcpp::Node {
 public:
  HFRSimulator() : Node("hfr_simulator") {
    // Create a subscription to the input point cloud topic
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sensing/lidar/top/pointcloud_raw_ex_in",
        rclcpp::QoS(10).best_effort(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          this->pointcloud_callback(msg);
        });

    // Create a publisher for the output point cloud topic
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/sensing/lidar/top/pointcloud_raw_ex", rclcpp::QoS(10).best_effort());

    this->declare_parameter("elimination_angle", 20.0);
    azimuth_range = this->get_parameter("elimination_angle").as_double();
    this->declare_parameter("success_rate_chrono", 0.1);
    sccess_rate_chrono = this->get_parameter("success_rate_chrono").as_double();
    this->declare_parameter("csv_path","/home/lab_awsim/Downloads/out2.csv");
    csv_path=this->get_parameter("csv_path").as_string();


    readCSVtoEigen(csv_path, success_rate_matrix);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    for (int i = 0; i < 100; i++) {
      random_numbers.push_back(dis(gen));
    }
    RCLCPP_INFO(this->get_logger(), "random_numbers: %f, %f, %f",
                random_numbers[0], random_numbers[1], random_numbers[2]);
  }

 private:
  std::string csv_path;

  std::vector<float> random_numbers;
  int idx = 0;
  int idx_chrono = 0;
  float sccess_rate_chrono = 0.0;
  float elimination_angle = 0;

  float azimuth_range = 20.0;  // range of array corresponding to, degree
  float alititude_range = 32.0;

  float csv_row = 0;
  float csv_column = 0;

  float azimuth_step = 0;
  float alititude_step = 0;

  Eigen::MatrixXd success_rate_matrix;

  void readCSVtoEigen(const std::string& filePath,
                      Eigen::MatrixXd& outputMatrix) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Error opening file!");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "File opened successfully!");

    std::string line;
    std::vector<std::vector<double>> data;

    // Read each line from the CSV file
    while (std::getline(file, line)) {
      std::stringstream lineStream(line);
      std::string cell;
      std::vector<double> row;

      // Read each cell in the line
      while (std::getline(lineStream, cell, ',')) {
        row.push_back(std::stod(cell));
      }

      data.push_back(row);
    }

    file.close();

    // Determine row and column count
    int rows = data.size();
    int cols = data[0].size();
    // RCLCPP_INFO(this->get_logger(), "Rows: %d, Columns: %d", rows, cols);

    // outputMatrix.resize(rows, cols);
    // for (int i = 0; i < rows; ++i) {
    //   for (int j = 0; j < cols; ++j) {
    //   outputMatrix(i, j) = data[i][j];
    //   }
    // }

    // //outputMatrix.transpose();

    // csv_row = rows;
    // csv_column = cols;
    // azimuth_step = azimuth_range / rows;
    // alititude_step = alititude_range / cols;
    // Return row and column count

    outputMatrix.resize(cols, rows);  // Resize to transposed dimensions
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        outputMatrix(cols - j - 1, rows - i - 1) =
            data[i][j];  // Swap i and j for transposition
      }
    }
    csv_row = cols;     // Now this refers to the transposed rows
    csv_column = rows;  // Now this refers to the transposed columns
    azimuth_step = azimuth_range / csv_row;
    alititude_step = alititude_range / csv_column;
    return;
  }

  inline std::pair<int, int> get_index(const float azimuth,
                                       const float altitude) {
    int i = (azimuth + azimuth_range / 2) / azimuth_step;
    int j = (altitude + alititude_range / 2) / alititude_step;
    if (i >= csv_row) {
      i = csv_row - 1;
    }
    if (j >= csv_column) {
      j = csv_column - 1;
    }
    if (i < 0) {
      i = 0;
    }
    if (j < 0) {
      j = 0;
    }
    return std::make_pair(i, j);
  }

  bool is_attack_successful(const float azimuth, const float altitude,
                            const Eigen::MatrixXd& matrix) {
    if (azimuth > azimuth_range / 2.0 || azimuth < -azimuth_range / 2.0 ||
        altitude > alititude_range / 2.0 || altitude < -alititude_range / 2.0) {
      return 0;
    }
    std::pair<int, int> index = get_index(azimuth, altitude);
    // RCLCPP_INFO(this->get_logger(), "index: %d, %d", index.first,
    // index.second);
    float success_rate = matrix(index.first, index.second);
    // Generate a random number between 0 and 1

    bool success = random_numbers[idx] < success_rate;
    idx++;
    if (idx >= random_numbers.size()) {
      idx = 0;
    }
    return success;
  }

  bool is_attack_successful_chrono() {
    bool success = random_numbers[idx_chrono] < sccess_rate_chrono;
    idx_chrono++;
    if (idx_chrono >= random_numbers.size()) {
      idx_chrono = 0;
    }
    return success;
  }

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // // Make a copy of the message

    // auto output_msg = msg;
    auto output_msg = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);

    size_t point_step = msg->point_step;

    size_t offset = 0;
    float cos_phi = 0;
    // const float attack_angle = 0.0f;
    // RCLCPP_INFO(this->get_logger(), "cos: %f", cos(attack_angle/2*M_PI/180));
    if(is_attack_successful_chrono()){
      
    
    for (size_t i = 0; i < output_msg->width * output_msg->height; ++i) {
      float x = *reinterpret_cast<float*>(&output_msg->data[offset + 0]);
      float y = *reinterpret_cast<float*>(&output_msg->data[offset + 4]);
      float z = *reinterpret_cast<float*>(&output_msg->data[offset + 8]);
      cos_phi = -y / std::sqrt(x * x + y * y);

      if (cos_phi > cos(azimuth_range / 2 * M_PI / 180)) {
        if (is_attack_successful(atan2(x, -y) * 180 / M_PI,
                                 atan2(z, sqrt(x * x + y * y)) * 180 / M_PI,
                                 success_rate_matrix)) {
          
          float value = 0.0f;
          //RCLCPP_INFO(this->get_logger(), "attack success");
          std::memcpy(&output_msg->data[offset + 0], &value, sizeof(value));
          std::memcpy(&output_msg->data[offset + 4], &value, sizeof(value));
          std::memcpy(&output_msg->data[offset + 8], &value, sizeof(value));

          // float intensity = 0.0f;
          // std::memcpy(&output_msg->data[offset + 16], &intensity,
          //             sizeof(intensity));
          // u_int16_t ring = 0;
          // std::memcpy(&output_msg->data[offset + 20], &ring, sizeof(ring));
          // float azimuth = 0.0f;
          // std::memcpy(&output_msg->data[offset + 24], &azimuth,
          //             sizeof(azimuth));
          // float distance = 0.0f;
          // std::memcpy(&output_msg->data[offset + 28], &distance,
          //             sizeof(distance));
        }
      }
      offset += point_step;
    }
    }
    // Publish the updated message
    publisher_->publish(*output_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HFRSimulator>());
  rclcpp::shutdown();
  return 0;
}
