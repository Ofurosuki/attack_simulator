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
    elimination_angle = this->get_parameter("elimination_angle").as_double();
    readCSVtoEigen("/home/awsim/Downloads/success_rate_matrix.csv",
                   success_rate_matrix);
  }

 private:
  float elimination_angle = 0;

  float azimuth_range = 40.0;  // range of array corresponding to, degree
  float alititude_range = 30.0;

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
    RCLCPP_INFO(this->get_logger(), "Rows: %d, Columns: %d", rows, cols);

    outputMatrix.resize(rows, cols);
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        outputMatrix(i, j) = data[i][j];
      }
    }
    csv_row = rows;
    csv_column = cols;
    azimuth_step = azimuth_range / rows;
    alititude_step = alititude_range / cols;
    // Return row and column count
    return;
  }

  std::pair<int, int> get_index(const float azimuth, const float altitude) {
    int i = (azimuth + azimuth_range / 2) / azimuth_step;
    int j = (altitude + alititude_range / 2) / alititude_step;
    return std::make_pair(i, j);
  }
  bool is_attack_successful(const float azimuth, const float altitude,
                            const Eigen::MatrixXd& matrix) {
    if (azimuth > azimuth_range / 2.0 || azimuth < -azimuth_range / 2.0 ||
        altitude > alititude_range / 2.0 || altitude < -alititude_range / 2.0) {
      return 0;
    }
    std::pair<int, int> index = get_index(azimuth, altitude);
    RCLCPP_INFO(this->get_logger(), "index: %d, %d", index.first, index.second);
    float success_rate = matrix(index.first, index.second);
    // Generate a random number between 0 and 1
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    if (dis(gen) < success_rate) {
      return 1;
    } else {
      return 0;
    }
  }

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // // Make a copy of the message
    // auto updated_msg = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);

    // // Update the timestamp
    // //updated_msg->header.stamp = this->now();

    // // Publish the updated message
    // publisher_->publish(*updated_msg);
    // //publisher_->publish(*msg);
    sensor_msgs::msg::PointCloud2 output_msg = *msg;

    // Set up iterators for reading the input data
    sensor_msgs::PointCloud2Iterator<float> iter_x(output_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(output_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(output_msg, "z");

    // Set up iterators for writing the output data
    std::vector<uint8_t> filtered_data;
    size_t point_step = msg->point_step;

    // Define the cropping bounds
    // float min_x = -1.0, max_x = 1.0;
    // float min_y = -1.3, max_y = 1.3;
    // float min_z = -1.0, max_z = 1.0;

    size_t offset = 0;
    float cos_phi = 0;
    // const float attack_angle = 0.0f;
    // RCLCPP_INFO(this->get_logger(), "cos: %f", cos(attack_angle/2*M_PI/180));
    for (size_t i = 0; i < msg->width * msg->height; ++i) {
      float x = *reinterpret_cast<float*>(&msg->data[offset + 0]);
      float y = *reinterpret_cast<float*>(&msg->data[offset + 4]);
      float z = *reinterpret_cast<float*>(&msg->data[offset + 8]);
      // phi is the angle between the pointcloud and the projection of the
      // pointcloud on the y plane

      cos_phi = x / sqrt(x * x + y * y);

      // if (x >= min_x && x <= max_x && y >= min_y && y <= max_y && z >= min_z
      // &&
      //     z <= max_z)
      //   if (x <= min_x || x >= max_x)
      //     if (!(y >= min_y && y <= max_y))
      if (cos_phi <= cos(azimuth_range / 2 * M_PI / 180)) return;
      if (!is_attack_successful(atan2(y, x) * 180 / M_PI,
                                atan2(z, sqrt(x * x + y * y)) * 180 / M_PI,
                                success_rate_matrix)) {
        filtered_data.insert(filtered_data.end(), msg->data.begin() + offset,
                             msg->data.begin() + offset + point_step);
      }

      offset += point_step;
    }

    // Set the filtered data in the output message
    output_msg.data = filtered_data;
    output_msg.width =
        filtered_data.size() /
        point_step;  // Update width to the number of filtered points
    output_msg.row_step = output_msg.width * point_step;

    // Update the timestamp
    // output_msg.header.stamp = this->now();

    // Publish the updated message
    publisher_->publish(output_msg);
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
