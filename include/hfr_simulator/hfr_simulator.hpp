// hfr_simulator.hpp
#ifndef HFR_SIMULATOR_HPP
#define HFR_SIMULATOR_HPP


#include <fstream>
#include <iostream>
#include <random>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
#include <string>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

class HFRSimulator : public rclcpp::Node {
 public:
  HFRSimulator();

 private:
  std::string csv_path;
  std::vector<float> random_numbers;
  int idx = 0;
  int idx_chrono = 0;
  float sccess_rate_chrono = 0.0;
  float elimination_angle = 0;
  float azimuth_range = 20.0;
  float alititude_range = 32.0;
  int csv_row;
  int csv_column = 0;
  float azimuth_step = 0;
  float alititude_step = 0;
  Eigen::MatrixXd success_rate_matrix;

  void readCSVtoEigen(const std::string& filePath, Eigen::MatrixXd& outputMatrix);
  std::pair<int, int> get_index(float azimuth, float altitude);
  bool is_attack_successful(float azimuth, float altitude, const Eigen::MatrixXd& matrix);
  bool is_attack_successful_chrono();
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

#endif // HFR_SIMULATOR_HPP