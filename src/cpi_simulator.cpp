#include <pcl/common/io.h>

#include "boost/algorithm/string.hpp"
#include "cpi_simulator/custom_point_types.hpp"
#include "pcl/conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"

class PointCloudProcessor : public rclcpp::Node {
 public:
  PointCloudProcessor() : Node("pointcloud_processor") {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sensing/lidar/top/pointcloud_raw_ex_in",
        rclcpp::QoS(10).best_effort(),
        std::bind(&PointCloudProcessor::topic_callback, this,
                  std::placeholders::_1));
    flag_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/injection_flag", 10,
        std::bind(&PointCloudProcessor::flag_callback, this,
                  std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/sensing/lidar/top/pointcloud_raw_ex", rclcpp::QoS(10).best_effort());

    this->declare_parameter("offset_x", 0.1);
    this->declare_parameter("offset_y", 0.1);
    this->declare_parameter("offset_z", 0.1);
    this->declare_parameter("offset_yaw", M_PI / 4);
    this->declare_parameter("csv_path", "/home/awsim/Downloads/human.csv");
    offset_x = this->get_parameter("offset_x").as_double();
    offset_y = this->get_parameter("offset_y").as_double();
    offset_z = this->get_parameter("offset_z").as_double();
    csv_path = this->get_parameter("csv_path").as_string();
    offset_yaw = this->get_parameter("offset_yaw").as_double();

    if (loadPointCloudFromCSV(object_for_injection) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load point cloud from CSV");
      object_for_injection.points.clear();
    }
    for (auto& point : object_for_injection.points) {
      point.x += offset_x;  // Add an offset to x coordinate
      point.y += offset_y;  // Add an offset to y coordinate
      point.z += offset_z;  // Add an offset to z coordinate
    }
  }

  int loadPointCloudFromCSV(pcl::PointCloud<PointXYZIRADT>& cloud) {
    std::ifstream ifs(csv_path, std::ios::in);
    if (!ifs) return 1;  // File open failed

    std::string buf;
    std::getline(
        ifs,
        buf);  // Skip first line
               // ("Points:0","Points:1","Points:2","intensity","laser_id","azimuth","distance_m","timestamp")
    while (ifs && std::getline(ifs, buf)) {
      std::vector<std::string> v;
      boost::algorithm::split(v, buf, boost::is_any_of(","));
      if (v.size() < 4) continue;
      PointXYZIRADT p;
      p.x = std::atof(v[4].c_str());
      p.y = std::atof(v[3].c_str());
      p.z = std::atof(v[5].c_str());
      p.intensity = std::atof(v[10].c_str());
      p.ring = std::atoi(v[2].c_str());
      p.azimuth = std::atof(v[8].c_str());
      p.distance = std::atof(v[9].c_str());
      p.return_type = std::atoi(v[11].c_str());
      p.time_stamp = std::atof(v[12].c_str());
      cloud.push_back(p);
    }
    return 0;
  }

 private:
  float offset_x = 0;
  float offset_y = 0;
  float offset_z = 0;
  std::string csv_path;
  float offset_yaw = 0;

  mutable bool injection_flag = false;
  pcl::PointCloud<PointXYZIRADT> object_for_injection;
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Convert PointCloud2 to PCL PointCloud
    pcl::PointCloud<PointXYZIRADT> target_cloud;
    pcl::fromROSMsg(*msg, target_cloud);

    pcl::PointCloud<PointXYZIRADT> injected_cloud;
    if (!injection_flag) {
      injected_cloud = target_cloud;
    } else {
      injected_cloud = target_cloud + object_for_injection;
    }
    // injected_cloud = target_cloud + object_for_injection;

    // Convert back to PointCloud2
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(injected_cloud, output);
    output.header = msg->header;  // Maintain the original header
    // output.header.stamp = this->now();  // Update timestamp

    // Publish the modified point cloud
    publisher_->publish(output);
  }

  void flag_callback(const std_msgs::msg::Bool::SharedPtr msg) const {
    // RCLCPP_INFO(this->get_logger(), "Received: %s",
    //             msg->data ? "true" : "false");
    if (msg->data) {
      injection_flag = true;
    } else {
      injection_flag = false;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_subscription_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudProcessor>());
  rclcpp::shutdown();
  return 0;
}
