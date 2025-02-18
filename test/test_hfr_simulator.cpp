#include <gtest/gtest.h>
#include <hfr_simulator/hfr_simulator.hpp>

#include <rclcpp/rclcpp.hpp>

TEST(HFRSimulator, HandlesZeroInput) {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("test_node");

    auto my_node = std::make_shared<HFRSimulator>(node);

    EXPECT_TRUE(!(my_node->is_attack_successful(0, 0, Eigen::MatrixXd::Zero(1, 1))));
    rclcpp::shutdown();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
