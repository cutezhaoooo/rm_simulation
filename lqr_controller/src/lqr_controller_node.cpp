#include "lqr_controller/lqr_controller.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LqrController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

