#ifndef BRIGHTNESS_CALCULATOR_HPP
#define BRIGHTNESS_CALCULATOR_HPP

#include "cv_bridge/cv_bridge.hpp"

#include <boost/asio.hpp>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>


namespace brightness_calculator {
class BrightnessCalculator : public rclcpp::Node {
 public:
  explicit BrightnessCalculator(const rclcpp::NodeOptions &options);

 private:
  void parse_parameters();
  void initialize();
  double calculate_brightness(const cv::Mat &image);
  int32_t brightness_threshold_;
  enum BrightnessStatus
  {
      DARK,
      LIGHT,
      UNKNOWN
  };
  BrightnessStatus is_light_;
  size_t width_;
  size_t height_;
  std::string reliability_;
  std::string durability_;
  std::string history_;
  size_t depth_;
  enum PublishMode
  {
      CONTINUOUS,
      ON_CHANGE
  };
  PublishMode publish_mode_;
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
};

}  // namespace brightness_calculator

#endif
