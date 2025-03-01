#ifndef COLOUR_SEGMENTER_HPP
#define COLOUR_SEGMENTER_HPP

#include <sstream>
#include <boost/asio.hpp>

#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace brightness_calculator {
class ColourSegmenter : public rclcpp::Node {
 public:
  explicit ColourSegmenter(const rclcpp::NodeOptions &options);

 private:
  void parse_parameters();
  void initialize();

  
  std::string reliability_;
  std::string durability_;
  std::string history_;
  size_t depth_;
  bool show_camera_;

  std::string colour_name_;
  std::string image_source_topic_;
  int32_t brightness_threshold_;
  uint32_t lower_hue_;
  uint32_t upper_hue_;
  uint32_t lower_saturation_;
  uint32_t upper_saturation_;
  uint32_t lower_value_;
  uint32_t upper_value_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_filter_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_filter_;
};

}  // namespace brightness_calculator

#endif