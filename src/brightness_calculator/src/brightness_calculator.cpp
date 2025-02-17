#include "../include/brightness_calculator.hpp"

namespace brightness_calculator {

BrightnessCalculator::BrightnessCalculator(const rclcpp::NodeOptions &options)
    : Node("brightness_calculator", options) {
  parse_parameters();
  initialize();
}

double BrightnessCalculator::calculate_brightness(const cv::Mat &image) {
  // Extract the V (Value) channel from the HSV image
  std::vector<cv::Mat> hsv_channels;
  cv::split(image, hsv_channels);
  cv::Mat value_channel = hsv_channels[2];

  // Calculate the mean brightness
  cv::Scalar mean_brightness = cv::mean(value_channel);
  return mean_brightness[0];
}

void BrightnessCalculator::initialize() {
  is_light_ = BrightnessStatus::UNKNOWN;
  auto qos = rclcpp::QoS(depth_);  // queue depth of=based on depth parameter
  reliability_ == "best_effort" ? qos.best_effort() : qos.reliable();
  durability_ == "transient_local" ? qos.transient_local()
                                   : qos.durability_volatile();
  history_ == "keep_all" ? qos.keep_all() : qos.keep_last(depth_);
  pub_ = this->create_publisher<std_msgs::msg::Bool>("is_light",
                                                     rclcpp::SensorDataQoS());
  auto image_callback = [this](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat hsv_image;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    double brightness = calculate_brightness(cv_ptr->image);
    BrightnessStatus new_is_light = brightness > brightness_threshold_
                                        ? BrightnessStatus::LIGHT
                                        : BrightnessStatus::DARK;

    if (publish_mode_ == PublishMode::CONTINUOUS || new_is_light != is_light_) {
      is_light_ = new_is_light;
      auto msg_out = std_msgs::msg::Bool();
      msg_out.data = is_light_ == BrightnessStatus::LIGHT;
      RCLCPP_INFO(get_logger(), "Publishing brightness status #%d", is_light_);

      pub_->publish(msg_out);
    }
  };

  sub_ = create_subscription<sensor_msgs::msg::Image>("image", qos,
                                                      image_callback);
}

void BrightnessCalculator::parse_parameters() {
  brightness_threshold_ = this->declare_parameter("brightness_threshold", 50);
  reliability_ = this->declare_parameter("reliability", "reliable");
  durability_ = this->declare_parameter("durability", "volatile");
  history_ = this->declare_parameter("history", "keep_last");
  depth_ = this->declare_parameter("depth", 10);
  publish_mode_ =
      (this->declare_parameter("publish_mode", "continuous") == "on_change")
          ? PublishMode::ON_CHANGE
          : PublishMode::CONTINUOUS;
}

}  // namespace brightness_calculator

RCLCPP_COMPONENTS_REGISTER_NODE(brightness_calculator::BrightnessCalculator)