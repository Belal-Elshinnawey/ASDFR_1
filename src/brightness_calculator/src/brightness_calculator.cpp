#include "../include/brightness_calculator.hpp"

namespace brightness_calculator {

BrightnessCalculator::BrightnessCalculator(const rclcpp::NodeOptions &options)
    : Node("brightness_calculator", options) {
  parse_parameters();
  initialize();
}

double BrightnessCalculator::calculate_brightness(const cv::Mat &image) {
  std::vector<cv::Mat> hsv_channels;
  cv::split(image, hsv_channels);
  cv::Mat value_channel = hsv_channels[2];

  return cv::mean(value_channel)[0];
}

void BrightnessCalculator::initialize() {
  is_light_ = BrightnessStatus::UNKNOWN;
  
  // Configure QoS
  auto brightness_qos = rclcpp::QoS(depth_);
  (reliability_ == "best_effort") ? brightness_qos.best_effort() : brightness_qos.reliable();
  (durability_ == "transient_local") ? brightness_qos.transient_local() : brightness_qos.durability_volatile();
  (history_ == "keep_all") ? brightness_qos.keep_all() : brightness_qos.keep_last(depth_);

  pub_brightness_ = this->create_publisher<std_msgs::msg::Bool>("is_light", brightness_qos);

  auto image_callback = [this](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    double brightness = calculate_brightness(cv_ptr->image);
    BrightnessStatus new_is_light = (brightness > brightness_threshold_) ? BrightnessStatus::LIGHT : BrightnessStatus::DARK;

    if (publish_mode_ == PublishMode::CONTINUOUS || new_is_light != is_light_) {
      is_light_ = new_is_light;
      auto msg_out = std_msgs::msg::Bool();
      msg_out.data = (is_light_ == BrightnessStatus::LIGHT);
      pub_brightness_->publish(msg_out);
      RCLCPP_INFO(get_logger(), "Is Light? #%s", msg_out.data ? "True" : "False");
    }
  };

  sub_brightness_ = create_subscription<sensor_msgs::msg::Image>("image", brightness_qos, image_callback);
}

void BrightnessCalculator::parse_parameters() {
  brightness_threshold_ = this->declare_parameter("brightness_threshold", 50);
  image_source_topic_ = this->declare_parameter("image_source_topic", "image");
  reliability_ = this->declare_parameter("reliability", "reliable");
  durability_ = this->declare_parameter("durability", "volatile");
  history_ = this->declare_parameter("history", "keep_last");
  depth_ = this->declare_parameter("depth", 10);
  publish_mode_ = (this->declare_parameter("publish_mode", "continuous") == "on_change") ? PublishMode::ON_CHANGE : PublishMode::CONTINUOUS;
}

}  // namespace brightness_calculator

RCLCPP_COMPONENTS_REGISTER_NODE(brightness_calculator::BrightnessCalculator)
