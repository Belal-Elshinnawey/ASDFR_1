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
  auto brightness_q =
      rclcpp::QoS(depth_);  // queue depth of=based on depth parameter
  reliability_ == "best_effort" ? brightness_q.best_effort()
                                : brightness_q.reliable();
  durability_ == "transient_local" ? brightness_q.transient_local()
                                   : brightness_q.durability_volatile();
  history_ == "keep_all" ? brightness_q.keep_all()
                         : brightness_q.keep_last(depth_);

  auto filter_q =
      rclcpp::QoS(depth_);  // queue depth of=based on depth parameter
  reliability_ == "best_effort" ? filter_q.best_effort() : filter_q.reliable();
  durability_ == "transient_local" ? filter_q.transient_local()
                                   : filter_q.durability_volatile();
  history_ == "keep_all" ? filter_q.keep_all() : filter_q.keep_last(depth_);
  pub_brightness_ = this->create_publisher<std_msgs::msg::Bool>(
      "is_light", rclcpp::SensorDataQoS());
  pub_filter_ =
      this->create_publisher<sensor_msgs::msg::Image>("bw_image", filter_q);
  // ###################################Image Callback############################################
  auto image_callback =
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
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
      // RCLCPP_INFO(get_logger(), "Publishing brightness status #%d",
      // is_light_);
      pub_brightness_->publish(msg_out);
    }
  };
  // ###################################Filter Callback############################################
    auto filter_callback = [this](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
      cv_bridge::CvImagePtr cv_ptr;
      cv::Mat hsv_image, green_mask;
      try {
          cv_ptr = cv_bridge::toCvCopy(msg,
          sensor_msgs::image_encodings::BGR8); cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);
          cv::Scalar lower_green(27, brightness_threshold_, brightness_threshold_);  // Lower bound for green (H, S, V) 
          cv::Scalar upper_green(85, 255, 255);  // Upper bound for green (H, S, V) 
          cv::inRange(hsv_image, lower_green, upper_green, green_mask);
          cv::Mat labels, stats, centroids;
          int num_components = cv::connectedComponentsWithStats(green_mask, labels, stats, centroids);
          int min_area = 500;
          for (int i = 1; i < num_components; ++i) {
              if (stats.at<int>(i, cv::CC_STAT_AREA) < min_area) {
                  green_mask.setTo(0, labels == i);
              }
          }
          cv_bridge::CvImage green_mask_msg;
          green_mask_msg.header = msg->header;
          green_mask_msg.encoding = sensor_msgs::image_encodings::MONO8;
          green_mask_msg.image = green_mask;
          auto msg = green_mask_msg.toImageMsg();
          pub_filter_->publish(*msg);

      } catch (cv_bridge::Exception &e) {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s",
          e.what()); return;
      }
      if (show_camera_) {
        cv::imshow("green mask", green_mask);
        cv::waitKey(1);
      }
  };
  // ###################################End of Callbacks############################################
  sub_brightness_ = create_subscription<sensor_msgs::msg::Image>(
      "image", brightness_q, image_callback);
  sub_filter_ = sub_brightness_ = create_subscription<sensor_msgs::msg::Image>(
      "image", brightness_q, filter_callback);
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
  show_camera_ = this->declare_parameter("show_camera", true);
}

}  // namespace brightness_calculator

RCLCPP_COMPONENTS_REGISTER_NODE(brightness_calculator::BrightnessCalculator)