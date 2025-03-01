#include "../include/colour_segmenter.hpp"

namespace brightness_calculator {

ColourSegmenter::ColourSegmenter(const rclcpp::NodeOptions &options)
    : Node("colour_segmenter", options) {
  parse_parameters();
  initialize();
}

void ColourSegmenter::initialize() {
  // Configure QoS
  auto filter_qos = rclcpp::QoS(depth_);
  (reliability_ == "best_effort") ? filter_qos.best_effort() : filter_qos.reliable();
  (durability_ == "transient_local") ? filter_qos.transient_local() : filter_qos.durability_volatile();
  (history_ == "keep_all") ? filter_qos.keep_all() : filter_qos.keep_last(depth_);

  pub_filter_ = this->create_publisher<sensor_msgs::msg::Image>("bw_image", filter_qos);

  auto filter_callback = [this](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat hsv_image, color_mask;
    
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

      cv::Scalar lower_bound(lower_hue_, lower_saturation_, lower_value_);
      cv::Scalar upper_bound(upper_hue_, upper_saturation_, upper_value_);
      cv::inRange(hsv_image, lower_bound, upper_bound, color_mask);

      cv_bridge::CvImage color_mask_msg;
      color_mask_msg.header = msg->header;
      color_mask_msg.encoding = sensor_msgs::image_encodings::MONO8;
      color_mask_msg.image = color_mask;
      pub_filter_->publish(*color_mask_msg.toImageMsg());
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    if (show_camera_) {
      cv::imshow(colour_name_ + " Mask", color_mask);
      cv::waitKey(1);
    }
  };

  sub_filter_ = create_subscription<sensor_msgs::msg::Image>(image_source_topic_, filter_qos, filter_callback);
}

void ColourSegmenter::parse_parameters() {
  colour_name_ = this->declare_parameter("colour_name", "Green");
  brightness_threshold_ = this->declare_parameter("brightness_threshold", 50);
  image_source_topic_ = this->declare_parameter("image_source_topic", "image");
  lower_hue_ = this->declare_parameter("lower_hue", 27);
  upper_hue_ = this->declare_parameter("upper_hue", 85);
  lower_saturation_ = this->declare_parameter("lower_saturation", brightness_threshold_);
  upper_saturation_ = this->declare_parameter("upper_saturation", 255);
  lower_value_ = this->declare_parameter("lower_value", brightness_threshold_);
  upper_value_ = this->declare_parameter("upper_value", 255);
  reliability_ = this->declare_parameter("reliability", "reliable");
  durability_ = this->declare_parameter("durability", "volatile");
  history_ = this->declare_parameter("history", "keep_last");
  depth_ = this->declare_parameter("depth", 10);
  show_camera_ = this->declare_parameter("show_camera", true);
}

}  // namespace brightness_calculator

RCLCPP_COMPONENTS_REGISTER_NODE(brightness_calculator::ColourSegmenter)