#ifndef ISLIGHT_HPP
#define ISLIGHT_HPP

#include "std_msgs/msg/bool.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace brightness_calculator {
    class IsLight : public rclcpp::Node {
    public:
        explicit IsLight(const rclcpp::NodeOptions &options);

    private:
        void initialize();
        void parse_parameters();

        std::string reliability_;
        std::string durability_;
        std::string history_;
        size_t depth_;


        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber;
    };


} // end of namespace

#endif 