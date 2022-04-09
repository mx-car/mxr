#ifndef MXR_BASE_NODE_H_INCLUDED
#define MXR_BASE_NODE_H_INCLUDED

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mxr_msgs/msg/ackermann_state_stamped.hpp"
#include "mxr_msgs/msg/serial_state.hpp"
#include "mxr_msgs/srv/control_parameter.hpp"

#include <car/com/pc/interface.h>

class MXRBaseNode : public rclcpp::Node
{
    class JoyControlParameter {
    public:
        JoyControlParameter()
            : max_velocity(1.0)
            , max_steering(0.5)
            , axis_velocity(4)
            , axis_steering(3) {}
        double max_velocity;
        double max_steering;
        int axis_velocity;
        int axis_steering;
    };
public:
    MXRBaseNode();

private:
    void init_com() ;
    void init_parameter();
    
    void callback_update_parameter();
    void callback_joy(const sensor_msgs::msg::Joy &msg) ;

    void callback_serial ( car::com::Message &header,  car::com::Objects & objects );
    void respond();

    void callback_srv_control_parameter(const std::shared_ptr<mxr_msgs::srv::ControlParameter::Request> request, std::shared_ptr<mxr_msgs::srv::ControlParameter::Response>  response);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_;
    rclcpp::Publisher<mxr_msgs::msg::AckermannStateStamped>::SharedPtr publisher_ackermann_state_;
    rclcpp::Publisher<mxr_msgs::msg::SerialState>::SharedPtr publisher_serial_state_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Service<mxr_msgs::srv::ControlParameter>::SharedPtr srv_control_param_request_;
    size_t count_;
    size_t count_callback_joy_;


    car::com::pc::SerialInterface serial_arduino;
    car::com::objects::AckermannState ackermann_command;
    car::com::objects::ControlParameter control_parameter_current_;
    car::com::objects::ControlParameter control_parameter_target_;
    car::com::objects::AckermannConfig ackermann_config_;
    JoyControlParameter joy_control_parameter_;
    mxr_msgs::msg::SerialState serial_state_;
    
};


#endif // MXR_BASE_NODE_H_INCLUDED
