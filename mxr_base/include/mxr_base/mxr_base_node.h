#ifndef MXR_BASE_NODE_H_INCLUDED
#define MXR_BASE_NODE_H_INCLUDED

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mxr_msgs/msg/ackermann_state.hpp"

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
    
    void callback_parameter();
    void callback_joy(const sensor_msgs::msg::Joy &msg) ;

    void callback_serial ( car::com::Message &header,  car::com::Objects & objects );
    void respond();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_;
    rclcpp::Publisher<mxr_msgs::msg::AckermannState>::SharedPtr publisher_ackermann_state_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    size_t count_;
    size_t count_callback_joy_;


    car::com::pc::SerialInterface serial_arduino;
    car::com::objects::AckermannState ackermann_command;
    car::com::objects::ControlParameter control_parameter;
    car::com::objects::AckermannConfig ackermann_config_;
    JoyControlParameter joy_control_parameter_;
    
};


#endif // MXR_BASE_NODE_H_INCLUDED
