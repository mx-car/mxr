#include <mxr_base/mxr_base_node.h>

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace car::com::objects;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

car::com::objects::Time tprev;

MXRBaseNode:: MXRBaseNode()
    : Node("mxr_base"), count_(0), count_callback_joy_(0)
{
    //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    publisher_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    publisher_ackermann_state_ = this->create_publisher<mxr_msgs::msg::AckermannState>("state", 10);
    
    
    init_parameter();
    init_com();
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&MXRBaseNode::callback_joy, this, _1));
}

void MXRBaseNode::init_parameter() {
    
    this->declare_parameter<double>("joy/max_velocity", joy_control_parameter_.max_velocity);
    this->declare_parameter<double>("joy/max_steering", joy_control_parameter_.max_steering);
    this->declare_parameter<int>("joy/axis_velocity", joy_control_parameter_.axis_velocity);
    this->declare_parameter<int>("joy/axis_steering", joy_control_parameter_.axis_steering);

    this->declare_parameter<double>("ackermann/wheel_diameter", 0.065);
    this->declare_parameter<double>("ackermann/wheel_displacement", 0.153);
    this->declare_parameter<double>("ackermann/wheel_axle_displacement", 0.26);
    
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        rcl_interfaces::msg::IntegerRange range;
        range.set__from_value(0).set__to_value(100).set__step(1);
        descriptor.integer_range= {range};
        this->declare_parameter("test", 1, descriptor);
    }
    

    callback_parameter();
    timer_ = this->create_wall_timer(1000ms, std::bind(&MXRBaseNode::callback_parameter, this));
}

void MXRBaseNode::init_com() {
    ackermann_command.set( 0, 0, 0, ackermann_command.MODE_PWM );


    auto  callback_serial_fnc ( std::bind ( &MXRBaseNode::callback_serial, this, std::placeholders::_1,  std::placeholders::_2 ) );

    car::com::pc::Parameters serial;
    serial.port  = "/dev/ttyACM0";
    serial.baudrate = 115200;

    serial_arduino.init ( serial, callback_serial_fnc );
    std::this_thread::sleep_for(std::chrono::seconds(1));
}
void MXRBaseNode::callback_parameter()
{
    
    this->get_parameter("joy/max_velocity", joy_control_parameter_.max_velocity);
    this->get_parameter("joy/max_steering", joy_control_parameter_.max_steering);
    this->get_parameter("joy/axis_velocity", joy_control_parameter_.axis_velocity);
    this->get_parameter("joy/axis_steering", joy_control_parameter_.axis_steering);

    this->get_parameter("ackermann/wheel_diameter", ackermann_config_.wheel_diameter);
    this->get_parameter("ackermann/wheel_displacement", ackermann_config_.wheel_displacement);
    this->get_parameter("ackermann/wheel_axle_displacement", ackermann_config_.wheel_axle_displacement);
    
    if(count_callback_joy_ == 0) RCLCPP_INFO(this->get_logger(), "no joy message received");
}

void MXRBaseNode::callback_joy(const sensor_msgs::msg::Joy &msg)
{
    count_callback_joy_++;
    ackermann_command.coubled[0] = true;
    ackermann_command.coubled[1] = true;
    ackermann_command.v[0] = msg.axes[joy_control_parameter_.axis_velocity] * joy_control_parameter_.max_velocity;
    ackermann_command.v[1] = -msg.axes[joy_control_parameter_.axis_velocity] * joy_control_parameter_.max_velocity;
    ackermann_command.steering = -msg.axes[joy_control_parameter_.axis_steering] * joy_control_parameter_.max_steering;
    ackermann_command.stamp = Time::now();

    ackermann_command.mode = AckermannState::MODE_PWM;
}

void MXRBaseNode::callback_serial ( car::com::Message &header,  car::com::Objects & objects )
{
    car::com::objects::Time tnow = car::com::objects::Time::now();
    car::com::objects::Duration dt = tnow - tprev;
    tprev = tnow;
    
    mxr_msgs::msg::AckermannState::SharedPtr msg_state;
    //mxr_msgs::msg::AckermannState msg_state;
//     state.coubled[0] = false;
//     msg_state = std::make_shared<mxr_msgs::msg::AckermannState>();
//     msg_state.coubled[0] = state.coubled[0];
        
    for ( car::com::Objects::iterator it=objects.begin(); it!=objects.end(); ++it ) {
        car::com::objects::Object &object = it->second;
        switch ( it->first ) {
        case car::com::objects::TYPE_SYNC_REQUEST: {
            std::cout << "Sync request" << std::endl;
            RCLCPP_INFO(this->get_logger(), "UART: Sync request");
        }
        break;
        case car::com::objects::TYPE_TEXT: {
            car::com::objects::Text text;
            object.get ( text );
            RCLCPP_INFO(this->get_logger(), "UART: %s", text.txt);
            //std::cout << "Text: " << text.txt << std::endl;
        }
        break;
        case car::com::objects::TYPE_STATE_ACKERMANN: {
            car::com::objects::StateAckermann state;
            object.get ( state );
            //std::cout << "StateAckermann  : " << state.getToStringReadable() << std::endl;
        }
        break;
        case car::com::objects::TYPE_COMMAND_ACKERMANN: {
            car::com::objects::CommandAckermann cmd;
            object.get ( cmd );
            //std::cout << "CommandAckermann: " << cmd.getToStringReadable() << std::endl;
        }
        break;
        case car::com::objects::TYPE_ACKERMANN_CONFIG: {
            car::com::objects::AckermannConfig config;
            object.get ( config );
            //std::cout << "AckermannConfig : " << config.getToStringReadable() << std::endl;
        }
        break;
        case car::com::objects::TYPE_ACKERMANN_STATE: {
            car::com::objects::AckermannState state;
            object.get ( state );
            msg_state = std::make_shared<mxr_msgs::msg::AckermannState>();
            msg_state->coubled[mxr_msgs::msg::AckermannState::BACK_LEFT] = state.coubled[0];
            msg_state->coubled[mxr_msgs::msg::AckermannState::BACK_RIGHT] = state.coubled[1];
            msg_state->velocity[mxr_msgs::msg::AckermannState::BACK_LEFT] = state.v[0];
            msg_state->velocity[mxr_msgs::msg::AckermannState::BACK_RIGHT] = state.v[1];
            msg_state->steering[mxr_msgs::msg::AckermannState::FRONT_LEFT] = state.steering;
            msg_state->steering[mxr_msgs::msg::AckermannState::FRONT_RIGHT] = state.steering;
            std::cout << "AckermannState : " << state.getToStringReadable() << std::endl;
        }
        break;
        case car::com::objects::TYPE_ACKERMANN_CMD: {
            car::com::objects::AckermannState cmd;
            object.get ( cmd );
            //std::cout << "AckermannCommand : " << cmd.getToStringReadable() << std::endl;
        }
        break;
        case car::com::objects::TYPE_POSE_STAMPED: {
            car::com::objects::PoseStamped pose;
            object.get ( pose );
            //std::cout << "PoseStamped : " << pose.getToStringReadable() << std::endl;
        }
        break;
        case car::com::objects::TYPE_CONTROL_PARAMETER: {
            car::com::objects::ControlParameter o;
            object.get ( o );
            //std::cout << "ControlParameter : " << o << std::endl;
        }
        break;
        default:
            std::cout << "Type id: " << object.type << ", of size: " << object.size << std::endl;
        }
    }
    serial_arduino.addObject ( car::com::objects::Object( ackermann_command, car::com::objects::TYPE_ACKERMANN_CMD ) );
    {
        if(msg_state)  publisher_ackermann_state_->publish(*msg_state);
    }

}

