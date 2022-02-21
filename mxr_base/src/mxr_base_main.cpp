#include "rclcpp/rclcpp.hpp"
#include <mxr_base/mxr_base_node.h>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MXRBaseNode>());
    rclcpp::shutdown();
    return 0;
}
