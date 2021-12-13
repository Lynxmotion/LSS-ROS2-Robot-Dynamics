//
// Created by guru on 12/30/19.
//
// Default main executor for any node, define NODE_CLASS_TYPE in your compile line
//

#include "rclcpp/rclcpp.hpp"

#include NODE_CLASS_INCLUDE

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    auto node = std::make_shared<NODE_CLASS_TYPE>();

    exe.add_node(node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}
