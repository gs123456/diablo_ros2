#include <iostream>
#include "diablo_ctrl.hpp"


int main(int argc, char **argv) {
    // 初始化ROS 2节点
    rclcpp::init(argc, argv);

    // 创建ROS 2节点
    auto node = std::make_shared<rclcpp::Node>("stand_mode_client");

    // 创建服务客户端
    auto client = node->create_client<motion_msgs::srv::SetMode>("set_mode_service");

    // 等待服务可用
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            std::cout << "Interrupted while waiting for the service. Exiting." << std::endl;
            return -1;
        }
        std::cout << "Service not available. Waiting..." << std::endl;
    }

    // 准备服务请求
    auto request = std::make_shared<motion_msgs::srv::SetMode::Request>();
    request->command = "stand";

    // 发送服务请求
    auto future_result = client->async_send_request(request);

    // 等待服务响应
    if (rclcpp::spin_until_future_complete(node, future_result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future_result.get();
        if (response->success) {
            std::cout << "Command 'stand' executed successfully." << std::endl;
        } else {
            std::cout << "Failed to execute command 'stand': " << response->message << std::endl;
        }
    } else {
        std::cout << "Service call failed to execute." << std::endl;
    }

    // 关闭ROS 2节点
    rclcpp::shutdown();
    return 0;
}
