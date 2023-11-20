#include "iostream"
#include "diablo_ctrl.hpp"

class StandingControlNode : public rclcpp::Node {
public:
    StandingControlNode() : Node("standing_control_node") {
        // 创建订阅器用于接收机器人状态消息
        robot_status_subscriber_ = this->create_subscription<your_robot_msgs::msg::RobotStatus>(
            "robot_status", 10, std::bind(&StandingControlNode::robotStatusCallback, this, std::placeholders::_1));

        // 创建服务，用于判断并触发站立和设置高度
        stand_and_set_height_service_ = this->create_service<your_robot_msgs::srv::SetStandingHeight>(
            "stand_and_set_standing_height", std::bind(&StandingControlNode::standAndSetHeightCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    rclcpp::Subscription<your_robot_msgs::msg::RobotStatus>::SharedPtr robot_status_subscriber_;
    rclcpp::Service<your_robot_msgs::srv::SetStandingHeight>::SharedPtr stand_and_set_height_service_;

    // 机器人状态回调函数
    void robotStatusCallback(const your_robot_msgs::msg::RobotStatus::SharedPtr msg) {
        if (msg->standing) {
            RCLCPP_INFO(this->get_logger(), "Robot is already standing.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Robot is not standing. Performing stand and set height...");
            // 如果机器人不处于站立状态，可以在这里调用函数进行站立并设置高度的操作
            standAndSetHeight(your_default_height);
        }
    }

    // 一键站立并设置高度的服务回调函数
    void standAndSetHeightCallback(
        const std::shared_ptr<your_robot_msgs::srv::SetStandingHeight::Request> request,
        std::shared_ptr<your_robot_msgs::srv::SetStandingHeight::Response> response) {
        // 在这里处理一键站立并设置高度的请求
        standAndSetHeight(request->desired_height);
        response->success = true; // 设置成功标志
    }

    // 模拟一键站立并设置高度的操作
    void standAndSetHeight(double desired_height) {
        //调用机器人控制接口的函数来触发站立并设置高度
        // setStandingAndHeight(desired_height);
        RCLCPP_INFO(this->get_logger(), "Performing stand and set height operation to %f...", desired_height);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StandingControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
