#include <iostream>
#include "diablo_ctrl.hpp"
#include "diablo_imu.hpp"
#include "rclcpp/rclcpp.hpp"
#include "diablo_battery.hpp"
#include "diablo_legmotors.hpp"
#include "diablo_body_state.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"
#include"motion_msgs/srv/set_mode.h"
#include"motion_msgs/srv/set_mode.hpp"
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

#define CMD_GO_FORWARD                               0x08
#define CMD_GO_LEFT                                  0x04
#define CMD_ROLL_RIGHT                               0x09

#define CMD_HEIGH_MODE                               0x01 //set 0 or 1
#define CMD_BODY_UP                                  0x11

#define CMD_STAND_UP                                 0x02
#define CMD_STAND_DOWN                               0x12

#define CMD_PITCH                                    0x03
#define CMD_PITCH_MODE                               0x13

#define CMD_SPEED_MODE                               0x05
using namespace std;

class StandModeSetter : public rclcpp::Node{
public:
    //创建服务
    StandModeSetter() :Node("stand_mode_service"){
        RCLCPP_INFO(this->get_logger(), "Service Run");
        set_mode_service_ = this->create_service<motion_msgs::srv::SetMode>(
            "set_mode_service",std::bind(&StandModeSetter::handleSetMode,this,std::placeholders::_1,std::placeholders::_2));
}
    std::shared_ptr<std::thread> thread_;
    DIABLO::OSDK::Movement_Ctrl* pMovementCtrl;
    DIABLO::OSDK::Telemetry* pTelemetry;

private:
    //服务回调函数
    void handleSetMode(
        const std::shared_ptr<motion_msgs::srv::SetMode::Request> request,
        std::shared_ptr<motion_msgs::srv::SetMode::Response>response)
    {
        if(request->command == "stand"){

            pMovementCtrl->obtain_control();
            RCLCPP_INFO(this->get_logger(), "Obtain_control Run");
            // ctrl_msg_.mode_mark= true;
            // ctrl_msg_.mode.stand_mode=true;
            pMovementCtrl->SendTransformUpCmd();
            RCLCPP_INFO(this->get_logger(), "SendTrans Run");
            // ctrl_msg_.mode_mark= false;
            ctrl_msg_.value.up=1.0;
            pMovementCtrl->ctrl_data.up = ctrl_msg_.value.up;
            pMovementCtrl->SendMovementCtrlCmd();
            RCLCPP_INFO(this->get_logger(), "Up Run");

            //设置成功响应
            response->success =true;
            response->message = "成功站立";           

        }else{
            response->success =false;
            response->message = "无效命令";      
        }
    }
    private:
        rclcpp::Service<motion_msgs::srv::SetMode>::SharedPtr set_mode_service_;
    private:
        rclcpp::Subscription<motion_msgs::msg::MotionCtrl>::SharedPtr sub_movement_cmd;
        OSDK_Movement_Ctrl_t    cmd_value;
        bool                onSend = true;
        bool        thd_loop_mark_ = true;
        motion_msgs::msg::MotionCtrl          ctrl_msg_;
};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<StandModeSetter>();


    DIABLO::OSDK::HAL_Pi Hal;
    if(Hal.init("/dev/ttyUSB_diablo")) return -1;

    DIABLO::OSDK::Vehicle vehicle(&Hal);                     
    if(vehicle.init()) return -1;

    vehicle.telemetry->activate();

    diablo_imu_publisher imuPublisher(node,&vehicle);
    imuPublisher.imu_pub_init();

    diablo_battery_publisher batteryPublisher(node,&vehicle);
    batteryPublisher.battery_pub_init();

	diablo_motors_publisher motorsPublisher(node,&vehicle);
    motorsPublisher.motors_pub_init();

    diablo_body_state_publisher bodyStatePublisher(node,&vehicle);
    bodyStatePublisher.body_pub_init();

    // vehicle.telemetry->enableLog(DIABLO::OSDK::TOPIC_POWER);
    // vehicle.telemetry->setMaxSpeed(1.0);
    node->pMovementCtrl = vehicle.movement_ctrl;
    node->pTelemetry = vehicle.telemetry;
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

