#ifndef BNO055_I2C_NODE_H
#define BNO055_I2C_NODE_H

/* std */
#include <chrono>

/* bno055 driver */
#include "bno055_i2c_driver.h"

/* ros */
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class Bno055I2cNode : public rclcpp::Node 
{
public:
    Bno055I2cNode(const std::string& node_name);

    void publish();

private:
    std::unique_ptr<imu_bno055::BNO055I2CDriver> imu_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

};

#endif /* BNO055_I2C_NODE_H */