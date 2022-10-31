// Copyright (c) 2022, Julian Arkenau All rights reserved.

// 3-Clause BSD License

// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation and/or
// other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
// ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef IMU_BNO055__BNO055_I2C_DRIVER_H_ 
#define IMU_BNO055__BNO055_I2C_DRIVER_H_

// std
#include <chrono>
#include <string>
#include <memory>

// bno055 driver
#include "bno055_i2c_driver.h"

// ros
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"

class Bno055I2cNode : public rclcpp::Node 
{
public:
    explicit Bno055I2cNode(const std::string& node_name);

    void publish();

private:
    std::unique_ptr<imu_bno055::BNO055I2CDriver> imu_;

    rclcpp::TimerBase::SharedPtr timer_;

    // publisher
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr imu_magnetic_field_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr imu_temperature_publisher_;

    void publish_raw(imu_bno055::IMURecord& record);
    void publish_magnetic_field(imu_bno055::IMURecord& record);
    void publish_temperature(imu_bno055::IMURecord& record);

    std::string frame_id_;
    rclcpp::Time current_time_; 
};

#endif // IMU_BNO055__BNO055_I2C_DRIVER_H_