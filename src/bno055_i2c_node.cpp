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

#include "imu_bno055/bno055_i2c_node.h"

Bno055I2cNode::Bno055I2cNode(const std::string& node_name)
    : Node(node_name)
{
    this->declare_parameter<std::string>("device");
    this->declare_parameter<int>("address");
    this->declare_parameter<std::string>("frame_id");

    imu_raw_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/raw", 10);
    imu_magnetic_field_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/magnetic_field", 10);
    imu_temperature_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("imu/temperature", 10);

    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Bno055I2cNode::publish, this));

    frame_id_ = this->get_parameter("frame_id").as_string();
    
    imu_ = std::make_unique<imu_bno055::BNO055I2CDriver>(
        this->get_parameter("device").as_string(),
        this->get_parameter("address").as_int()
    );
    imu_->init();
}


void Bno055I2cNode::publish_raw(imu_bno055::IMURecord& record){
    sensor_msgs::msg::Imu msg_raw;

    msg_raw.header.stamp = current_time_;
    msg_raw.header.frame_id = frame_id_;
    msg_raw.linear_acceleration.x = static_cast<double>(record.raw_linear_acceleration_x / 100.0);
    msg_raw.linear_acceleration.y = static_cast<double>(record.raw_linear_acceleration_y / 100.0);
    msg_raw.linear_acceleration.z = static_cast<double>(record.raw_linear_acceleration_z / 100.0);
    msg_raw.angular_velocity.x = static_cast<double>(record.raw_angular_velocity_x / 900.0);
    msg_raw.angular_velocity.y = static_cast<double>(record.raw_angular_velocity_y / 900.0);
    msg_raw.angular_velocity.z = static_cast<double>(record.raw_angular_velocity_z / 900.0);

    imu_raw_publisher_->publish(std::move(msg_raw));
}


void Bno055I2cNode::publish_magnetic_field(imu_bno055::IMURecord& record){
    sensor_msgs::msg::MagneticField msg_magnetic;

    msg_magnetic.header.stamp = current_time_;
    msg_magnetic.header.frame_id = frame_id_;
    msg_magnetic.magnetic_field.x = static_cast<double>(record.raw_magnetic_field_x / 16.0);
    msg_magnetic.magnetic_field.y = static_cast<double>(record.raw_magnetic_field_y / 16.0);
    msg_magnetic.magnetic_field.z = static_cast<double>(record.raw_magnetic_field_z / 16.0);

    imu_magnetic_field_publisher_->publish(std::move(msg_magnetic));
}


void Bno055I2cNode::publish_temperature(imu_bno055::IMURecord& record){
    sensor_msgs::msg::Temperature msg_temp;

    msg_temp.header.stamp = current_time_;
    msg_temp.header.frame_id = frame_id_;
    msg_temp.temperature = static_cast<double>(record.temperature);

    imu_temperature_publisher_->publish(std::move(msg_temp));
}

void Bno055I2cNode::publish()
{
    imu_bno055::IMURecord record;
    try {
        record = imu_->read();
    } catch(std::runtime_error const& e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), e.what());
        return;
    }

    current_time_ = this -> get_clock()->now();

    this->publish_raw(record);
    this->publish_magnetic_field(record);
    this->publish_temperature(record);

}

int main(int argc, char** argv){
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<Bno055I2cNode>("bno055_node"));
    rclcpp::shutdown();
    return 0;
}