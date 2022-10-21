#include "bno055_i2c_node.h"

Bno055I2cNode::Bno055I2cNode(const std::string& node_name)
    : Node(node_name)
{
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("raw", 10);
    timer_ = this->create_wall_timer(1s, std::bind(&Bno055I2cNode::publish, this));
    // TODO add device file and address via launch file
    imu_ = std::make_unique<imu_bno055::BNO055I2CDriver>("/dev/i2c-3", 40);
    imu_->init();
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

    sensor_msgs::msg::Imu msg_raw;
    // TODO set frame via launch file
    msg_raw.header.stamp = this->get_clock()->now(); 
    msg_raw.header.frame_id = "imu";
    msg_raw.linear_acceleration.x = (double) record.raw_linear_acceleration_x / 100.0;
    msg_raw.linear_acceleration.y = (double) record.raw_linear_acceleration_y / 100.0;
    msg_raw.linear_acceleration.z = (double) record.raw_linear_acceleration_z / 100.0;
    msg_raw.angular_velocity.x = (double) record.raw_angular_velocity_x / 900.0;
    msg_raw.angular_velocity.y = (double) record.raw_angular_velocity_y / 900.0;
    msg_raw.angular_velocity.z = (double) record.raw_angular_velocity_z / 900.0;

    imu_publisher_->publish(std::move(msg_raw));
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<Bno055I2cNode>("bno55_node"));
    rclcpp::shutdown();
    return 0;
}