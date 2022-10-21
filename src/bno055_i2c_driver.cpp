/**
Based on the work of Dheera Venkatraman for ROS1
Source: https://github.com/dheera/ros-imu-bno055 

3-Clause BSD License

Copyright 2019 Dheera Venkatraman 
Contact: `echo qurren | sed -e"s/\(.*\)/\1@\1.arg/" | tr a-z n-za-m`

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "bno055_i2c_driver.h"

namespace imu_bno055 {

BNO055I2CDriver::BNO055I2CDriver(std::string device_, int address_) {
    device = device_;
    address = address_;
}

bool BNO055I2CDriver::reset() {
    int i = 0;

    i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    // reset
    i2c_smbus_write_byte_data(file, BNO055_SYS_TRIGGER_ADDR, 0x20);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    // wait for chip to come back online
    while(i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if(i++ > 500) {
            throw std::runtime_error("chip did not come back online within 5 seconds of reset");
            return false;
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // normal power mode
    i2c_smbus_write_byte_data(file, BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    i2c_smbus_write_byte_data(file, BNO055_PAGE_ID_ADDR, 0);
    i2c_smbus_write_byte_data(file, BNO055_SYS_TRIGGER_ADDR, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_NDOF);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    return true;
}

void BNO055I2CDriver::init() {

    file = open(device.c_str(), O_RDWR);

    if(ioctl(file, I2C_SLAVE, address) < 0) {
        throw std::runtime_error("i2c device open failed");
    }

    if(i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
        throw std::runtime_error("incorrect chip ID");
    }

    std::cerr << "rev ids:"
      << " accel:" << i2c_smbus_read_byte_data(file, BNO055_ACCEL_REV_ID_ADDR)
      << " mag:" << i2c_smbus_read_byte_data(file, BNO055_MAG_REV_ID_ADDR)
      << " gyro:" << i2c_smbus_read_byte_data(file, BNO055_GYRO_REV_ID_ADDR)
      << " sw:" << i2c_smbus_read_word_data(file, BNO055_SW_REV_ID_LSB_ADDR)
      << " bl:" << i2c_smbus_read_byte_data(file, BNO055_BL_REV_ID_ADDR) << std::endl;

    if(!reset()) {
	    throw std::runtime_error("chip init failed");
    }
}

IMURecord BNO055I2CDriver::read() {
    IMURecord record;

    // can only read a length of 0x20 at a time, so do it in 2 reads
    // BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR is the start of the data block that aligns with the IMURecord struct
    if(i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_DATA_X_LSB_ADDR, 0x20, (uint8_t*)&record) != 0x20) {
        throw std::runtime_error("read error");
    }
    if(i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_DATA_X_LSB_ADDR + 0x20, 0x13, (uint8_t*)&record + 0x20) != 0x13) {
        throw std::runtime_error("read error");
    }

    return record;
}

}
