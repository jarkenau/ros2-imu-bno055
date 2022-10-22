from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros2_bno055_imu",
            executable="ros2_bno055_imu",
            name="bno55_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    # default for software i2c on raspberry pi
                    "device": "/dev/i2c-1",
                    # default i2c address of bno055 0x28 -> 40
                    "address": 40,
                    "frame_id": "imu"
                }
            ]
        )
    ])