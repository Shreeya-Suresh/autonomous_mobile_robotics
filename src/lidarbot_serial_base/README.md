# Lidarbot Serial Hardware Interface

This package implements a `hardware_interface::SystemInterface` for the Lidarbot using serial communication with an ESP32.

## Firmware
The corresponding ESP32 firmware is located in `src/lidarbot_firmware`.
Flash it to your ESP32 using Arduino IDE or PlatformIO.

## Configuration
To use this hardware interface, update your `ros2_control.xacro` (usually in `lidarbot_description`) with the following:

```xml
<ros2_control name="LidarbotSerial" type="system">
  <hardware>
    <plugin>lidarbot_serial_base/LidarbotSerialHardware</plugin>
    <param name="serial_port">/dev/ttyUSB0</param>
    <param name="baud_rate">115200</param>
    <param name="left_wheel_name">left_wheel_joint</param>
    <param name="right_wheel_name">right_wheel_joint</param>
    <param name="enc_ticks_per_rev">1084</param>
    <param name="loop_rate">30.0</param>
    <param name="imu_name">mpu6050</param>
  </hardware>
  <joint name="left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <sensor name="mpu6050">
    <state_interface name="orientation.x"/>
    <state_interface name="orientation.y"/>
    <state_interface name="orientation.z"/>
    <state_interface name="orientation.w"/>
    <state_interface name="angular_velocity.x"/>
    <state_interface name="angular_velocity.y"/>
    <state_interface name="angular_velocity.z"/>
    <state_interface name="linear_acceleration.x"/>
    <state_interface name="linear_acceleration.y"/>
    <state_interface name="linear_acceleration.z"/>
  </sensor>
</ros2_control>
```

## Build
Run `colcon build --packages-select lidarbot_serial_base` in your workspace root.
