/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 */

#include "src/CYdLidar.h"
#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <signal.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"

#define ROS2Verision "1.0.1"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");

  RCLCPP_INFO(
    node->get_logger(),
    "[YDLIDAR INFO] Current ROS Driver Version: %s",
    ROS2Verision
  );

  CYdLidar laser;

  /* ================= STRING PARAMETERS ================= */

  std::string port = "/dev/ttyUSB0";
  node->declare_parameter<std::string>("port", port);
  node->get_parameter("port", port);
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());

  std::string frame_id = "laser_frame";
  node->declare_parameter<std::string>("frame_id", frame_id);
  node->get_parameter("frame_id", frame_id);

  /* ================= INTEGER PARAMETERS ================= */

  int baudrate = 230400;
  node->declare_parameter<int>("baudrate", baudrate);
  node->get_parameter("baudrate", baudrate);
  laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));

  int lidar_type = TYPE_TRIANGLE;
  node->declare_parameter<int>("lidar_type", lidar_type);
  node->get_parameter("lidar_type", lidar_type);
  laser.setlidaropt(LidarPropLidarType, &lidar_type, sizeof(int));

  int device_type = YDLIDAR_TYPE_SERIAL;
  node->declare_parameter<int>("device_type", device_type);
  node->get_parameter("device_type", device_type);
  laser.setlidaropt(LidarPropDeviceType, &device_type, sizeof(int));

  int sample_rate = 9;
  node->declare_parameter<int>("sample_rate", sample_rate);
  node->get_parameter("sample_rate", sample_rate);
  laser.setlidaropt(LidarPropSampleRate, &sample_rate, sizeof(int));

  int abnormal_check_count = 4;
  node->declare_parameter<int>("abnormal_check_count", abnormal_check_count);
  node->get_parameter("abnormal_check_count", abnormal_check_count);
  laser.setlidaropt(
    LidarPropAbnormalCheckCount,
    &abnormal_check_count,
    sizeof(int)
  );

  /* ================= BOOLEAN PARAMETERS ================= */

  bool fixed_resolution = false;
  node->declare_parameter<bool>("fixed_resolution", fixed_resolution);
  node->get_parameter("fixed_resolution", fixed_resolution);
  laser.setlidaropt(
    LidarPropFixedResolution,
    &fixed_resolution,
    sizeof(bool)
  );

  bool reversion = true;
  node->declare_parameter<bool>("reversion", reversion);
  node->get_parameter("reversion", reversion);
  laser.setlidaropt(LidarPropReversion, &reversion, sizeof(bool));

  bool inverted = true;
  node->declare_parameter<bool>("inverted", inverted);
  node->get_parameter("inverted", inverted);
  laser.setlidaropt(LidarPropInverted, &inverted, sizeof(bool));

  bool auto_reconnect = true;
  node->declare_parameter<bool>("auto_reconnect", auto_reconnect);
  node->get_parameter("auto_reconnect", auto_reconnect);
  laser.setlidaropt(
    LidarPropAutoReconnect,
    &auto_reconnect,
    sizeof(bool)
  );

  bool isSingleChannel = false;
  node->declare_parameter<bool>("isSingleChannel", isSingleChannel);
  node->get_parameter("isSingleChannel", isSingleChannel);
  laser.setlidaropt(
    LidarPropSingleChannel,
    &isSingleChannel,
    sizeof(bool)
  );

  bool intensity = false;
  node->declare_parameter<bool>("intensity", intensity);
  node->get_parameter("intensity", intensity);
  laser.setlidaropt(LidarPropIntenstiy, &intensity, sizeof(bool));

  bool support_motor_dtr = false;
  node->declare_parameter<bool>("support_motor_dtr", support_motor_dtr);
  node->get_parameter("support_motor_dtr", support_motor_dtr);
  laser.setlidaropt(
    LidarPropSupportMotorDtrCtrl,
    &support_motor_dtr,
    sizeof(bool)
  );

  bool invalid_range_is_inf = false;
  node->declare_parameter<bool>(
    "invalid_range_is_inf",
    invalid_range_is_inf
  );
  node->get_parameter("invalid_range_is_inf", invalid_range_is_inf);

  /* ================= FLOAT PARAMETERS ================= */

  float angle_max = 180.0f;
  node->declare_parameter<float>("angle_max", angle_max);
  node->get_parameter("angle_max", angle_max);
  laser.setlidaropt(LidarPropMaxAngle, &angle_max, sizeof(float));

  float angle_min = -180.0f;
  node->declare_parameter<float>("angle_min", angle_min);
  node->get_parameter("angle_min", angle_min);
  laser.setlidaropt(LidarPropMinAngle, &angle_min, sizeof(float));

  float range_max = 8.0f;
  node->declare_parameter<float>("range_max", range_max);
  node->get_parameter("range_max", range_max);
  laser.setlidaropt(LidarPropMaxRange, &range_max, sizeof(float));

  float range_min = 0.1f;
  node->declare_parameter<float>("range_min", range_min);
  node->get_parameter("range_min", range_min);
  laser.setlidaropt(LidarPropMinRange, &range_min, sizeof(float));

  float frequency = 10.0f;
  node->declare_parameter<float>("frequency", frequency);
  node->get_parameter("frequency", frequency);
  laser.setlidaropt(
    LidarPropScanFrequency,
    &frequency,
    sizeof(float)
  );

  /* ================= START LIDAR ================= */

  bool ret = laser.initialize();
  if (ret) {
    ret = laser.turnOn();
  } else {
    RCLCPP_ERROR(node->get_logger(), "%s", laser.DescribeError());
    return 1;
  }

  auto laser_pub =
    node->create_publisher<sensor_msgs::msg::LaserScan>(
      "scan",
      rclcpp::SensorDataQoS()
    );

  rclcpp::WallRate loop_rate(20);

  while (rclcpp::ok()) {
    LaserScan scan;
    if (laser.doProcessSimple(scan)) {
      sensor_msgs::msg::LaserScan msg;
      msg.header.stamp =
        node->get_clock()->now();
      msg.header.frame_id = frame_id;

      msg.angle_min = scan.config.min_angle;
      msg.angle_max = scan.config.max_angle;
      msg.angle_increment = scan.config.angle_increment;
      msg.scan_time = scan.config.scan_time;
      msg.time_increment = scan.config.time_increment;
      msg.range_min = scan.config.min_range;
      msg.range_max = scan.config.max_range;

      int size =
        (scan.config.max_angle - scan.config.min_angle)
        / scan.config.angle_increment + 1;

      msg.ranges.assign(size, 0.0f);
      msg.intensities.assign(size, 0.0f);

      for (const auto &p : scan.points) {
        int idx = std::ceil(
          (p.angle - scan.config.min_angle)
          / scan.config.angle_increment
        );
        if (idx >= 0 && idx < size) {
          msg.ranges[idx] = p.range;
          msg.intensities[idx] = p.intensity;
        }
      }

      laser_pub->publish(msg);
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();
  return 0;
}
