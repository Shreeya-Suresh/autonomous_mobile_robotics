#ifndef LIDARBOT_SERIAL_BASE__LIDARBOT_SERIAL_HARDWARE_HPP_
#define LIDARBOT_SERIAL_BASE__LIDARBOT_SERIAL_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <termios.h>
#include <fcntl.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace lidarbot_serial_base
{

class LidarbotSerialHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LidarbotSerialHardware)

  LidarbotSerialHardware();
  virtual ~LidarbotSerialHardware();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial communication
  std::string device_name_;
  int baud_rate_;
  int serial_fd_;
  
  // Buffers
  std::string rx_buffer_;
  
  // Helpers
  void open_serial();
  void close_serial();
  void parse_data(const std::string & data);

  // Store the command for the robot
  double left_wheel_cmd_ = 0.0;
  double right_wheel_cmd_ = 0.0;

  // Store the state for the robot
  double left_wheel_pos_ = 0.0;
  double left_wheel_vel_ = 0.0;
  double right_wheel_pos_ = 0.0;
  double right_wheel_vel_ = 0.0;
  
  // IMU state
  double imu_orientation_x_ = 0.0;
  double imu_orientation_y_ = 0.0;
  double imu_orientation_z_ = 0.0;
  double imu_orientation_w_ = 1.0;
  double imu_ang_vel_x_ = 0.0;
  double imu_ang_vel_y_ = 0.0;
  double imu_ang_vel_z_ = 0.0;
  double imu_lin_acc_x_ = 0.0;
  double imu_lin_acc_y_ = 0.0;
  double imu_lin_acc_z_ = 0.0;

  // Parameters
  std::string left_wheel_name_;
  std::string right_wheel_name_;
  std::string imu_name_;
  int enc_ticks_per_rev_;
  double loop_rate_;
  
  // Logger
  rclcpp::Logger logger_;
};

}  // namespace lidarbot_serial_base

#endif  // LIDARBOT_SERIAL_BASE__LIDARBOT_SERIAL_HARDWARE_HPP_
