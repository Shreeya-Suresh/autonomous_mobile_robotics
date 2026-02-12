#include "lidarbot_serial_base/lidarbot_serial_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <sstream>
#include <iostream>
#include <unistd.h>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace lidarbot_serial_base
{

LidarbotSerialHardware::LidarbotSerialHardware()
  : logger_(rclcpp::get_logger("LidarbotSerialHardware"))
{
}

LidarbotSerialHardware::~LidarbotSerialHardware()
{
  if (serial_fd_ >= 0) {
    close(serial_fd_);
  }
}

hardware_interface::CallbackReturn LidarbotSerialHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Load parameters
  device_name_ = info_.hardware_parameters["serial_port"];
  try {
    baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  } catch (...) {
    baud_rate_ = 115200;
  }
  
  left_wheel_name_ = info_.hardware_parameters["left_wheel_name"];
  right_wheel_name_ = info_.hardware_parameters["right_wheel_name"];
  enc_ticks_per_rev_ = std::stoi(info_.hardware_parameters["enc_ticks_per_rev"]);
  loop_rate_ = std::stod(info_.hardware_parameters["loop_rate"]);
  imu_name_ = info_.hardware_parameters["imu_name"];
  
  if (info_.hardware_parameters.count("pid_p")) {
      // If PID implemented in ROS, we might need these.
      // But we are sending velocity commands to ESP32 (which might run PID or just open loop PWM).
      // Let's assume ESP32 takes raw PWM for now or we send target velocity and ESP32 does PID.
      // The current plan says "Publisher for Twist to Serial", so let's send velocity command.
  }

  // Verify joint count
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        logger_,
        "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        logger_,
        "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        logger_,
        "Joint '%s' has %zu state interface. 2 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION &&
        joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
       RCLCPP_FATAL(
        logger_,
        "Joint '%s' has %s state interface. Expected %s or %s.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR; 
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
LidarbotSerialHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    left_wheel_name_, hardware_interface::HW_IF_POSITION, &left_wheel_pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    left_wheel_name_, hardware_interface::HW_IF_VELOCITY, &left_wheel_vel_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    right_wheel_name_, hardware_interface::HW_IF_POSITION, &right_wheel_pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    right_wheel_name_, hardware_interface::HW_IF_VELOCITY, &right_wheel_vel_));
    
  // IMU
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    imu_name_, "orientation.x", &imu_orientation_x_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    imu_name_, "orientation.y", &imu_orientation_y_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    imu_name_, "orientation.z", &imu_orientation_z_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    imu_name_, "orientation.w", &imu_orientation_w_));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    imu_name_, "angular_velocity.x", &imu_ang_vel_x_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    imu_name_, "angular_velocity.y", &imu_ang_vel_y_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    imu_name_, "angular_velocity.z", &imu_ang_vel_z_));
    
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    imu_name_, "linear_acceleration.x", &imu_lin_acc_x_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    imu_name_, "linear_acceleration.y", &imu_lin_acc_y_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    imu_name_, "linear_acceleration.z", &imu_lin_acc_z_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
LidarbotSerialHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    left_wheel_name_, hardware_interface::HW_IF_VELOCITY, &left_wheel_cmd_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    right_wheel_name_, hardware_interface::HW_IF_VELOCITY, &right_wheel_cmd_));

  return command_interfaces;
}

hardware_interface::CallbackReturn LidarbotSerialHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Configuring... Opening serial port: %s", device_name_.c_str());
  
  serial_fd_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_fd_ < 0) {
      RCLCPP_ERROR(logger_, "Unable to open serial port.");
      return hardware_interface::CallbackReturn::ERROR;
  }
  
  struct termios options;
  tcgetattr(serial_fd_, &options);
  
  switch(baud_rate_) {
      case 9600: cfsetispeed(&options, B9600); cfsetospeed(&options, B9600); break;
      case 57600: cfsetispeed(&options, B57600); cfsetospeed(&options, B57600); break;
      case 115200: cfsetispeed(&options, B115200); cfsetospeed(&options, B115200); break;
      default: cfsetispeed(&options, B115200); cfsetospeed(&options, B115200); break;
  }
  
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  
  // Raw input
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;

  tcsetattr(serial_fd_, TCSANOW, &options);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LidarbotSerialHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LidarbotSerialHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type LidarbotSerialHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (serial_fd_ < 0) return hardware_interface::return_type::ERROR;

  char buf[256];
  int n = ::read(serial_fd_, buf, sizeof(buf) - 1);
  if (n > 0) {
      buf[n] = 0;
      rx_buffer_ += buf;
      
      size_t pos;
      while ((pos = rx_buffer_.find('\n')) != std::string::npos) {
          std::string line = rx_buffer_.substr(0, pos);
          rx_buffer_.erase(0, pos + 1);
          parse_data(line);
      }
  }

  return hardware_interface::return_type::OK;
}

void LidarbotSerialHardware::parse_data(const std::string & data) {
    if (data.rfind("D,", 0) != 0) return; // Must start with D,

    std::stringstream ss(data.substr(2)); // Skip D,
    std::string segment;
    std::vector<std::string> seglist;
    
    while(std::getline(ss, segment, ',')) {
       seglist.push_back(segment);
    }
    
    // Firmware sends: D,left_enc,right_enc,yaw
    if (seglist.size() < 3) return; 

    try {
        long enc_l = std::stol(seglist[0]);
        long enc_r = std::stol(seglist[1]);
        
        // Convert to position (rad)
        double pos_l = (enc_l / (double)enc_ticks_per_rev_) * 2 * M_PI;
        double pos_r = (enc_r / (double)enc_ticks_per_rev_) * 2 * M_PI;
        
        left_wheel_vel_ = (pos_l - left_wheel_pos_) * loop_rate_;
        right_wheel_vel_ = (pos_r - right_wheel_pos_) * loop_rate_;
        
        left_wheel_pos_ = pos_l;
        right_wheel_pos_ = pos_r;

        // IMU Parsing
        // Format: yaw (degrees) only
        
        double deg2rad = M_PI / 180.0;
        
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = std::stod(seglist[2]) * deg2rad;

        // Firmware does not send rates/accel anymore, so zero them out
        imu_ang_vel_x_ = 0.0;
        imu_ang_vel_y_ = 0.0;
        imu_ang_vel_z_ = 0.0;

        imu_lin_acc_x_ = 0.0;
        imu_lin_acc_y_ = 0.0;
        imu_lin_acc_z_ = 0.0;
        
        // Convert Euler to Quaternion
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);

        imu_orientation_w_ = cr * cp * cy + sr * sp * sy;
        imu_orientation_x_ = sr * cp * cy - cr * sp * sy;
        imu_orientation_y_ = cr * sp * cy + sr * cp * sy;
        imu_orientation_z_ = cr * cp * sy - sr * sp * cy;

    } catch (...) {
        // Ignore parsing errors
    }
}

hardware_interface::return_type LidarbotSerialHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (serial_fd_ < 0) return hardware_interface::return_type::ERROR;
    
  // Convert command (rad/s) to PWM.
  // Assumption: Max 10 rad/s maps to 255 PWM.
  // We add detailed logging to debug why motors aren't moving.
  
  double rad_s_to_pwm = 255.0 / 10.0; // Scale factor
  
  int pwm_l = (int)(left_wheel_cmd_ * rad_s_to_pwm);
  int pwm_r = (int)(right_wheel_cmd_ * rad_s_to_pwm);
  
  // Deadband/Min PWM check (Optional: motors might need min ~40 to move)
  // if (std::abs(pwm_l) < 40 && std::abs(pwm_l) > 0) pwm_l = (pwm_l > 0) ? 40 : -40;
  // if (std::abs(pwm_r) < 40 && std::abs(pwm_r) > 0) pwm_r = (pwm_r > 0) ? 40 : -40;

  // Clamp
  pwm_l = std::max(-255, std::min(255, pwm_l));
  pwm_r = std::max(-255, std::min(255, pwm_r));
  
  // Format command
  std::stringstream ss;
  ss << "V," << pwm_l << "," << pwm_r << "\n";
  std::string cmd = ss.str();
  
  // Send
  int bytes_written = ::write(serial_fd_, cmd.c_str(), cmd.length());

  // Log every 1 second
  if ((long)period.nanoseconds() % 100 == 0) { // Simple hack or use throttle
      RCLCPP_INFO_THROTTLE(logger_, *rclcpp::get_clock_t(), 1000, 
          "Write: Cmd(%.2f, %.2f) -> PWM(%d, %d) -> Serial(%s)", 
          left_wheel_cmd_, right_wheel_cmd_, pwm_l, pwm_r, cmd.substr(0, cmd.length()-1).c_str());
  }

  if (bytes_written < 0) {
      RCLCPP_ERROR_THROTTLE(logger_, *rclcpp::get_clock_t(), 1000, "Failed to write to serial port!");
      return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace lidarbot_serial_base

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  lidarbot_serial_base::LidarbotSerialHardware,
  hardware_interface::SystemInterface)
