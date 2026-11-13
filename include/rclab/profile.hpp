#pragma once
#include <array>
#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace rclab {
struct JointSettings {
  double kp = 30, kd = 2;
  double lower = -3.141592653589793, upper = 3.141592653589793;
};
struct JointGroup {
  std::string name;
  std::vector<std::string> joints;
};
struct RobotProfile {
  std::string name, base_body, controller = "pd", adapter = "direct_motor";
  std::string control_stack, command_mode = "velocity";
  bool manual_joint_control = true;
  nlohmann::json extension = nlohmann::json::object();
  std::filesystem::path source_directory;
  std::filesystem::path model_path;
  double control_dt = 0.002, suspend_height = 1, record_seconds = 120;
  size_t record_bytes = 256 * 1024 * 1024;
  bool start_suspended = true;
  std::array<double, 4> suspend_quat{1, 0, 0, 0};
  std::array<double, 3> command{};
  JointSettings defaults;
  std::map<std::string, double> initial_positions;
  std::map<std::string, JointSettings> joint_settings;
  std::vector<JointGroup> groups;
  std::vector<std::string> contact_bodies;
  static RobotProfile load(const std::filesystem::path& file,
                           const std::string& model_override = "");
};
}  // namespace rclab
