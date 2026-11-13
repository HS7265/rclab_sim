#include "rclab/profile.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace rclab {
namespace {
double finite_number(double x, const char* label, bool positive = false) {
  if (!std::isfinite(x) || (positive && x <= 0))
    throw std::runtime_error(std::string("Invalid ") + label);
  return x;
}
std::filesystem::path expand_path(std::string text) {
  if (text.rfind("~/", 0) == 0) {
    const char* home = std::getenv("HOME");
    if (!home) throw std::runtime_error("HOME is unset");
    text.replace(0, 1, home);
  }
  return text;
}
JointSettings settings(const nlohmann::json& j, JointSettings value) {
  value.kp = finite_number(j.value("kp", value.kp), "kp");
  value.kd = finite_number(j.value("kd", value.kd), "kd");
  value.lower = finite_number(j.value("lower", value.lower), "lower");
  value.upper = finite_number(j.value("upper", value.upper), "upper");
  if (value.kp < 0 || value.kd < 0 || value.lower >= value.upper)
    throw std::runtime_error("Invalid joint settings");
  return value;
}
}  // namespace
RobotProfile RobotProfile::load(const std::filesystem::path& file,
                                const std::string& model_override) {
  std::ifstream input(file);
  if (!input) throw std::runtime_error("Cannot open profile: " + file.string());
  nlohmann::json j;
  input >> j;
  if (j.at("schema_version").get<int>() != 1)
    throw std::runtime_error("Unsupported profile schema_version");
  RobotProfile p;
  p.source_directory = std::filesystem::absolute(file).parent_path();
  p.control_stack = j.value("control_stack", std::string());
  p.command_mode = j.value("command_mode", p.command_mode);
  if (p.command_mode != "velocity" && p.command_mode != "run_stop")
    throw std::runtime_error("command_mode must be velocity or run_stop");
  p.manual_joint_control = j.value("manual_joint_control", true);
  p.extension = j.value("extension", nlohmann::json::object());
  if (!p.extension.is_object()) throw std::runtime_error("extension must be an object");
  p.command = j.value("command", p.command);
  for (double value : p.command) finite_number(value, "command");
  p.name = j.at("name");
  p.base_body = j.at("base_body");
  if (p.name.empty() || p.base_body.empty())
    throw std::runtime_error("name and base_body must not be empty");
  p.model_path =
      expand_path(model_override.empty() ? j.at("model").get<std::string>() : model_override);
  if (p.model_path.is_relative())
    p.model_path = (model_override.empty() ? std::filesystem::absolute(file).parent_path()
                                           : std::filesystem::current_path()) /
                   p.model_path;
  p.model_path = p.model_path.lexically_normal();
  if (!std::filesystem::is_regular_file(p.model_path))
    throw std::runtime_error("Model not found: " + p.model_path.string() + " (use --model PATH)");
  p.controller = j.value("controller", p.controller);
  p.adapter = j.value("adapter", p.adapter);
  p.control_dt = finite_number(j.value("control_dt", p.control_dt), "control_dt", true);
  p.defaults = settings(j.value("joint_defaults", nlohmann::json::object()), p.defaults);
  if (j.contains("suspension")) {
    const auto& s = j.at("suspension");
    p.start_suspended = s.value("enabled", true);
    p.suspend_height = finite_number(s.at("height"), "suspension height");
    p.suspend_quat = s.value("quat", p.suspend_quat);
    double norm = 0;
    for (double q : p.suspend_quat) norm += finite_number(q, "suspension quaternion") * q;
    if (norm < 1e-12) throw std::runtime_error("Zero suspension quaternion");
    for (auto& q : p.suspend_quat) q /= std::sqrt(norm);
  }
  if (j.contains("initial_positions"))
    p.initial_positions = j.at("initial_positions").get<decltype(p.initial_positions)>();
  for (const auto& [name, q] : p.initial_positions) {
    (void)name;
    finite_number(q, "initial position");
  }
  if (j.contains("joints"))
    for (const auto& entry : j.at("joints").items())
      p.joint_settings[entry.key()] = settings(entry.value(), p.defaults);
  if (j.contains("groups"))
    for (const auto& g : j.at("groups"))
      p.groups.push_back({g.at("name"), g.at("joints").get<std::vector<std::string>>()});
  p.contact_bodies = j.value("contact_bodies", p.contact_bodies);
  if (j.contains("recording")) {
    const auto& r = j.at("recording");
    p.record_seconds =
        finite_number(r.value("seconds", p.record_seconds), "recording seconds", true);
    double mib = finite_number(r.value("max_mib", 256.0), "recording max_mib", true);
    if (mib > 4096) throw std::runtime_error("recording max_mib must be <= 4096");
    p.record_bytes = static_cast<size_t>(mib * 1024 * 1024);
  }
  return p;
}
}  // namespace rclab
