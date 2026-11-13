// Running student v1의 관측·상태 기계·구동기 계약을 이식한 로봇 확장.
// 참조: rok_4_running/training/sim/rok4_mujoco_sim/src/rok4/rok4_core.cpp.
// 모델/정책 파일은 외부에 두며 이 모듈은 mjModel/mjData를 소유하거나 적분하지 않는다.
#include "rclab/rok4_student.hpp"

#include <onnxruntime_cxx_api.h>
#include <openssl/evp.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>

#include "rclab/simulation.hpp"

namespace rclab {
namespace {
using Json = nlohmann::json;
constexpr size_t joints = 21;
constexpr double pi = 3.14159265358979323846;
using Axis = std::array<double, joints>;
Json read_json(const std::filesystem::path& file) {
  std::ifstream in(file);
  if (!in) throw std::runtime_error("Cannot open student metadata: " + file.string());
  Json result;
  in >> result;
  return result;
}
std::string sha256(const std::filesystem::path& file) {
  std::ifstream in(file, std::ios::binary);
  if (!in) throw std::runtime_error("Cannot hash bundle file: " + file.string());
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(EVP_MD_CTX_new(),
                                                                  EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("SHA256 initialization failed");
  std::array<char, 16384> bytes;
  while (in) {
    in.read(bytes.data(), bytes.size());
    if (EVP_DigestUpdate(context.get(), bytes.data(), in.gcount()) != 1)
      throw std::runtime_error("SHA256 update failed");
  }
  if (!in.eof()) throw std::runtime_error("Bundle read failed");
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned length = 0;
  if (EVP_DigestFinal_ex(context.get(), digest, &length) != 1)
    throw std::runtime_error("SHA256 finalization failed");
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (unsigned i = 0; i < length; ++i) out << std::setw(2) << static_cast<unsigned>(digest[i]);
  return out.str();
}
double number(const Json& value, const char* name, bool positive = false) {
  double result = value.get<double>();
  if (!std::isfinite(result) || (positive && result <= 0))
    throw std::runtime_error(std::string("Invalid student ") + name);
  return result;
}
Axis axis(const Json& j, const char* name, bool positive = false) {
  if (!j.is_array() || j.size() != joints)
    throw std::runtime_error(std::string("Expected 21 values: ") + name);
  Axis result{};
  for (size_t i = 0; i < joints; ++i) result[i] = number(j[i], name, positive);
  return result;
}
std::filesystem::path bundle_path(const RobotProfile& p) {
  std::string text = p.extension.at("bundle").get<std::string>();
  if (text.rfind("~/", 0) == 0) {
    const char* home = std::getenv("HOME");
    if (!home) throw std::runtime_error("HOME unset");
    text.replace(0, 1, home);
  }
  std::filesystem::path path = text;
  return (path.is_absolute() ? path : p.source_directory / path).lexically_normal();
}
double yaw(const double* q) {
  return std::atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
}

// 입력/출력 tensor와 이름 포인터를 초기화 때 고정한다. Run은 사전 확보 출력에 쓴다.
class FloatSession {
 public:
  FloatSession(Ort::Env& env, const std::filesystem::path& file)
      : session_(nullptr),
        memory_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    options.SetInterOpNumThreads(1);
    options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ = Ort::Session(env, file.c_str(), options);
    Ort::AllocatorWithDefaultOptions allocator;
    for (size_t i = 0; i < session_.GetInputCount(); ++i) {
      input_names_.emplace_back(session_.GetInputNameAllocated(i, allocator).get());
      allocate(session_.GetInputTypeInfo(i), inputs_, input_values_);
    }
    for (size_t i = 0; i < session_.GetOutputCount(); ++i) {
      output_names_.emplace_back(session_.GetOutputNameAllocated(i, allocator).get());
      allocate(session_.GetOutputTypeInfo(i), outputs_, output_values_);
    }
    for (const auto& name : input_names_) input_pointers_.push_back(name.c_str());
    for (const auto& name : output_names_) output_pointers_.push_back(name.c_str());
  }
  size_t index(std::string_view name) const {
    for (size_t i = 0; i < input_names_.size(); ++i)
      if (name == std::string_view(input_names_[i])) return i;
    throw std::runtime_error("Missing ONNX input: " + std::string(name));
  }
  float* input(std::string_view name) { return inputs_[index(name)].data(); }
  size_t input_size(std::string_view name) const { return inputs_[index(name)].size(); }
  std::vector<float>& input_values(size_t i) { return inputs_.at(i); }
  const std::vector<float>& output(size_t i) const { return outputs_.at(i); }
  const std::vector<std::string>& input_names() const { return input_names_; }
  const std::vector<std::string>& output_names() const { return output_names_; }
  double last_ms() const { return ms_; }
  void run() {
    auto started = std::chrono::steady_clock::now();
    session_.Run(Ort::RunOptions{nullptr}, input_pointers_.data(), input_values_.data(),
                 input_values_.size(), output_pointers_.data(), output_values_.data(),
                 output_values_.size());
    ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
              .count();
    for (const auto& values : outputs_)
      for (float x : values)
        if (!std::isfinite(x)) throw std::runtime_error("Nonfinite ONNX output");
  }

 private:
  void allocate(Ort::TypeInfo type, std::vector<std::vector<float>>& buffers,
                std::vector<Ort::Value>& values) {
    auto tensor = type.GetTensorTypeAndShapeInfo();
    if (tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
      throw std::runtime_error("Student requires float32 tensors");
    auto shape = tensor.GetShape();
    size_t count = 1;
    for (auto d : shape) {
      if (d <= 0 || count > 1000000 / static_cast<size_t>(d))
        throw std::runtime_error("Unsupported ONNX tensor shape");
      count *= d;
    }
    buffers.emplace_back(count, 0.f);
    values.push_back(Ort::Value::CreateTensor<float>(memory_, buffers.back().data(), count,
                                                     shape.data(), shape.size()));
  }
  Ort::Session session_;
  Ort::MemoryInfo memory_;
  std::vector<std::string> input_names_, output_names_;
  std::vector<const char*> input_pointers_, output_pointers_;
  std::vector<std::vector<float>> inputs_, outputs_;
  std::vector<Ort::Value> input_values_, output_values_;
  double ms_ = 0;
};

class StudentRuntime {
 public:
  explicit StudentRuntime(const RobotProfile& profile)
      : profile_(profile),
        bundle_(bundle_path(profile)),
        env_(ORT_LOGGING_LEVEL_WARNING, "rclab_student"),
        meta_(read_json(bundle_ / "rok4_meta.json")),
        act_(read_json(bundle_ / "actuator_meta.json")) {
    const auto exported = read_json(bundle_ / "student_export.json");
    for (auto name : {"policy.onnx", "motor_state.onnx", "actuator.onnx", "actuator.onnx.data",
                      "rok4_meta.json", "actuator_meta.json", "reference_tables.json"}) {
      auto digest = sha256(bundle_ / name);
      if (exported.at("files").at(name) != digest)
        throw std::runtime_error(std::string("Student bundle hash mismatch: ") + name);
      hashes_[name] = digest;
    }
    if (meta_.contains("jump") || !meta_.contains("mimic") || meta_["mimic"].contains("kind") ||
        meta_.contains("action_filter") ||
        meta_.at("student").at("format") != "sprint_student_v1" ||
        meta_["student"]["hidden_shape"] != Json::array({1, 1, 256}) ||
        meta_["obs"]["actor_dim"] != 81 || meta_["obs"]["history_length"] != 1)
      throw std::runtime_error("Expected running student v1 (obs81, GRU256)");
    if (meta_["student"].value("encoder_zero_deployment", 0.) != 0.)
      throw std::runtime_error("Nonzero encoder deployment offset is unsupported");
    names_ = meta_.at("canonical_joint_names").get<std::vector<std::string>>();
    if (names_.size() != joints ||
        std::set<std::string>(names_.begin(), names_.end()).size() != joints)
      throw std::runtime_error("Invalid canonical joint names");
    offset_ = axis(meta_.at("action_offset"), "action_offset");
    scale_ = axis(meta_.at("action_scale"), "action_scale", true);
    stand_ = axis(meta_.at("mimic").at("stand").at("joint_pos"), "stand");
    root_quat_ = meta_["mimic"]["stand"]["root_quat"].get<std::array<double, 4>>();
    double norm = 0;
    for (double q : root_quat_) {
      if (!std::isfinite(q)) throw std::runtime_error("Invalid stand quaternion");
      norm += q * q;
    }
    if (std::abs(norm - 1) > 1e-5) throw std::runtime_error("Stand quaternion must be normalized");
    stand_height_ = number(meta_["mimic"]["stand"]["base_z"], "stand height", true);
    const auto& mm = meta_.at("mimic");
    run_speed_ = number(mm.at("run_speed"), "run speed", true);
    threshold_ = number(mm.at("run_command_threshold"), "threshold", true);
    period_ = number(mm.at("run_period_steps"), "period", true);
    accel_length_ = mm.at("accel_length");
    decel_length_ = mm.at("decel_length");
    accel_end_ = number(mm.at("accel_end_phase"), "accel end");
    decel_entry_ = number(mm.at("decel_entry_phase"), "decel entry");
    if (accel_length_ < 2 || decel_length_ < 2 || accel_end_ < 0 || accel_end_ >= 1 ||
        decel_entry_ <= 0 || decel_entry_ >= 1)
      throw std::runtime_error("Invalid running state machine");
    dt_ = number(meta_["timing"]["physics_dt"], "physics dt", true);
    policy_dt_ = number(meta_["timing"]["policy_dt"], "policy dt", true);
    decimation_ = meta_["timing"]["decimation"];
    if (decimation_ < 1 || std::abs(policy_dt_ - dt_ * decimation_) > 1e-10 ||
        std::abs(profile.control_dt - policy_dt_) > 1e-10)
      throw std::runtime_error("Student profile timing mismatch");
    lag_ = profile.extension.value("lag_substeps", act_.value("delay_lag_substeps_default", 1));
    rnn_period_ = act_.at("rnn_update_period_substeps");
    thermal_init_ = number(act_.at("initial_state_thermal_load"), "thermal initial");
    if (lag_ < 0 || lag_ > 3 || rnn_period_ < 1 ||
        std::abs(number(act_.at("physics_dt"), "actuator dt", true) - dt_) > 1e-10)
      throw std::runtime_error("Invalid actuator timing");
    raw_clip_ = number(meta_.at("action_raw_clip"), "raw clip", true);
    have_bounds_ = meta_.contains("action_clip_bounds");
    if (have_bounds_) {
      const auto& b = meta_.at("action_clip_bounds");
      if (b.size() != joints) throw std::runtime_error("Invalid action bounds");
      for (size_t i = 0; i < joints; ++i) {
        low_[i] = number(b[i].at(0), "action lower");
        high_[i] = number(b[i].at(1), "action upper");
        if (low_[i] >= high_[i]) throw std::runtime_error("Invalid action range");
      }
    }
    reference_blend_ =
        number(meta_.value("action_clip_reference_blend", Json(1.)), "reference blend");
    if (reference_blend_ < 0 || reference_blend_ > 1)
      throw std::runtime_error("Invalid reference blend");
    if (meta_.contains("action_clip_transition")) {
      accel_blend_ = number(meta_["action_clip_transition"]["accel_s"], "accel blend", true);
      decel_blend_ = number(meta_["action_clip_transition"]["decel_s"], "decel blend", true);
    }
    interpolate_ = meta_.value("target_interpolation", false);
    interpolation_ = meta_.value("target_interpolation_mode", std::string("linear"));
    if (interpolation_ != "linear" && interpolation_ != "cubic" && interpolation_ != "smoothstep")
      throw std::runtime_error("Unsupported target interpolation");
    upper_tau_ =
        number(meta_.value("upper_torque_time_constant", Json(0.)), "upper torque response");
    if (upper_tau_ < 0) throw std::runtime_error("Negative torque response");
    full_diagnostics_ = profile.extension.value("full_policy_diagnostics", false);
    policy_ = std::make_unique<FloatSession>(env_, bundle_ / "policy.onnx");
    motor_ = std::make_unique<FloatSession>(env_, bundle_ / "motor_state.onnx");
    actuator_ = std::make_unique<FloatSession>(env_, bundle_ / "actuator.onnx");
    if (policy_->input_names() != std::vector<std::string>{"obs", "hidden"} ||
        policy_->output_names() != std::vector<std::string>{"actions", "hidden_out"} ||
        policy_->input_size("obs") != 81 || policy_->input_size("hidden") != 256 ||
        policy_->output(0).size() != 21 || policy_->output(1).size() != 256)
      throw std::runtime_error("Student ONNX signature mismatch");
    if (motor_->input_names() != std::vector<std::string>{"joint_pos", "joint_vel"} ||
        motor_->input_size("joint_pos") != 21 || motor_->input_size("joint_vel") != 21 ||
        motor_->output(0).size() != 21 || motor_->output(1).size() != 21)
      throw std::runtime_error("Motor ONNX signature mismatch");
    if (actuator_->input_names() != act_.at("inputs").get<std::vector<std::string>>() ||
        actuator_->output_names() != act_.at("outputs").get<std::vector<std::string>>() ||
        actuator_->output(0).size() != 21)
      throw std::runtime_error("Actuator ONNX signature mismatch");
    state_names_ = act_.at("state_names").get<std::vector<std::string>>();
    if (state_names_.empty() || state_names_.back() != "latched")
      throw std::runtime_error("Missing actuator latch state");
    for (size_t i = 0; i < state_names_.size(); ++i) {
      const auto& n = state_names_[i];
      size_t count = 1;
      for (int dim : act_.at("state_shapes").at(n)) {
        if (dim < 1 || count > 1000000 / static_cast<size_t>(dim))
          throw std::runtime_error("Invalid actuator state shape");
        count *= dim;
      }
      size_t index = actuator_->index(n);
      if (actuator_->input_size(n) != count || actuator_->output(i + 1).size() != count ||
          actuator_->output_names()[i + 1] != n + "_out")
        throw std::runtime_error("Actuator recurrent state mismatch");
      state_inputs_.push_back(index);
      states_.emplace_back(count, 0.f);
    }
    auto at = std::find(state_names_.begin(), state_names_.end(), "applied");
    if (at == state_names_.end()) throw std::runtime_error("Missing motor applied state");
    applied_state_ = at - state_names_.begin();
    if (states_[applied_state_].size() != 12)
      throw std::runtime_error("Expected applied state [1,2,6]");
    for (auto name : {"target", "pos", "vel"})
      if (actuator_->input_size(name) != 21)
        throw std::runtime_error("Actuator joint input size mismatch");
    if (actuator_->input_size("contact_force_z") != 2 || actuator_->input_size("update_rnn") != 1 ||
        actuator_->input_size("hold") != 1)
      throw std::runtime_error("Actuator sensor input size mismatch");
  }

  void bind(const mjModel& m, std::vector<JointInfo>& info) {
    if (std::abs(m.opt.timestep - dt_) > 1e-12)
      throw std::runtime_error("Student model timestep mismatch");
    auto find = [&](mjtObj type, const std::string& name) {
      int id = mj_name2id(&m, type, ("robot/" + name).c_str());
      if (id < 0) id = mj_name2id(&m, type, name.c_str());
      if (id < 0) throw std::runtime_error("Student model object missing: " + name);
      return id;
    };
    base_ = mj_name2id(&m, mjOBJ_BODY, profile_.base_body.c_str());
    for (int j = 0; j < m.njnt; ++j)
      if (m.jnt_bodyid[j] == base_ && m.jnt_type[j] == mjJNT_FREE) root_ = j;
    if (base_ < 1 || root_ < 0) throw std::runtime_error("Student base requires a free joint");
    for (size_t i = 0; i < joints; ++i) {
      int j = find(mjOBJ_JOINT, names_[i]), a = find(mjOBJ_ACTUATOR, names_[i]);
      if (m.jnt_type[j] != mjJNT_HINGE || m.actuator_trntype[a] != mjTRN_JOINT ||
          m.actuator_trnid[2 * a] != j || m.actuator_gear[6 * a] != 1 ||
          m.actuator_gaintype[a] != mjGAIN_FIXED || m.actuator_gainprm[mjNGAIN * a] != 1 ||
          m.actuator_biastype[a] != mjBIAS_NONE || m.actuator_dyntype[a] != mjDYN_NONE)
        throw std::runtime_error("Student requires unit torque motors: " + names_[i]);
      q_[i] = m.jnt_qposadr[j];
      v_[i] = m.jnt_dofadr[j];
      ctrl_[i] = a;
      auto row =
          std::find_if(info.begin(), info.end(), [&](const JointInfo& x) { return x.id == j; });
      if (row == info.end()) throw std::runtime_error("Student joint map incomplete");
      row->controllable = true;
      row->joint_target_available = false;
      row->actuator = a;
      row->gear = row->gain = 1;
      row->reason = "Policy controls motor targets; see motor diagnostic channels";
    }
    for (int leg = 0; leg < 2; ++leg) {
      std::string name = std::string("feet_ground_contact_") + "LR"[leg] + "_Foot_Link_force";
      int sensor = mj_name2id(&m, mjOBJ_SENSOR, name.c_str());
      if (sensor < 0 || m.sensor_dim[sensor] != 3)
        throw std::runtime_error("Student foot force sensor missing: " + name);
      force_adr_[leg] = m.sensor_adr[sensor];
    }
  }

  void initialize_state(const mjModel& m, mjData& d) {
    if (m.nkey > 0) mj_resetDataKeyframe(&m, &d, 0);
    for (size_t i = 0; i < joints; ++i) d.qpos[q_[i]] = stand_[i];
    const int q = m.jnt_qposadr[root_];
    d.qpos[q + 2] = stand_height_;
    std::copy(root_quat_.begin(), root_quat_.end(), d.qpos + q + 3);
  }
  void motor_state(const mjData& d) {
    for (size_t i = 0; i < joints; ++i) {
      motor_->input("joint_pos")[i] = static_cast<float>(d.qpos[q_[i]]);
      motor_->input("joint_vel")[i] = static_cast<float>(d.qvel[v_[i]]);
    }
    motor_->run();
    for (size_t i = 0; i < joints; ++i) {
      motor_position_[i] = motor_->output(0)[i];
      motor_velocity_[i] = motor_->output(1)[i];
    }
    motor_time_ = d.time;
  }
  void reset(const mjModel& m, const mjData& d) {
    for (size_t s = 0; s < states_.size(); ++s)
      std::fill(states_[s].begin(), states_[s].end(),
                state_names_[s] == "thermal_load" ? static_cast<float>(thermal_init_) : 0.f);
    substep_ = 0;
    delay_index_ = 0;
    delay_primed_ = false;
    mode_ = 0;
    progress_ = 0;
    run_command_ = pending_command_ = false;
    filtered_.fill(0);
    raw_.fill(0);
    target_.fill(0);
    last_torque_.fill(0);
    motor_torque_.fill(0);
    delayed_.fill(0);
    forces_.fill(0);
    upper_applied_.fill(0);
    prev_slope_.fill(0);
    hidden_.fill(0);
    observation_.fill(0);
    first_ = true;
    upper_primed_ = false;
    latched_ = false;
    heading_ = yaw(d.qpos + m.jnt_qposadr[root_] + 3);
    motor_time_ = -1;
    motor_state(d);
    if (have_bounds_ && reference_blend_ > 0)
      for (size_t i = 0; i < joints; ++i)
        filtered_[i] = raw_[i] = (motor_position_[i] - offset_[i]) / scale_[i];
  }

  void update(const mjModel&, const mjData& d, ControlState& c) {
    if (std::abs(c.command[1]) > 1e-12 || std::abs(c.command[2]) > 1e-12)
      throw std::runtime_error("Running student supports run/stop only, not lateral/yaw commands");
    pending_command_ = c.command[0] >= threshold_;
    motor_state(d);
    float* obs = policy_->input("obs");
    size_t k = 0;
    double rot[9];
    mju_quat2Mat(rot, d.xquat + 4 * base_);
    for (int i = 0; i < 3; ++i) obs[k++] = static_cast<float>(-rot[6 + i]);
    const double* w = d.cvel + 6 * base_;
    for (int i = 0; i < 3; ++i)
      obs[k++] = static_cast<float>((rot[i] * w[0] + rot[3 + i] * w[1] + rot[6 + i] * w[2]) * .25);
    for (size_t i = 0; i < joints; ++i)
      obs[k++] = static_cast<float>((motor_position_[i] - offset_[i]) / scale_[i]);
    for (size_t i = 0; i < joints; ++i)
      obs[k++] = static_cast<float>(motor_velocity_[i] * .1 / scale_[i]);
    for (double a : filtered_) obs[k++] = static_cast<float>(a);
    obs[k++] = static_cast<float>(run_command_ ? run_speed_ : 0.);
    obs[k++] = 0;
    obs[k++] = 0;
    for (int i = 0; i < 4; ++i) obs[k++] = mode_ == i ? 1.f : 0.f;
    obs[k++] = static_cast<float>(mode_ == 2 ? std::sin(progress_ * 2 * pi) : 0.);
    obs[k++] = static_cast<float>(mode_ == 2 ? std::cos(progress_ * 2 * pi) : 0.);
    obs[k++] = static_cast<float>(
        mode_ == 1 || mode_ == 3 ? progress_ / ((mode_ == 1 ? accel_length_ : decel_length_) - 1)
                                 : 0.);
    const double error = heading_ - yaw(d.xquat + 4 * base_);
    obs[k++] = static_cast<float>(std::sin(error));
    obs[k++] = static_cast<float>(std::cos(error));
    if (k != observation_.size()) throw std::runtime_error("Student observation layout mismatch");
    std::copy(obs, obs + 81, observation_.begin());
    std::copy(hidden_.begin(), hidden_.end(), policy_->input("hidden"));
    policy_->run();
    std::copy(policy_->output(1).begin(), policy_->output(1).end(), hidden_.begin());
    double blend = 1.;
    if (have_bounds_ && accel_blend_ > 0) {
      double x = 0;
      if (mode_ == 0)
        x = 1;
      else if (mode_ == 1)
        x = std::clamp(1 - progress_ * policy_dt_ / accel_blend_, 0., 1.);
      else if (mode_ == 3)
        x = std::clamp((progress_ - (decel_length_ - 1 - decel_blend_ / policy_dt_)) * policy_dt_ /
                           decel_blend_,
                       0., 1.);
      blend = x * x * x * (10 - 15 * x + 6 * x * x);
    }
    blend *= reference_blend_;
    for (size_t i = 0; i < joints; ++i) {
      raw_[i] = policy_->output(0)[i];
      filtered_[i] =
          have_bounds_ ? std::clamp(raw_[i], -1 + blend * (low_[i] + 1), 1 + blend * (high_[i] - 1))
                       : std::clamp(std::clamp(raw_[i], -1., 1.), -raw_clip_, raw_clip_);
      target_[i] = offset_[i] + filtered_[i] * scale_[i];
    }
    if (first_) {
      previous_ = target_;
      prev_slope_.fill(0);
    }
  }

  void apply(const mjModel& m, mjData& d) {
    if (motor_time_ != d.time) motor_state(d);
    const int within = substep_ % decimation_;
    Axis subtarget = target_;
    if (interpolate_) {
      double a = static_cast<double>(within + 1) / decimation_, a2 = a * a, a3 = a2 * a;
      for (size_t i = 0; i < joints; ++i) {
        if (interpolation_ == "cubic")
          subtarget[i] = (2 * a3 - 3 * a2 + 1) * previous_[i] + (a3 - 2 * a2 + a) * prev_slope_[i] +
                         (-2 * a3 + 3 * a2) * target_[i] + (a3 - a2) * (target_[i] - previous_[i]);
        else {
          subtarget[i] = previous_[i] + (interpolation_ == "smoothstep" ? a2 * (3 - 2 * a) : a) *
                                            (target_[i] - previous_[i]);
        }
      }
    }
    if (!delay_primed_) {
      for (auto& slot : delay_) slot = subtarget;
      delay_primed_ = true;
      delay_index_ = 0;
    } else
      delay_index_ = (delay_index_ + 1) % delay_.size();
    delay_[delay_index_] = subtarget;
    delayed_ = delay_[(delay_index_ + delay_.size() - lag_) % delay_.size()];
    for (size_t i = 0; i < joints; ++i) {
      actuator_->input("target")[i] = static_cast<float>(delayed_[i]);
      actuator_->input("pos")[i] = static_cast<float>(d.qpos[q_[i]]);
      actuator_->input("vel")[i] = static_cast<float>(d.qvel[v_[i]]);
    }
    for (int leg = 0; leg < 2; ++leg)
      actuator_->input("contact_force_z")[leg] = static_cast<float>(forces_[leg]);
    actuator_->input("update_rnn")[0] = substep_ % rnn_period_ == 0 ? 1.f : 0.f;
    actuator_->input("hold")[0] = 0;
    for (size_t s = 0; s < states_.size(); ++s)
      std::copy(states_[s].begin(), states_[s].end(),
                actuator_->input_values(state_inputs_[s]).begin());
    actuator_->run();
    for (size_t s = 0; s < states_.size(); ++s)
      std::copy(actuator_->output(s + 1).begin(), actuator_->output(s + 1).end(),
                states_[s].begin());
    latched_ = states_.back()[0] > .5f;
    mju_zero(d.ctrl, m.nu);
    for (size_t i = 0; i < joints; ++i) {
      double torque = actuator_->output(0)[i];
      if (i < 9 && upper_tau_ > 0) {
        if (!upper_primed_) upper_applied_[i] = torque;
        upper_applied_[i] += -std::expm1(-dt_ / upper_tau_) * (torque - upper_applied_[i]);
        torque = upper_applied_[i];
      }
      last_torque_[i] = motor_torque_[i] = d.ctrl[ctrl_[i]] = torque;
    }
    for (int leg = 0; leg < 2; ++leg)
      for (int i = 0; i < 4; ++i)
        motor_torque_[11 + 6 * leg + i] = states_[applied_state_][6 * leg + 2 + i];
    upper_primed_ = true;
  }
  void after_step(const mjModel&, const mjData& d) {
    for (int leg = 0; leg < 2; ++leg) forces_[leg] = std::abs(d.sensordata[force_adr_[leg] + 2]);
    ++substep_;
    if (substep_ % decimation_) return;
    if (interpolation_ == "cubic")
      for (size_t i = 0; i < joints; ++i) prev_slope_[i] = target_[i] - previous_[i];
    previous_ = target_;
    first_ = false;
    run_command_ = pending_command_;
    switch (mode_) {
      case 0:
        if (run_command_) {
          mode_ = 1;
          progress_ = 0;
        }
        break;
      case 1:
        progress_ += 1;
        if (progress_ >= accel_length_ - 1) {
          mode_ = 2;
          progress_ = accel_end_;
        }
        break;
      case 2: {
        double before = progress_;
        progress_ += 1. / period_;
        bool wrapped = progress_ >= 1;
        if (wrapped) progress_ -= 1;
        if (!run_command_ && before < decel_entry_ && progress_ >= decel_entry_ && !wrapped) {
          mode_ = 3;
          progress_ = 0;
        }
        break;
      }
      case 3:
        progress_ += 1;
        if (progress_ >= decel_length_ - 1) {
          mode_ = 0;
          progress_ = 0;
        }
        break;
    }
  }

  std::vector<DiagnosticChannel> channels() const {
    std::vector<DiagnosticChannel> out{{"student/mode", "0=STAND 1=ACCEL 2=RUN 3=DECEL"},
                                       {"student/progress", "phase or clip index"},
                                       {"student/run_command", "1"},
                                       {"student/hidden_norm", "1"},
                                       {"student/tracking_latched", "1"},
                                       {"student/policy_ms", "ms"},
                                       {"student/actuator_ms", "ms"}};
    for (size_t i = 0; i < joints; ++i)
      for (auto kind : {"motor_position", "motor_velocity", "motor_target", "delayed_motor_target",
                        "motor_torque", "joint_torque", "raw_action", "filtered_action"}) {
        std::string unit =
            std::string_view(kind) == "motor_velocity" ? "rad/s"
            : std::string_view(kind) == "motor_torque" || std::string_view(kind) == "joint_torque"
                ? "N m"
            : std::string_view(kind) == "raw_action" || std::string_view(kind) == "filtered_action"
                ? "1"
                : "rad";
        out.push_back({"student/" + names_[i] + "/" + kind, unit});
      }
    for (const char* side : {"left", "right"})
      out.push_back({std::string("student/") + side + "_foot_force_z", "N"});
    if (full_diagnostics_) {
      for (size_t i = 0; i < 81; ++i)
        out.push_back({"student/observation/" + std::to_string(i), "1"});
      for (size_t i = 0; i < 256; ++i) out.push_back({"student/hidden/" + std::to_string(i), "1"});
    }
    return out;
  }
  void diagnostics(double* out) const {
    *out++ = mode_;
    *out++ = progress_;
    *out++ = run_command_;
    double norm = 0;
    for (float x : hidden_) norm += x * x;
    *out++ = std::sqrt(norm);
    *out++ = latched_;
    *out++ = policy_->last_ms();
    *out++ = actuator_->last_ms();
    for (size_t i = 0; i < joints; ++i)
      for (double value : {motor_position_[i], motor_velocity_[i], target_[i], delayed_[i],
                           motor_torque_[i], last_torque_[i], raw_[i], filtered_[i]})
        *out++ = value;
    for (double f : forces_) *out++ = f;
    if (full_diagnostics_) {
      for (float x : observation_) *out++ = x;
      for (float x : hidden_) *out++ = x;
    }
  }
  Json metadata() const {
    return {{"stack", "rok4_running_student"},
            {"bundle_sha256", hashes_},
            {"lag_substeps", lag_},
            {"full_policy_diagnostics", full_diagnostics_},
            {"timing", meta_.at("timing")},
            {"student", meta_.at("student")}};
  }

 private:
  RobotProfile profile_;
  std::filesystem::path bundle_;
  Ort::Env env_;
  Json meta_, act_, hashes_;
  std::unique_ptr<FloatSession> policy_, motor_, actuator_;
  std::vector<std::string> names_, state_names_;
  std::vector<size_t> state_inputs_;
  std::vector<std::vector<float>> states_;
  std::array<int, joints> q_{}, v_{}, ctrl_{};
  std::array<int, 2> force_adr_{};
  int base_ = -1, root_ = -1, mode_ = 0, lag_ = 1, rnn_period_ = 2, decimation_ = 5,
      accel_length_ = 0, decel_length_ = 0;
  size_t substep_ = 0, delay_index_ = 0, applied_state_ = 0;
  Axis offset_{}, scale_{}, stand_{}, low_{}, high_{}, raw_{}, filtered_{}, target_{}, previous_{},
      prev_slope_{}, delayed_{}, last_torque_{}, motor_position_{}, motor_velocity_{},
      motor_torque_{};
  std::array<Axis, 4> delay_{};
  std::array<double, 9> upper_applied_{};
  std::array<double, 2> forces_{};
  std::array<double, 4> root_quat_{};
  std::array<float, 81> observation_{};
  std::array<float, 256> hidden_{};
  double dt_ = .002, policy_dt_ = .01, stand_height_ = 0, run_speed_ = 0, threshold_ = .005,
         period_ = 80, accel_end_ = 0, decel_entry_ = 0, raw_clip_ = 1, reference_blend_ = 1,
         accel_blend_ = 0, decel_blend_ = 0, upper_tau_ = 0, thermal_init_ = 0, progress_ = 0,
         heading_ = 0, motor_time_ = -1;
  bool have_bounds_ = false, interpolate_ = false, full_diagnostics_ = false, first_ = true,
       delay_primed_ = false, upper_primed_ = false, latched_ = false, run_command_ = false,
       pending_command_ = false;
  std::string interpolation_;
};

class StudentController : public IController {
 public:
  explicit StudentController(std::shared_ptr<StudentRuntime> runtime)
      : runtime_(std::move(runtime)) {}
  void initialize_state(const mjModel& m, mjData& d) override { runtime_->initialize_state(m, d); }
  void reset(const mjModel& m, const mjData& d) override { runtime_->reset(m, d); }
  void update(const mjModel& m, const mjData& d, ControlState& c) override {
    runtime_->update(m, d, c);
  }
  std::vector<DiagnosticChannel> channels() const override { return runtime_->channels(); }
  void diagnostics(const mjModel&, const mjData&, double* out) const override {
    runtime_->diagnostics(out);
  }
  Json recording_metadata() const override { return runtime_->metadata(); }

 private:
  std::shared_ptr<StudentRuntime> runtime_;
};
class StudentAdapter : public IActuationAdapter {
 public:
  explicit StudentAdapter(std::shared_ptr<StudentRuntime> runtime) : runtime_(std::move(runtime)) {}
  void bind(const mjModel& m, std::vector<JointInfo>& info) override { runtime_->bind(m, info); }
  void apply(const mjModel& m, mjData& d, const std::vector<JointInfo>&,
             const ControlState&) override {
    runtime_->apply(m, d);
  }
  void after_step(const mjModel& m, const mjData& d) override { runtime_->after_step(m, d); }

 private:
  std::shared_ptr<StudentRuntime> runtime_;
};
}  // namespace
void register_rok4_student_stack() {
  register_control_stack("rok4_running_student", [](const RobotProfile& profile) {
    auto runtime = std::make_shared<StudentRuntime>(profile);
    return ControlStack{std::make_unique<StudentController>(runtime),
                        std::make_unique<StudentAdapter>(runtime)};
  });
}
}  // namespace rclab
