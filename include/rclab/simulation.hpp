#pragma once
#include <mujoco/mujoco.h>

#include <functional>
#include <memory>
#include <vector>

#include "rclab/profile.hpp"

namespace rclab {
struct ModelDelete {
  void operator()(mjModel* p) const { mj_deleteModel(p); }
};
struct DataDelete {
  void operator()(mjData* p) const { mj_deleteData(p); }
};
using ModelPtr = std::unique_ptr<mjModel, ModelDelete>;
using DataPtr = std::unique_ptr<mjData, DataDelete>;
struct JointInfo {
  std::string name, group, reason;
  int id = -1, qpos = -1, dof = -1, type = -1, actuator = -1;
  double lower = 0, upper = 0, gear = 0, gain = 0;
  bool scalar = false, controllable = false;
  bool joint_target_available = true;
};
struct ControlState {
  std::vector<double> target, kp, kd;
  std::vector<unsigned char> enabled;
  std::array<double, 3> command{};
};
struct DiagnosticChannel {
  std::string name, unit;
};

// 확장 제어기는 실제 제어 주기에만 호출된다. GUI 렌더링/재생에서는 호출하지 않는다.
class IController {
 public:
  virtual ~IController() = default;
  virtual void initialize_state(const mjModel&, mjData&) {}
  virtual void reset(const mjModel&, const mjData&) {}
  virtual void update(const mjModel&, const mjData&, ControlState&) = 0;
  virtual std::vector<DiagnosticChannel> channels() const { return {}; }
  virtual void diagnostics(const mjModel&, const mjData&, double*) const {}
  virtual nlohmann::json recording_metadata() const { return nlohmann::json::object(); }
};
class IActuationAdapter {
 public:
  virtual ~IActuationAdapter() = default;
  virtual void bind(const mjModel&, std::vector<JointInfo>&) = 0;
  virtual void reset(const mjModel&, const mjData&) {}
  virtual void apply(const mjModel&, mjData&, const std::vector<JointInfo>&,
                     const ControlState&) = 0;
  virtual void after_step(const mjModel&, const mjData&) {}
  virtual std::vector<DiagnosticChannel> channels() const { return {}; }
  virtual void diagnostics(const mjModel&, const mjData&, double*) const {}
};
using ControllerFactory = std::function<std::unique_ptr<IController>()>;
using AdapterFactory = std::function<std::unique_ptr<IActuationAdapter>()>;
void register_controller(const std::string&, ControllerFactory);
void register_adapter(const std::string&, AdapterFactory);
struct ControlStack {
  std::unique_ptr<IController> controller;
  std::unique_ptr<IActuationAdapter> adapter;
};
using ControlStackFactory = std::function<ControlStack(const RobotProfile&)>;
void register_control_stack(const std::string&, ControlStackFactory);
void register_builtin_control_stacks();

class Simulation {
 public:
  explicit Simulation(RobotProfile profile);
  void reset();
  void step(const std::function<void(const Simulation&)>& before_integrate = {});
  void suspend(bool active, double height, const std::array<double, 4>& quat);
  std::vector<DiagnosticChannel> channels() const;
  void diagnostics(double* values) const;
  std::string fingerprint() const;
  nlohmann::json control_metadata() const { return controller_->recording_metadata(); }
  mjModel* model() const { return model_.get(); }
  mjData* data() const { return data_.get(); }
  const RobotProfile& profile() const { return profile_; }
  const std::vector<JointInfo>& joints() const { return joints_; }
  ControlState& controls() { return controls_; }
  const ControlState& controls() const { return controls_; }
  int base_body() const { return base_body_; }
  int hold_equality() const { return hold_eq_; }
  int control_decimation() const { return decimation_; }
  const std::vector<std::string>& warnings() const { return warnings_; }
  bool suspended() const;

 private:
  RobotProfile profile_;
  ModelPtr model_;
  DataPtr data_;
  std::vector<JointInfo> joints_;
  std::vector<int> old_base_constraints_;
  std::vector<int> contact_body_ids_;
  std::vector<std::string> warnings_;
  ControlState controls_;
  std::unique_ptr<IController> controller_;
  std::unique_ptr<IActuationAdapter> adapter_;
  int base_body_ = -1, root_joint_ = -1, hold_eq_ = -1, mocap_id_ = -1, decimation_ = 1;
  size_t steps_ = 0;
  size_t controller_channels_ = 0;
};
}  // namespace rclab
