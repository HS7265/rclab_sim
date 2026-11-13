#include "rclab/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace rclab {
namespace {
class PDController : public IController {
 public:
  void update(const mjModel&, const mjData&, ControlState&) override {}
};
class PassiveController : public IController {
 public:
  void update(const mjModel&, const mjData&, ControlState& c) override {
    std::fill(c.enabled.begin(), c.enabled.end(), 0);
  }
};
class DirectMotor : public IActuationAdapter {
 public:
  void bind(const mjModel& m, std::vector<JointInfo>& joints) override {
    for (auto& j : joints) {
      if (!j.scalar) {
        j.reason = "ball/free: inspect only";
        continue;
      }
      int count = 0;
      for (int a = 0; a < m.nu; ++a)
        if (m.actuator_trntype[a] == mjTRN_JOINT && m.actuator_trnid[2 * a] == j.id) {
          j.actuator = a;
          ++count;
        }
      if (count != 1) {
        j.reason = count ? "multiple actuators: custom adapter required" : "passive joint";
        continue;
      }
      int a = j.actuator;
      j.gear = m.actuator_gear[6 * a];
      j.gain = m.actuator_gainprm[mjNGAIN * a];
      bool scalar_gear = true;
      for (int k = 1; k < 6; ++k) scalar_gear &= m.actuator_gear[6 * a + k] == 0;
      j.controllable = scalar_gear && std::abs(j.gear * j.gain) > 1e-12 &&
                       m.actuator_gaintype[a] == mjGAIN_FIXED &&
                       m.actuator_biastype[a] == mjBIAS_NONE && m.actuator_dyntype[a] == mjDYN_NONE;
      if (!j.controllable) j.reason = "non-direct actuator: custom adapter required";
    }
  }
  void apply(const mjModel& m, mjData& d, const std::vector<JointInfo>& joints,
             const ControlState& c) override {
    mju_zero(d.ctrl, m.nu);
    for (size_t i = 0; i < joints.size(); ++i) {
      const auto& j = joints[i];
      if (!j.controllable || !c.enabled[i]) continue;
      const double tau = c.kp[i] * (std::clamp(c.target[i], j.lower, j.upper) - d.qpos[j.qpos]) -
                         c.kd[i] * d.qvel[j.dof];
      double u = tau / (j.gear * j.gain);
      if (m.actuator_forcelimited[j.actuator]) {
        double lo = m.actuator_forcerange[2 * j.actuator] / j.gain,
               hi = m.actuator_forcerange[2 * j.actuator + 1] / j.gain;
        u = std::clamp(u, std::min(lo, hi), std::max(lo, hi));
      }
      if (m.actuator_ctrllimited[j.actuator])
        u = std::clamp(u, m.actuator_ctrlrange[2 * j.actuator],
                       m.actuator_ctrlrange[2 * j.actuator + 1]);
      d.ctrl[j.actuator] = u;
    }
  }
};
std::map<std::string, ControllerFactory>& controllers() {
  static std::map<std::string, ControllerFactory> map{
      {"pd", [] { return std::make_unique<PDController>(); }},
      {"passive", [] { return std::make_unique<PassiveController>(); }}};
  return map;
}
std::map<std::string, AdapterFactory>& adapters() {
  static std::map<std::string, AdapterFactory> map{
      {"direct_motor", [] { return std::make_unique<DirectMotor>(); }}};
  return map;
}
std::map<std::string, ControlStackFactory>& control_stacks() {
  static std::map<std::string, ControlStackFactory> stacks;
  return stacks;
}
std::string object_name(const mjModel* m, mjtObj type, int id) {
  const char* p = mj_id2name(m, type, id);
  return p ? p : "unnamed_" + std::to_string(id);
}
}  // namespace
void register_controller(const std::string& n, ControllerFactory f) {
  if (!controllers().emplace(n, std::move(f)).second)
    throw std::runtime_error("Controller already registered: " + n);
}
void register_adapter(const std::string& n, AdapterFactory f) {
  if (!adapters().emplace(n, std::move(f)).second)
    throw std::runtime_error("Adapter already registered: " + n);
}
void register_control_stack(const std::string& n, ControlStackFactory f) {
  if (!control_stacks().emplace(n, std::move(f)).second)
    throw std::runtime_error("Control stack already registered: " + n);
}

Simulation::Simulation(RobotProfile p) : profile_(std::move(p)) {
  register_builtin_control_stacks();
  if (mj_version() != mjVERSION_HEADER || mj_version() < 3011000)
    throw std::runtime_error("MuJoCo headers/library mismatch; require >= 3.11.0");
  char error[4096]{};
  std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
      mj_parseXML(profile_.model_path.c_str(), nullptr, error, sizeof(error)), mj_deleteSpec);
  if (!spec) throw std::runtime_error(error);
  auto* base = mjs_findBody(spec.get(), profile_.base_body.c_str());
  if (!base) throw std::runtime_error("base_body not found: " + profile_.base_body);
  if (mjs_findBody(spec.get(), "rclab_hold_anchor"))
    throw std::runtime_error("Reserved body name: rclab_hold_anchor");
  auto* anchor = mjs_addBody(mjs_findBody(spec.get(), "world"), nullptr);
  mjs_setName(anchor->element, "rclab_hold_anchor");
  anchor->mocap = true;
  anchor->pos[2] = profile_.suspend_height;
  std::copy(profile_.suspend_quat.begin(), profile_.suspend_quat.end(), anchor->quat);
  auto* eq = mjs_addEquality(spec.get(), nullptr);
  mjs_setName(eq->element, "rclab_base_hold");
  eq->type = mjEQ_WELD;
  eq->objtype = mjOBJ_BODY;
  eq->active = false;
  mjs_setString(eq->name1, profile_.base_body.c_str());
  mjs_setString(eq->name2, "rclab_hold_anchor");
  std::fill(eq->data, eq->data + mjNEQDATA, 0);
  eq->data[6] = 1;
  eq->data[10] = 1;
  eq->solref[0] = 0.01;
  eq->solref[1] = 1;
  model_.reset(mj_compile(spec.get(), nullptr));
  if (!model_) throw std::runtime_error(mjs_getError(spec.get()));
  auto* m = model();
  base_body_ = mj_name2id(m, mjOBJ_BODY, profile_.base_body.c_str());
  hold_eq_ = mj_name2id(m, mjOBJ_EQUALITY, "rclab_base_hold");
  mocap_id_ = m->body_mocapid[mj_name2id(m, mjOBJ_BODY, "rclab_hold_anchor")];
  for (int j = 0; j < m->njnt; ++j)
    if (m->jnt_bodyid[j] == base_body_ && m->jnt_type[j] == mjJNT_FREE) root_joint_ = j;
  for (int e = 0; e < m->neq; ++e) {
    if (e == hold_eq_ || (m->eq_type[e] != mjEQ_WELD && m->eq_type[e] != mjEQ_CONNECT)) continue;
    const int a = m->eq_obj1id[e], b = m->eq_obj2id[e];
    const int ba = m->eq_objtype[e] == mjOBJ_SITE ? m->site_bodyid[a] : a;
    const int bb = m->eq_objtype[e] == mjOBJ_SITE ? m->site_bodyid[b] : b;
    if ((ba == base_body_ && bb == 0) || (bb == base_body_ && ba == 0))
      old_base_constraints_.push_back(e);
  }
  double ratio = profile_.control_dt / m->opt.timestep;
  if (!std::isfinite(ratio) || ratio < 1 || ratio > 100000 ||
      std::abs(ratio - std::round(ratio)) > 1e-7)
    throw std::runtime_error("control_dt must be an integer multiple of model timestep");
  decimation_ = static_cast<int>(std::round(ratio));
  std::map<std::string, std::string> groups;
  for (const auto& group : profile_.groups)
    for (const auto& n : group.joints) {
      if (mj_name2id(m, mjOBJ_JOINT, n.c_str()) < 0)
        throw std::runtime_error("Group joint not found: " + n);
      if (!groups.emplace(n, group.name).second)
        throw std::runtime_error("Joint belongs to multiple groups: " + n);
    }
  for (const auto& [n, value] : profile_.initial_positions) {
    int id = mj_name2id(m, mjOBJ_JOINT, n.c_str());
    if (id < 0) throw std::runtime_error("Initial joint not found: " + n);
    if (m->jnt_type[id] != mjJNT_HINGE && m->jnt_type[id] != mjJNT_SLIDE)
      throw std::runtime_error("Initial positions require scalar joints: " + n);
    if (m->jnt_limited[id] && (value < m->jnt_range[2 * id] || value > m->jnt_range[2 * id + 1]))
      throw std::runtime_error("Initial position outside joint range: " + n);
  }
  for (const auto& [n, setting] : profile_.joint_settings) {
    (void)setting;
    if (mj_name2id(m, mjOBJ_JOINT, n.c_str()) < 0)
      throw std::runtime_error("Settings joint not found: " + n);
  }
  for (const auto& n : profile_.contact_bodies) {
    int id = mj_name2id(m, mjOBJ_BODY, n.c_str());
    if (id <= 0) throw std::runtime_error("Contact body not found / world is invalid: " + n);
    contact_body_ids_.push_back(id);
  }
  for (int id = 0; id < m->njnt; ++id) {
    JointInfo j;
    j.id = id;
    j.name = object_name(m, mjOBJ_JOINT, id);
    j.type = m->jnt_type[id];
    j.qpos = m->jnt_qposadr[id];
    j.dof = m->jnt_dofadr[id];
    j.scalar = j.type == mjJNT_HINGE || j.type == mjJNT_SLIDE;
    auto setting = profile_.joint_settings.count(j.name) ? profile_.joint_settings.at(j.name)
                                                         : profile_.defaults;
    j.lower = m->jnt_limited[id] ? m->jnt_range[2 * id] : setting.lower;
    j.upper = m->jnt_limited[id] ? m->jnt_range[2 * id + 1] : setting.upper;
    j.group = groups.count(j.name) ? groups.at(j.name) : "Other / passive";
    joints_.push_back(j);
    controls_.target.push_back(0);
    controls_.kp.push_back(setting.kp);
    controls_.kd.push_back(setting.kd);
    controls_.enabled.push_back(0);
  }
  if (!profile_.control_stack.empty()) {
    if (!control_stacks().count(profile_.control_stack))
      throw std::runtime_error("Unknown control_stack: " + profile_.control_stack +
                               " (check optional build features)");
    auto stack = control_stacks().at(profile_.control_stack)(profile_);
    controller_ = std::move(stack.controller);
    adapter_ = std::move(stack.adapter);
    if (!controller_ || !adapter_)
      throw std::runtime_error("Control stack returned null component");
  } else {
    if (!controllers().count(profile_.controller))
      throw std::runtime_error("Unknown controller: " + profile_.controller);
    if (!adapters().count(profile_.adapter))
      throw std::runtime_error("Unknown adapter: " + profile_.adapter);
    controller_ = controllers().at(profile_.controller)();
    adapter_ = adapters().at(profile_.adapter)();
  }
  controller_channels_ = controller_->channels().size();
  adapter_->bind(*m, joints_);
  for (const auto& j : joints_)
    if (j.scalar && !j.controllable) warnings_.push_back(j.name + ": " + j.reason);
  if (root_joint_ < 0) warnings_.push_back("base_body has no free joint: suspension unavailable");
  data_.reset(mj_makeData(m));
  if (!data_) throw std::runtime_error("Cannot allocate mjData");
  reset();
}
void Simulation::reset() {
  mj_resetData(model(), data());
  steps_ = 0;
  controller_->initialize_state(*model(), *data());
  for (int id : old_base_constraints_) data()->eq_active[id] = 0;
  for (const auto& [n, q] : profile_.initial_positions)
    data()->qpos[model()->jnt_qposadr[mj_name2id(model(), mjOBJ_JOINT, n.c_str())]] = q;
  for (size_t i = 0; i < joints_.size(); ++i) {
    controls_.target[i] = joints_[i].scalar ? data()->qpos[joints_[i].qpos] : 0;
    controls_.enabled[i] = joints_[i].controllable;
    auto settings = profile_.joint_settings.count(joints_[i].name)
                        ? profile_.joint_settings.at(joints_[i].name)
                        : profile_.defaults;
    controls_.kp[i] = settings.kp;
    controls_.kd[i] = settings.kd;
  }
  controls_.command = profile_.command;
  mj_forward(model(), data());
  if (profile_.start_suspended && root_joint_ >= 0)
    suspend(true, profile_.suspend_height, profile_.suspend_quat);
  controller_->reset(*model(), *data());
  adapter_->reset(*model(), *data());
}
bool Simulation::suspended() const {
  return data()->eq_active[hold_eq_] != 0;
}
void Simulation::suspend(bool active, double height, const std::array<double, 4>& quat) {
  if (root_joint_ < 0) {
    if (active) throw std::runtime_error("Suspension requires a free joint on base_body");
    return;
  }
  double norm = 0;
  for (double q : quat) norm += q * q;
  if (!std::isfinite(height) || !std::isfinite(norm) || norm < 1e-12)
    throw std::runtime_error("Invalid suspension pose");
  for (int id : old_base_constraints_) data()->eq_active[id] = 0;
  if (active) {
    const int q = model()->jnt_qposadr[root_joint_], v = model()->jnt_dofadr[root_joint_];
    double* pos = data()->mocap_pos + 3 * mocap_id_;
    double* orientation = data()->mocap_quat + 4 * mocap_id_;
    pos[0] = data()->qpos[q];
    pos[1] = data()->qpos[q + 1];
    pos[2] = height;
    for (int k = 0; k < 4; ++k) orientation[k] = quat[k] / std::sqrt(norm);
    std::copy(pos, pos + 3, data()->qpos + q);
    std::copy(orientation, orientation + 4, data()->qpos + q + 3);
    mju_zero(data()->qvel + v, 6);
  }
  data()->eq_active[hold_eq_] = active;
  mj_forward(model(), data());
}
void Simulation::step(const std::function<void(const Simulation&)>& before) {
  if (steps_ % decimation_ == 0) controller_->update(*model(), *data(), controls_);
  adapter_->apply(*model(), *data(), joints_, controls_);
  for (int a = 0; a < model()->nu; ++a)
    if (!std::isfinite(data()->ctrl[a])) throw std::runtime_error("Nonfinite control input");
  // 기록 시각은 적분 전 t로 통일한다. 힘도 같은 qpos/qvel/ctrl에서 계산한다.
  mj_forward(model(), data());
  if (before) before(*this);
  const double previous = data()->time;
  mj_step(model(), data());
  adapter_->after_step(*model(), *data());
  ++steps_;
  if (data()->time <= previous || !std::isfinite(data()->time))
    throw std::runtime_error("MuJoCo reset / nonfinite simulation state");
}
std::vector<DiagnosticChannel> Simulation::channels() const {
  std::vector<DiagnosticChannel> result;
  for (const auto& j : joints_)
    if (j.scalar) {
      result.push_back({j.name + "/position", j.type == mjJNT_HINGE ? "rad" : "m"});
      result.push_back({j.name + "/velocity", j.type == mjJNT_HINGE ? "rad/s" : "m/s"});
      result.push_back({j.name + "/actuation", j.type == mjJNT_HINGE ? "N m" : "N"});
      if (j.joint_target_available)
        result.push_back({j.name + "/target", j.type == mjJNT_HINGE ? "rad" : "m"});
    }
  for (int a = 0; a < model()->nu; ++a)
    result.push_back({object_name(model(), mjOBJ_ACTUATOR, a) + "/ctrl", "actuator input"});
  for (auto axis : {"vx", "vy", "vz", "wx", "wy", "wz"})
    result.push_back({std::string("base/") + axis, axis[0] == 'v' ? "m/s" : "rad/s"});
  if (profile_.command_mode == "run_stop")
    result.push_back({"command/run_requested", "1"});
  else
    for (auto axis : {"vx", "vy", "yaw"})
      result.push_back({std::string("command/") + axis, axis[0] == 'v' ? "m/s" : "rad/s"});
  for (const auto& name : profile_.contact_bodies)
    result.push_back({name + "/contact_force_norm", "N"});
  const auto cc = controller_->channels(), ac = adapter_->channels();
  result.insert(result.end(), cc.begin(), cc.end());
  result.insert(result.end(), ac.begin(), ac.end());
  std::set<std::string> names;
  for (const auto& c : result)
    if (c.name.empty() || c.unit.empty() || !names.insert(c.name).second)
      throw std::runtime_error("Invalid / duplicate diagnostic channel: " + c.name);
  return result;
}
void Simulation::diagnostics(double* out) const {
  const auto* m = model();
  const auto* d = data();
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto& j = joints_[i];
    if (j.scalar) {
      *out++ = d->qpos[j.qpos];
      *out++ = d->qvel[j.dof];
      *out++ = d->qfrc_actuator[j.dof];
      if (j.joint_target_available) *out++ = controls_.target[i];
    }
  }
  for (int a = 0; a < m->nu; ++a) *out++ = d->ctrl[a];
  mjtNum vel[6];
  mj_objectVelocity(m, d, mjOBJ_BODY, base_body_, vel, 1);
  for (int k : {3, 4, 5, 0, 1, 2}) *out++ = vel[k];
  if (profile_.command_mode == "run_stop")
    *out++ = controls_.command[0];
  else
    for (double command : controls_.command) *out++ = command;
  for (int body : contact_body_ids_) {
    double force = 0;
    for (int c = 0; c < d->ncon; ++c) {
      const auto& con = d->contact[c];
      bool match = false;
      for (int g : {con.geom[0], con.geom[1]})
        if (g >= 0) {
          int b = m->geom_bodyid[g];
          while (b && b != body) b = m->body_parentid[b];
          match |= b == body;
        }
      if (match) {
        mjtNum f[6];
        mj_contactForce(m, d, c, f);
        force += mju_norm3(f);
      }
    }
    *out++ = force;
  }
  controller_->diagnostics(*m, *d, out);
  out += controller_channels_;
  adapter_->diagnostics(*m, *d, out);
}
std::string Simulation::fingerprint() const {
  // MJB에는 컴파일된 메시·관성·옵션·이름까지 포함된다. 같은 차원의 다른 모델을 거부한다.
  std::vector<char> buffer(mj_sizeModel(model()));
  mj_saveModel(model(), nullptr, buffer.data(), static_cast<int>(buffer.size()));
  uint64_t hash = 14695981039346656037ULL;
  for (unsigned char b : buffer) {
    hash ^= b;
    hash *= 1099511628211ULL;
  }
  std::ostringstream out;
  out << std::hex << std::setw(16) << std::setfill('0') << hash;
  return out.str();
}
}  // namespace rclab
