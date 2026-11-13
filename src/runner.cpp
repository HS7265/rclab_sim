#include "rclab/runner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace rclab {
SimulationRunner::SimulationRunner(RobotProfile p)
    : simulation_(std::move(p)),
      telemetry_(simulation_.channels().size() + 1),
      pending_controls_(simulation_.controls()) {
  mjv_defaultPerturb(&perturb_);
  telemetry_[0] = simulation_.data()->time;
  simulation_.diagnostics(telemetry_.data() + 1);
}
SimulationRunner::~SimulationRunner() {
  stop();
}
void SimulationRunner::start() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (worker_.joinable()) return;
  stop_ = false;
  worker_ = std::thread(&SimulationRunner::run, this);
}
void SimulationRunner::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
    wake_.notify_all();
  }
  if (worker_.joinable()) worker_.join();
}
void SimulationRunner::check_error() const {
  if (error_) std::rethrow_exception(error_);
}
void SimulationRunner::set_running(bool value) {
  std::lock_guard<std::mutex> lock(mutex_);
  check_error();
  running_ = value;
  wake_.notify_all();
}
void SimulationRunner::set_speed(double value) {
  if (!std::isfinite(value) || value < 0.05 || value > 4)
    throw std::runtime_error("Speed must be 0.05..4");
  std::lock_guard<std::mutex> lock(mutex_);
  speed_ = value;
  wake_.notify_all();
}
void SimulationRunner::single_step() {
  std::lock_guard<std::mutex> lock(mutex_);
  check_error();
  running_ = false;
  ++pending_steps_;
  wake_.notify_all();
}
void SimulationRunner::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  check_error();
  reset_requested_ = true;
  wake_.notify_all();
}
void SimulationRunner::update_controls(const ControlState& c) {
  const auto count = simulation_.joints().size();
  if (c.target.size() != count || c.kp.size() != count || c.kd.size() != count ||
      c.enabled.size() != count)
    throw std::runtime_error("Control dimensions mismatch");
  for (size_t i = 0; i < count; ++i)
    if (!std::isfinite(c.target[i]) || !std::isfinite(c.kp[i]) || !std::isfinite(c.kd[i]) ||
        c.kp[i] < 0 || c.kd[i] < 0)
      throw std::runtime_error("Invalid control value");
  for (double x : c.command)
    if (!std::isfinite(x)) throw std::runtime_error("Invalid command value");
  std::lock_guard<std::mutex> lock(mutex_);
  pending_controls_ = c;
  controls_dirty_ = true;
  wake_.notify_all();
}
void SimulationRunner::set_suspend(bool active, double height, const std::array<double, 4>& quat) {
  std::lock_guard<std::mutex> lock(mutex_);
  hold_active_ = active;
  hold_height_ = height;
  hold_quat_ = quat;
  hold_dirty_ = true;
  wake_.notify_all();
}
void SimulationRunner::set_perturbation(const mjvPerturb& p) {
  std::lock_guard<std::mutex> lock(mutex_);
  perturb_ = p;
}
void SimulationRunner::start_recording() {
  std::lock_guard<std::mutex> lock(mutex_);
  check_error();
  if (recorder_.episode().frames())
    throw std::runtime_error("Finish current recording before starting another");
  recorder_.start(simulation_, simulation_.profile().record_seconds,
                  simulation_.profile().record_bytes);
}
Episode SimulationRunner::finish_recording() {
  std::lock_guard<std::mutex> lock(mutex_);
  check_error();
  running_ = false;
  return recorder_.release();
}
void SimulationRunner::snapshot(mjData* dst, TimingStats& stats, size_t& frames, bool& recording,
                                std::string& status, std::vector<double>* telemetry) {
  std::lock_guard<std::mutex> lock(mutex_);
  check_error();
  mj_copyData(dst, simulation_.model(), simulation_.data());
  stats = stats_;
  frames = recorder_.episode().frames();
  recording = recorder_.active();
  status = recorder_.status();
  if (telemetry) {
    if (telemetry->size() != telemetry_.size())
      throw std::runtime_error("Telemetry buffer dimension mismatch");
    std::copy(telemetry_.begin(), telemetry_.end(), telemetry->begin());
  }
}
void SimulationRunner::do_step() {
  mju_zero(simulation_.data()->xfrc_applied, 6 * simulation_.model()->nbody);
  if (perturb_.active) mjv_applyPerturbForce(simulation_.model(), simulation_.data(), &perturb_);
  simulation_.step([this](const Simulation& s) {
    telemetry_[0] = s.data()->time;
    s.diagnostics(telemetry_.data() + 1);
    recorder_.capture(s);
  });
}
void SimulationRunner::run() {
  using Clock = std::chrono::steady_clock;
  auto deadline = Clock::now();
  double active_wall = 0, active_sim = 0;
  try {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!stop_) {
      if (reset_requested_) {
        recorder_.stop();
        simulation_.reset();
        pending_controls_ = simulation_.controls();
        controls_dirty_ = false;
        reset_requested_ = false;
        stats_ = {};
        active_wall = active_sim = 0;
        deadline = Clock::now();
        telemetry_[0] = simulation_.data()->time;
        simulation_.diagnostics(telemetry_.data() + 1);
      }
      if (hold_dirty_) {
        simulation_.suspend(hold_active_, hold_height_, hold_quat_);
        hold_dirty_ = false;
      }
      if (controls_dirty_) {
        simulation_.controls() = pending_controls_;
        controls_dirty_ = false;
      }
      if (!running_ && !pending_steps_) {
        wake_.wait(lock, [this] {
          return stop_ || running_ || pending_steps_ || reset_requested_ || controls_dirty_ ||
                 hold_dirty_;
        });
        deadline = Clock::now();
        continue;
      }
      const auto begin = Clock::now();
      const bool paced = running_;
      if (pending_steps_) --pending_steps_;
      do_step();
      const auto end = Clock::now();
      const double ms = std::chrono::duration<double, std::milli>(end - begin).count();
      ++stats_.steps;
      stats_.mean_ms += (ms - stats_.mean_ms) / stats_.steps;
      stats_.max_ms = std::max(ms, stats_.max_ms);
      const double dt = simulation_.model()->opt.timestep;
      const auto budget =
          std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(dt / speed_));
      deadline += budget;
      if (paced && end > deadline) {
        ++stats_.overruns;
        deadline = end;
      }
      if (paced) {
        wake_.wait_until(lock, deadline, [this] { return stop_ || !running_ || reset_requested_; });
        active_wall += std::chrono::duration<double>(Clock::now() - begin).count();
        active_sim += dt;
        stats_.realtime_factor = active_wall > 0 ? active_sim / active_wall : 0;
      }
    }
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    error_ = std::current_exception();
    running_ = false;
  }
}
}  // namespace rclab
