#pragma once
#include <atomic>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <thread>

#include "rclab/recording.hpp"

namespace rclab {
struct TimingStats {
  uint64_t steps = 0, overruns = 0;
  double mean_ms = 0, max_ms = 0, realtime_factor = 0;
};
// 하나의 worker만 live mjData를 변경한다. GUI는 짧은 복사 구간에만 mutex를 잡는다.
class SimulationRunner {
 public:
  explicit SimulationRunner(RobotProfile);
  ~SimulationRunner();
  void start();
  void stop();
  void set_running(bool);
  void set_speed(double);
  void single_step();
  void reset();
  void update_controls(const ControlState&);
  void set_suspend(bool, double, const std::array<double, 4>&);
  void set_perturbation(const mjvPerturb&);
  void start_recording();
  Episode finish_recording();
  void snapshot(mjData* destination, TimingStats&, size_t& frames, bool& recording,
                std::string& status, std::vector<double>* telemetry = nullptr);
  const Simulation& simulation() const { return simulation_; }

 private:
  void run();
  void do_step();
  void check_error() const;
  Simulation simulation_;
  std::vector<double> telemetry_;
  EpisodeRecorder recorder_;
  ControlState pending_controls_;
  bool controls_dirty_ = false, reset_requested_ = false, hold_dirty_ = false, hold_active_ = true;
  double hold_height_ = 1;
  std::array<double, 4> hold_quat_{1, 0, 0, 0};
  mjvPerturb perturb_{};
  std::mutex mutex_;
  std::condition_variable wake_;
  std::thread worker_;
  bool stop_ = false, running_ = false;
  unsigned pending_steps_ = 0;
  double speed_ = 1;
  TimingStats stats_;
  std::exception_ptr error_;
};
}  // namespace rclab
