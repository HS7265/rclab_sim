#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "rclab/extensions.hpp"
#include "rclab/recording.hpp"

int main(int argc, char** argv) {
  try {
    rclab::register_extensions();
    std::string profile, model, record;
    double duration = 1;
    double run_start = -1, run_stop = -1;
    bool realtime = false;
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      auto next = [&] {
        if (++i >= argc) throw std::runtime_error("Missing value for " + arg);
        return std::string(argv[i]);
      };
      if (arg == "--robot")
        profile = next();
      else if (arg == "--model")
        model = next();
      else if (arg == "--duration")
        duration = std::stod(next());
      else if (arg == "--record")
        record = next();
      else if (arg == "--run-start")
        run_start = std::stod(next());
      else if (arg == "--run-stop")
        run_stop = std::stod(next());
      else if (arg == "--realtime")
        realtime = true;
      else if (arg == "--help") {
        std::cout << "rclab_headless --robot PROFILE [--model XML] [--duration SECONDS] [--record "
                     "FILE.rclog] [--realtime] [--run-start SECONDS --run-stop SECONDS]\n";
        return 0;
      } else
        throw std::runtime_error("Unknown argument: " + arg);
    }
    if (profile.empty() || !std::isfinite(duration) || duration <= 0)
      throw std::runtime_error("--robot and positive finite --duration required");
    rclab::Simulation sim(rclab::RobotProfile::load(profile, model));
    if (!std::isfinite(run_start) || !std::isfinite(run_stop) ||
        (run_start >= 0 && (run_stop <= run_start || sim.profile().command_mode != "run_stop")))
      throw std::runtime_error("Run schedule requires run_stop profile and stop > start >= 0");
    rclab::EpisodeRecorder recorder;
    if (!record.empty())
      recorder.start(sim, sim.profile().record_seconds, sim.profile().record_bytes);
    using Clock = std::chrono::steady_clock;
    auto begin = Clock::now(), deadline = begin;
    size_t count = 0, late = 0;
    double mean = 0, maximum = 0;
    while (sim.data()->time < duration - 1e-10) {
      if (run_start >= 0 && count % sim.control_decimation() == 0)
        sim.controls().command = {
            sim.data()->time + 1e-9 >= run_start && sim.data()->time + 1e-9 < run_stop ? 1. : 0.,
            0., 0.};
      auto t = Clock::now();
      sim.step([&](const rclab::Simulation& s) { recorder.capture(s); });
      double ms = std::chrono::duration<double, std::milli>(Clock::now() - t).count();
      ++count;
      mean += (ms - mean) / count;
      maximum = std::max(maximum, ms);
      if (realtime) {
        deadline += std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(sim.model()->opt.timestep));
        if (Clock::now() > deadline) {
          ++late;
          deadline = Clock::now();
        }
        std::this_thread::sleep_until(deadline);
      }
    }
    if (!record.empty()) {
      recorder.stop();
      recorder.episode().save(record);
    }
    double wall = std::chrono::duration<double>(Clock::now() - begin).count();
    size_t scalar = 0, controlled = 0;
    for (const auto& j : sim.joints()) {
      scalar += j.scalar;
      controlled += j.controllable;
    }
    std::cout << "model=" << sim.profile().name << " nq=" << sim.model()->nq
              << " nv=" << sim.model()->nv << " nu=" << sim.model()->nu << " scalar=" << scalar
              << " controlled=" << controlled << "\nsteps=" << count
              << " simulated_s=" << sim.data()->time << " wall_s=" << wall << " mean_ms=" << mean
              << " max_ms=" << maximum << " overruns=" << late << " rtf=" << sim.data()->time / wall
              << " recorded=" << recorder.episode().frames() << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << '\n';
    return 1;
  }
}
