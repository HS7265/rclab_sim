// 외부 참조 코어와 서브스텝 단위로 계약을 대조한다. 연구실 자산은 포함하지 않는다.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "rclab/recording.hpp"
#include "rok4/rok4_core.h"

int main(int argc, char** argv) {
  try {
    if (argc != 3) throw std::runtime_error("Expected profile and bundle");
    auto profile = rclab::RobotProfile::load(argv[1]);
    profile.extension["full_policy_diagnostics"] = true;
    rclab::Simulation sim(profile);
    rok4::Rok4Core reference(profile.model_path.string(), argv[2], 1);
    const auto channels = sim.channels();
    auto index = [&](const std::string& name) {
      for (size_t i = 0; i < channels.size(); ++i)
        if (channels[i].name == name) return i;
      throw std::runtime_error("Missing channel " + name);
    };
    const auto obs = index("student/observation/0"), hidden = index("student/hidden/0");
    std::vector<int> controls;
    std::vector<size_t> actions;
    for (int i = 0; i < 21; ++i) {
      int actuator = -1;
      for (int a = 0; a < sim.model()->nu; ++a)
        if (sim.model()->jnt_qposadr[sim.model()->actuator_trnid[2 * a]] ==
            reference.qpos_address(i))
          actuator = a;
      if (actuator < 0) throw std::runtime_error("Missing torque actuator");
      controls.push_back(actuator);
      std::string name = mj_id2name(sim.model(), mjOBJ_ACTUATOR, actuator);
      if (name.rfind("robot/", 0) == 0) name.erase(0, 6);
      actions.push_back(index("student/" + name + "/filtered_action"));
    }
    std::vector<double> diagnostics(channels.size());
    struct Sample {
      std::vector<double> q, v, torque, action, obs, hidden;
    };
    std::vector<Sample> samples;
    reference.set_substep_hook([&] {
      auto* d = reference.data();
      samples.push_back({{d->qpos, d->qpos + reference.model()->nq},
                         {d->qvel, d->qvel + reference.model()->nv},
                         {reference.last_torque().begin(), reference.last_torque().end()},
                         {reference.last_action().begin(), reference.last_action().end()},
                         {reference.last_observation().begin(), reference.last_observation().end()},
                         {reference.policy_hidden().begin(), reference.policy_hidden().end()}});
    });
    double max_q = 0, max_v = 0, max_obs = 0, max_hidden = 0, max_torque = 0, max_action = 0;
    // 초기 정지·가속·달리기 진입까지 비교한다. 폐루프 오차는 별도로 보고한다.
    for (int block = 0; block < 450; ++block) {
      rok4::Command command;
      command.vx = block >= 100 ? 1. : 0.;
      samples.clear();
      reference.step(command);
      sim.controls().command = {command.vx, 0, 0};
      for (int sub = 0; sub < 5; ++sub)
        sim.step([&](const rclab::Simulation& s) {
          const auto& expected = samples.at(sub);
          s.diagnostics(diagnostics.data());
          for (int i = 0; i < s.model()->nq; ++i)
            max_q = std::max(max_q, std::abs(s.data()->qpos[i] - expected.q[i]));
          for (int i = 0; i < s.model()->nv; ++i)
            max_v = std::max(max_v, std::abs(s.data()->qvel[i] - expected.v[i]));
          for (size_t i = 0; i < 81; ++i)
            max_obs = std::max(max_obs, std::abs(diagnostics[obs + i] - expected.obs[i]));
          for (size_t i = 0; i < 256; ++i)
            max_hidden =
                std::max(max_hidden, std::abs(diagnostics[hidden + i] - expected.hidden[i]));
          for (int i = 0; i < 21; ++i)
            max_torque =
                std::max(max_torque, std::abs(s.data()->ctrl[controls[i]] - expected.torque[i]));
          for (int i = 0; i < 21; ++i)
            max_action =
                std::max(max_action, std::abs(diagnostics[actions[i]] - expected.action[i]));
        });
    }
    std::cout << "parity 4.5s max_q=" << max_q << " max_v=" << max_v << " max_obs=" << max_obs
              << " max_hidden=" << max_hidden << " max_torque=" << max_torque
              << " max_action=" << max_action << '\n';
    if (max_q > 1e-8 || max_v > 1e-7 || max_obs > 1e-6 || max_hidden > 1e-6 || max_torque > 1e-5 ||
        max_action > 1e-6)
      throw std::runtime_error("Reference parity failed");
    sim.reset();
    reference.reset();
    samples.clear();
    reference.step({});
    sim.step();
    sim.diagnostics(diagnostics.data());
    for (size_t i = 0; i < 81; ++i)
      if (diagnostics[obs + i] != samples.front().obs[i])
        throw std::runtime_error("Reset observation mismatch");
    for (size_t i = 0; i < 256; ++i)
      if (diagnostics[hidden + i] != samples.front().hidden[i])
        throw std::runtime_error("Reset hidden mismatch");
    for (int i = 0; i < 21; ++i)
      if (sim.data()->ctrl[controls[i]] != samples.front().torque[i])
        throw std::runtime_error("Reset actuator state mismatch");
    std::cout << "reset observation / hidden / torque PASS\n";
    sim.reset();
    rclab::EpisodeRecorder recorder;
    recorder.start(sim, 31, 256 * 1024 * 1024);
    double min_height = 10, run_speed = 0;
    size_t running = 0;
    const auto mode = index("student/mode");
    for (int step = 0; step < 15000; ++step) {
      sim.controls().command = {step >= 500 && step < 5500 ? 1. : 0., 0, 0};
      sim.step([&](const rclab::Simulation& s) {
        recorder.capture(s);
        s.diagnostics(diagnostics.data());
        for (double value : diagnostics)
          if (!std::isfinite(value)) throw std::runtime_error("Nonfinite recorded diagnostics");
        min_height = std::min(min_height, s.data()->qpos[2]);
        if (diagnostics[mode] == 2) {
          run_speed += s.data()->qvel[0];
          ++running;
        }
      });
    }
    recorder.stop();
    std::cout << "scenario 30s samples=" << recorder.episode().frames()
              << " min_height=" << min_height << " final_height=" << sim.data()->qpos[2]
              << " run_world_vx=" << run_speed / running << '\n';
    if (min_height < .6 || !running || diagnostics[mode] != 0 ||
        recorder.episode().frames() != 15000)
      throw std::runtime_error("Run/stop scenario failed");
    auto path =
        std::filesystem::temp_directory_path() /
        ("rclab-student-" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".rclog");
    recorder.episode().save(path);
    auto loaded = rclab::Episode::load(path, sim);
    if (loaded.rows != recorder.episode().rows)
      throw std::runtime_error("Record roundtrip mismatch");
    std::vector<double> live(sim.data()->qpos, sim.data()->qpos + sim.model()->nq);
    rclab::ReplaySession replay(sim, std::move(loaded));
    auto selected = replay.seek(6.1234);
    if (std::abs(replay.data()->time - 6.124) > 1e-8 || selected != 3062 ||
        !std::equal(live.begin(), live.end(), sim.data()->qpos))
      throw std::runtime_error("Replay selection/live isolation failed");
    auto metadata = recorder.episode().metadata;
    metadata["control_contract"]["lag_substeps"] = 3;
    std::ifstream input(path.string() + ".json");
    nlohmann::json stored;
    input >> stored;
    input.close();
    stored["control_contract"] = metadata["control_contract"];
    {
      std::ofstream output(path.string() + ".json");
      output << stored;
    }
    bool rejected = false;
    try {
      rclab::Episode::load(path, sim);
    } catch (const std::exception&) {
      rejected = true;
    }
    if (!rejected) throw std::runtime_error("Incompatible control metadata accepted");
    std::filesystem::remove(path);
    std::filesystem::remove(path.string() + ".json");
    std::cout << "record roundtrip / replay isolation / metadata rejection PASS\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
