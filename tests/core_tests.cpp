#include <unistd.h>

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <thread>

#include "rclab/runner.hpp"

namespace {
void require(bool ok, const std::string& message) {
  if (!ok) throw std::runtime_error(message);
}
template <class F>
void rejects(F&& f, const std::string& msg) {
  bool caught = false;
  try {
    f();
  } catch (const std::exception&) {
    caught = true;
  }
  require(caught, msg);
}
std::filesystem::path source = RCLAB_SOURCE_DIR;
std::filesystem::path scratch;
using Json = nlohmann::json;
void write_json(const std::filesystem::path& p, const Json& j) {
  std::ofstream o(p);
  o << j.dump(2);
}
class StepTarget : public rclab::IController {
 public:
  void update(const mjModel&, const mjData& d, rclab::ControlState& c) override {
    for (size_t i = 0; i < c.target.size(); ++i) {
      c.target[i] = d.time >= 0.01 - 1e-10 ? 0.2 : 0;
      c.kp[i] = 4;
      c.kd[i] = 0;
    }
  }
  std::vector<rclab::DiagnosticChannel> channels() const override {
    return {{"example/pulse", "1"}};
  }
  void diagnostics(const mjModel&, const mjData& d, double* out) const override {
    out[0] = d.time >= 0.01 - 1e-10 ? 1 : 0;
  }
};
void models() {
  for (const auto& [name, count] : std::vector<std::pair<std::string, int>>{
           {"biped", 6}, {"quadruped", 12}, {"hexapod", 18}, {"humanoid", 21}}) {
    rclab::Simulation s(rclab::RobotProfile::load(source / "profiles" / (name + ".json")));
    int actual = 0;
    for (const auto& j : s.joints()) {
      actual += j.controllable;
      if (j.scalar)
        require(j.qpos == s.model()->jnt_qposadr[j.id] && j.dof == s.model()->jnt_dofadr[j.id],
                "address mapping");
    }
    require(actual == count, "dynamic controls count: " + name);
    for (int i = 0; i < 1000; ++i) s.step();
    mj_forward(s.model(), s.data());
    require(std::abs(s.data()->xpos[3 * s.base_body() + 2] - s.profile().suspend_height) < .005,
            "suspended base height: " + name);
    double held = s.data()->xpos[3 * s.base_body() + 2];
    s.suspend(false, s.profile().suspend_height, s.profile().suspend_quat);
    for (int i = 0; i < 50; ++i) s.step();
    mj_forward(s.model(), s.data());
    require(s.data()->xpos[3 * s.base_body() + 2] < held - .01, "release falls");
  }
}
void recording_catalog() {
  const auto folder = scratch / "catalog";
  require(rclab::list_recordings(folder).empty(), "missing catalog is empty");
  std::filesystem::create_directories(folder);
  require(rclab::list_recordings(folder).empty(), "empty catalog");
  {
    std::ofstream(folder / "older.rclog") << "log";
    std::ofstream(folder / "older.rclog.json") << "{}";
    std::ofstream(folder / "newer.rclog") << "new log";
    std::ofstream(folder / "ignored.txt") << "ignore";
  }
  const auto now = std::filesystem::file_time_type::clock::now();
  std::filesystem::last_write_time(folder / "older.rclog", now - std::chrono::seconds(10));
  std::filesystem::last_write_time(folder / "newer.rclog", now);
  const auto files = rclab::list_recordings(folder);
  require(files.size() == 2 && files[0].path.filename() == "newer.rclog" &&
              files[1].path.filename() == "older.rclog",
          "catalog filters and sorts newest first");
  require(!files[0].has_metadata && files[1].has_metadata && files[1].bytes == 3,
          "catalog identifies incomplete recording and byte size");
  rejects([&] { rclab::list_recordings(folder / "ignored.txt"); },
          "catalog rejects file as directory");
}
void recordings() {
  rclab::register_controller("test_step", [] { return std::make_unique<StepTarget>(); });
  auto p = rclab::RobotProfile::load(source / "profiles/biped.json");
  p.controller = "test_step";
  rclab::Simulation s(p);
  rclab::EpisodeRecorder r;
  r.start(s, .1, 1024 * 1024);
  for (int k = 0; k < 60; ++k) s.step([&](const rclab::Simulation& state) { r.capture(state); });
  require(!r.active() && r.episode().frames() == 50, "duration cap");
  const auto& ep = r.episode();
  auto channel = [&](const std::string& name) {
    const auto& channels = ep.metadata.at("channels");
    for (size_t i = 0; i < channels.size(); ++i)
      if (channels[i].at("name") == name) return i;
    throw std::runtime_error("Missing channel: " + name);
  };
  for (size_t j = 0; j < s.joints().size(); ++j)
    if (s.joints()[j].scalar) {
      for (size_t k = 0; k < ep.frames(); ++k) {
        double t = ep.time(k), q = ep.value(k, channel(s.joints()[j].name + "/position")),
               tau = ep.value(k, channel(s.joints()[j].name + "/actuation"));
        double target = ep.frame(k)[ep.state_size + ep.channel_count + j];
        require(ep.value(k, channel(s.joints()[j].name + "/target")) == target,
                "target graph matches recorded target");
        require(std::abs(target - (t >= .01 - 1e-10 ? .2 : 0)) < 1e-12,
                "target transition timestamp");
        require(std::abs(tau - 4 * (target - q)) < 1e-9, "force and state time alignment");
      }
    }
  require(ep.nearest(.010000000000001) == 5 && ep.nearest(-1) == 0 && ep.nearest(100) == 49,
          "nearest samples and endpoints");
  const auto file = scratch / "roundtrip.rclog";
  ep.save(file);
  auto loaded = rclab::Episode::load(file, s);
  try {
    rclab::Episode::load(scratch / "missing.rclog", s);
    require(false, "missing recording accepted");
  } catch (const std::exception& error) {
    require(std::string(error.what()) == "기록 파일을 찾을 수 없습니다. 경로를 확인해 주세요.",
            "missing recording has actionable Korean message");
  }
  require(loaded.rows.size() == ep.rows.size() &&
              std::memcmp(loaded.rows.data(), ep.rows.data(), ep.rows.size() * sizeof(double)) == 0,
          "compressed bit-exact roundtrip");
  const size_t raw_bytes = ep.rows.size() * sizeof(double);
  require(std::filesystem::file_size(file) < 24 + raw_bytes, "compression reduces fixture size");
  rejects([&] { rclab::Episode::load(file, s, raw_bytes - 1); },
          "memory limit applies to decoded size");
  auto metadata = Json::parse(std::ifstream(file.string() + ".json"));
  require(metadata.at("storage").at("codec") == "zstd", "compressed storage metadata");
  std::ifstream bytes(file, std::ios::binary);
  std::vector<char> payload{std::istreambuf_iterator<char>(bytes), {}};
  require(std::string(payload.data(), 8) == "RCLOG002", "compressed format header");
  const auto damaged = scratch / "damaged.rclog";
  auto write_damaged = [&](const std::vector<char>& data) {
    std::ofstream out(damaged, std::ios::binary);
    out.write(data.data(), static_cast<std::streamsize>(data.size()));
    write_json(damaged.string() + ".json", metadata);
  };
  auto bad = payload;
  bad.pop_back();
  write_damaged(bad);
  rejects([&] { rclab::Episode::load(damaged, s); }, "truncated compressed frame");
  bad = payload;
  bad.push_back(0);
  write_damaged(bad);
  rejects([&] { rclab::Episode::load(damaged, s); }, "trailing compressed bytes");
  bad = payload;
  bad.insert(bad.end(), payload.begin() + 24, payload.end());
  write_damaged(bad);
  rejects([&] { rclab::Episode::load(damaged, s); }, "concatenated compressed frames");
  write_damaged(payload);
  auto wrong = metadata;
  wrong["storage"]["uncompressed_bytes"] = raw_bytes + 8;
  write_json(damaged.string() + ".json", wrong);
  rejects([&] { rclab::Episode::load(damaged, s); }, "decoded length metadata mismatch");
  wrong = metadata;
  wrong["checksum_fnv1a64"] = wrong["checksum_fnv1a64"].get<uint64_t>() ^ 1;
  write_json(damaged.string() + ".json", wrong);
  rejects([&] { rclab::Episode::load(damaged, s); }, "decoded checksum mismatch");
  wrong = metadata;
  wrong["frames"] = std::numeric_limits<uint64_t>::max();
  bad = payload;
  uint64_t huge = std::numeric_limits<uint64_t>::max();
  std::memcpy(bad.data() + 8, &huge, sizeof(huge));
  write_damaged(bad);
  write_json(damaged.string() + ".json", wrong);
  rejects([&] { rclab::Episode::load(damaged, s); }, "oversized decoded header");
  // 기존 비압축 v1 파일을 직접 만들어 호환성과 압축 재저장을 확인한다.
  const auto legacy_file = scratch / "legacy.rclog";
  auto legacy_meta = metadata;
  legacy_meta.erase("storage");
  write_json(legacy_file.string() + ".json", legacy_meta);
  {
    std::ofstream legacy(legacy_file, std::ios::binary);
    const uint64_t counts[] = {ep.frames(), ep.stride};
    legacy.write("RCLOG001", 8);
    legacy.write(reinterpret_cast<const char*>(counts), sizeof(counts));
    legacy.write(reinterpret_cast<const char*>(ep.rows.data()), raw_bytes);
  }
  auto legacy = rclab::Episode::load(legacy_file, s);
  require(std::memcmp(legacy.rows.data(), ep.rows.data(), raw_bytes) == 0,
          "legacy uncompressed bit-exact load");
  legacy.save(scratch / "converted.rclog");
  auto converted = rclab::Episode::load(scratch / "converted.rclog", s);
  require(std::memcmp(converted.rows.data(), ep.rows.data(), raw_bytes) == 0,
          "legacy to compressed conversion is bit-exact");
  auto precision = ep;
  precision.rows[precision.state_size] = -0.0;
  precision.rows[precision.state_size + 1] = std::nextafter(1.0, 2.0);
  precision.save(scratch / "precision.rclog");
  const auto exact = rclab::Episode::load(scratch / "precision.rclog", s);
  require(std::memcmp(exact.rows.data(), precision.rows.data(), raw_bytes) == 0,
          "signed zero and float64 precision preserved");
  std::cout
      << "PASS compressed/legacy bit-exact roundtrip, conversion, corruption, decoded limits; "
      << "raw_bytes=" << 24 + raw_bytes << " compressed_bytes=" << std::filesystem::file_size(file)
      << '\n';
  rejects([&] { ep.save(file); }, "do not overwrite recordings");
  std::vector<double> live(mj_stateSize(s.model(), mjSTATE_INTEGRATION));
  mj_getState(s.model(), s.data(), live.data(), mjSTATE_INTEGRATION);
  rclab::ReplaySession replay(s, std::move(loaded));
  replay.seek(.014);
  double frozen = replay.data()->time;
  for (int i = 0; i < 20; ++i) mj_forward(s.model(), replay.data());
  require(replay.data()->time == frozen, "paused replay does not integrate");
  std::vector<double> after(live.size());
  mj_getState(s.model(), s.data(), after.data(), mjSTATE_INTEGRATION);
  require(live == after, "replay preserves live state");
  auto jp = Json::parse(std::ifstream(file.string() + ".json"));
  jp["model_fingerprint"] = "wrong";
  write_json(file.string() + ".json", jp);
  rejects([&] { rclab::Episode::load(file, s); }, "reject mismatched model");
  jp["model_fingerprint"] = s.fingerprint();
  write_json(file.string() + ".json", jp);
  {
    std::fstream binary(file, std::ios::in | std::ios::out | std::ios::binary);
    binary.seekp(25);
    char b = 42;
    binary.write(&b, 1);
  }
  rejects([&] { rclab::Episode::load(file, s); }, "reject corrupt bytes");
  rclab::EpisodeRecorder small;
  const size_t frame_bytes = ep.stride * sizeof(double);
  small.start(s, 120, frame_bytes * 2);
  for (int i = 0; i < 4; ++i) s.step([&](const rclab::Simulation& st) { small.capture(st); });
  require(small.episode().frames() == 2 && !small.active(), "memory budget cap");
}
void edge_cases() {
  std::ofstream xml(scratch / "edge.xml");
  xml << R"(<mujoco><option timestep="0.002" gravity="0 0 0"/><worldbody>
    <body name="object" pos="0 0 5"><freejoint name="object_root"/><geom type="sphere" size=".05"/></body>
    <body name="base" pos="0 0 1"><freejoint name="robot_root"/><geom type="sphere" size=".1"/>
      <body name="slide"><joint name="linear" type="slide" axis="1 0 0" limited="true" range="-.2 .2"/><geom type="sphere" size=".05"/></body>
      <body name="ball"><joint name="ball_joint" type="ball"/><geom type="sphere" size=".05"/></body>
      <body name="hinge"><joint name="rotary" axis="0 1 0"/><geom type="sphere" size=".05"/></body>
    </body></worldbody><equality><weld name="original_pin" body1="base"/></equality>
    <actuator><motor name="negative_gear" joint="linear" gear="-2" ctrlrange="-3 3" ctrllimited="true"/><position name="servo" joint="rotary" kp="10"/></actuator></mujoco>)";
  xml.close();
  Json j = {{"schema_version", 1}, {"name", "edge"},     {"model", "edge.xml"},
            {"base_body", "base"}, {"control_dt", .002}, {"suspension", {{"height", 2}}}};
  const auto file = scratch / "edge.json";
  write_json(file, j);
  rclab::Simulation s(rclab::RobotProfile::load(file));
  require(s.base_body() != 1 && s.model()->nq != s.model()->nv, "additional free body");
  int pin = mj_name2id(s.model(), mjOBJ_EQUALITY, "original_pin");
  require(!s.data()->eq_active[pin], "existing base pin disabled");
  size_t linear = 0;
  bool found_ball = false, found_servo = false;
  for (size_t i = 0; i < s.joints().size(); ++i) {
    const auto& a = s.joints()[i];
    if (a.name == "linear") {
      linear = i;
      require(a.controllable && a.type == mjJNT_SLIDE && a.qpos > 7, "slide addresses");
    }
    if (a.name == "ball_joint") found_ball = !a.scalar && !a.controllable;
    if (a.name == "rotary") found_servo = !a.controllable;
  }
  require(found_ball && found_servo, "unsupported types are inspect-only");
  s.controls().target[linear] = .2;
  s.controls().kp[linear] = 1000;
  s.controls().kd[linear] = 0;
  s.step();
  require(s.data()->ctrl[0] == -3, "negative gear and input clamp");
  auto invalid = j;
  invalid["base_body"] = "missing";
  write_json(scratch / "bad.json", invalid);
  rejects([&] { rclab::Simulation bad(rclab::RobotProfile::load(scratch / "bad.json")); },
          "bad base name");
  invalid = j;
  invalid["control_dt"] = .003;
  write_json(scratch / "bad.json", invalid);
  rejects([&] { rclab::Simulation bad(rclab::RobotProfile::load(scratch / "bad.json")); },
          "nonintegral decimation");
  invalid = j;
  invalid["groups"] = Json::array({{{"name", "oops"}, {"joints", {"missing"}}}});
  write_json(scratch / "bad.json", invalid);
  rejects([&] { rclab::Simulation bad(rclab::RobotProfile::load(scratch / "bad.json")); },
          "bad group joint");
  rejects([&] { rclab::RobotProfile::load(file, "nonexistent.xml"); }, "bad model path");
}
void threaded() {
  rclab::SimulationRunner runner(rclab::RobotProfile::load(source / "profiles/hexapod.json"));
  rclab::DataPtr copy(mj_makeData(runner.simulation().model()));
  runner.start();
  runner.start_recording();
  runner.set_running(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  runner.set_running(false);
  rclab::TimingStats timing;
  size_t frames;
  bool recording;
  std::string status;
  runner.snapshot(copy.get(), timing, frames, recording, status);
  require(frames > 5, "worker records");
  double t = copy->time;
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  runner.snapshot(copy.get(), timing, frames, recording, status);
  require(copy->time == t, "worker pauses");
  runner.single_step();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  runner.snapshot(copy.get(), timing, frames, recording, status);
  require(std::abs(copy->time - t - .002) < 1e-9, "single physical step");
  mjvScene scene;
  mjvCamera camera;
  mjvOption option;
  mjvPerturb perturb;
  mjv_defaultScene(&scene);
  mjv_defaultCamera(&camera);
  mjv_defaultOption(&option);
  mjv_defaultPerturb(&perturb);
  const auto* model = runner.simulation().model();
  mjv_makeScene(model, &scene, 1000);
  mj_forward(model, copy.get());
  mjv_updateScene(model, copy.get(), &option, nullptr, &camera, mjCAT_ALL, &scene);
  perturb.select = runner.simulation().base_body();
  mjv_initPerturb(model, copy.get(), &scene, &perturb);
  perturb.refselpos[0] += .1;
  perturb.active = mjPERT_TRANSLATE;
  runner.set_perturbation(perturb);
  runner.single_step();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  runner.snapshot(copy.get(), timing, frames, recording, status);
  require(std::abs(copy->xfrc_applied[6 * perturb.select]) > 1e-5,
          "mouse perturbation applies force");
  perturb.active = 0;
  runner.set_perturbation(perturb);
  runner.single_step();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  runner.snapshot(copy.get(), timing, frames, recording, status);
  require(copy->xfrc_applied[6 * perturb.select] == 0, "mouse release clears force");
  mjv_freeScene(&scene);
  auto ep = runner.finish_recording();
  require(ep.frames() > 0, "worker releases recording");
  runner.stop();
}
void external(const std::string& profile, const std::string& model) {
  rclab::Simulation s(rclab::RobotProfile::load(profile, model));
  size_t scalar = 0, controlled = 0;
  for (const auto& j : s.joints()) {
    scalar += j.scalar;
    controlled += j.controllable;
    require(j.qpos == s.model()->jnt_qposadr[j.id] && j.dof == s.model()->jnt_dofadr[j.id],
            "external joint mapping");
  }
  require(scalar == static_cast<size_t>(s.model()->nu) && controlled == scalar,
          "external controls count");
  int root = -1;
  for (int j = 0; j < s.model()->njnt; ++j)
    if (s.model()->jnt_bodyid[j] == s.base_body() && s.model()->jnt_type[j] == mjJNT_FREE) root = j;
  require(root >= 0, "external base freejoint");
  size_t steps = static_cast<size_t>(.5 / s.model()->opt.timestep);
  for (size_t i = 0; i < steps; ++i) s.step();
  mj_forward(s.model(), s.data());
  double held = s.data()->xpos[3 * s.base_body() + 2];
  require(std::abs(held - s.profile().suspend_height) < .005, "external suspended height");
  s.suspend(false, s.profile().suspend_height, s.profile().suspend_quat);
  for (size_t i = 0; i < steps / 5; ++i) s.step();
  mj_forward(s.model(), s.data());
  require(s.data()->xpos[3 * s.base_body() + 2] < held - .005, "external hold release");
  rclab::EpisodeRecorder recorder;
  recorder.start(s, .05, 1024 * 1024);
  for (size_t i = 0; i < steps / 10; ++i)
    s.step([&](const rclab::Simulation& st) { recorder.capture(st); });
  auto ep = recorder.release();
  ep.save(scratch / "external.rclog");
  rclab::Simulation reopened(rclab::RobotProfile::load(profile, model));
  auto loaded = rclab::Episode::load(scratch / "external.rclog", reopened);
  require(ep.rows == loaded.rows, "external record roundtrip");
  rclab::ReplaySession replay(reopened, std::move(loaded));
  replay.select(ep.frames() - 1);
  int q = s.model()->jnt_qposadr[root];
  require(std::abs(replay.data()->qpos[q] - ep.frame(ep.frames() - 1)[1 + q]) < 1e-10,
          "external replay root");
  std::cout << "PASS external " << s.profile().name << " scalar=" << scalar
            << " controlled=" << controlled << " held_z=" << held << " root_qpos=" << q << '\n';
}
}  // namespace
int main(int argc, char** argv) {
  try {
    scratch = std::filesystem::temp_directory_path() / ("rclab-tests-" + std::to_string(getpid()));
    std::filesystem::create_directories(scratch);
    if (argc == 4 && std::string(argv[1]) == "--external") {
      external(argv[2], argv[3]);
      std::filesystem::remove_all(scratch);
      return 0;
    }
    models();
    std::cout << "PASS robot counts, hold, release\n";
    recordings();
    recording_catalog();
    std::cout << "PASS recording catalog filter, order, missing metadata, missing directory\n";
    std::cout << "PASS timestamp, roundtrip, replay isolation, corruption, caps\n";
    edge_cases();
    std::cout << "PASS free objects, units, ball, gear, unsupported actuator, invalid profiles\n";
    threaded();
    std::cout << "PASS worker recording, pause, single step, perturbation apply/release\n";
    std::filesystem::remove_all(scratch);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "FAIL " << e.what() << '\n';
    return 1;
  }
}
