#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <future>
#include <iostream>
#include <map>
#include <set>

#include "rclab/extensions.hpp"
#include "rclab/runner.hpp"

namespace {
using Clock = std::chrono::steady_clock;
constexpr double rad_to_deg = 180.0 / 3.141592653589793;
constexpr ImGuiWindowFlags fixed_panel = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                                         ImGuiWindowFlags_NoCollapse |
                                         ImGuiWindowFlags_NoSavedSettings;
void load_korean_font() {
  auto& io = ImGui::GetIO();
  static ImVector<ImWchar> ranges;
  ImFontGlyphRangesBuilder builder;
  builder.AddRanges(io.Fonts->GetGlyphRangesKorean());
  builder.AddText("←→·");
  builder.BuildRanges(&ranges);
  const char* custom = std::getenv("RCLAB_FONT");
  for (const char* path : {custom, "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
                           "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"}) {
    if (!path || !std::filesystem::is_regular_file(path)) continue;
    ImFontConfig config;
    config.FontNo = std::filesystem::path(path).extension() == ".ttc" &&
                            std::string(path).find("NotoSansCJK") != std::string::npos
                        ? 1
                        : 0;
    if (io.Fonts->AddFontFromFileTTF(path, 17.f, &config, ranges.Data)) return;
  }
  throw std::runtime_error(
      "한글 글꼴이 없습니다. fonts-noto-cjk 설치 또는 RCLAB_FONT 경로를 지정하세요.");
}
std::string channel_label(const rclab::DiagnosticChannel& channel) {
  std::string name = channel.name;
  const auto split = name.rfind('/');
  const std::string field = name.substr(split == std::string::npos ? 0 : split + 1);
  const std::map<std::string, std::string> labels{
      {"position", "위치"},
      {"velocity", "속도"},
      {"actuation", channel.unit == "N m" ? "토크 (torque)" : "힘"},
      {"target", "목표"},
      {"ctrl", "제어 입력"},
      {"motor_position", "모터 위치"},
      {"motor_velocity", "모터 속도"},
      {"motor_target", "모터 목표"},
      {"delayed_motor_target", "지연 후 모터 목표"},
      {"motor_torque", "모터 토크"},
      {"joint_torque", "관절 토크"},
      {"raw_action", "원시 행동"},
      {"filtered_action", "제한 후 행동"},
      {"contact_force_norm", "접촉력 크기"},
      {"mode", "동작 단계"},
      {"progress", "진행도"},
      {"run_command", "달리기 명령"},
      {"run_requested", "달리기 요청"},
      {"hidden_norm", "내부 상태 크기"},
      {"tracking_latched", "추종 보호 상태"},
      {"policy_ms", "정책 추론 시간"},
      {"actuator_ms", "구동기 추론 시간"},
      {"left_foot_force_z", "왼발 수직 힘"},
      {"right_foot_force_z", "오른발 수직 힘"},
      {"vx", "전후 속도"},
      {"vy", "좌우 속도"},
      {"vz", "수직 속도"},
      {"wx", "전후축 각속도"},
      {"wy", "좌우축 각속도"},
      {"wz", "수직축 각속도"},
      {"yaw", "회전 속도"}};
  if (auto found = labels.find(field); found != labels.end())
    name.replace(split == std::string::npos ? 0 : split + 1, field.size(), found->second);
  for (const auto& prefix :
       {std::pair{"student/", "정책/"}, {"base/", "몸통/"}, {"command/", "명령/"}})
    if (name.rfind(prefix.first, 0) == 0)
      name.replace(0, std::char_traits<char>::length(prefix.first), prefix.second);
  return name;
}
struct PlotAccess {
  const rclab::Episode* ep;
  size_t channel;
};
ImPlotPoint point_at(int index, void* context) {
  const auto& source = *static_cast<PlotAccess*>(context);
  return {source.ep->time(index), source.ep->value(index, source.channel)};
}
std::string cursor_text(double x, double y, const std::string& unit) {
  const std::string suffix = unit == "N m" ? "Nm" : unit == "1" ? " (무차원)" : unit;
  char text[256];
  std::snprintf(text, sizeof(text), "%.4gs, %.4g%s", x, y, suffix.c_str());
  return text;
}
bool show_cursor_coordinates(const std::string& unit) {
  if (!ImPlot::IsPlotHovered()) return false;
  const auto mouse = ImPlot::GetPlotMousePos();
  const auto text = cursor_text(mouse.x, mouse.y, unit);
  const auto pos = ImPlot::GetPlotPos(), size = ImPlot::GetPlotSize();
  const auto text_size = ImGui::CalcTextSize(text.c_str());
  const ImVec2 corner{pos.x + size.x - 7, pos.y + size.y - 5};
  const ImVec2 start{corner.x - text_size.x, corner.y - text_size.y};
  ImPlot::PushPlotClipRect();
  auto* draw = ImPlot::GetPlotDrawList();
  draw->AddRectFilled({start.x - 3, start.y - 2}, {corner.x + 3, corner.y + 2},
                      IM_COL32(15, 15, 15, 225));
  draw->AddText(start, IM_COL32(245, 245, 245, 255), text.c_str());
  ImPlot::PopPlotClipRect();
  return true;
}
struct Gui {
  rclab::SimulationRunner runner;
  rclab::DataPtr live;
  std::unique_ptr<rclab::ReplaySession> replay;
  rclab::ControlState controls;
  rclab::ControlState initial_controls;
  rclab::Episode preview;
  std::vector<double> telemetry;
  mjvCamera camera{};
  mjvOption options{};
  mjvPerturb perturb{};
  mjvScene scene{};
  mjrContext render{};
  GLFWwindow* window = nullptr;
  bool running = false, follow = true, playing = false, hold = true, selftest = false;
  double speed = 1, playback_speed = 1, requested_time = 0, x_min = 0, x_max = 1;
  float height = 1, euler[3]{}, control_speed = 1;
  double initial_camera_distance = 2.5;
  char search[128]{}, path[512] = "recordings/episode.rclog";
  std::string message, error_detail;
  rclab::TimingStats timing;
  size_t frame_count = 0;
  bool recording = false;
  std::string record_status;
  std::vector<rclab::DiagnosticChannel> channels;
  std::array<int, 3> selected_channels{0, 1, 2};
  std::future<std::string> save_task;
  std::future<rclab::Episode> load_task;
  std::future<std::vector<rclab::RecordingFile>> catalog_task;
  std::vector<rclab::RecordingFile> recording_files;
  char catalog_directory[512] = "recordings";
  std::string catalog_folder = "recordings", scanned_folder, catalog_error;
  std::string loading_recording, opened_recording;
  bool refresh_catalog = true;
  Clock::time_point catalog_next_scan{};
  ImVec2 view_min{}, view_size{}, render_min{}, render_size{}, graph_probe{}, view_probe{};
  double graph_probe_time = 0, graph_clicked_time = 0;
  bool graph_click_observed = false, follow_live = true;
  std::string screenshot_path;
  int layout_width = 0, layout_height = 0;
  bool fit_layout = true;
  float left_width = 290, right_width = 335;
  bool layout_verified = true;
  int resize_checks = 0;
  bool replay_keys_verified = true;
  ImVec2 edit_probe{};
  ImVec2 hover_probe{};
  bool cursor_units_verified = false;
  ImPlotRect graph_limits[3];
  double zoom_initial_width = 0;
  ImVec2 panel_drag_probe{};
  ImVec2 library_toggle_probe{}, library_row_probe{};
  void request_open(const std::filesystem::path& file) {
    if (save_task.valid() || load_task.valid()) return;
    if (recording || frame_count) {
      message = "현재 기록을 종료한 후 파일을 열어 주세요.";
      return;
    }
    const auto filename = file.string();
    if (filename.size() >= sizeof(path)) {
      message = "파일 경로가 너무 깁니다. 더 짧은 경로를 사용해 주세요.";
      return;
    }
    runner.set_running(false);
    running = false;
    playing = false;
    std::snprintf(path, sizeof(path), "%s", filename.c_str());
    loading_recording = filename;
    load_task = std::async(std::launch::async, [this, filename] {
      return rclab::Episode::load(filename, sim(), sim().profile().record_bytes);
    });
  }
  void recording_library() {
    if (catalog_task.valid() &&
        catalog_task.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        auto files = catalog_task.get();
        if (scanned_folder == catalog_folder) {
          recording_files = std::move(files);
          catalog_error.clear();
        } else
          refresh_catalog = true;
      } catch (const std::exception& e) {
        recording_files.clear();
        catalog_error = "목록을 읽지 못했습니다. 폴더 경로와 접근 권한을 확인해 주세요.";
        error_detail = e.what();
      }
    }
    const bool expanded = ImGui::CollapsingHeader("저장된 기록");
    const auto toggle_min = ImGui::GetItemRectMin(), toggle_max = ImGui::GetItemRectMax();
    library_toggle_probe = {toggle_min.x + 70, (toggle_min.y + toggle_max.y) * .5f};
    if (!expanded) return;
    ImGui::SetNextItemWidth(std::max(80.f, ImGui::GetContentRegionAvail().x - 120));
    bool changed = ImGui::InputText("폴더", catalog_directory, sizeof(catalog_directory),
                                    ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::SameLine();
    changed |= ImGui::Button("새로고침");
    if (changed) {
      catalog_folder = catalog_directory;
      recording_files.clear();
      refresh_catalog = true;
    }
    if (!catalog_task.valid() && (refresh_catalog || Clock::now() >= catalog_next_scan)) {
      scanned_folder = catalog_folder;
      catalog_task = std::async(
          std::launch::async, [folder = catalog_folder] { return rclab::list_recordings(folder); });
      refresh_catalog = false;
      catalog_next_scan = Clock::now() + std::chrono::seconds(2);
    }
    if (!catalog_error.empty())
      ImGui::TextWrapped("%s", catalog_error.c_str());
    else if (recording_files.empty())
      ImGui::TextUnformatted(catalog_task.valid() ? "기록을 찾는 중입니다…"
                                                  : "저장된 기록이 없습니다.");
    if (recording || frame_count)
      ImGui::TextWrapped("현재 기록을 종료하면 저장된 기록을 열 수 있습니다.");
    if (!recording_files.empty()) {
      const float row_height = ImGui::GetTextLineHeightWithSpacing() * 2;
      ImGui::BeginChild("record_list",
                        {0, std::min(180.f, recording_files.size() * row_height + 16)}, true);
      ImGui::BeginDisabled(save_task.valid() || load_task.valid() || recording || frame_count);
      ImGuiListClipper clipper;
      clipper.Begin(static_cast<int>(recording_files.size()), row_height);
      while (clipper.Step())
        for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; ++row) {
          const auto& file = recording_files[row];
          const auto filename = file.path.string();
          ImGui::PushID(filename.c_str());
          ImGui::BeginDisabled(!file.has_metadata);
          if (ImGui::Selectable(file.path.filename().string().c_str(),
                                filename == opened_recording))
            request_open(file.path);
          if (row == 0) {
            const auto min = ImGui::GetItemRectMin(), max = ImGui::GetItemRectMax();
            library_row_probe = {(min.x + max.x) * .5f, (min.y + max.y) * .5f};
          }
          ImGui::EndDisabled();
          ImGui::TextDisabled(file.has_metadata ? "%.2f MiB" : "%.2f MiB · 기록 정보 파일 없음",
                              file.bytes / 1048576.);
          ImGui::PopID();
        }
      ImGui::EndDisabled();
      ImGui::EndChild();
    }
  }
  void check_layout(float x, float y, float width, float height) {
    if (!selftest) return;
    const auto pos = ImGui::GetWindowPos(), size = ImGui::GetWindowSize();
    layout_verified &= std::abs(pos.x - x) <= 1 && std::abs(pos.y - y) <= 1 &&
                       std::abs(size.x - width) <= 1 && std::abs(size.y - height) <= 1;
    if (fit_layout) ++resize_checks;
  }
  explicit Gui(rclab::RobotProfile p)
      : runner(std::move(p)),
        live(mj_makeData(runner.simulation().model())),
        controls(runner.simulation().controls()),
        initial_controls(controls),
        channels(runner.simulation().channels()) {
    // 메뉴 표기만 바꾼다. 기존 기록의 채널 식별자와 호환성은 유지한다.
    for (auto& channel : channels) {
      channel.name = channel_label(channel);
      if (channel.unit == "0=STAND 1=ACCEL 2=RUN 3=DECEL")
        channel.unit = "0정지 1가속 2달리기 3감속";
      if (channel.unit == "phase or clip index") channel.unit = "진행도";
      if (channel.unit == "actuator input") channel.unit = "제어 입력";
    }
    preview.state_size = 1;
    preview.channel_count = channels.size();
    preview.stride = 1 + channels.size();
    telemetry.resize(preview.stride);
    preview.rows.reserve(preview.stride * 2048);
    mjv_defaultCamera(&camera);
    mjv_defaultOption(&options);
    mjv_defaultPerturb(&perturb);
    mjv_defaultScene(&scene);
    mjr_defaultContext(&render);
    height = runner.simulation().profile().suspend_height;
    hold = runner.simulation().suspended();
    const auto& q = runner.simulation().profile().suspend_quat;
    euler[0] = std::atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])) *
               rad_to_deg;
    euler[1] = std::asin(std::clamp(2 * (q[0] * q[2] - q[3] * q[1]), -1., 1.)) * rad_to_deg;
    euler[2] = std::atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3])) *
               rad_to_deg;
    // 투사체나 지면이 포함된 전체 장면 대신 로봇 하위 트리의 크기로 카메라를 맞춘다.
    double radius = .5;
    const auto* m = sim().model();
    const auto* d = sim().data();
    for (int g = 0; g < m->ngeom; ++g) {
      int b = m->geom_bodyid[g];
      while (b && b != sim().base_body()) b = m->body_parentid[b];
      if (b == sim().base_body()) {
        mjtNum offset[3];
        mju_sub3(offset, d->geom_xpos + 3 * g, d->xpos + 3 * sim().base_body());
        radius = std::max(radius, mju_norm3(offset) + m->geom_rbound[g]);
      }
    }
    camera.distance = initial_camera_distance = std::max(1.5, radius * 2.7);
    camera.azimuth = 135;
    camera.elevation = -20;
  }
  ~Gui() {
    runner.stop();
    if (window) {
      glfwMakeContextCurrent(window);
      mjr_freeContext(&render);
      mjv_freeScene(&scene);
      ImPlot::DestroyContext();
      ImGui_ImplOpenGL3_Shutdown();
      ImGui_ImplGlfw_Shutdown();
      ImGui::DestroyContext();
      glfwDestroyWindow(window);
    }
    glfwTerminate();
  }
  const rclab::Simulation& sim() const { return runner.simulation(); }
  mjData* shown() const { return replay ? replay->data() : live.get(); }
  void open_episode(rclab::Episode ep) {
    if (!ep.frames()) {
      message = "아직 기록된 데이터가 없습니다.";
      return;
    }
    runner.set_running(false);
    running = false;
    playing = false;
    perturb.active = 0;
    runner.set_perturbation(perturb);
    replay = std::make_unique<rclab::ReplaySession>(sim(), std::move(ep));
    const auto& episode = replay->episode();
    x_min = episode.time(0);
    x_max = episode.time(episode.frames() - 1) + sim().model()->opt.timestep;
    requested_time = x_min;
    opened_recording.clear();
  }
  void inspect_recording() { open_episode(runner.finish_recording()); }
  void seek(double t) {
    if (!replay) return;
    requested_time = t;
    playing = false;
    replay->seek(t);
  }
  void select_relative(int delta) {
    if (!replay) return;
    long long value = static_cast<long long>(replay->selected()) + delta;
    value = std::clamp(value, 0LL, static_cast<long long>(replay->episode().frames() - 1));
    replay->select(value);
    requested_time = replay->data()->time;
    playing = false;
  }
  void toggle_replay() {
    if (!replay) return;
    if (!playing && replay->selected() + 1 == replay->episode().frames()) {
      replay->select(0);
      requested_time = replay->data()->time;
    }
    playing = !playing;
  }
  void replay_shortcuts() {
    if (!replay || ImGui::GetIO().WantTextInput || ImGui::IsAnyItemActive() ||
        ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId | ImGuiPopupFlags_AnyPopupLevel))
      return;
    if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow)) select_relative(-1);
    if (ImGui::IsKeyPressed(ImGuiKey_RightArrow)) select_relative(1);
    if (ImGui::IsKeyPressed(ImGuiKey_Space, false)) toggle_replay();
  }
  void peak(int direction) {
    if (!replay) return;
    const auto& ep = replay->episode();
    size_t channel = selected_channels[0], current = replay->selected();
    if (direction == 0) {
      size_t best = current;
      double maximum = -1;
      for (size_t i = 0; i < ep.frames(); ++i)
        if (ep.time(i) >= x_min && ep.time(i) <= x_max &&
            std::abs(ep.value(i, channel)) > maximum) {
          best = i;
          maximum = std::abs(ep.value(i, channel));
        }
      seek(ep.time(best));
      return;
    }
    for (long long i = static_cast<long long>(current) + direction;
         i > 0 && i < static_cast<long long>(ep.frames()) - 1; i += direction) {
      double v = std::abs(ep.value(i, channel));
      if (v > std::abs(ep.value(i - 1, channel)) && v >= std::abs(ep.value(i + 1, channel))) {
        seek(ep.time(i));
        return;
      }
    }
  }
  void controls_panel(int w, int h) {
    ImGui::SetNextWindowPos({0, 0}, ImGuiCond_Always);
    const float panel_height = sim().profile().manual_joint_control ? h * .5f : h;
    ImGui::SetNextWindowSize({left_width, panel_height}, ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(.6f);
    ImGui::Begin("시뮬레이션", nullptr, fixed_panel);
    check_layout(0, 0, left_width, panel_height);
    ImGui::TextUnformatted(sim().profile().name.c_str());
    ImGui::Text("시간 %.6f s", shown()->time);
    if (replay) {
      ImGui::TextColored({.3f, .8f, 1, 1}, "기록 재생 중");
      if (ImGui::Button("실시간 화면으로 돌아가기")) {
        replay.reset();
        playing = false;
        follow_live = true;
      }
      ImGui::TextWrapped("그래프는 기록된 힘, 3D 화살표는 재계산한 힘입니다.");
    } else {
      if (ImGui::Button(running ? "일시정지" : "실행")) {
        running = !running;
        runner.set_running(running);
      }
      ImGui::SameLine();
      if (ImGui::Button("한 스텝")) {
        running = false;
        runner.single_step();
      }
      ImGui::SameLine();
      if (ImGui::Button("초기화")) {
        running = false;
        runner.set_running(false);
        runner.reset();
        controls = initial_controls;
        preview.rows.clear();
        follow_live = true;
        hold = sim().profile().start_suspended;
        height = sim().profile().suspend_height;
      }
      if (ImGui::SliderFloat("실행 배속", &control_speed, .05f, 4.f, "%.2fx")) {
        speed = control_speed;
        runner.set_speed(speed);
      }
      ImGui::Separator();
      if (recording || frame_count) {
        if (ImGui::Button("기록 종료 후 분석")) inspect_recording();
      } else if (ImGui::Button("기록 시작"))
        runner.start_recording();
      ImGui::Text("기록 %zu개 · 용량 한도 %.1f MiB", frame_count,
                  sim().profile().record_bytes / 1048576.);
      const char* status = record_status == "Recording"           ? "기록 중"
                           : record_status == "Recording stopped" ? "기록 종료"
                           : record_status.empty()
                               ? ""
                               : "시간 또는 용량 한도에 도달해 기록을 종료했습니다.";
      ImGui::TextWrapped("%s", status);
      if (ImGui::CollapsingHeader("공중 고정")) {
        bool changed = ImGui::Checkbox("몸통 고정", &hold);
        changed |= ImGui::DragFloat("높이 (m)", &height, .005f, -10, 100);
        changed |= ImGui::DragFloat3("기울기·회전 (deg)", euler, .5f, -180, 180);
        if (changed) {
          const double r = euler[0] / rad_to_deg / 2, p = euler[1] / rad_to_deg / 2,
                       y = euler[2] / rad_to_deg / 2;
          std::array<double, 4> quat{
              std::cos(r) * std::cos(p) * std::cos(y) + std::sin(r) * std::sin(p) * std::sin(y),
              std::sin(r) * std::cos(p) * std::cos(y) - std::cos(r) * std::sin(p) * std::sin(y),
              std::cos(r) * std::sin(p) * std::cos(y) + std::sin(r) * std::cos(p) * std::sin(y),
              std::cos(r) * std::cos(p) * std::sin(y) - std::sin(r) * std::sin(p) * std::cos(y)};
          runner.set_suspend(hold, height, quat);
        }
      }
      if (ImGui::CollapsingHeader("로봇 명령", ImGuiTreeNodeFlags_DefaultOpen)) {
        bool changed = false;
        if (sim().profile().command_mode == "run_stop") {
          bool requested = controls.command[0] >= .005;
          changed = ImGui::Checkbox("달리기", &requested);
          controls.command = {requested ? 1. : 0., 0., 0.};
          ImGui::TextWrapped("실행 후 선택하면 달립니다. 해제하면 감속 후 정지합니다.");
        } else {
          for (int k = 0; k < 3; ++k)
            changed |= ImGui::InputDouble(k == 0   ? "전후 속도 (m/s)"
                                          : k == 1 ? "좌우 속도 (m/s)"
                                                   : "회전 속도 (rad/s)",
                                          &controls.command[k], .05, .1, "%.3f");
          ImGui::TextWrapped("속도 명령은 이를 지원하는 제어기에서만 사용합니다.");
        }
        if (changed) runner.update_controls(controls);
      }
    }
    ImGui::Separator();
    ImGui::Checkbox("로봇 따라가기", &follow);
    if (ImGui::Button("시점 초기화")) {
      camera.azimuth = 135;
      camera.elevation = -20;
      camera.distance = initial_camera_distance;
    }
    if (ImGui::CollapsingHeader("화면 표시")) {
      auto flag = [&](const char* name, int index) {
        bool v = options.flags[index];
        if (ImGui::Checkbox(name, &v)) options.flags[index] = v;
      };
      flag("관절축", mjVIS_JOINT);
      flag("접촉점", mjVIS_CONTACTPOINT);
      flag("접촉력", mjVIS_CONTACTFORCE);
      flag("무게중심", mjVIS_COM);
      flag("반투명", mjVIS_TRANSPARENT);
      int frame = options.frame;
      const char* frames[] = {"없음", "링크", "형상", "기준점", "카메라", "조명", "접촉", "세계"};
      if (ImGui::Combo("좌표계", &frame, frames, 8)) options.frame = frame;
      for (int g = 0; g < 6; ++g) {
        ImGui::PushID(g);
        bool visible = options.geomgroup[g];
        if (ImGui::Checkbox("형상 그룹", &visible)) options.geomgroup[g] = visible;
        ImGui::SameLine();
        ImGui::Text("%d", g);
        ImGui::PopID();
      }
    }
    if (ImGui::CollapsingHeader("고급 정보")) {
      ImGui::Text("계산 시간: 평균 %.3f / 최대 %.3f ms", timing.mean_ms, timing.max_ms);
      ImGui::Text("주기 초과 %llu회 / 전체 %llu스텝",
                  static_cast<unsigned long long>(timing.overruns),
                  static_cast<unsigned long long>(timing.steps));
      ImGui::Text("실시간 배율 %.3f", timing.realtime_factor);
      ImGui::Text(
          "위치 좌표 %lld · 자유도 %lld · 모터 %lld", static_cast<long long>(sim().model()->nq),
          static_cast<long long>(sim().model()->nv), static_cast<long long>(sim().model()->nu));
      if (!sim().warnings().empty() && ImGui::TreeNode("모델 진단 상세")) {
        for (const auto& warning : sim().warnings()) ImGui::TextWrapped("%s", warning.c_str());
        ImGui::TreePop();
      }
      if (!error_detail.empty() && ImGui::TreeNode("오류 상세")) {
        ImGui::TextWrapped("%s", error_detail.c_str());
        ImGui::TreePop();
      }
    }
    ImGui::End();
    (void)w;
  }
  void joints_panel(int w, int h) {
    ImGui::SetNextWindowPos({0, h * .5f}, ImGuiCond_Always);
    ImGui::SetNextWindowSize({left_width, h * .5f}, ImGuiCond_Always);
    ImGui::Begin("관절 조작", nullptr, fixed_panel);
    check_layout(0, h * .5f, left_width, h * .5f);
    ImGui::InputText("검색", search, sizeof(search));
    ImGui::BeginDisabled(static_cast<bool>(replay) || !sim().profile().manual_joint_control);
    if (ImGui::Button("전체 활성화")) {
      for (size_t i = 0; i < sim().joints().size(); ++i)
        controls.enabled[i] = sim().joints()[i].controllable;
      runner.update_controls(controls);
    }
    ImGui::SameLine();
    if (ImGui::Button("전체 비활성화")) {
      std::fill(controls.enabled.begin(), controls.enabled.end(), 0);
      runner.update_controls(controls);
    }
    ImGui::EndDisabled();
    std::set<std::string> groups;
    for (const auto& j : sim().joints()) groups.insert(j.group);
    const std::map<std::string, std::string> group_names{
        {"Left leg", "왼쪽 다리"}, {"Right leg", "오른쪽 다리"},
        {"Left arm", "왼팔"},      {"Right arm", "오른팔"},
        {"Torso", "몸통"},         {"Other / passive", "기타 관절"}};
    for (const auto& group : groups)
      if (ImGui::CollapsingHeader(
              (group_names.count(group) ? group_names.at(group) : group).c_str(),
              ImGuiTreeNodeFlags_DefaultOpen)) {
        for (size_t i = 0; i < sim().joints().size(); ++i) {
          const auto& joint = sim().joints()[i];
          if (joint.group != group ||
              (!std::string(search).empty() && joint.name.find(search) == std::string::npos))
            continue;
          ImGui::PushID(static_cast<int>(i));
          if (ImGui::TreeNodeEx(joint.name.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
            if (joint.scalar) {
              bool angle = joint.type == mjJNT_HINGE;
              double scale = angle ? rad_to_deg : 1;
              ImGui::Text("현재 %.3f %s · 속도 %.3f %s", shown()->qpos[joint.qpos] * scale,
                          angle ? "deg" : "m", shown()->qvel[joint.dof], angle ? "rad/s" : "m/s");
              if (joint.controllable && joint.joint_target_available &&
                  sim().profile().manual_joint_control) {
                ImGui::BeginDisabled(static_cast<bool>(replay));
                bool enabled = controls.enabled[i];
                bool changed = ImGui::Checkbox("제어", &enabled);
                controls.enabled[i] = enabled;
                float target = controls.target[i] * scale;
                if (replay)
                  target = replay->episode().frame(
                               replay->selected())[replay->episode().state_size +
                                                   replay->episode().channel_count + i] *
                           scale;
                ImGui::SetNextItemWidth(std::max(80.f, ImGui::GetContentRegionAvail().x - 105));
                if (ImGui::SliderFloat(angle ? "목표 (deg)" : "목표 (m)", &target,
                                       joint.lower * scale, joint.upper * scale, "%.3f")) {
                  controls.target[i] = target / scale;
                  changed = true;
                }
                if (!replay) {
                  changed |= ImGui::InputDouble("Kp", &controls.kp[i], 1, 10, "%.3f");
                  changed |= ImGui::InputDouble("Kd", &controls.kd[i], .1, 1, "%.3f");
                }
                controls.kp[i] = std::max(0., controls.kp[i]);
                controls.kd[i] = std::max(0., controls.kd[i]);
                if (changed) runner.update_controls(controls);
                ImGui::Text("전달비 %.3g · 입력 %.3g", joint.gear, shown()->ctrl[joint.actuator]);
                ImGui::EndDisabled();
              } else
                ImGui::TextWrapped("이 관절은 수동 제어를 지원하지 않습니다.");
            } else
              ImGui::TextUnformatted("자유·구면 관절: 수동 조작 불가");
            ImGui::TreePop();
          }
          ImGui::PopID();
        }
      }
    ImGui::End();
  }
  void graph_panel(int w, int h) {
    ImGui::SetNextWindowPos({w - right_width, 0}, ImGuiCond_Always);
    ImGui::SetNextWindowSize({right_width, static_cast<float>(h)}, ImGuiCond_Always);
    ImGui::Begin("그래프 · 3D 재생", nullptr, fixed_panel);
    const auto graph_pos = ImGui::GetWindowPos();
    panel_drag_probe = {graph_pos.x + 100, graph_pos.y + 10};
    check_layout(w - right_width, 0, right_width, h);
    ImGui::SetNextItemWidth(std::max(80.f, ImGui::GetContentRegionAvail().x - 90));
    ImGui::InputText("##record_file", path, sizeof(path));
    const auto edit_min = ImGui::GetItemRectMin(), edit_max = ImGui::GetItemRectMax();
    edit_probe = {edit_min.x + 15, (edit_min.y + edit_max.y) * .5f};
    ImGui::SameLine();
    ImGui::BeginDisabled(save_task.valid() || load_task.valid());
    if (ImGui::Button("열기")) request_open(std::string(path));
    ImGui::SameLine();
    ImGui::BeginDisabled(!replay);
    if (ImGui::Button("저장")) {
      std::string filename = path;
      // 독립 복사본을 백그라운드에서 저장하므로 물리 worker가 파일 I/O를 기다리지 않는다.
      auto copy = replay->episode();
      save_task = std::async(std::launch::async, [copy = std::move(copy), filename] {
        copy.save(filename);
        return "저장했습니다: " + filename;
      });
    }
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    if (save_task.valid() &&
        save_task.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        message = save_task.get();
        refresh_catalog = true;
      } catch (const std::exception& e) {
        message = "기록을 저장하지 못했습니다. 파일명과 저장 위치를 확인해 주세요.";
        error_detail = e.what();
      }
    }
    if (load_task.valid() &&
        load_task.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        open_episode(load_task.get());
        opened_recording = loading_recording;
        message = "기록을 불러왔습니다.";
        error_detail.clear();
      } catch (const std::exception& e) {
        error_detail = e.what();
        message = error_detail.rfind("기록", 0) == 0
                      ? error_detail
                      : "기록을 열지 못했습니다. 파일 상태와 로봇·정책 설정을 확인해 주세요.";
      }
    }
    recording_library();
    if (!message.empty()) {
      ImGui::TextWrapped("%s", message.c_str());
      if (ImGui::SmallButton("알림 닫기")) message.clear();
    }
    if (!replay) {
      ImGui::TextWrapped(
          "실시간 미리보기입니다. 기록 중 그래프를 클릭하면 해당 시점의 3D 자세를 확인합니다.");
      if (ImGui::Button("최신 구간 보기")) follow_live = true;
      if (preview.frames() && follow_live) {
        x_min = preview.time(0);
        x_max = preview.time(preview.frames() - 1) + sim().model()->opt.timestep;
      }
      double seek_time = 0;
      bool clicked = false;
      if (preview.frames() > 1) {
        const float plot_height = std::max(70.f, ImGui::GetContentRegionAvail().y / 3 - 34);
        for (int plot = 0; plot < 3; ++plot) {
          ImGui::PushID(plot);
          ImGui::SetNextItemWidth(std::max(100.f, ImGui::GetContentRegionAvail().x - 65));
          if (ImGui::BeginCombo("항목", channels[selected_channels[plot]].name.c_str())) {
            for (size_t k = 0; k < channels.size(); ++k)
              if (ImGui::Selectable(channels[k].name.c_str(),
                                    selected_channels[plot] == static_cast<int>(k)))
                selected_channels[plot] = k;
            ImGui::EndCombo();
          }
          if (ImPlot::BeginPlot("##live", {ImGui::GetContentRegionAvail().x, plot_height},
                                ImPlotFlags_NoMenus | ImPlotFlags_NoMouseText)) {
            ImPlot::SetupAxes("시간 (s)", channels[selected_channels[plot]].unit.c_str(),
                              ImPlotAxisFlags_None,
                              ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit);
            ImPlot::SetupAxisLinks(ImAxis_X1, &x_min, &x_max);
            PlotAccess access{&preview, static_cast<size_t>(selected_channels[plot])};
            ImPlot::PlotLineG("실시간", point_at, &access, static_cast<int>(preview.frames()));
            if ((ImPlot::IsPlotHovered() || ImPlot::IsAxisHovered(ImAxis_X1)) &&
                (ImGui::GetIO().MouseWheel != 0 ||
                 ImGui::IsMouseDragging(ImGuiMouseButton_Middle) ||
                 ImGui::IsMouseDragging(ImGuiMouseButton_Right)))
              follow_live = false;
            if (frame_count && ImPlot::IsPlotHovered() && ImGui::IsMouseReleased(0) &&
                ImGui::GetMouseDragDelta(0).x * ImGui::GetMouseDragDelta(0).x +
                        ImGui::GetMouseDragDelta(0).y * ImGui::GetMouseDragDelta(0).y <
                    16) {
              seek_time = ImPlot::GetPlotMousePos().x;
              clicked = true;
            }
            show_cursor_coordinates(channels[selected_channels[plot]].unit);
            ImPlot::EndPlot();
          }
          ImGui::PopID();
        }
      }
      if (clicked) {
        inspect_recording();
        seek(seek_time);
      }
      ImGui::End();
      return;
    }
    const auto& ep = replay->episode();
    if (ImGui::Button(playing ? "정지 [Space]" : "재생 [Space]")) toggle_replay();
    ImGui::SameLine();
    if (ImGui::Button("이전 [←]")) select_relative(-1);
    ImGui::SameLine();
    if (ImGui::Button("다음 [→]")) select_relative(1);
    if (ImGui::Button("이전 피크")) peak(-1);
    ImGui::SameLine();
    if (ImGui::Button("다음 피크")) peak(1);
    ImGui::SameLine();
    if (ImGui::Button("최댓값")) peak(0);
    ImGui::SameLine();
    if (ImGui::Button("전체 범위")) {
      x_min = ep.time(0);
      x_max = ep.time(ep.frames() - 1) + sim().model()->opt.timestep;
    }
    ImGui::SetNextItemWidth(70);
    ImGui::InputDouble("재생 배속", &playback_speed, .1, .5, "%.2f");
    playback_speed = std::clamp(playback_speed, .05, 4.);
    double cursor = replay->data()->time, minimum = ep.time(0), maximum = ep.time(ep.frames() - 1);
    ImGui::SetNextItemWidth(std::max(80.f, ImGui::GetContentRegionAvail().x - 45));
    if (ImGui::SliderScalar("시간", ImGuiDataType_Double, &cursor, &minimum, &maximum, "%.6f s"))
      seek(cursor);
    ImGui::TextWrapped("선택 %.6f s · 기록 %.6f s · 차이 %.3f ms", requested_time,
                       replay->data()->time, 1000 * (replay->data()->time - requested_time));
    float graph_height = std::max(140.f, ImGui::GetContentRegionAvail().y / 3);
    const float plot_controls_height =
        ImGui::GetFrameHeightWithSpacing() + ImGui::GetTextLineHeightWithSpacing() + 8;
    for (int plot = 0; plot < 3; ++plot) {
      ImGui::PushID(plot);
      ImGui::SetNextItemWidth(std::max(100.f, ImGui::GetContentRegionAvail().x - 65));
      if (ImGui::BeginCombo("항목", channels[selected_channels[plot]].name.c_str())) {
        for (size_t k = 0; k < channels.size(); ++k)
          if (ImGui::Selectable(channels[k].name.c_str(),
                                selected_channels[plot] == static_cast<int>(k))) {
            selected_channels[plot] = k;
          }
        ImGui::EndCombo();
      }
      if (ImPlot::BeginPlot(
              "##signal",
              {ImGui::GetContentRegionAvail().x,
               std::max(60.f, graph_height - plot_controls_height)},
              ImPlotFlags_Crosshairs | ImPlotFlags_NoMenus | ImPlotFlags_NoMouseText)) {
        // AutoFit은 Y축 직접 조작을 막고, RangeFit은 보이는 X구간의 데이터만 맞춘다.
        ImPlot::SetupAxes("시간 (s)", channels[selected_channels[plot]].unit.c_str(),
                          ImPlotAxisFlags_None, ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit);
        ImPlot::SetupAxisLinks(ImAxis_X1, &x_min, &x_max);
        PlotAccess access{&ep, static_cast<size_t>(selected_channels[plot])};
        ImPlot::PlotLineG("기록", point_at, &access, static_cast<int>(ep.frames()));
        double t = replay->data()->time;
        ImPlot::PlotInfLines("선택 시각", &t, 1);
        if (ImPlot::IsPlotHovered() && ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
            ImGui::GetMouseDragDelta(ImGuiMouseButton_Left).x *
                        ImGui::GetMouseDragDelta(ImGuiMouseButton_Left).x +
                    ImGui::GetMouseDragDelta(ImGuiMouseButton_Left).y *
                        ImGui::GetMouseDragDelta(ImGuiMouseButton_Left).y <
                16) {
          graph_clicked_time = ImPlot::GetPlotMousePos().x;
          seek(graph_clicked_time);
          graph_click_observed = true;
        }
        if (plot == 0) {
          double probe_time = ep.time(ep.frames() / 2);
          graph_probe =
              ImPlot::PlotToPixels(probe_time, ep.value(ep.frames() / 2, selected_channels[plot]));
          graph_probe.x = std::floor(graph_probe.x);
          graph_probe.y = std::floor(graph_probe.y);
          graph_probe_time = ImPlot::PixelsToPlot(graph_probe).x;
        }
        if (plot == 2) {
          const auto pos = ImPlot::GetPlotPos(), size = ImPlot::GetPlotSize();
          hover_probe = {pos.x + size.x * .55f, pos.y + size.y * .4f};
        }
        if (show_cursor_coordinates(channels[selected_channels[plot]].unit) && plot == 2)
          cursor_units_verified = true;
        graph_limits[plot] = ImPlot::GetPlotLimits();
        ImPlot::EndPlot();
      }
      ImGui::Text("값 %.6g %s", ep.value(replay->selected(), selected_channels[plot]),
                  channels[selected_channels[plot]].unit.c_str());
      ImGui::PopID();
    }
    ImGui::End();
  }
  void view_panel(int w, int h) {
    ImGui::SetNextWindowPos({left_width, 0}, ImGuiCond_Always);
    ImGui::SetNextWindowSize({w - left_width - right_width, static_cast<float>(h)},
                             ImGuiCond_Always);
    ImGui::Begin("3D 화면", nullptr, fixed_panel | ImGuiWindowFlags_NoBackground);
    check_layout(left_width, 0, w - left_width - right_width, h);
    ImGui::TextUnformatted("왼쪽 드래그: 회전 · 오른쪽: 이동 · 휠: 확대/축소");
    if (replay) ImGui::TextUnformatted("← / → 한 스텝 이동 · Space 재생/정지");
    view_min = ImGui::GetCursorScreenPos();
    view_size = ImGui::GetContentRegionAvail();
    view_size.x = std::max(1.f, view_size.x);
    view_size.y = std::max(1.f, view_size.y);
    // 왼쪽 패널 뒤에도 장면을 그려 반투명 배경 너머로 공간이 이어지게 한다.
    render_min = {0, view_min.y};
    render_size = {view_min.x + view_size.x, view_size.y};
    view_probe = {view_min.x + view_size.x * .5f, view_min.y + view_size.y * .5f};
    ImGui::InvisibleButton("viewport", view_size,
                           ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight |
                               ImGuiButtonFlags_MouseButtonMiddle);
    bool hovered = ImGui::IsItemHovered(), active = ImGui::IsItemActive();
    const auto& io = ImGui::GetIO();
    if (hovered && ImGui::IsMouseDoubleClicked(0)) {
      mjtNum hit[3];
      int geom = -1, flex = -1, skin = -1;
      int body = mjv_select(sim().model(), shown(), &options, render_size.x / render_size.y,
                            (io.MousePos.x - render_min.x) / render_size.x,
                            1 - (io.MousePos.y - render_min.y) / render_size.y, &scene, hit, &geom,
                            &flex, &skin);
      if (body > 0) {
        perturb.select = body;
        perturb.skinselect = skin;
        mjtNum offset[3];
        mju_sub3(offset, hit, shown()->xpos + 3 * body);
        mju_mulMatTVec(perturb.localpos, shown()->xmat + 9 * body, offset, 3, 3);
        mjv_initPerturb(sim().model(), shown(), &scene, &perturb);
      }
    }
    bool drag = active && (ImGui::IsMouseDragging(0) || ImGui::IsMouseDragging(1) ||
                           ImGui::IsMouseDragging(2));
    if (drag) {
      int action = ImGui::IsMouseDown(1) ? (io.KeyShift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V)
                                         : (io.KeyShift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V);
      if (ImGui::IsMouseDown(2)) action = mjMOUSE_ZOOM;
      if (io.KeyCtrl && perturb.select && !replay) {
        if (!perturb.active) {
          mjv_initPerturb(sim().model(), shown(), &scene, &perturb);
          perturb.active = ImGui::IsMouseDown(1) ? mjPERT_ROTATE : mjPERT_TRANSLATE;
        }
        mjv_movePerturb(sim().model(), shown(), action, io.MouseDelta.x / view_size.y,
                        io.MouseDelta.y / view_size.y, &scene, &perturb);
      } else
        mjv_moveCamera(sim().model(), action, io.MouseDelta.x / view_size.y,
                       io.MouseDelta.y / view_size.y, &camera);
    } else
      perturb.active = 0;
    if (hovered && io.MouseWheel)
      mjv_moveCamera(sim().model(), mjMOUSE_ZOOM, 0, -.05 * io.MouseWheel, &camera);
    runner.set_perturbation(perturb);
    ImGui::End();
  }
  void screenshot(const std::string& filename, int w, int h) {
    std::vector<unsigned char> pixels(static_cast<size_t>(w) * h * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_BACK);
    glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
    std::ofstream out(filename, std::ios::binary);
    out << "P6\n" << w << ' ' << h << "\n255\n";
    for (int y = h - 1; y >= 0; --y)
      out.write(reinterpret_cast<char*>(pixels.data() + static_cast<size_t>(y) * w * 3), w * 3);
  }
  int run(double benchmark_seconds) {
    glfwSetErrorCallback(
        [](int code, const char* error) { std::cerr << "GLFW " << code << ": " << error << '\n'; });
    if (!glfwInit()) throw std::runtime_error("GLFW initialization failed (requires DISPLAY)");
    // MuJoCo의 고정 기능 렌더러와 ImGui가 함께 쓰는 호환 프로필을 요청한다.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
    // 자동 검사에는 실제 데스크톱의 마우스 이벤트가 섞이지 않도록 숨김 창을 사용한다.
    if (selftest || benchmark_seconds > 0) glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    // 숨김 GLX 창의 drawable 크기 제한을 피하도록 최대 검사 크기로 생성한다.
    window = glfwCreateWindow(selftest ? 1920 : 1500, selftest ? 1080 : 950,
                              "RCLab | robot simulator", nullptr, nullptr);
    if (!window) throw std::runtime_error("Cannot create GLFW window");
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    load_korean_font();
    ImPlot::CreateContext();
    ImGui::StyleColorsDark();
    if (selftest || benchmark_seconds > 0) ImGui::GetIO().IniFilename = nullptr;
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    ImPlot::GetInputMap().Pan = ImGuiMouseButton_Middle;
    ImPlot::GetStyle().FitPadding.y = .08f;
    mjv_makeScene(sim().model(), &scene, 10000);
    mjr_makeContext(sim().model(), &render, mjFONTSCALE_150);
    runner.start();
    if (selftest || benchmark_seconds > 0) {
      runner.start_recording();
      runner.set_running(true);
      running = true;
    }
    auto started = Clock::now(), previous = started;
    int phase = 0, phase_frame = 0;
    double camera_before = 0, selected_before = 0;
    size_t gui_frames = 0;
    nlohmann::json gui_checks;
    bool gui_checks_passed = false;
    std::filesystem::path library_fixture;
    int library_click_state = 0, library_loaded_frames = 0;
    while (!glfwWindowShouldClose(window)) {
      if (selftest && gui_frames == 5) glfwSetWindowSize(window, 1500, 950);
      if (selftest && gui_frames == 25) glfwSetWindowSize(window, 1920, 1080);
      if (selftest && gui_frames == 15) glfwSetWindowSize(window, 1280, 800);
      glfwPollEvents();
      auto now = Clock::now();
      double dt = std::chrono::duration<double>(now - previous).count();
      previous = now;
      double elapsed = std::chrono::duration<double>(now - started).count();
      runner.snapshot(live.get(), timing, frame_count, recording, record_status, &telemetry);
      mj_forward(sim().model(), live.get());
      if (preview.frames() && telemetry[0] < preview.time(preview.frames() - 1))
        preview.rows.clear();
      if (!preview.frames() || telemetry[0] > preview.time(preview.frames() - 1)) {
        if (preview.frames() >= 2048)
          preview.rows.erase(preview.rows.begin(),
                             preview.rows.begin() + static_cast<std::ptrdiff_t>(preview.stride));
        preview.rows.insert(preview.rows.end(), telemetry.begin(), telemetry.end());
      }
      if (replay && playing) {
        requested_time += dt * playback_speed;
        const auto& ep = replay->episode();
        if (requested_time >= ep.time(ep.frames() - 1)) {
          requested_time = ep.time(ep.frames() - 1);
          playing = false;
        }
        replay->seek(requested_time);
      }
      if ((selftest && elapsed > 1 && phase == 0) ||
          (benchmark_seconds > 0 && elapsed > benchmark_seconds && phase == 0)) {
        inspect_recording();
        phase = 1;
        phase_frame = 0;
      }
      // 실제 ImGui 입력 경로로 그래프 클릭과 3D 카메라 드래그를 검증한다.
      ImGui_ImplOpenGL3_NewFrame();
      ImGui_ImplGlfw_NewFrame();
      auto& io = ImGui::GetIO();
      if ((selftest || benchmark_seconds > 0) && phase >= 1) {
        ++phase_frame;
        if (selftest) {
          if (phase_frame == 1) io.AddKeyEvent(ImGuiKey_RightArrow, true);
          if (phase_frame == 2) {
            io.AddKeyEvent(ImGuiKey_RightArrow, false);
            replay_keys_verified &= replay->selected() == 1 && !playing;
          }
          if (phase_frame == 3) io.AddKeyEvent(ImGuiKey_LeftArrow, true);
          if (phase_frame == 4) {
            io.AddKeyEvent(ImGuiKey_LeftArrow, false);
            replay_keys_verified &= replay->selected() == 0 && !playing;
          }
          if (phase_frame == 5 || phase_frame == 7) io.AddKeyEvent(ImGuiKey_Space, true);
          if (phase_frame == 6 || phase_frame == 8) {
            io.AddKeyEvent(ImGuiKey_Space, false);
            replay_keys_verified &= playing == (phase_frame == 6);
          }
          if (phase_frame == 20 || phase_frame == 21) {
            io.AddMousePosEvent(edit_probe.x, edit_probe.y);
            io.AddMouseButtonEvent(0, phase_frame == 20);
          }
          if (phase_frame == 22 || phase_frame == 23) {
            io.AddKeyEvent(ImGuiKey_Space, phase_frame == 22);
            io.AddKeyEvent(ImGuiKey_RightArrow, phase_frame == 22);
          }
          if (phase_frame == 24)
            replay_keys_verified &=
                !playing && replay->data()->time == selected_before && io.WantTextInput;
          if (phase_frame == 25) io.AddMousePosEvent(hover_probe.x, hover_probe.y);
          if (phase_frame == 26) zoom_initial_width = graph_limits[2].X.Size();
          if (phase_frame == 27) io.AddMouseWheelEvent(0, 1);
          if (phase_frame == 31) {
            io.AddMousePosEvent(panel_drag_probe.x, panel_drag_probe.y);
            io.AddMouseButtonEvent(0, true);
          }
          if (phase_frame >= 32 && phase_frame <= 34)
            io.AddMousePosEvent(panel_drag_probe.x + (phase_frame - 31) * 30,
                                panel_drag_probe.y + 50);
          if (phase_frame == 35) io.AddMouseButtonEvent(0, false);
          if (phase_frame == 36) io.AddMousePosEvent(hover_probe.x, hover_probe.y);
          if (phase_frame == 39 || phase_frame == 40) {
            io.AddMousePosEvent(library_toggle_probe.x, library_toggle_probe.y);
            io.AddMouseButtonEvent(0, phase_frame == 39);
          }
          if (phase_frame >= 43 && library_click_state == 0 && !recording_files.empty() &&
              !save_task.valid()) {
            io.AddMousePosEvent(library_row_probe.x, library_row_probe.y);
            io.AddMouseButtonEvent(0, true);
            library_click_state = 1;
          } else if (library_click_state == 1) {
            io.AddMouseButtonEvent(0, false);
            library_click_state = 2;
          }
        }
        if (phase_frame == 8) {
          io.AddMousePosEvent(graph_probe.x, graph_probe.y);
          io.AddMouseButtonEvent(0, true);
        }
        if (phase_frame == 9) {
          io.AddMousePosEvent(graph_probe.x, graph_probe.y);
          io.AddMouseButtonEvent(0, false);
        }
        if (phase_frame == 13) {
          selected_before = replay->data()->time;
          camera_before = camera.azimuth;
          io.AddMousePosEvent(view_probe.x, view_probe.y);
          io.AddMouseButtonEvent(0, true);
        }
        if (phase_frame >= 14 && phase_frame <= 18)
          io.AddMousePosEvent(view_probe.x + (phase_frame - 13) * 12, view_probe.y + 10);
        if (phase_frame == 19) {
          io.AddMousePosEvent(view_probe.x + 60, view_probe.y + 10);
          io.AddMouseButtonEvent(0, false);
        }
      }
      ImGui::NewFrame();
      int width, height_pixels;
      glfwGetWindowSize(window, &width, &height_pixels);
      // 최대화/복원 직후 화면 전체를 다시 채운다. 다음 프레임부터 수동 조절을 허용한다.
      fit_layout = width != layout_width || height_pixels != layout_height;
      layout_width = width;
      layout_height = height_pixels;
      left_width = std::min(290.f, width * .25f);
      right_width = std::min(760.f, width * .4f);
      controls_panel(width, height_pixels);
      if (sim().profile().manual_joint_control) joints_panel(width, height_pixels);
      graph_panel(width, height_pixels);
      view_panel(width, height_pixels);
      replay_shortcuts();
      if (follow) mju_copy3(camera.lookat, shown()->xpos + 3 * sim().base_body());
      mjv_updateScene(sim().model(), shown(), &options, &perturb, &camera, mjCAT_ALL, &scene);
      ImGui::Render();
      int fw, fh;
      glfwGetFramebufferSize(window, &fw, &fh);
      glViewport(0, 0, fw, fh);
      glClearColor(.07f, .08f, .1f, 1);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      const float sx = static_cast<float>(fw) / width, sy = static_cast<float>(fh) / height_pixels;
      mjrRect viewport{static_cast<int>(render_min.x * sx),
                       static_cast<int>((height_pixels - render_min.y - render_size.y) * sy),
                       static_cast<int>(render_size.x * sx), static_cast<int>(render_size.y * sy)};
      if (viewport.width > 0 && viewport.height > 0) mjr_render(viewport, &scene, &render);
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
      ++gui_frames;
      if ((selftest || benchmark_seconds > 0) && phase_frame == (selftest ? 38 : 24)) {
        const auto& ep = replay->episode();
        // 긴 기록에서는 한 화면 픽셀이 여러 물리 표본을 포함한다. 실제 클릭 좌표의 시각을 검증한다.
        bool selected = graph_click_observed &&
                        std::abs(requested_time - graph_clicked_time) < 1e-8 &&
                        replay->selected() == ep.nearest(graph_clicked_time);
        bool frozen = replay->data()->time == selected_before;
        bool rotated = std::abs(camera.azimuth - camera_before) > 1;
        double recorded_base = ep.frame(replay->selected())[ep.stride - 5];
        bool pose =
            std::abs(recorded_base - replay->data()->xpos[3 * sim().base_body() + 2]) < 1e-10;
        if (!screenshot_path.empty()) screenshot(screenshot_path, fw, fh);
        nlohmann::json result = {{"graph_click_selects_sample", selected},
                                 {"camera_moves_while_time_frozen", frozen && rotated},
                                 {"replayed_base_matches_record", pose},
                                 {"recorded_frames", ep.frames()},
                                 {"gui_frames", gui_frames},
                                 {"selected_time", replay->data()->time},
                                 {"compute_mean_ms", timing.mean_ms},
                                 {"compute_max_ms", timing.max_ms},
                                 {"overruns", timing.overruns},
                                 {"steps", timing.steps},
                                 {"realtime_factor", timing.realtime_factor}};
        bool duration_met = benchmark_seconds <= 0 ||
                            timing.steps * sim().model()->opt.timestep >= benchmark_seconds * .95;
        result["benchmark_duration_met"] = duration_met;
        result["simulated_seconds"] = timing.steps * sim().model()->opt.timestep;
        result["requested_time"] = requested_time;
        result["click_expected_time"] = graph_clicked_time;
        const int expected_layout_checks = sim().profile().manual_joint_control ? 16 : 12;
        if (selftest) {
          bool zoomed =
              zoom_initial_width > 0 && graph_limits[2].X.Size() < zoom_initial_width * .95;
          bool auto_y = true;
          for (int plot = 0; plot < 3; ++plot) {
            double minimum = INFINITY, maximum = -INFINITY;
            const auto& limits = graph_limits[plot];
            for (size_t sample = 0; sample < ep.frames(); ++sample)
              if (ep.time(sample) >= limits.X.Min && ep.time(sample) <= limits.X.Max) {
                double value = ep.value(sample, selected_channels[plot]);
                minimum = std::min(minimum, value);
                maximum = std::max(maximum, value);
              }
            auto_y &= std::isfinite(minimum) && limits.Y.Min <= minimum + 1e-7 &&
                      limits.Y.Max >= maximum - 1e-7;
            if (maximum - minimum > 1e-7) auto_y &= limits.Y.Size() < 1.5 * (maximum - minimum);
          }
          result["x_zoom_with_visible_y_autofit"] = zoomed && auto_y;
          result["panels_stay_fixed_when_dragged"] = layout_verified;
          result["resize_layout_fills_window"] =
              layout_verified && resize_checks >= expected_layout_checks;
          result["resize_panel_checks"] = resize_checks;
          result["replay_keyboard_step_and_play_pause"] = replay_keys_verified;
          bool formatted = cursor_text(3.4, 57, "N m") == "3.4s, 57Nm" &&
                           cursor_text(3.4, 2, "rad/s") == "3.4s, 2rad/s" &&
                           cursor_text(3.4, 1, "m") == "3.4s, 1m";
          result["cursor_coordinates_have_units"] = cursor_units_verified && formatted;
          const auto* font = ImGui::GetFont();
          result["korean_font_loaded"] =
              font->FindGlyphNoFallback(0xD55C) && font->FindGlyphNoFallback(0xAE00);
          replay_keys_verified &= formatted && cursor_units_verified &&
                                  result["korean_font_loaded"].get<bool>() && zoomed && auto_y;
        }
        const bool passed =
            selected && frozen && rotated && pose && duration_met &&
            (!selftest ||
             (layout_verified && resize_checks >= expected_layout_checks && replay_keys_verified));
        if (!selftest) {
          std::cout << result.dump(2) << '\n';
          return passed ? 0 : 2;
        }
        gui_checks = std::move(result);
        gui_checks_passed = passed;
        library_fixture =
            std::filesystem::temp_directory_path() /
            ("rclab-library-" + std::to_string(Clock::now().time_since_epoch().count()));
        std::filesystem::create_directories(library_fixture);
        catalog_folder = library_fixture.string();
        std::snprintf(catalog_directory, sizeof(catalog_directory), "%s", catalog_folder.c_str());
        refresh_catalog = true;
        auto copy = ep;
        save_task =
            std::async(std::launch::async, [copy = std::move(copy), folder = library_fixture] {
              copy.save(folder / "검사 기록.rclog");
              return std::string("검사 기록 저장 완료");
            });
      }
      if (selftest && phase_frame > 43 &&
          ((opened_recording == (library_fixture / "검사 기록.rclog").string() &&
            ++library_loaded_frames >= 3) ||
           phase_frame > 180)) {
        const bool loaded =
            library_click_state == 2 &&
            opened_recording == (library_fixture / "검사 기록.rclog").string() && replay &&
            replay->episode().frames() == gui_checks.at("recorded_frames").get<size_t>();
        gui_checks["recording_list_click_opens_saved_log"] = loaded;
        if (!screenshot_path.empty()) screenshot(screenshot_path, fw, fh);
        std::cout << gui_checks.dump(2) << '\n';
        std::filesystem::remove_all(library_fixture);
        return gui_checks_passed && loaded ? 0 : 2;
      }
      glfwSwapBuffers(window);
      if (selftest || benchmark_seconds > 0)
        std::this_thread::sleep_until(now + std::chrono::microseconds(16667));
    }
    return 0;
  }
};
}  // namespace
int main(int argc, char** argv) {
  try {
    rclab::register_extensions();
    std::string profile, model, record, screenshot;
    bool selftest = false;
    double benchmark = 0;
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
      else if (arg == "--open")
        record = next();
      else if (arg == "--self-test")
        selftest = true;
      else if (arg == "--screenshot")
        screenshot = next();
      else if (arg == "--benchmark-seconds")
        benchmark = std::stod(next());
      else if (arg == "--help") {
        std::cout << "rclab_sim --robot PROFILE [--model XML] [--open FILE.rclog] [--self-test] "
                     "[--screenshot FILE.ppm] [--benchmark-seconds 60]\n";
        return 0;
      } else
        throw std::runtime_error("Unknown argument: " + arg);
    }
    if (profile.empty() || !std::isfinite(benchmark) || benchmark < 0)
      throw std::runtime_error("--robot PROFILE required; benchmark must be finite and >= 0");
    Gui app(rclab::RobotProfile::load(profile, model));
    app.selftest = selftest;
    app.screenshot_path = screenshot;
    if (!record.empty())
      app.open_episode(rclab::Episode::load(record, app.sim(), app.sim().profile().record_bytes));
    return app.run(benchmark);
  } catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << '\n';
    return 1;
  }
}
