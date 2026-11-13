#pragma once
#include <nlohmann/json.hpp>

#include "rclab/simulation.hpp"

namespace rclab {
struct RecordingFile {
  std::filesystem::path path;
  std::filesystem::file_time_type modified;
  uintmax_t bytes = 0;
  bool has_metadata = false;
};
// 지정 폴더의 로그만 조회한다. 모델/정책 호환성은 열 때 검사한다.
std::vector<RecordingFile> list_recordings(const std::filesystem::path& directory);
// 한 행은 해당 상태 시각의 진단값과 다음 스텝에 적용할 입력을 포함한다.
// [integration state | diagnostics | targets | body position/quaternion]
struct Episode {
  nlohmann::json metadata;
  size_t state_size = 0, channel_count = 0, target_count = 0, stride = 0;
  std::vector<double> rows;
  size_t frames() const { return stride ? rows.size() / stride : 0; }
  const double* frame(size_t i) const;
  double time(size_t i) const { return frame(i)[0]; }
  double value(size_t i, size_t channel) const;
  size_t nearest(double t) const;
  void save(const std::filesystem::path&) const;
  static Episode load(const std::filesystem::path&, const Simulation&,
                      size_t max_bytes = 256 * 1024 * 1024);
};
class EpisodeRecorder {
 public:
  void start(const Simulation&, double seconds, size_t bytes);
  bool capture(const Simulation&);
  void stop() { active_ = false; }
  bool active() const { return active_; }
  const std::string& status() const { return status_; }
  const Episode& episode() const { return episode_; }
  Episode release();

 private:
  Episode episode_;
  bool active_ = false;
  size_t capacity_ = 0;
  std::string status_;
};
class ReplaySession {
 public:
  ReplaySession(const Simulation&, Episode episode);
  size_t seek(double t);
  void select(size_t i);
  size_t selected() const { return selected_; }
  mjData* data() const { return data_.get(); }
  const Episode& episode() const { return episode_; }

 private:
  const mjModel* model_;
  DataPtr data_;
  Episode episode_;
  size_t selected_ = 0;
};
}  // namespace rclab
