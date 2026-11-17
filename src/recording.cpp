#include "rclab/recording.hpp"

#include <zstd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace rclab {
namespace {
using Json = nlohmann::json;
constexpr char legacy_magic[8] = {'R', 'C', 'L', 'O', 'G', '0', '0', '1'};
constexpr char compressed_magic[8] = {'R', 'C', 'L', 'O', 'G', '0', '0', '2'};
uint64_t checksum(const std::vector<double>& rows) {
  uint64_t value = 14695981039346656037ULL;
  auto* bytes = reinterpret_cast<const unsigned char*>(rows.data());
  for (size_t i = 0; i < rows.size() * sizeof(double); ++i) {
    value ^= bytes[i];
    value *= 1099511628211ULL;
  }
  return value;
}
void check_platform() {
  uint16_t endian = 1;
  if (*reinterpret_cast<unsigned char*>(&endian) != 1 || sizeof(double) != 8 ||
      !std::numeric_limits<double>::is_iec559)
    throw std::runtime_error("Record format requires IEEE754 little-endian float64");
}
Json describe(const Simulation& sim) {
  const auto* m = sim.model();
  Json channels = Json::array(), names = Json::array(), actuators = Json::array();
  for (const auto& c : sim.channels()) channels.push_back({{"name", c.name}, {"unit", c.unit}});
  for (const auto& j : sim.joints())
    names.push_back({{"name", j.name}, {"type", j.type}, {"qpos", j.qpos}, {"dof", j.dof}});
  for (int a = 0; a < m->nu; ++a) {
    const char* n = mj_id2name(m, mjOBJ_ACTUATOR, a);
    actuators.push_back(n ? n : "unnamed_" + std::to_string(a));
  }
  Json metadata = {
      {"schema_version", 1},
      {"mujoco_version", mj_versionString()},
      {"model_fingerprint", sim.fingerprint()},
      {"dimensions", {{"nq", m->nq}, {"nv", m->nv}, {"nu", m->nu}, {"na", m->na}}},
      {"state_signature", mjSTATE_INTEGRATION},
      {"state_size", mj_stateSize(m, mjSTATE_INTEGRATION)},
      {"target_count", sim.joints().size()},
      {"base_body", sim.profile().base_body},
      {"timestep", m->opt.timestep},
      {"control_dt", sim.profile().control_dt},
      {"sampling",
       "pre-integration state at t; controls apply on [t,t+timestep); diagnostics evaluated at t"},
      {"joints", names},
      {"actuators", actuators},
      {"channels", channels},
      {"layout", "integration_state,diagnostics,joint_targets,base_world_pos_quat"}};
  if (!sim.profile().control_stack.empty()) {
    metadata["control_contract"] = sim.control_metadata();
    metadata["joint_target_available"] = Json::array();
    for (const auto& joint : sim.joints())
      metadata["joint_target_available"].push_back(joint.joint_target_available);
  }
  return metadata;
}
}  // namespace
std::vector<RecordingFile> list_recordings(const std::filesystem::path& directory) {
  std::error_code error;
  const bool exists = std::filesystem::exists(directory, error);
  if (error) throw std::runtime_error("기록 폴더에 접근할 수 없습니다.");
  if (!exists) return {};
  if (!std::filesystem::is_directory(directory))
    throw std::runtime_error("기록 폴더 경로를 확인해 주세요.");
  std::vector<RecordingFile> result;
  for (const auto& entry : std::filesystem::directory_iterator(directory)) {
    if (!entry.is_regular_file() || entry.path().extension() != ".rclog") continue;
    result.push_back({entry.path(), entry.last_write_time(), entry.file_size(),
                      std::filesystem::is_regular_file(entry.path().string() + ".json")});
  }
  std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
    return a.modified != b.modified ? a.modified > b.modified : a.path < b.path;
  });
  return result;
}
const double* Episode::frame(size_t i) const {
  if (i >= frames()) throw std::out_of_range("Episode frame");
  return rows.data() + i * stride;
}
double Episode::value(size_t i, size_t channel) const {
  if (channel >= channel_count) throw std::out_of_range("Episode channel");
  return frame(i)[state_size + channel];
}
size_t Episode::nearest(double t) const {
  if (!frames() || !std::isfinite(t))
    throw std::runtime_error("Cannot seek empty episode / nonfinite time");
  size_t lo = 0, hi = frames();
  while (lo < hi) {
    size_t mid = lo + (hi - lo) / 2;
    if (time(mid) < t)
      lo = mid + 1;
    else
      hi = mid;
  }
  if (lo == frames()) return frames() - 1;
  if (lo && t - time(lo - 1) <= time(lo) - t) return lo - 1;
  return lo;
}
void Episode::save(const std::filesystem::path& path) const {
  check_platform();
  if (!frames()) throw std::runtime_error("Cannot save an empty episode");
  std::filesystem::path meta = path.string() + ".json";
  if (std::filesystem::exists(path) || std::filesystem::exists(meta))
    throw std::runtime_error("Recording already exists; choose another filename");
  if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
  Json document = metadata;
  document["frames"] = frames();
  document["stride"] = stride;
  document["checksum_fnv1a64"] = checksum(rows);
  const size_t raw_bytes = rows.size() * sizeof(double);
  // 백그라운드 저장에서만 압축한다. 물리 스텝과 메모리 내 표본은 변경하지 않는다.
  std::vector<char> compressed(ZSTD_compressBound(raw_bytes));
  const size_t compressed_bytes =
      ZSTD_compress(compressed.data(), compressed.size(), rows.data(), raw_bytes, 3);
  if (ZSTD_isError(compressed_bytes))
    throw std::runtime_error("기록 압축 실패: " + std::string(ZSTD_getErrorName(compressed_bytes)));
  document["storage"] = {
      {"format_version", 2}, {"codec", "zstd"}, {"uncompressed_bytes", raw_bytes}};
  auto tmp = path.string() + ".tmp", meta_tmp = meta.string() + ".tmp";
  try {
    std::ofstream out(tmp, std::ios::binary);
    const uint64_t counts[2] = {frames(), stride};
    out.write(compressed_magic, sizeof(compressed_magic));
    out.write(reinterpret_cast<const char*>(counts), sizeof(counts));
    out.write(compressed.data(), static_cast<std::streamsize>(compressed_bytes));
    out.flush();
    if (!out) throw std::runtime_error("Recording write failed");
    out.close();
    std::ofstream json(meta_tmp);
    json << document.dump(2) << '\n';
    json.flush();
    if (!json) throw std::runtime_error("Metadata write failed");
    json.close();
    std::filesystem::rename(meta_tmp, meta);
    std::filesystem::rename(tmp, path);
  } catch (...) {
    std::filesystem::remove(tmp);
    std::filesystem::remove(meta_tmp);
    throw;
  }
}
Episode Episode::load(const std::filesystem::path& path, const Simulation& sim, size_t max_bytes) {
  check_platform();
  const auto meta_path = path.string() + ".json";
  if (!std::filesystem::is_regular_file(path))
    throw std::runtime_error("기록 파일을 찾을 수 없습니다. 경로를 확인해 주세요.");
  if (!std::filesystem::is_regular_file(meta_path))
    throw std::runtime_error(
        "기록 정보 파일(.rclog.json)이 없습니다. 기록 파일과 같은 폴더에 두세요.");
  if (std::filesystem::file_size(meta_path) > 1024 * 1024)
    throw std::runtime_error("Metadata is too large");
  std::ifstream json(meta_path);
  Json metadata;
  json >> metadata;
  const Json expected = describe(sim);
  for (const auto* key : {"control_contract", "joint_target_available"}) {
    if (metadata.contains(key) != expected.contains(key) ||
        (expected.contains(key) && metadata.at(key) != expected.at(key)))
      throw std::runtime_error(std::string("Recording control configuration mismatch: ") + key);
  }
  for (auto key : {"schema_version", "mujoco_version", "model_fingerprint", "dimensions",
                   "state_signature", "state_size", "target_count", "base_body", "timestep",
                   "control_dt", "joints", "actuators", "channels", "layout", "sampling"}) {
    if (!metadata.contains(key) || metadata[key] != expected[key])
      throw std::runtime_error(std::string("Recording is incompatible: ") + key);
  }
  Episode ep;
  ep.metadata = metadata;
  ep.state_size = metadata.at("state_size");
  ep.channel_count = metadata.at("channels").size();
  ep.target_count = metadata.at("target_count");
  ep.stride = ep.state_size + ep.channel_count + ep.target_count + 7;
  std::ifstream input(path, std::ios::binary);
  char tag[8]{};
  uint64_t counts[2]{};
  input.read(tag, 8);
  input.read(reinterpret_cast<char*>(counts), sizeof(counts));
  const bool compressed = std::memcmp(tag, compressed_magic, 8) == 0;
  const bool legacy = std::memcmp(tag, legacy_magic, 8) == 0;
  if (!input || (!compressed && !legacy) || counts[1] != ep.stride ||
      metadata.at("stride") != ep.stride || counts[0] == 0 || metadata.at("frames") != counts[0] ||
      counts[0] > max_bytes / sizeof(double) / ep.stride)
    throw std::runtime_error("Invalid / oversized recording header");
  const size_t elements = static_cast<size_t>(counts[0]) * ep.stride;
  const size_t raw_bytes = elements * sizeof(double);
  const auto file_bytes = std::filesystem::file_size(path);
  if (compressed) {
    if (!metadata.contains("storage") || metadata["storage"].value("format_version", 0) != 2 ||
        metadata["storage"].value("codec", "") != "zstd" ||
        metadata["storage"].at("uncompressed_bytes") != raw_bytes)
      throw std::runtime_error("압축 기록의 저장 정보가 일치하지 않습니다.");
    // 헤더에서 복원 용량을 먼저 제한하고, 단일 프레임과 실제 출력 크기를 모두 검사한다.
    if (file_bytes <= 24 || file_bytes - 24 > ZSTD_compressBound(raw_bytes))
      throw std::runtime_error("압축 기록의 파일 크기가 올바르지 않습니다.");
    std::vector<char> payload(static_cast<size_t>(file_bytes - 24));
    input.read(payload.data(), static_cast<std::streamsize>(payload.size()));
    const auto frame_bytes = ZSTD_findFrameCompressedSize(payload.data(), payload.size());
    if (!input || ZSTD_isError(frame_bytes) || frame_bytes != payload.size() ||
        ZSTD_getFrameContentSize(payload.data(), payload.size()) != raw_bytes)
      throw std::runtime_error("압축 기록이 손상되었거나 복원 크기가 일치하지 않습니다.");
    ep.rows.resize(elements);
    const auto decoded = ZSTD_decompress(ep.rows.data(), raw_bytes, payload.data(), payload.size());
    if (ZSTD_isError(decoded) || decoded != raw_bytes)
      throw std::runtime_error("기록 압축을 해제하지 못했습니다.");
  } else {
    if (metadata.contains("storage") || file_bytes != 24 + raw_bytes)
      throw std::runtime_error("Recording length / storage mismatch");
    ep.rows.resize(elements);
    input.read(reinterpret_cast<char*>(ep.rows.data()), static_cast<std::streamsize>(raw_bytes));
  }
  if (!input || checksum(ep.rows) != metadata.at("checksum_fnv1a64").get<uint64_t>())
    throw std::runtime_error("Recording checksum mismatch");
  for (double value : ep.rows)
    if (!std::isfinite(value)) throw std::runtime_error("Nonfinite recording value");
  for (size_t i = 1; i < ep.frames(); ++i)
    if (std::abs((ep.time(i) - ep.time(i - 1)) - sim.model()->opt.timestep) > 1e-8)
      throw std::runtime_error("Recording timestamps are not contiguous");
  return ep;
}
void EpisodeRecorder::start(const Simulation& sim, double seconds, size_t bytes) {
  if (!std::isfinite(seconds) || seconds <= 0) throw std::runtime_error("Invalid record duration");
  episode_ = {};
  episode_.metadata = describe(sim);
  episode_.state_size = mj_stateSize(sim.model(), mjSTATE_INTEGRATION);
  episode_.channel_count = sim.channels().size();
  episode_.target_count = sim.joints().size();
  episode_.stride = episode_.state_size + episode_.channel_count + episode_.target_count + 7;
  const double requested = std::ceil(seconds / sim.model()->opt.timestep);
  capacity_ = std::min(static_cast<double>(bytes / sizeof(double) / episode_.stride), requested);
  if (!capacity_) throw std::runtime_error("Recording budget is smaller than one frame");
  episode_.rows.reserve(capacity_ * episode_.stride);
  active_ = true;
  status_ = "Recording";
}
bool EpisodeRecorder::capture(const Simulation& sim) {
  if (!active_) return false;
  if (episode_.frames() >= capacity_) {
    active_ = false;
    status_ = "Recording limit reached (duration or memory)";
    return false;
  }
  size_t offset = episode_.rows.size();
  episode_.rows.resize(offset + episode_.stride);
  double* row = episode_.rows.data() + offset;
  mj_getState(sim.model(), sim.data(), row, mjSTATE_INTEGRATION);
  sim.diagnostics(row + episode_.state_size);
  std::copy(sim.controls().target.begin(), sim.controls().target.end(),
            row + episode_.state_size + episode_.channel_count);
  double* pose = row + episode_.stride - 7;
  std::copy(sim.data()->xpos + 3 * sim.base_body(), sim.data()->xpos + 3 * sim.base_body() + 3,
            pose);
  std::copy(sim.data()->xquat + 4 * sim.base_body(), sim.data()->xquat + 4 * sim.base_body() + 4,
            pose + 3);
  if (episode_.frames() == capacity_) {
    active_ = false;
    status_ = "Recording limit reached (duration or memory)";
  }
  return true;
}
Episode EpisodeRecorder::release() {
  active_ = false;
  status_ = "Recording stopped";
  return std::move(episode_);
}
ReplaySession::ReplaySession(const Simulation& sim, Episode ep)
    : model_(sim.model()), data_(mj_makeData(model_)), episode_(std::move(ep)) {
  if (!data_ || !episode_.frames()) throw std::runtime_error("Cannot replay empty episode");
  select(0);
}
size_t ReplaySession::seek(double t) {
  select(episode_.nearest(t));
  return selected_;
}
void ReplaySession::select(size_t i) {
  const double* row = episode_.frame(i);
  mj_setState(model_, data_.get(), row, mjSTATE_INTEGRATION);
  // 재생은 적분하거나 제어기를 호출하지 않는다. 표시용 파생값만 다시 계산한다.
  mj_forward(model_, data_.get());
  selected_ = i;
}
}  // namespace rclab
