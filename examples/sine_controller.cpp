#include <cmath>

#include "rclab/extensions.hpp"
#include "rclab/simulation.hpp"

namespace rclab {
namespace {
// 첫 번째 지원 관절에만 작은 목표 변화를 준다. 보행 정책이 아닌 연결 예시다.
class ExampleSineController : public IController {
 public:
  void reset(const mjModel&, const mjData& d) override {
    started_ = d.time;
    initialized_ = false;
  }
  void update(const mjModel&, const mjData& d, ControlState& controls) override {
    for (size_t i = 0; i < controls.target.size(); ++i) {
      if (!controls.enabled[i]) continue;
      if (!initialized_) {
        initial_ = controls.target[i];
        initialized_ = true;
      }
      controls.target[i] =
          initial_ + 0.1 * std::sin(2 * 3.141592653589793 * 0.5 * (d.time - started_));
      break;
    }
  }
  std::vector<DiagnosticChannel> channels() const override { return {{"example/sine", "1"}}; }
  void diagnostics(const mjModel&, const mjData& d, double* values) const override {
    values[0] = std::sin(2 * 3.141592653589793 * 0.5 * (d.time - started_));
  }

 private:
  double started_ = 0, initial_ = 0;
  bool initialized_ = false;
};
}  // namespace
void register_extensions() {
  register_controller("example_sine", [] { return std::make_unique<ExampleSineController>(); });
}
}  // namespace rclab
