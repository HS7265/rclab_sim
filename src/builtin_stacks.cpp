#include <mutex>

#include "rclab/simulation.hpp"
#ifdef RCLAB_WITH_ONNX
#include "rclab/rok4_student.hpp"
#endif

namespace rclab {
void register_builtin_control_stacks() {
  static std::once_flag once;
  std::call_once(once, [] {
#ifdef RCLAB_WITH_ONNX
    register_rok4_student_stack();
#endif
  });
}
}  // namespace rclab
