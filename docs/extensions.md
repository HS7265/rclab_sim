# C++ 제어기와 구동기 확장

프로필 추가로 해결할 수 없는 수식·보행 정책은 C++ 확장에 둔다. 공개 인터페이스는 `include/rclab/simulation.hpp`, 시작 등록 지점은 `include/rclab/extensions.hpp`다. 동적 플러그인 로더나 Python 콜백을 사용하지 않는다.

## 제어기 예제 실행

기본 빌드에는 `examples/sine_controller.cpp`가 등록되어 있다.

```bash
./build/rclab_sim --robot profiles/sine.example.json
```

`example_sine` 제어기는 첫 번째 활성 지원 관절에 진폭 0.1, 주파수 0.5Hz의 목표 변화를 준다. 예제 모델은 회전 관절이므로 진폭은 rad이다. 다른 직선 관절 로봇에 그대로 적용하지 않는다. 보행 정책이 아닌 인터페이스 연결 예시다. `example/sine` 진단 채널도 기록된다.

## IController

- `initialize_state(model, data)`: `mj_resetData` 직후 로봇 초기 자세를 설정한다. 그 뒤 프로필 초기 자세·공중 고정·`mj_forward`를 적용하고 `reset`을 호출한다.
- `reset(model, data)`: 초기화/리셋 시 내부 상태를 초기화한다. 필요한 메모리는 여기서 확보한다.
- `update(model, data, controls)`: `control_dt`마다 호출된다. 관측은 모델 주소로 읽고 `controls.target`, `kp`, `kd`, `enabled`를 갱신한다. `controls.command`는 GUI 명령이다.
- `channels()`: 초기화 때 고정된 이름·단위의 채널 목록을 반환한다. 실행 중 개수나 순서를 바꾸지 않는다.
- `diagnostics(model, data, output)`: 지정된 길이의 버퍼를 채운다. 모든 물리 스텝의 동일 시점 상태에 대한 값이다.

물리 루프에서 파일을 읽거나 메모리를 반복 할당하거나 GUI에 접근하지 않는다. ONNX를 연결할 경우 입력 순서·단위·정규화·history·hidden state와 제어 주기를 명시적으로 맞춰야 한다. `recording_metadata()`는 번들 해시·제어 설정 등 기록 호환성 계약을 반환한다.

## IActuationAdapter

- `bind(model, joints)`: 초기화 시 한 번 호출된다. 모델에서 지원하는 구동 관계와 주소를 확인하고 제어 가능한 관절을 표시한다.
- `reset(model, data)`: 지연·필터·구동기 내부 상태를 초기화한다.
- `apply(model, data, joints, controls)`: 매 물리 스텝에 호출해 `data.ctrl`을 채운다. 힘/속도 한계·전달 관계를 적용한다.
- `after_step(model, data)`: `mj_step` 직후 센서·지연 상태를 갱신한다. 물리를 추가 적분하지 않는다.
- `channels/diagnostics`: 실제 계산한 모터 좌표·모터 속도·힘 등만 이름과 단위를 붙여 출력한다.

어댑터는 연결되지 않은 입력을 명시적으로 0으로 초기화하고 필요한 입력만 작성해야 한다. `qpos[joint_id]` 또는 `qvel[joint_id]`를 사용하지 말고 `jnt_qposadr`·`jnt_dofadr`로 찾는다. 고정 gain·gear의 기본 구현은 `src/simulation.cpp`의 `DirectMotor`를 참고한다.

프로필의 관절 목표는 기본적으로 관절 좌표다. 실제 모터 목표가 필요한 로봇은 어댑터 계약에서 좌표·단위를 명시하고 전용 입력 GUI도 연결해야 한다. 기본 GUI의 관절값에 물리 모터각이라는 이름만 붙이지 않는다.

## 등록과 빌드

기본 예제 대신 사용할 파일에서 다음 함수를 구현한다. 여러 파일로 나눌 수 있지만 `register_extensions()`의 정의는 하나여야 한다.

```cpp
#include "rclab/extensions.hpp"
#include "rclab/simulation.hpp"
#include "my_controller.hpp"
#include "my_actuation.hpp"

namespace rclab {
void register_extensions() {
  register_controller("my_walk", [] { return std::make_unique<MyWalkController>(); });
  register_adapter("my_motors", [] { return std::make_unique<MyActuationAdapter>(); });
}
}
```

```bash
cmake -S . -B build \
  -DRCLAB_EXTENSION_SOURCES="/path/to/register.cpp;/path/to/my_controller.cpp;/path/to/my_actuation.cpp"
cmake --build build -j2
```

프로필에서 `"controller": "my_walk"`, `"adapter": "my_motors"`를 지정한다. 존재하지 않거나 중복 등록된 이름은 오류다. `pd`·`passive`와 `direct_motor`는 코어에 기본 등록된다.

제어기와 구동기가 hidden·지연 버퍼를 공유해야 한다면 `register_control_stack(name, factory)`를 사용한다. factory는 `const RobotProfile&`을 받아 두 객체를 담은 `ControlStack`을 반환한다. 프로필의 `control_stack`이 있으면 개별 controller/adapter 대신 이 factory를 사용하며 `extension` JSON으로 로봇 설정을 전달한다. 공유 런타임은 인스턴스별로 생성한다. 선택 빌드 예시는 `src/rok4_student.cpp`다.

`command_mode: "run_stop"`은 GUI 명령을 달리기 요청으로 표시하고 기록 단위를 무차원으로 설정한다. 기본값은 `velocity`다. `manual_joint_control: false`는 수동 슬라이더와 PD 게인 편집을 비활성화한다. 모터 목표만 존재하는 축은 adapter가 `joint_target_available=false`로 표시해야 한다. 해당 축의 관절 목표 채널은 생성하지 않으며 실제 모터 목표를 별도 진단 채널로 등록한다.

## 기록·재생 계약

호출 순서는 `필요 시 controller.update → adapter.apply → mj_forward → 시각 t 기록 → mj_step → adapter.after_step`이다. `mj_forward`는 적분 전 상태와 제어 입력에 맞는 공통 진단값을 계산한다. 힘과 자세의 시점 정렬을 위해 추가 계산 비용을 성능 수치에 포함한다. 제어기 관측은 이 forward 이전에 구성하므로 마지막 적분에서 남은 파생값을 사용하는 정책 계약을 유지할 수 있다. 로봇 전용 진단값의 갱신 시점도 문서화해야 한다.

현재 재생은 기록 관찰용이다. 저장 상태로 `mj_forward`만 호출하며 제어기나 사용자 어댑터를 실행하지 않는다. 재생 중 원본의 내부 상태도 진행하지 않는다. 따라서 과거 시점에서 새 정책 실험을 시작하는 기능으로 사용할 수 없다. 그 기능을 추가할 때는 RNN hidden state, 필터·지연 버퍼, 난수 상태 등 사용자 제어기 상태까지 별도로 직렬화해야 한다.
