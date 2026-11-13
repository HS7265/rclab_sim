# RCLab Robot Simulator

연구실에서 여러 로봇을 같은 프로그램으로 검사하고 제어하는 **C++ MuJoCo + Dear ImGui + ImPlot** 시뮬레이터다. 로봇 프로필 JSON을 추가하면 모델의 관절 수·축·범위를 읽어 GUI를 구성한다. 실행에 ROS나 Python이 필요하지 않다.

기록한 토크·각속도 등의 그래프를 클릭하면 **그 시점의 로봇을 3D로 표시하고 정지**한다. 카메라를 자유롭게 움직이며 자세·충돌 형상·좌표계를 살펴볼 수 있다. 동영상 생성 과정은 없다.

## 지원 환경과 빌드

- 검증 기준: Ubuntu 22.04, GCC 11, CMake 3.22, C++17, MuJoCo **3.11.0**.
- GUI: GLFW 3.3, OpenGL 3.3 compatibility profile, Dear ImGui **1.89.9**, ImPlot **0.16**.
- JSON: nlohmann/json 3.10 이상. ImGui와 ImPlot은 CMake가 지정 버전을 내려받는다.
- 실제 화면에는 X11/GLFW가 접근할 수 있는 `DISPLAY`가 필요하다. GUI 없는 실행은 디스플레이가 필요 없다.

```bash
sudo apt-get update
sudo apt-get install build-essential cmake pkg-config libglfw3-dev libgl1-mesa-dev nlohmann-json3-dev fonts-noto-cjk
```

[MuJoCo 공식 배포본](https://github.com/google-deepmind/mujoco/releases/tag/3.11.0)을 설치한 뒤 설치 경로를 지정한다.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DMUJOCO_ROOT="$HOME/.mujoco/mujoco-3.11.0"
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

MuJoCo 경로는 `MUJOCO_PATH` 환경 변수로도 지정할 수 있다. 로컬 SDK에 SONAME 심링크가 빠져 있으면 CMake가 빌드 폴더에 링크를 만든다. 원본 SDK는 변경하지 않는다.

네트워크를 쓸 수 없는 경우 지정 버전의 소스 폴더를 전달한다.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DMUJOCO_ROOT=/path/to/mujoco-3.11.0 \
  -DFETCHCONTENT_SOURCE_DIR_IMGUI=/path/to/imgui-1.89.9 \
  -DFETCHCONTENT_SOURCE_DIR_IMPLOT=/path/to/implot-0.16
```

GUI를 빌드하지 않으려면 `-DRCLAB_BUILD_GUI=OFF`를 추가한다. 이 경우 GLFW·OpenGL·ImGui·ImPlot은 필요 없다.

한글 글꼴은 시스템의 Noto Sans CJK 또는 나눔고딕을 사용한다. 다른 한글 글꼴은 `RCLAB_FONT=/path/to/font.ttf`로 지정할 수 있다. GUI 없는 실행에는 글꼴이 필요 없다.

## 바로 실행하기

```bash
./build/rclab_sim --robot profiles/hexapod.json
./build/rclab_sim --robot profiles/humanoid.json
./build/rclab_headless --robot profiles/quadruped.json --duration 10
```

자체 제작 예제는 2족 6축, 4족 12축, 6족 18축, 휴머노이드 21축이다. 범용 GUI 검증을 위한 단순 형상 모델이며, 보행 정책이나 실제 로봇의 동역학 모델은 아니다.

RUBI·ROK-4의 모델·메시는 저장소에 포함하지 않는다. 로컬 모델을 연결한다.

```bash
./build/rclab_sim --robot profiles/rubi.example.json --model /path/to/RUBI/rubi.xml
./build/rclab_sim --robot profiles/rok4.example.json --model /path/to/rok_4_v4_flat.xml
```

`--model`은 프로필의 모델 경로를 대체한다. 모델에 포함된 include·메시·텍스처 파일도 원래 상대 경로에 있어야 한다. 프로필에 적은 상대 모델 경로는 **프로필 파일이 있는 폴더 기준**, 명령행의 상대 경로는 **현재 실행 폴더 기준**이다. `~/` 경로도 지원한다.

새 로봇 등록 방법과 복사 가능한 AI 프롬프트는 [로봇 추가 가이드](docs/adding_a_robot.md), 제어기·구동기 확장은 [C++ 확장 가이드](docs/extensions.md)를 참고한다.

ROK-4 Running student round400은 선택 빌드로 실행할 수 있다. **ONNX Runtime C++ SDK 1.22.0**과 OpenSSL 개발 패키지(`libssl-dev`)를 준비하고 기존 CMake 옵션에 다음을 추가한다.

```bash
cmake -S . -B build -DRCLAB_WITH_ONNX=ON -DONNXRUNTIME_ROOT=/path/to/onnxruntime-linux-x64-1.22.0
cmake --build build -j2
./build/rclab_sim --robot profiles/local/rok4-running-student.json
```

로컬 실행 프로필은 [공개 예시](profiles/rok4-running-student.example.json)를 복사한 뒤 모델·번들 경로를 수정한다. GUI의 `실행`을 누르고 `로봇 명령 → 달리기`을 선택한다. 해제하면 감속 후 정지한다. 임의 속도·회전 명령은 지원하지 않는다. 선택 정책·구동기·기록 계약과 검증은 [ROK-4 실행 가이드](docs/rok4_running_student.md)를 참고한다.

## GUI 사용법

기본 배치는 **왼쪽 조작, 중앙 3D, 오른쪽 세로 그래프 3개**다. 수동 제어가 가능한 프로필에서만 왼쪽 아래에 관절 조작 패널을 표시한다. 정책 전용 실행에서는 조작할 수 없는 관절 목록을 숨긴다. 패널 위치·크기는 고정하며, 창 크기 변경·최대화·복원 시 화면에 맞춰 자동 배치한다. 그래프 칸은 넓게 두고, 왼쪽 시뮬레이션 칸은 3D 장면이 비치는 반투명 배경을 사용한다. GUI는 한글을 기본으로 사용하며 로봇 관절 식별자·단위·필요한 기술 표기는 유지한다. 성능 수치와 모델 진단은 접힌 `고급 정보`에서 확인한다.

| 화면 | 기능 |
|---|---|
| 시뮬레이션 | 실행/일시정지, 한 스텝, 초기화, 실행 배율, 기록, 몸통 고정, 표시 옵션 |
| 3D 화면 | 마우스 카메라 조작, 부위 선택, 외란 |
| 관절 조작 | 이름 검색, 부위별 그룹, 현재 위치/속도, 목표값, Kp/Kd, 제어 활성화 |
| 그래프 · 3D 재생 | 실시간 그래프, 기록 열기/저장, 시간 선택·재생, 피크 탐색 |

그래프 선택 메뉴는 회전 관절의 구동 토크를 `토크 (torque)`(Nm), 직선 관절의 구동 힘을 `힘`(N)로 표시한다. 기존 기록과의 호환성을 위해 파일 내부의 해당 채널 식별자 `actuation`은 유지한다.

- 3D 화면에서 왼쪽 드래그: 회전, 오른쪽 드래그: 이동, 휠: 확대/축소. Shift로 조작 축을 바꾼다.
- 물체를 더블클릭해 선택한 뒤 Ctrl+드래그하면 실행 중 힘을 가한다. 기록 재생 중에는 외란을 적용하지 않는다.
- `Hold base`: 지정 몸통을 공중에서 고정. Height는 월드 Z 높이(m), Roll/Pitch/Yaw는 월드 자세(deg, ZYX 조합)다.
- 고정은 MuJoCo weld 제약이므로 미세한 오차가 있다. 로봇의 관절·접촉은 계속 계산된다.
- 기존 몸통↔월드 weld/connect는 해제하고 앱의 고정 제약을 사용한다. 로봇 내부 제약은 유지한다. 몸통에 free joint가 없는 모델은 관찰할 수 있지만 공중 이동 고정은 지원하지 않는다.
- 회전 관절의 GUI 목표값은 deg, 내부 계산과 기록은 rad이다. 직선 관절은 m이다.
- `Step`은 제어 주기가 아닌 **물리 스텝 한 번**이다. 초기화는 초기 자세·게인·명령으로 돌아가며 기록을 정지한다.
- 기본 PD 제어기는 속도 명령으로 보행하지 않는다. 속도 명령은 추가 제어기에 전달하는 공통 입력이다.

## 그래프를 클릭해서 3D 분석하기

1. `기록 시작`을 누르고 `실행`한다. 실시간 그래프에서 채널을 선택한다.
2. 기록 중 그래프를 클릭하거나 `기록 종료 후 분석`을 누른다. 물리 실행은 정지된다.
3. 기록 그래프의 원하는 시점을 클릭한다. 가장 가까운 **물리 스텝 표본**을 표시하며 요청 시각·선택 시각·시간 차이를 보여준다.
4. 카메라를 회전하거나 좌표계·충돌 형상을 켜서 확인한다. 이 조작으로 로봇 시간이 진행되지는 않는다.
5. **`←` / `→`로 1물리 스텝씩 이동**, **`Space`로 재생·정지**한다. 방향키를 누르면 재생을 멈추고 인접 표본을 선택한다. 누르고 있으면 반복 이동하며 기록 양끝에서 멈춘다. 입력칸 편집 중에는 단축키가 작동하지 않는다. 화면의 재생/스텝 버튼·배속·시간 슬라이더도 사용할 수 있다. 기록 끝에서 다시 재생하면 처음부터 시작한다.
6. `실시간 화면으로 돌아가기`를 누르면 보존된 마지막 실제 상태로 돌아간다. `실행`으로 이어간다.

실시간 미리보기 그래프는 렌더링 시점의 최근 2,048표본이다. 확대·이동하면 선택한 시간 범위를 유지하고, `최신 구간 보기`를 누르면 최신 구간 추적을 재개한다. **파일에 기록하는 데이터는 렌더링과 무관하게 모든 물리 스텝을 저장**한다. 정밀 분석에는 기록 그래프를 사용한다.

기록 그래프의 세 패널은 시간축을 공유한다. 휠은 시간축(X축)만 확대/축소하고 가운데 버튼 드래그로 시간 구간을 이동한다. 세로축(Y축)은 현재 보이는 구간의 데이터에 맞춰 자동 조정하며 직접 조작하지 않는다. `전체 범위`는 전체 시간과 값 범위를 맞춘다. `이전/다음 피크`와 `최댓값`은 첫 번째 패널의 **절댓값 피크**를 탐색한다.

그래프 오른쪽 아래의 커서 좌표에도 단위를 표시한다. 예를 들어 토크는 `3.4s, 57Nm`, 속도는 `3.4s, 2rad/s`처럼 표시한다.

기본 채널은 스칼라 관절 위치·속도·목표값·액추에이터가 가한 일반화 힘, 액추에이터 입력, 몸통 로컬 좌표계 속도, 명령이다. 프로필에서 접촉 부위를 지정하면 해당 부위와 하위 링크의 접촉력 크기 합도 기록한다. 이는 순벡터 합이나 발바닥 높이 추정값이 아니다. 추가 모터 좌표·행동 등은 제어기/구동기가 등록한 채널만 제공한다.

**그래프의 힘은 당시 기록값이다. 3D 화면의 접촉력 화살표는 재생 상태에서 다시 계산한 표시값이다.** 기록은 적분 직전 시각 `t`의 상태와 힘을 저장하며, 제어 입력은 `[t, t + 물리 간격)` 구간에 적용한다. 끝의 마지막 적분 결과는 다음 표본이 기록될 때 포함된다.

### 저장과 다시 열기

파일명을 바꾸고 `저장`을 누르면 백그라운드로 저장한다. 같은 이름의 파일은 덮어쓰지 않는다. 그래프 칸의 **`저장된 기록`을 펼치면 `recordings/`의 로그가 최신순으로 자동 표시**된다. 파일을 클릭하면 해당 기록을 연다. 목록은 펼쳐진 동안 2초마다 갱신되며 저장 직후에도 갱신한다. `폴더`를 입력하고 Enter 또는 `새로고침`으로 다른 폴더를 볼 수 있다. 하위 폴더는 자동 탐색하지 않는다. 정보 파일이 없는 로그는 비활성화하고, 로봇·정책 호환성은 열 때 검사한다. 직접 경로를 입력해 `열기` 또는 시작 옵션으로 여는 방식도 유지한다.

```bash
./build/rclab_headless --robot profiles/hexapod.json \
  --duration 5 --record recordings/hexapod_01.rclog
./build/rclab_sim --robot profiles/hexapod.json \
  --open recordings/hexapod_01.rclog
```

`.rclog` 바이너리와 `.rclog.json` 메타데이터는 한 쌍이다. 형식 버전·MuJoCo 버전·컴파일된 모델 식별값·차원·이름/주소·채널/단위·주기·체크섬을 검사한다. 다른 모델이나 다른 기록 채널 구성으로는 열지 않는다. 원본 모델 파일·메시도 별도로 보관해야 한다. 파일 형식은 little-endian IEEE754 float64 기준이다.

기본 기록 한도는 **120초 또는 256MiB 중 먼저 도달한 조건**이다. 도달하면 기록만 정지하며 화면에 알린다. 시뮬레이션은 계속할 수 있다. 프로필의 `recording`에서 변경할 수 있다.

## 제어·실시간 구조와 제한

- 물리·제어 전용 C++ worker와 메인 GUI/렌더링 스레드를 분리한다. 상태는 잠금 구간에서 복사하며 GUI 명령은 스텝 사이에 반영한다.
- 모델의 timestep을 따르고 `control_dt`는 그 정수배여야 한다. PD 구동기는 매 물리 스텝 피드백을 갱신하고 `IController::update`는 지정 제어 주기로 호출한다.
- 제어/기록 버퍼를 미리 확보한다. 저장은 백그라운드 작업이며 재생은 별도 `mjData`를 사용한다.
- 일반 Linux에서의 **소프트 실시간**이다. 평균/최대 스텝 계산 시간·마감 초과 횟수·실시간 배율을 측정하며 하드 실시간을 보장하지 않는다. 최대 성능은 모델 복잡도·제어기·CPU 부하에 따라 달라진다.
- 지원 기본 구동기는 단일 hinge/slide 관절에 연결된 고정 이득·무바이어스·무내부동역학 모터다. 전달비·이득·입력/힘 제한을 반영한다. 다중 모터/차동/링크/서보/텐던 구동은 별도 어댑터가 필요하다.
- ball/free 관절을 스칼라 슬라이더로 제어하지 않는다. 지원하지 않는 제어는 모델 진단에 표시하고 입력을 0으로 둔다. 모델의 수동 동역학은 계속 존재한다.
- ROK-4 Running student v1의 ONNX 실행은 선택 빌드로 제공한다. 다른 로봇의 보행 정책, 게임패드, 기존 NPZ/HTML 기록 입력, 과거 상태에서 새 실험 분기는 포함하지 않는다.

## 검증

```bash
ctest --test-dir build --output-on-failure
./build/rclab_sim --robot profiles/hexapod.json --self-test \
  --screenshot build/hexapod.ppm
./build/rclab_sim --robot profiles/hexapod.json --benchmark-seconds 60
```

GUI self-test는 실제 ImGui 입력 경로로 그래프를 클릭하고 카메라를 움직인 뒤 JSON 결과와 종료 코드로 판정한다. 자동 검사는 데스크톱 입력 간섭을 피하는 숨김 OpenGL 창에서 실행하며 `DISPLAY`가 필요하다. `--benchmark-seconds`는 기록·실시간 그래프·렌더링을 약 60fps로 켠 채 측정한 뒤 같은 조작 검사를 실행한다. headless의 `--realtime`은 GUI 없이 벽시계 간격을 맞추는 검사다.

연구실 모델을 보유한 환경에서는 선택 검사를 추가한다.

```bash
cmake -S . -B build \
  -DRCLAB_RUBI_MODEL=/path/to/RUBI/rubi.xml \
  -DRCLAB_ROK4_MODEL=/path/to/rok_4_v4_flat.xml
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

검사 항목과 실제 측정 결과는 [검증 기록](docs/validation.md)에 적는다. GitHub Actions는 자체 제작 모델로 빌드·CTest·가상 디스플레이 GUI 검사를 실행한다.

## 저장소 구성

```text
include/rclab/   프로필·제어·구동기·기록·worker 공개 인터페이스
src/            C++ 코어, GUI, headless 실행기
assets/         메시 없이 만든 검증용 MJCF
profiles/       공용 프로필과 연구실 모델 연결 예시
examples/       C++ 제어기 연결 예시
tests/          동작·기록·호환성 검사
docs/           로봇 추가·AI 프롬프트·확장·검증
```

개인 모델 경로가 든 설정은 `profiles/local/`, 기록은 `recordings/`에 보관한다. 두 폴더와 빌드 결과, 정책 가중치, 실험 데이터, 인증 정보, 로컬 에이전트 설정·작업 일지는 `.gitignore`로 제외한다. 소스·공개 프로필·자체 제작 XML 시험 모델·문서·CI 설정을 배포한다. 외부 라이브러리와 연구실 로봇 자산의 배포 조건은 [서드파티 안내](THIRD_PARTY.md)를 확인한다.
