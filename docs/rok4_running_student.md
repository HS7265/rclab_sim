# ROK-4 Running student 실행 — 2026-11-13

## 선택한 정책

Running student의 최신 학습본으로 다음 번들을 선택했다. 경로는 외부 `rok_4_running/training/` 기준이다.

| 항목 | 선택값 |
|---|---|
| 체크포인트 | `runs/run/student/student_25396_resume120_s24_b64e8/student_0400.pt` |
| 학습 | teacher 25396에서 증류한 student, 최종 round 400 |
| 체크포인트 SHA256 | `0c08728510cd6b56961b8d6de3edf686e141485c76d2cf51b0b52c117ba915b5` |
| ONNX 번들 | `exports/run/onnx_student_25396_s24_round400/` |
| 학습용 장면 | `sim/rok4_mujoco_sim/model/rok4/xml/rok4_scene_mimic.xml` |
| 근거 일지 | `ai/journal/Oct/4주차/2026-10-23.md`의 18:53 항목 |

student 실행 폴더별 체크포인트 번호를 숫자로 비교하고 실제 저장 시각도 확인했다. 11월의 새 달리기 게인·보간 실험은 teacher이므로 최신 student로 대체하지 않았다. 평가용 미학습 preview도 제외했다.

## 필수 실행 계약

- 정책: 관측 **81**, 행동 **21**, GRU hidden **[1, 1, 256]**. hidden은 정책 스텝 사이 유지하고 에피소드 리셋에서 초기화한다.
- 정책 주기 **10ms**, 물리 주기 **2ms**, decimation **5**.
- `motor_state.onnx`의 관절↔모터 좌표 변환과 `actuator.onnx`의 내부 상태·지연·토크 응답을 함께 사용한다.
- 구동기 RNN과 정책 GRU는 별도 상태다. 구동기 RNN 갱신 간격은 2물리 스텝, 기본 지연은 1물리 스텝이다.
- 하체 모터 응답은 6/6/8.5/9/8.3/8.3ms, 무릎 모터 P100/D7이다. 다른 모터 게인·한계·초기 상태도 메타데이터를 따른다.
- 팔 확장 행동 범위와 전환 메타데이터를 유지한다. 모든 행동을 임의로 ±1로 잘라서는 안 된다.
- 명령은 기존 상태 기계의 정지/달리기 요청이다. 임의 속도·횡이동·회전이 학습된 정책으로 해석하지 않는다.
- 몸통은 `robot/Base_Link`, 관절도 `robot/` 접두어가 있다. 기존 기본 프로필의 `Base_Link`와 배열 주소를 재사용하지 않는다.

필요 파일은 `policy.onnx`, `motor_state.onnx`, `actuator.onnx`, **`actuator.onnx.data`**, `rok4_meta.json`, `actuator_meta.json`, `reference_tables.json`이다. `student_export.json`은 출처·해시·내보내기 검증 근거로 보관한다.

## 이번에 수행한 준비와 검증

- 번들의 위 7개 파일, student 체크포인트, teacher 체크포인트, 원본 reference 2개의 해시를 내보내기 manifest와 대조했다. 모두 일치했다.
- 기존 **참조 C++ headless**에서 30초를 실행했다. 1초에 달리기 요청, 11초에 정지 요청, lag 1 조건이다.
- 3,000정책 스텝·15,000물리 스텝 기록의 길이와 전체 유한값을 검사했다.
- 별도 1초 실행에서 reset 후 첫 관측·행동·GRU hidden 재현 및 replay 초기화 검사가 PASS였다.
- 새 시뮬레이터에서도 학습용 장면을 로드하고 28 qpos / 27 qvel / 21 actuator 주소를 확인했다. **이 검사는 passive 모델 검사이며 student 구동 검사가 아니다.**

| 참조 실행 결과 | 측정값 |
|---|---:|
| RUN 평균 전진 속도 | 1.863m/s |
| 최종 몸통 높이 | 0.924312m |
| 기록 중 최소 몸통 높이 | 0.863672m |
| 최대 몸통 기울기 | 14.5도 |
| 기존 실행기의 첫 이상 판정 | 없음 |
| 정책 스텝 평균 / p99 / 최대 | 6.61 / 9.58 / 16.47ms |

첫 이상 판정은 기존 실행기의 낮은 몸통/추종 보호 조건이다. 이 단일 공칭 실행을 전체 DR 성능이나 완전한 걸음새 검증으로 확대하지 않는다. 이번 실행은 벽시계 간격을 강제하지 않았다. 최대 계산 시간이 10ms 예산을 넘으므로 새 앱 연결 후 기록·렌더링을 함께 켜고 실시간 성능을 다시 검증해야 한다.

## 로컬 준비 파일

다음 파일은 개인 경로를 포함하므로 Git에서 제외되는 위치에 만들었다.

- `profiles/local/rok4-running-student.selection.json`: 선택 checkpoint·번들·scene 경로, 해시, 실행 계약. 실행용 RobotProfile이 아닌 자산 선택 manifest다.
- `profiles/local/rok4-student-inspection.json`: 학습용 장면과 초기 관절 자세를 확인하는 **passive** 프로필. 공중 고정 상태에서 모델을 검사하며 정책을 실행하지 않는다.
- `build/rok4-student-preflight/`: 참조 실행 기록, 몸통 궤적 CSV, 수치 요약 JSON, reset 검사 출력.

모델 검사만 실행하려면:

```bash
./build/rclab_sim --robot profiles/local/rok4-student-inspection.json
```

모델·메시·정책 파일을 프로젝트에 복사하거나 원본을 다시 export하지 않았다.

## C++ 실행 연결

`RCLAB_WITH_ONNX=ON`으로 빌드하면 `rok4_running_student` control stack을 등록한다. ONNX Runtime 1.22.0 C++ SDK와 OpenSSL 개발 패키지가 필요하다. 기본 OFF 빌드는 기존 PD 시뮬레이터만 빌드한다.

```bash
cmake -S . -B build -DRCLAB_WITH_ONNX=ON \
  -DONNXRUNTIME_ROOT=/path/to/onnxruntime-linux-x64-1.22.0
cmake --build build -j2
./build/rclab_sim --robot profiles/local/rok4-running-student.json
```

`profiles/rok4-running-student.example.json`을 `profiles/local/`에 복사하고 `model`과 `extension.bundle`을 자신의 경로로 수정한다. 번들은 외부 파일을 직접 읽으며 시작할 때 manifest의 7개 SHA256을 검사한다. `extension.lag_substeps` 기본값은 1, 허용 범위는 0~3이다. 학습용 scene의 2ms 간격과 프로필의 10ms 제어 주기를 검사한다. 메타데이터가 다른 teacher·walking·jumping 정책은 거부한다.

GUI 사용 순서:

1. `실행`으로 물리를 실행한다. 초기 명령은 STAND다.
2. `로봇 명령 → 달리기`을 선택하면 가속 후 달린다.
3. 선택을 해제하면 지정된 걸음 위상에서 감속을 시작하고 정지한다. `일시정지`는 즉시 물리 시간을 멈춘다.
4. `기록 시작` 후 `기록 종료 후 분석`을 누르면 기록을 분석한다. 그래프 채널에서 `student/` 항목을 선택한다.
5. `초기화`은 초기 자세, 정책 hidden, 구동기 RNN·열·필터·지연, 상태 기계를 함께 초기화한다.

수동 관절 목표·PD 게인 편집은 비활성화한다. 정책은 결합 구동기의 **모터 좌표 목표**를 생성하므로 이를 관절 목표로 표시하지 않는다. 공중 고정은 검사에 사용할 수 있지만 떠 있는 상태는 학습된 정상 달리기 조건과 다르다.

화면 없이 같은 달리기/정지 시나리오를 실행하려면:

```bash
./build/rclab_headless --robot profiles/local/rok4-running-student.json \
  --duration 30 --run-start 1 --run-stop 11 --record recordings/student400.rclog
```

파일 이름은 새 이름을 사용한다. 기존 기록을 덮어쓰지 않는다. 실행 시간은 시뮬레이션 시간이며 벽시계 간격을 맞추려면 `--realtime`을 추가한다.

## 진단 채널과 시각

- `command/run_requested`: 사용자가 보낸 달리기 요청(0/1). 실제 속도 명령이 아니다.
- `student/mode`, `progress`, `run_command`: 관측에 쓰인 STAND/ACCEL/RUN/DECEL과 진행도·요청. 새 요청은 참조 구현처럼 해당 10ms 블록 적분 뒤 상태 기계에 반영한다.
- `student/<canonical joint>/motor_position`, `motor_velocity`: 해당 물리 스텝 직전 관절 상태를 motor_state ONNX로 변환한 실제 모터 좌표(rad, rad/s).
- `motor_target`, `delayed_motor_target`: 현재 정책 목표와 지연 버퍼를 통과해 이번 구동기 추론에 입력한 목표(rad).
- `motor_torque`, `joint_torque`: 구동기가 산출한 실제 모터 응답 상태와 관절에 전달할 토크(N·m). 결합 4축은 구동기의 applied 상태에서 읽는다.
- `raw_action`, `filtered_action`: 원시 정책 행동과 메타데이터의 전환별 범위를 적용한 행동. 무차원이다.
- `student/left_foot_force_z`, `right_foot_force_z`: **이번 구동기 입력에 사용한 직전 mj_step 센서값**이다. 공통 접촉력 채널은 현재 기록 시각에서 forward로 계산한 값이므로 두 채널의 시점·의미를 구분한다.
- `hidden_norm`, `tracking_latched`, `policy_ms`, `actuator_ms`: 현재 정책 hidden의 크기·보호 상태와 최근 개별 추론 시간이다. 전체 계산 비용은 고급 정보로 확인한다.

`extension.full_policy_diagnostics: true`는 관측 81개·정책 hidden 256개를 추가 기록한다. 대조 검사에 사용하며 일반 실행은 false로 두어 기록량을 줄인다. hidden 채널은 각 10ms 추론 직후 값이고 물리 서브스텝 사이 유지된다.

물리는 공통 Simulation만 적분한다. 정책 갱신 전에 임의의 forward를 추가하지 않아 참조 실행기의 마지막 물리 계산에서 남은 몸통 파생값과 최신 관절값 조합을 유지한다. 기록 직전 forward는 공통 힘 진단을 정렬한다. 21축 이름·구동기 수식은 로봇 확장 내부에 있다.

기록에는 번들 해시·지연·주기·student 메타데이터와 채널 구성을 저장한다. 다른 번들 또는 설정으로 열면 오류를 낸다. 재생은 별도 mjData에서 자세만 복원하며 ONNX를 다시 실행하지 않는다. 내부 정책 상태를 복원해 과거에서 새 실험을 시작하는 기능은 제공하지 않는다.

## 통합 검증

외부 참조 소스가 있는 로컬 환경에서는 아래 CMake 옵션으로 선택 대조 검사를 켠다. 공개 CI는 외부 자산 없이 기본 모델 검사를 실행한다.

```bash
cmake -S . -B build \
  -DRCLAB_ROK4_REFERENCE_ROOT=/path/to/rok4_mujoco_sim \
  -DRCLAB_ROK4_STUDENT_PROFILE=/path/to/local-student-profile.json \
  -DRCLAB_ROK4_STUDENT_BUNDLE=/path/to/onnx-student-bundle
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

2026-11-13 실제 결과:

- ONNX 선택 빌드 CTest **8/8 PASS**, ONNX 없는 별도 headless 빌드 **5/5 PASS**.
- 4.5초·2,250물리 스텝에서 참조 코어와 qpos·qvel·관측81·hidden256·행동21·관절 토크의 최대 차이가 **모두 0**이었다. 초기 정지·가속·달리기 진입을 포함한다.
- Reset 후 첫 관측·hidden·구동기 토크가 참조와 동일했다.
- 30초·15,000표본에서 전체 진단값이 유한했고, 달리기 후 최종 STAND를 확인했다. 최소 높이 **0.863650m**, 최종 **0.924312m**, RUN 동안 세계 x 속도 평균 **1.86395m/s**였다.
- 바이너리 저장/열기 후 전체 행이 동일했다. 6.1234초 선택은 최근접 6.124초 표본으로 이동했으며 실제 시뮬레이션 자세는 보존됐다. 제어 계약 메타데이터가 다른 기록은 거부했다.

위 대조는 이 PC의 MuJoCo 3.11.0·ONNX Runtime 1.22.0에서 수행했다. 다른 하드웨어/버전에서 같은 비트 수준 일치를 보장하는 의미는 아니다.

### GUI·기록 동시 성능 측정

실제 OpenGL 창을 숨김 모드로 생성해 60초 벽시계 동안 달리기 요청을 유지하며 기록·그래프·렌더링을 함께 실행했다. `--benchmark-seconds 60 --screenshot ...` 검사에는 ImGui 마우스 입력으로 그래프 표본 선택과 카메라 회전을 수행하는 절차가 포함된다.

| 항목 | 결과 |
|---|---:|
| 기록 표본 / 물리 진행 시간 | 29,312 / 58.624초 |
| 실시간 배율 | 0.9768배 |
| 물리 스텝 계산 평균 / 최대 | 1.525 / 4.018ms |
| 물리 주기 deadline 초과 | 3,601회 / 29,312스텝 |
| GUI 프레임 | 3,603 |
| 그래프 클릭 → 최근접 표본 | PASS |
| 카메라 조작 중 재생 시각 유지 | PASS |
| 3D 몸통 자세와 기록 일치 | PASS |

정책 추론이 포함된 스텝은 2ms 물리 주기를 넘을 수 있다. 이 PC에서도 deadline 초과가 있으므로 하드 실시간 실행으로 사용하지 않는다. 측정값은 CPU ONNX Runtime 단일 스레드와 GUI를 함께 쓴 소프트 실시간 결과다. 화면 캡처와 수치 JSON은 로컬 `build/rok4-student-running.png`, `build/rok4-student-gui-benchmark.json`에 있다.
