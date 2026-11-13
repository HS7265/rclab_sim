# 외부 소스와 자산

| 구성 요소 | 사용 버전 | 라이선스 / 출처 |
|---|---|---|
| MuJoCo | 3.11.0 | Apache-2.0, https://github.com/google-deepmind/mujoco |
| Dear ImGui | 1.89.9 | MIT, https://github.com/ocornut/imgui |
| ImPlot | 0.16 | MIT, https://github.com/epezent/implot |
| GLFW | 시스템 3.3 이상 | zlib/libpng, https://github.com/glfw/glfw |
| nlohmann/json | 시스템 3.10 이상 | MIT, https://github.com/nlohmann/json |
| ONNX Runtime (선택) | 1.22.0 | MIT, https://github.com/microsoft/onnxruntime |
| OpenSSL (선택) | 시스템 3.0 | Apache-2.0, https://www.openssl.org/ |

외부 라이브러리 소스는 이 저장소에 복사하지 않고 시스템 설치 또는 CMake 다운로드로 연결한다. 바이너리를 재배포할 때는 해당 배포본의 저작권·라이선스 고지도 포함해야 한다.

`assets/`의 단순 형상 예제는 이 프로젝트에서 제작했다. RUBI·ROK-4 모델과 메시·정책은 포함하지 않는다. 이를 별도로 배포할 때는 해당 자산 소유자의 배포 조건을 확인해야 한다.

기존 시뮬레이터는 기능과 사용 흐름을 참고했다. `src/rok4_student.cpp`의 관측·상태 기계·행동 제한·구동기 계약은 연구실 `rok_4_running/training/sim/rok4_mujoco_sim/src/rok4/rok4_core.cpp`를 기반으로 공통 확장 인터페이스에 이식했다. 원본 모델·정책·참조 코어는 저장소에 포함하지 않는다. 선택 대조 테스트만 외부 참조 소스를 직접 컴파일한다. 외부 공개 전 연구실 파생 제어 코드의 배포 권한도 확인해야 한다.
