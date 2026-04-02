# ui_ads_ver

`ui_ads_ver`는 ADS 로봇 상태를 ROS 2 topic으로 받아서 ImGui 기반 데스크톱 UI로 보여주는 패키지입니다.

이 패키지는 다음 정보를 실시간으로 표시합니다.

- 현재 joint reference 값과 actual 값 비교
- joint velocity / effort 요약
- motor reference 값과 actual 값 비교
- flat array 형태의 ADS 상태에서 tip position 요약
- joint별 reference-vs-actual 그래프
- TEST mode motor / amplitude / period 조절

특히 `ADS2ROS` 워크스페이스에서 ADS 런타임이 publish하는 아래 topic들과 함께 사용하기 좋도록 구성되어 있습니다.

- `/joint_states`
- `/joint_states_ref`
- `/motor_states`
- `/motor_states_ref`
- `/ads/mju_state`
- `/ads/test_config_state`
- `/ads/test_config_cmd`

## 주요 기능

- `rclcpp` 기반 C++ ROS 2 노드
- Dear ImGui + GLFW + OpenGL2 기반 UI
- 한글/영문 가독성을 고려한 시스템 폰트 적용
- ROS parameter로 topic 이름 변경 가능
- TEST config 명령 publish 지원
- `ESC` 키로 창 종료 가능

## 패키지 구조

```text
ui_ads_ver/
├── includes/ui_ads_ver/ads_state_ui.hpp
├── src/ads_state_ui.cpp
├── src/main.cpp
├── CMakeLists.txt
└── package.xml
```

## 요구 사항

- Ubuntu 22.04
- ROS 2 Humble
- OpenGL
- GLFW 3
- 로컬에 설치된 Dear ImGui 소스

이 패키지는 Dear ImGui를 내부에 포함하지 않습니다.
대신 `IMGUI_PATH` 환경변수 경로를 사용하며, 없으면 아래 경로를 기본값으로 사용합니다.

```bash
$HOME/.ImGUI/imgui
```

해당 경로에는 아래 파일들이 있어야 합니다.

- `imgui.h`
- `imgui.cpp`
- `imgui_draw.cpp`
- `imgui_tables.cpp`
- `imgui_widgets.cpp`
- `backends/imgui_impl_glfw.cpp`
- `backends/imgui_impl_opengl2.cpp`

## 환경 설정

이미 `~/.zshrc`에 아래 설정이 있다면 그대로 사용하면 됩니다.

```bash
export IMGUI_PATH=$HOME/.ImGUI/imgui
```

설정을 반영하려면:

```bash
source ~/.zshrc
```

## 빌드

워크스페이스 루트에서 실행합니다.

```bash
cd /home/drcl-rss/Music/test/ADS2ROS
source /opt/ros/humble/setup.zsh
colcon build --packages-select ui_ads_ver
source install/setup.zsh
```

## 실행

기본 topic을 사용할 경우:

```bash
ros2 run ui_ads_ver ads_state_ui
```

topic 이름을 직접 지정하고 싶다면:

```bash
ros2 run ui_ads_ver ads_state_ui --ros-args \
  -p joint_state_topic:=/joint_states \
  -p joint_state_ref_topic:=/joint_states_ref \
  -p motor_state_topic:=/motor_states \
  -p motor_state_ref_topic:=/motor_states_ref \
  -p mju_state_topic:=/ads/mju_state \
  -p test_config_state_topic:=/ads/test_config_state \
  -p test_config_cmd_topic:=/ads/test_config_cmd
```

## 사용 topic

### `/joint_states`

- 타입: `sensor_msgs/msg/JointState`
- actual joint state로 사용

### `/joint_states_ref`

- 타입: `sensor_msgs/msg/JointState`
- reference joint state로 사용

### `/ads/mju_state`

- 타입: `std_msgs/msg/Float64MultiArray`
- 추가 ADS 상태 데이터 표시용으로 사용
- flat array에서 tip position 정보를 읽어옵니다

### `/ads/test_config_state`

- 타입: `std_msgs/msg/Float64MultiArray`
- 현재 적용된 TEST config 상태 표시용으로 사용

### `/ads/test_config_cmd`

- 타입: `std_msgs/msg/Float64MultiArray`
- UI에서 TEST motor / amplitude / period를 변경할 때 publish

## 조작 방법

- `ESC`: 창 종료
- 좌측 `테스트 명령` 패널에서 motor / amplitude / period를 설정한 뒤 `테스트 설정 적용`
- `현재 적용값 불러오기`를 누르면 현재 로봇에 적용된 TEST 설정을 편집창으로 다시 가져옵니다

## 참고 사항

- UI는 `joint1`, `joint2`, `joint3`, `joint4` 이름의 4개 joint를 기준으로 동작합니다.
- `JointState.name` 순서가 정확히 일치하지 않으면 첫 4개 항목을 사용하고 경고를 표시합니다.
- 폰트는 시스템 폰트를 사용하며, 현재는 `Noto Sans CJK KR`를 우선 사용하고 없으면 `DejaVu Sans`로 대체합니다.
- TEST 입력은 UI에서 `period [s]`로 편집하고, 내부 publish 시 `frequency [Hz]`로 변환합니다.

## 문제 해결

### ImGui 경로를 찾지 못하는 경우

아래 명령으로 환경변수와 경로를 확인합니다.

```bash
echo $IMGUI_PATH
ls $IMGUI_PATH
```

### 그래프에 데이터가 보이지 않는 경우

topic이 실제로 publish되고 있는지 확인합니다.

```bash
ros2 topic list
ros2 topic echo /joint_states
ros2 topic echo /joint_states_ref
ros2 topic echo /ads/mju_state
ros2 topic echo /ads/test_config_state
```

### GLFW / OpenGL 관련 빌드 문제가 있는 경우

`pkg-config`가 GLFW를 찾는지 확인합니다.

```bash
pkg-config --modversion glfw3
```

## 라이선스

MIT
