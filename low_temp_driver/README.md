# low_temp_driver

새 저온 로봇을 위한 ROS 2 드라이버다. 사용자가 제공한 `mobile_keti_261109.zip`의
`driver/mobile_keti_driver.py`와 통신 프로토콜을 기준으로 만들었다.
`kang-hyun/meta_cmd` 저장소의 `low_temp` 브랜치 안에서 별도 폴더로 관리하며,
이 로봇의 바퀴 반지름 **0.15 m**, 휠 간격 **0.60 m**를 사용한다.

`cmd_vel`을 좌우 바퀴 목표 각속도로 변환하여 시리얼로 계속 전송하고, 수신한 모터 상태를 ROS에 발행한다.
**호스트의 출발 저속 구간, 출발 대기 시간, 가속·감속 램프는 없다.** 첫 유효 명령부터 다음 송신 주기에
계산된 목표를 그대로 보낸다. 실제 바퀴 응답에는 펌웨어와 모터 제어기의 동작이 적용된다.

## 기본값과 변경점

| 항목 | 이번 드라이버 |
|---|---|
| 기준 패키지 | `mobile_keti_261109.zip` / 2026-11 판 |
| ROS 노드 이름 | `low_temp_driver` |
| 보드레이트 | **500000**, 8N1 — 이 ZIP의 펌웨어 기준 |
| 바퀴 반지름 / 좌우 간격 | **0.15 m / 0.60 m** |
| 송신 주기 | 20 Hz, 정지 명령도 같은 주기로 전송 |
| 출발 제한·호스트 램프 | 없음 |
| 피드백 없이 명령 전송 | 가능, 피드백 유무로 목표를 바꾸지 않음 |
| 바퀴 목표 상한 | 좌우 각각 절댓값 **20 rad/s**, ZIP 기본값 유지 |
| `cmd_vel` 끊김 | 마지막 유효 명령부터 0.5 s 후 0 목표 전송 |
| 잘못된 명령 | NaN/무한대/상한 초과를 거부하고 다음 주기에 0 목표 전송 |
| 종료 | 20 ms 간격으로 0 목표를 최대 5회 시도하고 포트 닫기 |
| 통신 오류 | 0 목표 전송을 시도하고 오류 종료; 재연결은 노드 재시작 |

납품 드라이버의 반지름 `0.155`를 요청한 `0.15`로 바꿨다. 패킷 ID, CRC, 데이터 순서,
little-endian f64 형식은 그대로다. 수신 프레임 검증, 불완전한 쓰기 감지, 명령/피드백 진단 로그를 보완했다.
납품본에서는 잘못된 새 명령을 무시하고 직전 목표가 타임아웃까지 남을 수 있었으나,
이번 드라이버는 그 경우 0 목표로 바꾼다.

설정은 시작할 때 읽으며, 실행 중 변경을 거부하도록 읽기 전용으로 선언했다.
YAML 또는 실행 인자를 수정하고 노드를 재시작하면 적용된다.

## 실행 — ROS 2 Jazzy

GitHub에서 새 폴더로 받으려면:

```bash
git clone --branch low_temp --single-branch https://github.com/kang-hyun/meta_cmd.git meta_cmd_low_temp
cd meta_cmd_low_temp/low_temp_driver
```

ROS 2 Jazzy가 설치된 Ubuntu에서 아래 의존성을 설치한다.

```bash
sudo apt install python3-serial ros-jazzy-rclpy ros-jazzy-geometry-msgs ros-jazzy-std-msgs ros-jazzy-rcl-interfaces
source /opt/ros/jazzy/setup.bash
python3 low_temp_driver_node.py --ros-args --params-file config/low_temp_driver.yaml
```

ZIP을 받았다면 먼저 압축을 풀고 생성된 `low_temp_driver` 폴더에서 실행한다.
`low_temp_driver_node.py`와 `low_temp_driver.py` 두 파일을 같은 폴더에 둔다. `colcon build`는 필요 없다.

포트나 기구 값을 실행 인자로 지정할 수도 있다.

```bash
python3 low_temp_driver_node.py --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=500000 \
  -p wheel_radius:=0.15 \
  -p wheel_separation:=0.60
```

시리얼 포트는 이 드라이버 하나만 연다. 기존 로봇 드라이버를 같은 포트에 동시에 실행하지 않는다.
노드 실행만으로는 움직이지 않으며, 새로운 `cmd_vel`을 받아야 한다.

## Nav2 / teleop 연결

이 드라이버의 입력은 `geometry_msgs/msg/Twist`의 `linear.x`와 `angular.z`다.
Nav2의 최종 속도 출력 또는 teleop 출력을 `cmd_vel`로 연결한다. 다른 토픽이면
`-p cmd_vel_topic:=/원하는_토픽`으로 지정한다. 실제 형식은 `ros2 topic type /cmd_vel`로 확인한다.
`TwistStamped`를 사용하는 구성에서는 발행 측을 `Twist`로 맞추거나 별도 변환 노드를 사용해야 한다.

명령은 10 Hz 이상으로 계속 발행한다. Nav2와 teleop이 동시에 같은 토픽에 쓰면
마지막으로 받은 명령이 적용되므로, 하나의 명령원만 사용하거나 외부 mux로 선택한다.
드라이버의 20 Hz 재전송은 명령 발행 주기와 별개이며, 새 명령이 끊기면 0.5 s 후 0을 보낸다.

처음에는 바퀴를 띄우거나 주변을 비우고 낮은 속도로 방향과 정지 응답을 확인한다.
다음은 다른 명령 발행기가 없는 상태에서의 간단한 통신 시험이다.

```bash
# 별도 터미널
source /opt/ros/jazzy/setup.bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
```

위 발행기를 Ctrl-C로 종료하면 명령 타임아웃 후 0 목표를 보낸다.
0 명령을 직접 시험하려면 먼저 이동 명령 발행기를 종료하고 아래를 실행한다.

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

## ROS 입력과 출력

아래 이름은 네임스페이스 없이 실행했을 때 기준이다.

| 방향 | 토픽 / 타입 | 값과 단위 |
|---|---|---|
| 입력 | `/cmd_vel` / `geometry_msgs/msg/Twist` | `linear.x`: m/s, `angular.z`: 차체 rad/s, 반시계 + |
| 출력 | `/wheel_command` / `std_msgs/msg/Float64MultiArray` | 시리얼 write가 완료된 `[좌 목표, 우 목표]`, 바퀴 rad/s |
| 출력 | `/motor_state` / `std_msgs/msg/Float64MultiArray` | `[좌 속도, 좌 PWM, 좌 위치, 우 속도, 우 PWM, 우 위치]` |

`motor_state`의 속도는 rad/s, PWM은 0~1 크기, 위치는 부팅 후 누적 rad다.
PWM은 전류나 토크가 아니다. 피드백 패킷을 받았을 때만 상태를 발행하며,
수신이 끊겨도 마지막 값을 새 데이터처럼 반복 발행하지 않는다.
프로토콜상 상태 송신은 50 Hz다. 호스트가 20 Hz로 묶어 읽으므로 ROS 메시지가 묶음으로 도착할 수 있다.

`wheel_command`는 **전송 목표**이며 실제 회전 속도나 보드의 수신 확인이 아니다.
실제 속도는 `motor_state`에서 확인한다. 피드백에 실제로 들어온 0을 보정하거나 숨기지 않는다.

```bash
ros2 topic echo /wheel_command
ros2 topic echo /motor_state
ros2 topic hz /motor_state
ros2 topic info /cmd_vel -v
```

1초마다 출력하는 `IO` 로그는 다음처럼 구분해서 읽는다.

| 로그 | 의미 |
|---|---|
| `cmd_rx=0`, `reason=waiting_cmd_vel` | ROS 명령을 아직 못 받음 |
| `cmd_rx` 증가, `rejected` 증가 | 비정상 수치 또는 바퀴 속도 상한 초과 명령 |
| `TX_wheels=(...)`, `tx_packets` 증가 | PC에서 시리얼 write 완료; 실제 모터 동작 확인은 아님 |
| `rx_bytes=0`, `feedback_age=never` | 시리얼 수신 데이터 없음 |
| `rx_bytes`만 증가, `motor_packets`는 정지 | 유효한 모터 상태로 해석되지 않음; baud/펌웨어/프레임 확인 |
| `motor_packets` 증가, `feedback_age`가 작음 | 유효한 모터 상태 수신 중 |
| `bad_frames` 증가 | 길이·CRC·상태 형식·비정상 수치 오류 감지 |
| `reason=cmd_vel_timeout` | 명령 갱신이 끊겨 0 목표 전송 중 |

`cmd_age`는 마지막 유효 명령의 경과 시간이다. 잘못된 새 명령을 받으면 해당 목표를 지우므로 `never`가 표시될 수 있다.

## 속도 단위와 제한

```text
left_rad_s  = (linear_x - angular_z * wheel_separation / 2) / wheel_radius
right_rad_s = (linear_x + angular_z * wheel_separation / 2) / wheel_radius
```

반지름 0.15 m, 간격 0.60 m일 때:

| ROS 명령 | 좌 목표 | 우 목표 |
|---|---:|---:|
| 직진 0.30 m/s | 2.0 rad/s | 2.0 rad/s |
| 직진 0.60 m/s | 4.0 rad/s | 4.0 rad/s |
| 제자리 반시계 1.0 rad/s | -2.0 rad/s | +2.0 rad/s |
| 직진 0.30 m/s + 반시계 0.50 rad/s | 1.0 rad/s | 3.0 rad/s |

바퀴 20 rad/s는 반지름 0.15 m에서 직진 환산 **3.0 m/s**다. 이는 수학적 환산이며,
실기에서 확인한 주행 가능 속도나 권장 운용 속도가 아니다.
직진과 회전을 함께 요청하면 한쪽 바퀴가 먼저 상한에 걸릴 수 있다.
상한 초과 시 비율 축소나 잘라내기를 하지 않고 명령을 거부하고 0을 보낸다.
Nav2/teleop의 운용 속도는 해당 프로그램에서 설정한다.

## 파라미터

| 이름 | 기본값 | 의미 |
|---|---:|---|
| `port` | `/dev/ttyUSB0` | 시리얼 장치 |
| `baudrate` | `500000` | 보드 펌웨어와 일치시킬 값 |
| `wheel_radius` | `0.15` | m |
| `wheel_separation` | `0.60` | 좌우 구동륜 간격 m |
| `max_wheel_speed` | `20.0` | 각 바퀴 목표의 절댓값 상한 rad/s |
| `control_rate` | `20.0` | 주기 송신 Hz, 최소 5.0 |
| `cmd_vel_timeout` | `0.5` | 명령 갱신 타임아웃 s |
| `cmd_vel_topic` | `cmd_vel` | 입력 토픽 |
| `motor_state_topic` | `motor_state` | 실제 수신한 상태 출력 |
| `wheel_command_topic` | `wheel_command` | 시리얼 전송 목표 출력 |
| `status_period` | `1.0` | 진단 로그 주기 s, 0.0이면 주기 로그 끄기 |

호스트 송신 타이머와 타임아웃은 실제 경과 시간을 사용한다.
ROS의 `/clock`이 정지해도 명령 타임아웃과 0 목표 전송은 계속된다.

## 펌웨어가 맡는 동작

제공된 [프로토콜 원문](docs/VENDOR_PROTOCOL.md)에 따르면 보드 내부에는 약 **24.516 rad/s²**의
가감속 제한이 있다. 호스트의 출발 제한을 없애도 이 값은 그대로다.
프로토콜 원문은 반지름 0.155 m를 기준으로 설명하며, 이 프로젝트는 사용자의 실측 반지름 0.15 m로
ROS 속도를 변환한다. 호스트 반지름 설정을 바꿔도 보드 내부 게인이 바뀌지는 않는다.

또한 원문에는 10초 이상 정지 후 모터 연결을 다시 맺는 동안 약 3초간 출발 명령에 반응하지 않을 수 있다고
명시되어 있다. 이 동작을 바꾸려면 펌웨어 쪽 수정이 필요하다.

0 속도 명령은 능동 브레이크 명령이 아니다. 보드 내부 감속과 모터 제어기의 중립 동작을 거치므로,
실제 정지 거리·시간은 실기에서 확인해야 한다. 시리얼 명령이 끊겼을 때의 250 ms watchdog은
제공된 해당 펌웨어의 기능이며, 이 호스트 테스트가 실제 바퀴의 250 ms 내 정지를 보장하지 않는다.

이 저장소는 드라이버와 문서·검사 코드로 구성한다. 보드 펌웨어 이미지는 원래 제공된 ZIP에서 관리하며,
이 드라이버는 게인 변경(`0x03`)이나 보드 리셋(`0xFF`)을 자동 전송하지 않는다.

## 검사

ROS나 로봇 없이 실행할 수 있다.

```bash
python3 -m unittest discover -s tests -v
python3 -m py_compile low_temp_driver.py low_temp_driver_node.py tests/test_driver.py
```

검사는 제공 문서의 패킷 바이트 일치, 조각 수신·잡음·CRC 오류 복구, 실제 0 피드백 보존,
기구학 변환, 첫 명령의 제한 없는 반영, 주기 재전송, 타임아웃, 잘못된 명령 처리,
포트 오류·불완전 쓰기·종료 시 0 전송을 다룬다.
ROS 콜백은 모의 객체로 검사한다. 실제 Jazzy의 DDS 통신, USB 시리얼, 로봇 주행은 이 환경에서 실행하지 않았다.

## 원본 추적

| 항목 | 값 |
|---|---|
| 원본 ZIP | `mobile_keti_261109.zip` |
| 원본 ZIP SHA-256 | `451fd0daa9531b582729bfbc113f383cea253e5818f2e849bd4cc403a32c73af` |
| 원본 드라이버 SHA-256 | `a257ae28bacc269af6424c6294dd3f24d8975a5c0f85cdce1388468676feffe7` |
| 프로토콜 원문 SHA-256 | `b729a9d5c8c598e265125df7916b95977007611142ce611c4972481476c4b72a` |
| 원본이 가리키는 펌웨어 커밋 | `1f75b6442525a39faea7fb70110b3fde97a77e6e` |

`docs/VENDOR_PROTOCOL.md`는 원본 그대로 보관했다. 원본의 `mobile_keti_driver.py`·노드 이름·반지름 등은
납품본 설명이며, 이 프로젝트 실행 방법과 기본값은 이 README를 따른다.
원본 ZIP에 별도의 라이선스 파일은 없으며, 재배포 권한을 새로 부여하는 라이선스는 추가하지 않았다.
