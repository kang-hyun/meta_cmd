# MF Robot Console

중앙 좌우 구동륜 2개 + 수동 캐스터 4개 로봇용 운용 화면입니다. 구동륜 간격은 사용자가 제공한 **0.70m**, 반지름은 기존 드라이버와 같은 **0.12m**를 사용합니다. 반지름은 실제 장비와 확인하십시오.

## 포함 기능

- 좌우 바퀴 속도, PWM, 엔코더 누적 위치 표시
- 엔코더 기반 차체 선속도·각속도 환산, 전송 목표와 비교 그래프
- 버튼/키보드를 누르는 동안만 주행 명령을 보내는 수동 조작
- 선속도 0.02~0.30m/s, 각속도 0.05~0.50rad/s 조정
- `/driver_stop` 정지 유지, `/driver_reset` 복구
- 피드백·연결 상태 확인, 이벤트 기록 및 CSV 내보내기
- ROS 토픽 값 확인, 리프트 연결 시 HOME·목표 높이·정지
- 실제 ROS를 사용하지 않는 브라우저 데모

웹에 게시한 화면은 **데모 전용**입니다. 실제 로봇의 상태나 네트워크에 연결하지 않습니다. 실제 운용은 아래 프로그램을 로봇 PC에서 실행합니다. 화면에 데모/실제 연결을 구분해서 표시합니다. `?demo=1`을 붙이면 localhost에서도 명시적으로 데모가 실행됩니다.

## 실행: 로봇 PC에서 두 터미널

Ubuntu / ROS2 Humble / Python 3.10 이상 기준입니다. 관제 브리지는 Python 표준 라이브러리와 기존 ROS2 패키지만 사용합니다. rosbridge, Node.js, pip 웹 프레임워크 설치는 필요하지 않습니다. 드라이버에는 기존처럼 pyserial이 필요합니다.

처음 받는 경우:

```bash
git clone https://github.com/kang-hyun/meta_cmd.git
cd meta_cmd
```

이미 받은 저장소가 있으면 그 폴더에서 `git pull --ff-only origin main`으로 업데이트합니다. 아래 실행 명령의 `/path/to/meta_cmd`는 실제 저장소 폴더 경로로 바꾸십시오.

| 파일 | 역할 |
|---|---|
| `ros_dashboard_bridge.py` | ROS2 토픽·서비스와 브라우저 연결 |
| `dist/` | 관제 화면 HTML·JS·CSS·아이콘 |
| `mf_mobile_driver.py`, `mf_mobile_driver_node.py` | 기존 최신 드라이버, 기본 보드레이트 921600bps |
| `README.md` | 드라이버 실행·설정 안내 |
| `DASHBOARD.md` | 관제 실행·연결 안내(이 문서) |
| `tests/test_bridge.py`, `tests/test_driver.py` | 브리지·드라이버 모의 시험 |

필요한 패키지가 없다면:

```bash
sudo apt install python3-serial ros-humble-rclpy \
  ros-humble-geometry-msgs ros-humble-std-msgs ros-humble-std-srvs
```

**터미널 1: 드라이버 실행**

```bash
source /opt/ros/humble/setup.bash
cd /path/to/meta_cmd
python3 mf_mobile_driver_node.py --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=921600 \
  -p wheel_separation:=0.70 \
  -p require_motor_feedback:=false \
  -p enable_lift:=false
```

이미 같은 드라이버를 실행 중이면 중복 실행하지 않습니다. 시리얼 포트와 반지름은 실제 장비의 값을 사용합니다.

**터미널 2: 관제 브리지 실행**

```bash
source /opt/ros/humble/setup.bash
cd /path/to/meta_cmd
python3 ros_dashboard_bridge.py
```

같은 PC의 브라우저에서 **http://127.0.0.1:8765**를 엽니다.

`/driver_status`와 시리얼 연결이 정상이고 바퀴가 멈춘 것을 확인한 뒤 수동 제어 스위치를 켭니다. 이번 드라이버는 기본 `require_motor_feedback:=false`로 모터 피드백 없이 주행할 수 있습니다. 브리지도 드라이버의 명시적 `motor_feedback_required:false` 상태를 따라 모터 피드백 없이 수동 제어를 허용합니다. 이전 드라이버나 true 설정은 피드백이 필요합니다. 드라이버와 브리지를 함께 교체하고 재시작하십시오. 최초 실행 시에는 움직이지 않습니다. ROS_DOMAIN_ID, RMW 설정, namespace는 드라이버와 맞아야 합니다.

namespace 예시:

```bash
python3 ros_dashboard_bridge.py --ros-args -r __ns:=/robot1
```

속도 토픽 이름을 변경한 경우 브리지에도 같은 remap을 적용합니다. 예:

```bash
python3 ros_dashboard_bridge.py --ros-args -r cmd_vel:=/robot1/velocity
```

브리지는 기본적으로 **127.0.0.1에만 바인딩**합니다. 외부 인터넷에 로봇 제어 포트를 공개하지 않습니다. 별도 제어 PC에서 접속해야 한다면 기존 SSH 접근 권한을 이용해 터널을 열 수 있습니다. 아래의 `robot_user`와 `ROBOT_IP`를 실제 값으로 바꿉니다.

```bash
ssh -N -L 8765:127.0.0.1:8765 robot_user@ROBOT_IP
```

터널이 열린 제어 PC에서 http://127.0.0.1:8765를 엽니다. 연결 지연이 크면 조작 신호가 만료되어 제어가 해제될 수 있습니다. 브리지 포트를 바꾸면 터널의 양쪽 포트도 맞춰야 합니다.

## 조작

| 입력 | 동작 |
|---|---|
| W / ↑ 또는 전진 버튼 누르기 | 전진 |
| S / ↓ 또는 후진 버튼 누르기 | 후진 |
| A / ← 또는 좌회전 버튼 누르기 | 제자리 좌회전 |
| D / → 또는 우회전 버튼 누르기 | 제자리 우회전 |
| 전진+좌회전 등 함께 누르기 | 선속도·각속도 동시 명령 |
| 방향 입력 놓기 / 중앙 정지 버튼 | 0속도 명령 |
| Esc / 수동 스위치 끄기 / 창 전환 | 수동 제어 해제 |
| Space / 정지 유지 버튼 | `/driver_stop` 요청 |
| 복구 버튼 | `/driver_reset` 요청; 자동 재출발 없음 |

입력창이나 설정 대화상자에서 방향키는 주행 명령으로 처리하지 않습니다. 설정/기록 창을 열 때 수동 제어를 해제합니다. 스크롤 중에도 위쪽 정지 유지 버튼을 사용할 수 있습니다.

브라우저는 활성화된 수동 제어에서 약 100ms마다 조작 신호를 보냅니다. 브리지는 50ms 타이머에서 최신 명령을 ROS에 발행합니다. 브라우저 신호가 0.35초 동안 없으면 제어 세션을 종료하고 0속도를 한 번 보낸 뒤 발행을 멈춥니다. 초기 활성화의 첫 명령에는 최대 0.5초를 허용합니다. 이전 세션 토큰·역순 명령은 거부하므로 지연된 요청이 정지 뒤 다시 출발시키지 않습니다.

수동 제어가 꺼진 동안에는 브리지가 0속도를 계속 보내지 않습니다. 따라서 단순 모니터링으로 사용할 수 있습니다. 다만 브리지의 ROS publisher는 존재하므로 노드/발행기 수에는 포함됩니다.

다른 `/cmd_vel` 발행기가 있으면 수동 활성화를 거부합니다. 수동 제어 중 다른 발행기가 감지되면 제어를 해제하고 드라이버 정지를 요청합니다. **이 검사는 0.5초 간격의 ROS 그래프 관측이며 정식 제어권 중재기가 아닙니다.** Nav2/teleop 등 기존 발행기를 먼저 중지한 후 사용하십시오. 자동·수동 모드 통합에는 별도의 명령 선택기 설계가 필요합니다.

브리지·브라우저의 정지 처리는 소프트웨어 명령입니다. 프로세스/OS 정지나 통신 단선 시 물리적 정지를 보장하지 않습니다. 드라이버의 기존 cmd_vel 타임아웃, 펌웨어 watchdog, 하드웨어 비상정지는 별도로 필요합니다.

## cmd_vel 직접 시험

관제 수동 제어는 끄고, 같은 ROS 환경의 새 터미널에서 10Hz로 보냅니다.

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

발행을 Ctrl+C로 종료하면 기본 0.5초 명령 타임아웃 뒤 0속도를 전송합니다.
`--once`보다 주기 발행으로 확인하십시오. `/cmd_vel_applied`는 전송 목표이며 실제 속도가 아닙니다.

```bash
ros2 topic echo /cmd_vel_applied
ros2 topic echo /driver_status
```

`fault`가 비어 있지 않으면 원인을 해소하고 명령 발행기를 멈춘 뒤 `ros2 service call /driver_reset std_srvs/srv/Trigger "{}"`로 복구합니다. 피드백 필수 방식으로 되돌리려면 드라이버를 `-p require_motor_feedback:=true`로 재시작합니다. 리프트 피드백 검사에는 영향이 없습니다.

## 표시값의 의미

`현재 선속도` = `(왼쪽 바퀴 rad/s + 오른쪽 바퀴 rad/s) × 반지름 / 2`

`현재 회전 속도` = `(오른쪽 바퀴 rad/s - 왼쪽 바퀴 rad/s) × 반지름 / 좌우 간격`

이는 **엔코더 기반 환산값**으로 실제 차체의 GPS/IMU 측정이 아닙니다. 미끄러짐이나 캐스터에 의한 횡방향 이탈을 측정하지 않습니다. 구성도는 실제 로봇 위치 지도나 캐스터 각도 센싱 화면이 아닙니다.

`전송 목표`는 `/cmd_vel_applied`의 값입니다. 드라이버가 저속 출발과 속도 제한을 적용한 값이며, 측정 속도와 구분합니다. PWM은 펌웨어 원시 값이고 백분율로 표시하지 않습니다.

피드백 경과 시간은 브리지의 마지막 수신 시각 기준입니다. 통신 패킷의 원래 생성 시각은 알 수 없습니다. 드라이버 상태 1.2초, ROS 그래프 확인 2초를 넘으면 수동 제어를 막습니다. 모터 피드백 0.5초 제한은 피드백 필수 모드에서만 주행을 차단합니다. 기본 모드에서는 모터 피드백 없이 주행할 수 있지만 화면에 이를 알리고 실측값은 `—`로 표시합니다. 실제 속도·물리적 정지는 확인할 수 없습니다. 신선한 모터 피드백으로 이동 중임이 확인되면 수동 제어 활성화를 거부합니다. 네트워크 응답이 끊기면 화면 값은 `—`로 바뀌고 자동으로 제어를 다시 켜지 않습니다.

상태 기록은 세션 메모리에 최대 150개 보관합니다. 브리지를 종료하면 사라지므로 필요한 기록은 CSV로 저장하십시오. `ready`/`running` 등은 드라이버의 소프트웨어 상태이고 물리적 이동을 확정하지 않습니다.

## ROS 인터페이스

| 방향 | 이름 | 타입 |
|---|---|---|
| 발행 | cmd_vel | geometry_msgs/msg/Twist |
| 구독 | motor_state | std_msgs/msg/Float64MultiArray |
| 구독 | cmd_vel_applied | geometry_msgs/msg/Twist |
| 구독 | driver_status | std_msgs/msg/String |
| 서비스 호출 | driver_stop, driver_reset | std_srvs/srv/Trigger |
| 발행 | lift_target | std_msgs/msg/Float64 |
| 구독 | lift_state | std_msgs/msg/Float64MultiArray |
| 서비스 호출 | lift_set_home, lift_stop | std_srvs/srv/Trigger |

모니터링 구독은 BEST_EFFORT/VOLATILE로 RELIABLE 또는 BEST_EFFORT 발행기의 피드백을 받을 수 있습니다. 속도 명령은 기존 드라이버에 맞는 RELIABLE 발행입니다.

서비스는 비동기로 호출합니다. `success=false`를 성공으로 표시하지 않습니다. 응답이 3초 넘게 오지 않으면 실행 여부 미확인으로 표시하고 자동 재시도하지 않습니다. 같은 서비스는 이전 결과가 확인될 때까지 중복 호출을 막습니다. HOME 성공 응답은 요청 접수이며 완료 여부는 별도의 리프트 피드백으로 확인해야 합니다.

## 리프트

기본값은 비활성입니다. 실제 장착된 기구의 허용 높이를 확인한 경우에만 드라이버의 `enable_lift`, `lift_max_height`를 설정하고 브리지의 `--lift-max-height`에도 동일한 높이(m)를 지정하십시오. 반지름·간격이 변경되었다면 브리지의 `--wheel-radius`, `--wheel-separation`도 드라이버와 맞춥니다.

브리지는 리프트 목표/HOME 요청 시 바퀴 정지·센서 유효성·피드백 시각을 확인합니다. 높이 이동은 HOME 완료 후 0~설정한 최대 높이까지만 허용합니다. 리프트 목표 토픽 발행 성공은 실제 높이 도달을 뜻하지 않습니다.

## 로컬 HTTP 경계

표준 라이브러리 HTTP 서버를 로컬 PC UI 용도로만 사용합니다. 허용된 Host/Origin, CSRF 토큰, 고정 API 경로, 본문 크기, 명령 범위를 검사합니다. 임의 토픽·임의 서비스·셸 명령을 요청하는 API가 없습니다. 프런트엔드는 폰트/CDN/외부 스크립트 없이 실행됩니다. 이 구성은 공개 인터넷용 서버가 아닙니다.

## 검증

```bash
python3 -m unittest discover -s tests -v
```

브리지의 제어 세션·타임아웃·명령 순서·피드백 조건·서비스 중복·HTTP 경계 모의 시험 35개와 드라이버의 모의 시험 49개, 총 84개를 통과했습니다. 기존 브라우저 데모에서는 버튼 누름/해제, 정지·복구, 설정·토픽·로그 화면을 확인했습니다. 실제 ROS2 런타임과 실물 로봇 통합은 검증하지 않았습니다. 배포된 데모의 동작이 실물 로봇의 성능 검증을 의미하지 않습니다.

API 참고: https://github.com/ros2/rclpy/tree/humble/rclpy/rclpy 및 https://docs.python.org/3.10/library/http.server.html
