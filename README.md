# MF 모바일 드라이버 — 중앙 구동륜 + 수동 캐스터 보완본

작성일: 2026-10-29

**중앙 좌·우 구동륜 2개 + 앞뒤 수동 회전 캐스터 4개** 구성의 ROS2 드라이버입니다. 차동구동 식은 그대로 사용하며, 빨간 캐스터에 제어 명령을 보내지는 않습니다.

이번 버전은 출발·방향 전환 때의 명령 변화를 완화합니다. 캐스터 각도, 차체 자세, 실제 이동 경로를 측정하는 기능은 없습니다. **캐스터 정렬 완료를 감지하거나 초기 방향 틀어짐을 완전히 없애는 코드가 아닙니다.** IMU/GPS 기반 방향·경로 보정도 추가하지 않았습니다. 실제 로봇에서의 개선 정도는 검증되지 않았습니다.

## 1. 파일과 적용 방법

| 파일 | 역할 |
|---|---|
| `mf_mobile_driver.py` | 기존 MFPacket 통신 + 출발/가감속/역회전 프로파일 |
| `mf_mobile_driver_node.py` | ROS2 연결, 피드백 감시, 장애·복구, 리프트 처리 |
| `CHANGELOG.md` | 기존 코드와의 변경점 및 검증 범위 |
| `tests/test_driver.py` | ROS2·실물 장치 없이 실행하는 모의 시험 |
| `tests/fixtures/command_packets.json` | 원본 드라이버에서 추출한 송신 패킷 기준값 |

기존 노드를 종료하고 **두 Python 파일을 함께** 교체하십시오. 두 파일은 같은 디렉터리에 있어야 합니다. 같은 포트를 사용하는 다른 프로그램을 함께 실행하지 마십시오. 이후 변경 이력은 Git으로 관리합니다.

기존 설치 조건을 유지합니다: Python 3.10+, ROS2 Humble, `pyserial`, `rclpy`, `geometry_msgs`, `std_msgs`, `std_srvs`. `mf_mobile_base`, `mf_msgs`는 필요하지 않습니다.

```bash
sudo apt install python3-serial ros-humble-rclpy \
  ros-humble-geometry-msgs ros-humble-std-msgs ros-humble-std-srvs
```

포트 접근 권한이 없다면 `sudo usermod -aG dialout $USER` 후 다시 로그인합니다.

```bash
source /opt/ros/humble/setup.bash
python3 mf_mobile_driver_node.py --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=921600 \
  -p require_motor_feedback:=false \
  -p enable_lift:=false \
  -p wheel_radius:=0.12 \
  -p wheel_separation:=0.70
```

**좌우 구동륜 간격은 사용자가 알려준 0.70m를 반영했습니다. 반지름 0.12m는 기존 코드의 값이며 아직 실측 확인이 필요합니다.** `wheel_radius`는 하중이 실린 흰색 구동륜의 유효 반지름, `wheel_separation`은 좌·우 흰색 구동륜 중심 간 거리입니다. 앞뒤 빨간 캐스터 간격을 넣는 값이 아닙니다.

**기본 보드레이트는 실제 로봇에서 사용자가 동작을 확인한 921600bps입니다.** `python3 mf_mobile_driver_node.py`만 실행해도 이 값이 적용됩니다. 다른 통신 속도가 필요한 경우 `-p baudrate:=값`으로 지정하십시오.

**이번 수정본의 기본값은 `require_motor_feedback:=false`입니다.** 시리얼 연결 후에는 모터 피드백이 한 번도 없거나 중단되어도 새 `cmd_vel`을 받아 속도 명령을 전송합니다. `/motor_state`를 가짜로 만들지 않으므로 실측값은 비어 있을 수 있습니다. 기존 피드백 필수 동작은 실행 시 `-p require_motor_feedback:=true`로 선택합니다. 이 설정은 시작 시 적용되므로 변경하려면 노드를 재시작하십시오.

## 2. 출발·정지·전후진 동작

### 실행되지만 움직이지 않을 때

`Serial connected`는 PC에서 포트를 열었다는 뜻입니다. 로봇의 응답이나 실제 이동을 확인한 메시지는 아닙니다. 노드만 실행하면 0속도를 전송하며, 별도 발행기에서 속도 명령을 계속 보내야 움직입니다.

터미널 1에서 기본 실행만 해도 1초마다 진단 요약이 나옵니다. 원본 송수신 HEX까지 보려면:

```bash
python3 mf_mobile_driver_node.py --ros-args -p debug_serial:=true
```

같은 ROS 환경의 터미널 2에서:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

`--once`나 기본 1Hz 발행은 0.5초 타임아웃과 출발 대기 때문에 출발이 반복 중단될 수 있습니다. 진단 편의를 위해 타임아웃을 없애거나 노드 시작 시 자동 주행을 넣지는 않았습니다.

예시 출력(아래 숫자는 설명용):

```text
IO cmd_rx=0 accepted=0 rejected=0 cmd_age=never TX_wheels=(0.000,0.000)rad/s tx_packets=20 rx_bytes=0 motor_packets=0 bad_frames=0 reason=waiting_cmd_vel
IO cmd_rx=30 accepted=30 rejected=0 cmd_age=0.05s TX_wheels=(0.833,0.833)rad/s tx_packets=60 rx_bytes=0 motor_packets=0 bad_frames=0 reason=nonzero_command_sent
```

| 표시 | 의미 및 다음 확인 |
|---|---|
| `cmd_rx=0`, `waiting_cmd_vel` | 이 노드가 명령을 받지 못함. 시작 로그의 실제 구독 토픽, 메시지 타입, 발행기, ROS_DOMAIN_ID 확인 |
| `rejected>0`, `reason=rejected:...` 또는 `fault:...` | 명령 수신 후 거부/장애 유지. 표시된 이유 확인 |
| `CMD_TIMEOUT`, `cmd_vel_timeout` | 유효 명령 간격이 0.5초를 초과함. 10Hz 이상 주기 발행 확인 |
| `zero_command` | 0속도 명령 수신. 다른 발행기가 0을 보내는지도 확인 |
| `zero_dwell`, `waiting_wheel_stop` | 출발/역회전 대기 또는 신선한 피드백상 이동 중 |
| `TX_wheels`가 0이 아님 | 해당 바퀴 목표속도를 시리얼 드라이버에 성공적으로 기록. 모터가 수신·실행했다는 확인은 아님 |
| `tx_packets` 증가 | 0속도·리프트 명령을 포함한 송신 완료 패킷 수. 증가 자체가 주행 명령을 뜻하지 않음 |
| `rx_bytes=0` | 현재 포트 연결 이후 읽은 원본 수신 바이트가 없음 |
| `rx_bytes>0`, `motor_packets=0` | 데이터는 들어오지만 정상 모터 상태 프레임을 아직 해석하지 못함. HEX·프로토콜·통신속도 확인 |
| `motor_packets` 증가 | 정상 모터 상태 프레임 수신·해석 성공. 최신 여부는 motor_feedback_fresh로 확인 |

`TX_HEX`는 마지막 송신 완료 패킷, `RX_HEX`는 마지막 읽기의 최대 128바이트입니다. 읽기 구간은 패킷 경계와 일치하지 않을 수 있습니다. 시리얼 카운터는 재연결할 때 초기화되며, 명령 카운터는 노드가 재시작될 때 초기화됩니다. 포트가 끊기면 `transmitted_wheel_speeds`는 null(알 수 없음)입니다.

ROS에서 같은 정보를 보려면 `ros2 topic echo /driver_status`, 발행기·타입·QoS를 보려면 `ros2 topic info /cmd_vel -v`를 사용합니다. namespace/remap을 썼다면 시작 로그에 표시되는 실제 토픽명을 사용하십시오.

1. 시작/정지 후에는 기본 0.2초의 0속도 명령 대기 구간을 거칩니다. 신선한 피드백이 있으면 양쪽 바퀴 속도가 기준 이하인지도 확인합니다. 기본 모드에서는 피드백이 없어도 시간 대기만으로 출발하며, 실제 정지는 확인할 수 없습니다. `require_motor_feedback:=true`이면 피드백이 없을 때 출발하지 않습니다.
2. 출발 후 기본 0.8초 동안 선속도 0.08m/s, 각속도 0.20rad/s 범위로 제한합니다. 선속도와 각속도를 같은 비율로 줄여 요청한 회전 비율을 유지합니다.
3. 동시에 좌·우 바퀴 속도 변화율을 제한합니다. 이후 정상 속도 상한까지 점진적으로 올라갑니다.
4. 어느 한 바퀴라도 기존 출력과 반대 방향을 요구하면 양쪽 명령 출력을 0까지 감속하고 기본 0.2초 대기한 뒤 새 방향으로 저속 출발합니다. 신선한 피드백이 있으면 바퀴 정지도 확인합니다. 피드백이 없으면 실제 감속/정지 여부는 확인하지 못합니다. 전후진과 한쪽 바퀴가 역회전하는 회전 전환 모두 적용됩니다.
5. **명시적인 `cmd_vel=0`, 명령 타임아웃, 잘못된 수치, 피드백 필수 모드의 피드백 장애, 정지 서비스는 감속 프로파일을 우회하고 다음 제어 주기에 0속도를 전송합니다.** 물리적 정지 시간은 하위제어기와 기구에 달려 있습니다.

저속 구간은 시간 기준입니다. 캐스터가 돌았는지 확인하는 센서가 없으므로, 0.8초 후 정렬 완료를 보장하지 않습니다. 구동륜도 움직이지 않을 정도로 속도를 낮추면 개선되지 않을 수 있습니다. 자동 왕복·흔들기·제자리 회전으로 캐스터를 맞추는 동작은 넣지 않았습니다.

곡선 출발에서는 좌·우 속도비를 유지합니다. 주행 도중 목표가 바뀔 때는 두 바퀴에 공통 보간 비율을 적용하므로 기존 출력에서 새 출력까지 부드럽게 전환되며, 전환 중 궤적은 순간적인 원래 `cmd_vel`과 다를 수 있습니다. 상위 경로제어기와 함께 사용할 때 실제 추종 응답을 확인하십시오.

## 3. 최초 확인과 튜닝

먼저 바퀴를 띄운 상태에서 통신, 좌우 대응, 전후진 부호, 정지 동작을 확인합니다. 캐스터 효과는 바닥 접촉이 필요하므로 이후 비어 있는 평탄한 구역에서 낮은 속도로 확인합니다. 물리적 정지 수단을 확보하고 시행하십시오.

```bash
ros2 topic echo motor_state
ros2 topic echo driver_status
ros2 topic echo cmd_vel_applied
```

시리얼 연결 후 별도 터미널에서 10Hz로 계속 발행합니다. `--once`는 출발 대기·명령 타임아웃 때문에 움직임을 확인하기에 적합하지 않습니다:

```bash
ros2 topic pub -r 10 cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

발행 터미널의 Ctrl+C 후 마지막 유효 명령으로부터 0.5초가 지나면 0속도를 전송합니다. 정지 서비스를 사용할 수도 있습니다.

```bash
ros2 service call driver_stop std_srvs/srv/Trigger "{}"
```

이 서비스는 정지를 유지하며, 복구하려면 명령 발행기를 먼저 중지하고 바퀴가 정지한 것을 확인한 뒤:

```bash
ros2 service call driver_reset std_srvs/srv/Trigger "{}"
```

리셋 후 새 주행 명령을 보내십시오. **발행기가 계속 켜져 있으면 리셋 후 도착하는 메시지도 새 명령이므로 다시 움직일 수 있습니다.** 정지 서비스는 하드웨어 비상정지를 대체하지 않습니다.

현장 비교는 동일 하중·바닥·명령으로 시행합니다. 캐스터가 전방 정렬된 상태, 좌우로 틀어진 상태, 후진 직후 상태를 나누어 출발 시 최대 방향 오차와 처음 0.5m 동안의 횡방향 이탈을 기록하십시오. 출발 구간만 느려졌는지, 실제 이탈도 줄었는지를 구분해야 합니다.

처음에는 기본값으로 확인하고 한 항목씩 조정합니다. 출발 충격이 크면 `wheel_acceleration`을 낮추고, 캐스터가 돌아오는 데 시간이 더 필요하면 `startup_duration`을 늘립니다. 지나치게 느린 출발에서 구동륜이 정체되면 속도를 더 낮추기보다 접지와 캐스터 회전 저항부터 확인합니다. 기존 상위제어기에도 속도 완화가 있으면 두 단계 지연이 겹칠 수 있습니다.

## 4. 파라미터

모든 값은 **초기 시험용 설정**입니다. 검증된 최적값이 아닙니다. 값은 노드 시작 시 읽으므로 변경 후 재시작하십시오.

| 이름 | 기본값 | 의미 |
|---|---:|---|
| `port` | `/dev/ttyUSB0` | 시리얼 장치 |
| `baudrate` | 921600 | bps, 실제 로봇에서 사용자 동작 확인 |
| `wheel_radius` | 0.12 | 구동륜 반지름 m, 실측 필요 |
| `wheel_separation` | 0.70 | 좌우 구동륜 간격 m, 사용자 제공값 |
| `control_rate` | 20.0 | 제어 주기 Hz |
| `max_linear_speed` | 0.30 | 차체 선속도 상한 m/s |
| `max_angular_speed` | 0.50 | 차체 각속도 상한 rad/s |
| `max_wheel_speed` | 20.0 | 바퀴 속도 상한 rad/s |
| `wheel_acceleration` | 2.0 | 바퀴 각가속도 상한 rad/s², 직진 약 0.24m/s² |
| `wheel_deceleration` | 4.0 | 일반 감속 상한 rad/s², 직진 약 0.48m/s² |
| `startup_duration` | 0.8 | 저속 출발 유지시간 s |
| `startup_linear_speed` | 0.08 | 저속 구간 선속도 상한 m/s |
| `startup_angular_speed` | 0.20 | 저속 구간 각속도 상한 rad/s |
| `reversal_pause` | 0.20 | 출발/역회전 전 0속도 대기시간 s; 신선한 피드백이 있으면 정지도 확인 |
| `stopped_wheel_speed` | 0.10 | 정지로 간주하는 바퀴 속도 절댓값 상한 rad/s |
| `cmd_vel_timeout` | 0.5 | 마지막 유효 속도 명령 이후 정지 s |
| `cmd_vel_reliability` | `best_effort` | BEST_EFFORT/RELIABLE 발행기 모두 수신 가능; `reliable` 선택 시 RELIABLE 발행기만 수신 |
| `diagnostic_period` | 1.0 | 터미널 명령·송수신 진단 출력 간격 s |
| `debug_serial` | false | true이면 진단 시 마지막 TX/RX HEX도 표시 |
| `require_motor_feedback` | false | false: 모터 피드백 없이도 주행 허용, true: 필수 감시 |
| `motor_state_timeout` | 0.5 | 피드백 신선도 기준 s; 필수 모드에서만 주행 차단/장애 유지 |
| `control_loop_timeout` | 0.25 | 제어 루프 지연 시 장애 처리 기준 s |
| `reconnect_interval` | 1.0 | 포트 재연결 시도 간격 s |
| `cmd_vel_topic` | `cmd_vel` | 입력 토픽 |
| `motor_state_topic` | `motor_state` | 상태 출력 토픽 |
| `enable_lift` | false | 리프트 활성화 |
| `lift_state_topic` | `lift_state` | 리프트 피드백 토픽 |
| `lift_target_topic` | `lift_target` | 리프트 목표 높이 토픽 |
| `lift_tolerance` | 0.02 | 높이 허용오차 m |
| `lift_max_height` | 0.0 | 실제 허용 최대 높이 m. 리프트 활성화 시 양수 필수 |
| `lift_state_timeout` | 0.5 | 리프트 상태 수신 중단 허용시간 s |
| `lift_motion_timeout` | 30.0 | 목표 높이 이동 제한시간 s |
| `lift_home_timeout` | 30.0 | HOME 전체 제한시간 s |
| `lift_home_start_timeout` | 2.0 | HOME 시작 상태 확인 제한시간 s |

시작 속도 상한은 일반 속도 상한보다 클 수 없습니다. `control_loop_timeout`은 두 제어 주기 이상이어야 합니다. 실제 피드백이 느린 펌웨어라면 수신 주기를 먼저 측정하고 타임아웃을 맞추십시오.

ROS 노드에서는 유한한 초과 속도를 선속도·각속도·바퀴속도 상한에 맞춰 **공통 비율로 축소**합니다. 이는 원본 노드의 거부 정책과 다릅니다. 통신 클래스의 `set_wheel_speeds()` 직접 호출은 기존대로 상한 초과를 거부합니다.

## 5. ROS 인터페이스와 장애 복구

모든 이름은 namespace 적용이 가능한 상대 이름입니다.

| 구분 | 이름 | 타입 | 내용 |
|---|---|---|---|
| 입력 | `cmd_vel` | `geometry_msgs/msg/Twist` | x 선속도, z 각속도 |
| 출력 | `motor_state` | `std_msgs/msg/Float64MultiArray` | 원본과 동일한 6개 값 |
| 출력 | `cmd_vel_applied` | `geometry_msgs/msg/Twist` | 제한·프로파일 적용 후 전송한 목표값, 실제 측정 속도가 아님 |
| 출력 | `driver_status` | `std_msgs/msg/String` | 상태 변경 시/최소 2Hz, JSON 상태 |
| 서비스 | `driver_stop` | `std_srvs/srv/Trigger` | 주행·리프트 정지 요청 및 정지 유지 |
| 서비스 | `driver_reset` | `std_srvs/srv/Trigger` | 시리얼 연결 확인 후 장애 해제·기존 목표 삭제; 피드백 필수 모드에서는 정상 피드백과 바퀴 정지도 필요 |
| 입력 | `lift_target` | `std_msgs/msg/Float64` | 목표 높이 m, 리프트 사용 시만 생성 |
| 출력 | `lift_state` | `std_msgs/msg/Float64MultiArray` | 높이, ADC, flags |
| 서비스 | `lift_set_home` | `std_srvs/srv/Trigger` | HOME 요청 접수; 완료 응답 아님 |
| 서비스 | `lift_stop` | `std_srvs/srv/Trigger` | 리프트 정지·목표 취소 |

`cmd_vel` 구독은 기본 BEST_EFFORT / VOLATILE / depth=1입니다. RELIABLE과 BEST_EFFORT 발행기 모두 연결할 수 있으며 재전송은 요구하지 않습니다. 유실·중단 시 명령 타임아웃은 계속 적용합니다. 기존 RELIABLE 수신은 실행 시 `-p cmd_vel_reliability:=reliable`로 선택할 수 있습니다. 메시지 타입은 여전히 `geometry_msgs/msg/Twist`이며 `TwistStamped`는 받지 않습니다. 송신원이 여러 개라면 상위 명령 선택기를 사용하십시오. 이 코드는 제어권 중재나 수동/자율 모드 전환을 하지 않습니다.

`motor_state.data`:

| 인덱스 | 값 | 단위 |
|---:|---|---|
| 0 | left_speed | rad/s |
| 1 | left_pwm | 펌웨어 PWM 상태 |
| 2 | left_position | 엔코더 누적 rad |
| 3 | right_speed | rad/s |
| 4 | right_pwm | 펌웨어 PWM 상태 |
| 5 | right_position | 엔코더 누적 rad |

`driver_status`의 `state`는 `waiting_feedback`, `ready`, `idle`, `startup`, `running`, `braking`, `reversal_wait`, `cmd_vel_timeout`, `fault` 등입니다. `fault`에 장애 사유, `phase`에 프로파일 상태, `serial_connected`와 `motor_feedback_fresh`에 연결/피드백 여부, `motor_feedback_required`에 피드백 필수 설정, `standstill_verified`에 신선한 피드백 기반 정지 확인 여부, `lift_state`와 `lift_fault`에 리프트 상태가 들어갑니다.

제어 루프 지연·시리얼 오류·운전자 정지는 장애를 유지합니다. 모터 피드백 중단은 `require_motor_feedback:=true`일 때만 장애로 처리합니다. 기본 false 모드는 피드백이 없어도 주행·복구가 가능합니다. 신선한 피드백으로 이동 중임이 확인되면 복구를 거부합니다. 포트 오류 때 1초 간격으로 재연결하고, 연결이 돌아와도 자동 재출발하지 않습니다. 바퀴가 멈춘 것을 확인한 뒤 `driver_reset`과 새 명령이 필요합니다. 피드백 없이 복구하면 응답에 `standstill UNVERIFIED`가 표시됩니다. `cmd_vel` 타임아웃 자체는 장애 유지가 아니므로 새 명령으로 다시 출발할 수 있습니다.

타임아웃은 호스트 수신 시각 기준입니다. MFPacket에 송신 시각·시퀀스가 없으므로 지연/재전송된 오래된 패킷을 완전히 식별할 수 없습니다. 8192byte를 넘는 수신 적체는 오류 처리합니다. 프로세스 강제 종료·OS 정지·케이블 단선 시에는 Python에서 STOP을 전달할 수 없으므로 펌웨어 측 명령 watchdog이 별도로 필요합니다.

## 6. 리프트 보완

리프트가 없는 사진 속 구성에는 `enable_lift:=false`로 사용하십시오. 이때 리프트 명령·서비스는 생성/전송하지 않습니다.

리프트를 사용하려면 `enable_lift:=true`와 함께 **실제 기구의 허용 최대 높이**를 `lift_max_height`에 지정해야 합니다. 기본 0.0인 상태에서는 실행을 거부합니다. 임의의 높이를 넣지 마십시오.

- `SENSOR_VALID`와 `WIRE_FAULT`, 상태 수신 시각을 검사합니다.
- 목표 높이는 0 이상, 지정한 최대 높이 이하여야 합니다. HOME 미완료/진행 중에는 목표를 거부합니다.
- 목표 도달 시 STOP을 보내고 목표를 해제합니다. **원본과 달리 이후 높이 변화를 자동으로 재보정하지 않습니다.** 새 이동에는 새 목표가 필요합니다.
- 상태가 끊기거나 오류/이동 제한시간 초과가 발생하면 STOP을 보내고 목표를 폐기합니다. 이후 피드백이 회복되어도 `driver_reset`과 새 목표가 필요합니다.
- 같은 목표를 반복 발행하거나 이동 중 목표를 바꾸어도 기존 이동 제한시간을 계속 연장하지 않습니다.
- HOME 요청을 아직 전송하지 않은 상태의 이전 피드백은 완료 판단에 쓰지 않습니다. 전송 후 수신한 모든 상태를 순서대로 처리하므로 `HOMING → HOMED`가 한꺼번에 와도 인식합니다.
- 기존에 HOMED였던 리프트를 다시 HOME할 때는 HOMING 관측이 필요합니다. 진행 상태가 누락되면 성공으로 추측하지 않고 시작 제한시간 후 STOP합니다. 이미 영점 부근이고 HOMED이면 동작 없이 완료된 상태를 반환합니다.
- 원본의 펌웨어 전제를 유지해 HOME 진행 중 `HOME=3` 명령은 주기 전송합니다. 반복 HOME이 펌웨어에서 작업을 재시작시키지 않는지 제조사와 확인해야 합니다.
- `lift_stop`은 현재 목표를 취소합니다. 이후 새 목표는 허용합니다. `driver_stop`은 리프트와 주행 모두를 정지 유지합니다.
- 주행과 리프트의 기구적 동시동작 인터록은 추가하지 않았습니다. 장비에 필요한 인터록은 상위제어기/펌웨어에서 별도로 적용해야 합니다.

flags 비트: 0 SENSOR_VALID, 1 RELAY_UP, 2 RELAY_DOWN, 3 WIRE_FAULT, 4 HOMING, 5 HOMED. 원래의 의미를 유지합니다.

## 7. 통신 규격과 직접 사용

MFPacket 송신 바이트 형식은 원본과 동일합니다.

| 방향 | ID | payload |
|---|---:|---|
| Python → 펌웨어 | 0x02 | float64 좌·우 바퀴 rad/s, 총 27byte |
| 펌웨어 → Python | 0x01 | float64 6개 모터 상태, 총 67byte |
| Python → 펌웨어 | 0x06 | int32 STOP=0, UP=1, DOWN=2, HOME=3, 총 13byte |
| 펌웨어 → Python | 0x05 | float64 높이, float64 ADC, int32 flags, 총 33byte |

헤더 `FF F9`, 길이 1byte(ID부터 payload 끝), ID, ACK, payload 개수, typed payload, 체크섬 순서입니다. 실수는 type=0x02/length=8/little-endian float64, 정수는 type=0x00/length=4/little-endian int32입니다. 속도 명령의 길이 필드는 0x17입니다.

원본의 체크섬을 그대로 사용합니다: 초기값 0xFF, 각 byte XOR 후 8회 오른쪽 시프트, 이전 최하위 비트가 1이면 0x1D XOR. 일반적인 이름의 CRC 라이브러리로 바꾸지 않았습니다.

수신 검사는 고정 패킷 길이, 개수, ACK(원본 모터 상태의 허용값 0/2/3을 리프트에도 적용), 자료형, 유한한 실수, 체크섬을 확인합니다. 허용 ACK 값은 실제 펌웨어와 확인하십시오. 손상 프레임은 한 byte씩 다시 동기화하여 뒤의 정상 프레임을 보존합니다. 다른 ID의 패킷은 처리하지 않습니다. ACK값 검사와 별개로 명령별 응답 대응/재전송 프로토콜을 구현한 것은 아닙니다.

`MfMobileDriver`의 기존 `set_velocity`, `set_wheel_speeds`, `read_states`, `read_lift_states`, `set_lift_command`, `stop`, `close` 인터페이스를 유지했습니다. **통신 클래스 단독 호출에는 ROS 노드의 타임아웃·상태 감시·출발 프로파일이 자동 적용되지 않습니다.** 직접 사용할 때는 별도의 제어 루프가 필요합니다.

PID, 모터 방향, CAN 변환, GPIO 모드, 구동허가는 펌웨어의 기존 설정을 사용합니다. 엔코더에 직접 명령을 보내지 않습니다. `/odom`, TF, GPS 또는 IMU 방향제어는 이 패키지에 없습니다.

## 8. 기구적으로 확인할 부분

사진과 증상으로 캐스터 방향 전환 저항이 유력하지만 실제 원인을 확정하지는 않았습니다. 적재 상태에서 네 캐스터 회전부의 뻑뻑함·브레이크 걸림·손상, 좌우 차이, 흰색 구동륜 접지와 미끄러짐을 확인하십시오. 캐스터 회전 저항은 접촉 면적·회전 베어링·회전축 오프셋에 영향을 받습니다. 구조 변경은 하중·바닥·회전 동작을 고려해 제작사와 검토해야 합니다.

정확한 직진 방향 유지가 필요하면 차체 IMU 또는 위치추정 피드백을 이용하는 상위 방향제어가 추가로 필요합니다. 엔코더 속도가 같다는 사실만으로 차체의 방향 틀어짐·횡미끄러짐을 알 수는 없습니다. 상위 경로제어기가 이미 yaw를 제어한다면 드라이버에 별도 보정을 중복 삽입하기 전에 통합 설계를 해야 합니다.

근거 자료:
- Blickle, Starting, rolling and swivel resistance: https://www.blickle.com/guide/info-box-wheel-series
- Kittisares et al., Error Prediction of a Differential Drive Wheeled Robot with a Swivel Caster Wheel, IFAC-PapersOnLine 56(2), 2023: https://doi.org/10.1016/j.ifacol.2023.10.394

## 9. 검증 범위

```bash
python3 -m unittest discover -s tests -v
```

모의 시험 49개 통과. 피드백 없는 10Hz 주행, 무명령 정지, 타임아웃 사유 유지, QoS 설정, 원본 바이트/정상 프레임 구분, HEX 길이 제한과 기존 시험을 검사합니다. ROS2와 시리얼을 가짜 객체로 대체하므로 실제 장치를 구동하지 않습니다. 실제 DDS 통신, ROS2 런타임, 펌웨어 응답, 로봇 주행, 캐스터 방향 틀어짐 개선량은 검증하지 않았습니다. 제공된 시작 로그만으로 실물의 미주행 원인을 확정하지 않았습니다.
