# MF Robot Console — 주행 모니터링

주행 조작은 teleop 등 외부 프로그램에서 하고, 관제는 ROS2 명령·모터 피드백·드라이버 상태를 확인합니다. 관제에는 방향 버튼, 주행 속도 슬라이더, 주행 키보드 단축키가 없습니다.

## 연결 구조

- 드라이버: 시리얼 921600bps로 로봇 보드와 통신합니다.
- teleop: `/cmd_vel`로 주행 명령을 보냅니다.
- 관제 브리지: `/cmd_vel`, `/cmd_vel_applied`, `/motor_state`, `/driver_status`를 구독합니다. **`/cmd_vel` 발행기를 만들지 않으며 0속도도 보내지 않습니다.**
- 정지 유지·복구 버튼은 사용자가 눌렀을 때만 드라이버 서비스를 호출합니다. 기존 리프트 기능도 명시적 요청으로만 동작합니다.

관제를 열거나 닫기, 탭 전환, 피드백 끊김, 다른 cmd_vel 발행기 감지는 주행 명령 또는 자동 정지 요청을 발생시키지 않습니다. 여러 발행기는 화면에 안내만 표시합니다. 제어권 중재 기능은 없으므로 teleop과 자율주행 노드의 동시 명령은 별도로 관리해야 합니다.

## 실행

Ubuntu / ROS2 Humble / Python 3.10 이상 기준입니다. 브리지는 Python 표준 라이브러리와 ROS2 패키지만 사용합니다. Node.js나 rosbridge를 설치할 필요가 없습니다.

처음 받는 경우:

```bash
git clone https://github.com/kang-hyun/meta_cmd.git
cd meta_cmd
```

이미 받은 저장소에서는 `git pull --ff-only origin main`으로 업데이트합니다. 아래 `/path/to/meta_cmd`는 실제 저장소 폴더로 바꾸십시오.

필요한 패키지가 없다면:

```bash
sudo apt install python3-serial ros-humble-rclpy \
  ros-humble-geometry-msgs ros-humble-std-msgs ros-humble-std-srvs
```

**터미널 1 — 드라이버**

```bash
source /opt/ros/humble/setup.bash
cd /path/to/meta_cmd
python3 mf_mobile_driver_node.py
```

기본 시리얼 포트는 `/dev/ttyUSB0`, 보드레이트는 921600입니다. 이미 드라이버를 실행 중이면 중복 실행하지 않습니다.

**터미널 2 — 관제 브리지**

```bash
source /opt/ros/humble/setup.bash
cd /path/to/meta_cmd
python3 ros_dashboard_bridge.py
```

같은 PC의 브라우저에서 **http://127.0.0.1:8765**를 엽니다. 별도 터미널에서 기존 teleop 프로그램을 실행해 `/cmd_vel`에 `geometry_msgs/msg/Twist`를 발행합니다. 드라이버·teleop·브리지의 ROS_DOMAIN_ID와 namespace를 맞추십시오.

드라이버의 기본 명령 타임아웃은 0.5초입니다. 주행 중에는 teleop이 명령을 지속적으로 발행해야 하며 10Hz 이상을 권장합니다. 키를 한 번 눌러 한 번만 발행하는 설정이면 주행이 중단될 수 있습니다. 실제 주행 종료·정지는 teleop과 드라이버의 설정에 따라 처리됩니다.

브리지는 localhost에만 연결합니다. 별도 PC에서는 기존 SSH 접속으로 터널을 만들 수 있습니다.

```bash
ssh -N -L 8765:127.0.0.1:8765 robot_user@ROBOT_IP
```

터널을 연 PC에서 http://127.0.0.1:8765를 엽니다. 관제 브리지 포트를 바꾸면 터널의 포트도 맞춥니다.

## 화면 값

| 표시 | 데이터 |
|---|---|
| 입력 선속도·각속도 | 외부 `/cmd_vel`에서 마지막으로 수신한 명령 |
| 드라이버 전송 목표 | `/cmd_vel_applied`; 출발 완화·제한을 적용해 전송한 목표 |
| 좌우 바퀴 속도·PWM·누적 위치 | `/motor_state` |
| 현재 선속도 | `(왼쪽 rad/s + 오른쪽 rad/s) × 반지름 / 2` |
| 현재 회전 속도 | `(오른쪽 rad/s - 왼쪽 rad/s) × 반지름 / 좌우 간격` |
| 발행기 수 | 관제가 구독하는 cmd_vel 토픽의 발행기 수; 관제 자신은 포함되지 않음 |

입력 명령이 수신된 사실은 드라이버가 승인했거나 로봇이 움직인다는 뜻이 아닙니다. 전송 목표와 모터 피드백도 함께 확인하십시오. 명령 수신 후 0.5초가 지나면 입력 속도는 `—`로 표시하고 경과 시간은 남깁니다. 이 표시 기준은 드라이버의 실제 타임아웃 설정을 변경하지 않습니다. ROS 인터페이스 창에는 마지막 명령과 수신 경과 시간을 표시합니다.

모터 피드백이 0.5초 넘게 없거나 브리지 응답이 끊기면 실측값은 `—`로 표시합니다. 피드백 없이 주행을 허용하는 드라이버에서도 가짜 측정값은 만들지 않습니다. 그래프 세로 범위는 수신한 전송 목표·측정값에 따라 자동으로 넓어집니다. 화면의 속도 제한 슬라이더나 고정 최대속도 표시는 없습니다. 실제 제한값은 드라이버의 설정을 따릅니다.

반지름 기본값은 0.12m, 구동륜 간격은 0.70m입니다. 바꾸면 드라이버와 브리지에 동일한 값을 설정합니다.

```bash
python3 ros_dashboard_bridge.py --wheel-radius 0.12 --wheel-separation 0.70
```

엔코더 환산값은 차체의 GPS/IMU 측정값이 아닙니다. 바퀴 미끄러짐과 캐스터에 의한 횡방향 이동은 측정하지 않습니다.

## 정지 유지·복구·리프트

- **정지 유지**: `/driver_stop` 호출. 드라이버가 정지 상태를 유지합니다.
- **복구**: `/driver_reset` 호출. 복구 전에 외부 teleop 명령을 멈추고 바퀴 정지와 장애 해소를 확인하십시오. 외부 프로그램이 계속 명령을 보내면 복구 후 그 새 명령으로 다시 움직일 수 있습니다.
- **리프트**: 기존 HOME·목표 높이·정지 기능을 유지합니다. 드라이버의 `enable_lift`, `lift_max_height`와 브리지의 `--lift-max-height`를 맞추어야 합니다. 이동 중에는 조작을 거부하며, 작업 중 외부 주행 명령을 보내지 마십시오.

서비스는 비동기로 호출합니다. 실패를 성공으로 표시하지 않으며, 응답이 3초 넘게 없으면 실행 여부 미확인으로 표시합니다. 자동 재시도하지 않고 기존 결과가 확인될 때까지 같은 서비스의 중복 호출을 막습니다. HOME 응답은 요청 접수이고 실제 완료는 리프트 피드백으로 확인합니다.

정지 버튼은 소프트웨어 서비스이며 하드웨어 비상정지를 대신하지 않습니다. 관제·브리지 종료 자체는 정지 동작이 아닙니다.

## ROS 인터페이스

| 방향 | 이름 | 타입 |
|---|---|---|
| 구독 | cmd_vel, cmd_vel_applied | geometry_msgs/msg/Twist |
| 구독 | motor_state, lift_state | std_msgs/msg/Float64MultiArray |
| 구독 | driver_status | std_msgs/msg/String |
| 서비스 호출 | driver_stop, driver_reset, lift_set_home, lift_stop | std_srvs/srv/Trigger |
| 발행 | lift_target | std_msgs/msg/Float64 |

구독은 BEST_EFFORT/VOLATILE입니다. RELIABLE 또는 BEST_EFFORT 발행기를 수신할 수 있습니다. 필요하면 드라이버와 브리지의 remap을 맞춥니다.

```bash
python3 ros_dashboard_bridge.py --ros-args -r __ns:=/robot1
python3 ros_dashboard_bridge.py --ros-args -r cmd_vel:=/robot1/velocity
```

## 데모와 검증

`?demo=1`을 URL에 붙이면 로컬에서도 가상 외부 명령·피드백을 재생합니다. 데모는 실제 로봇에 연결하지 않습니다. 정지·복구도 데모 내에서만 적용됩니다.

```bash
python3 -m unittest discover -s tests -q
```

드라이버 모의 시험 49개와 관제 모의 시험 22개, 총 71개를 통과했습니다. 관제 시험은 cmd_vel 발행기 부재, 외부 명령 구독, 복수 발행기·피드백 중단 시 자동 정지 요청 부재, 명시적 서비스 호출, 폐기된 주행 HTTP 경로 차단을 포함합니다. 실제 ROS2/DDS와 로봇 통합 시험은 별도입니다.
