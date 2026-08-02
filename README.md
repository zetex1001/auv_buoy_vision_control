# AUV 부표 비전/제어 브리지

노트북에서 YOLO로 buoy/stick을 검출하고, AUV NUC의 미션 상태머신이 MAVROS RC override로 제어하는 ROS 2 패키지입니다.

```text
AUV NUC
  /camera/camera/color/image_raw/compressed
      |
      | 테더 / ROS 2 DDS
      v
노트북 PC
  yolo_buoy_detector (Python / Ultralytics / .pt)
  /vision/buoy_bbox  (+ /vision/yolo/annotated/compressed)
      |
      | 테더 / ROS 2 DDS
      v
AUV NUC
  mission_state_machine_node (C++)
  /mavros/rc/override
  /mission/rc_command   (모니터링용 동일 PWM)
  /mission/state
```

> `bbox_controller_node`는 단순 추적용 레거시 노드입니다. 기본 launch는 사용하지 않으며, `mission_state_machine_node`와 동시에 실행하면 RC override가 충돌합니다.

## 미션 흐름

<!-- [ACOUSTIC-VISION HANDSHAKE] near zone 요청 → confirm → grant, 또는 외부 강제 grant. -->
```text
                     buoy 안정 검출 + grant
IDLE ─request→ TARGET_CONFIRM ─────────────────→ APPROACH_BUOY
  │                   │ buoy 안정 검출                  │ buoy 유실
  │ grant             └────────────→ WAIT_CONTROL_GRANT ┴──────→ SEARCH
  └───────────────────────────────────────────────────────→ SEARCH
                                                              │ buoy 안정 검출
                                                              v
                                                        APPROACH_BUOY
                                                              │ buoy가 충분히 크고 stick 검출
                                                              v
                                                         ALIGN_STICK
                                                              │ 목표점 정렬 유지
                                                              v
INSERT_FORK ←────────────────────────────────────────────────┘
     │ 삽입 시간 경과 → DETACH → BACKOFF → VERIFY_RELEASE
     │                                  │       │
     │                                  │       ├─ buoy 미검출: SEARCH (임시 성공)
     │                                  │       └─ buoy 잔존: 재시도 또는 SEARCH (포기)

SEARCH ── search_timeout_sec 경과 → AREA_VERIFY ── area_verify_sec 경과 → ASCEND → COMPLETE
               ▲                         │ buoy 안정 검출
               └─────────────────────────┴──────────────→ APPROACH_BUOY

어느 제어 상태에서든 수심 입력 유실 또는 최대 수심 초과 → FAILSAFE
```

수심 유실 또는 최대수심 초과 시 `FAILSAFE`로 전환하고 제어 채널을 `RELEASE`합니다.
이 노드는 하강 단계(`DIVE`)를 수행하지 않습니다. Acoustic/상위 제어기가 near zone과 적정 수심까지 유도한 뒤 Vision에 제어권을 넘기는 구조입니다.

| 상태 | 제어 동작 | 진입 및 다음 전이 |
|------|-----------|------------------|
| `IDLE` | RC override를 발행하지 않고 Acoustic의 요청을 기다린다. | `/homing/vision_search_active=true` → `TARGET_CONFIRM`. 외부 `/homing/vision_control_granted=true`가 오면, 이미 안정 검출한 buoy가 있으면 `APPROACH_BUOY`, 없으면 `SEARCH`. |
| `TARGET_CONFIRM` | RC를 발행하지 않는다. 가까운 buoy 후보의 연속 검출을 확인하고 `/vision/target_confirmed`를 발행한다. | 요청 후 `target_confirm_hits` frame이 `target_confirm_sec` 동안 유지되면 `WAIT_CONTROL_GRANT`. grant가 먼저/동시에 오고 안정 buoy가 있으면 `APPROACH_BUOY`, 없으면 `SEARCH`. |
| `WAIT_CONTROL_GRANT` | RC를 발행하지 않고 Acoustic이 기존 제어를 해제해 Vision에 넘기기를 기다린다. | `/homing/vision_control_granted=true` → 안정 buoy가 있으면 `APPROACH_BUOY`, 없으면 `SEARCH`. |
| `SEARCH` | `work_depth_m` 수심을 유지하고, 전진은 중립으로 둔 채 `search_yaw_pwm`으로 제자리 회전한다. | 가까운 buoy가 `target_confirm_hits` frame/`target_confirm_sec` 조건을 만족하면 `APPROACH_BUOY`. `search_timeout_sec`(기본 40초) 동안 못 찾으면 `AREA_VERIFY`. |
| `APPROACH_BUOY` | buoy를 화면 중앙으로 yaw 정렬하며, 박스 면적이 작을수록 더 빠르게 전진한다(`approach_forward_pwm`→`approach_forward_min_pwm`). throttle은 비전 상하 오차와 작업수심 제어를 블렌딩한다. | buoy가 `detection_timeout_sec` 동안 끊기면 `SEARCH`. buoy 면적이 `approach_area_ratio` 이상이고 stick도 최근 검출되면 `ALIGN_STICK`. |
| `ALIGN_STICK` | 전진은 중립으로 두고 stick을 `fork_target_x/y`로 yaw·throttle 정렬하며 작업수심도 유지한다. | stick이 사라지면 buoy가 남아 있으면 `APPROACH_BUOY`, 아니면 `SEARCH`. stick이 deadband 안에 `align_stable_sec` 동안 유지되면 `INSERT_FORK`. |
| `INSERT_FORK` | 작업수심을 유지하면서 `insert_pwm` 전진 펄스를 낸다. | `insert_duration_sec` 경과 → `DETACH`. |
| `DETACH` | 작업수심을 유지하면서 `detach_pwm` 전진 펄스로 분리를 시도한다. | `detach_duration_sec` 경과 → `BACKOFF`. |
| `BACKOFF` | 작업수심을 유지하면서 `backoff_pwm`으로 후진해 대상에서 떨어진다. | `backoff_duration_sec` 경과 → `VERIFY_RELEASE`. |
| `VERIFY_RELEASE` | yaw/전진은 중립, 작업수심을 유지하며 buoy가 사라졌는지 확인한다. | buoy가 `verify_clear_sec` 동안 안 보이면 **임시 성공**으로 `SEARCH`. `verify_timeout_sec` 안에 남아 있으면 최대 `max_target_retries`까지 `ALIGN_STICK` 또는 `APPROACH_BUOY` 재시도하고, 한도를 넘으면 타깃을 포기하고 `SEARCH`. |
| `AREA_VERIFY` | SEARCH와 같이 작업수심을 유지하고 전진 중립·yaw 회전으로 마지막 재탐색을 한다. | buoy가 `min_detection_hits` 연속 검출되면 `APPROACH_BUOY`. `area_verify_sec`(기본 12초) 동안 못 찾으면 `ASCEND`. |
| `ASCEND` | yaw/전진 중립, 양성 부력 보정 없이 `surface_depth_m`를 목표로 부상한다. | 수심이 `surface_depth_m + depth_tolerance_m` 이내 → `COMPLETE`. |
| `COMPLETE` | throttle/yaw/forward RC override를 `RELEASE`하여 제어권을 반환한다. | 종료 상태. |
| `FAILSAFE` | throttle/yaw/forward RC override를 즉시 `RELEASE`한다. | 수심 입력이 `depth_timeout_sec`보다 오래됐거나 수심이 `max_depth_m`를 초과하면 진입. 종료 상태. |

### 다중 부표 선택

SEARCH/AREA_VERIFY에서 buoy가 여러 개면:

1. 박스 **면적**이 큰 것
2. 비슷하면 **confidence**(박스 확률)가 높은 것
3. 그것도 비슷하면 이미지 **오른쪽**

APPROACH/ALIGN 중에는 고른 부표를 유지합니다.

## 토픽

### 카메라

```text
/camera/camera/color/image_raw/compressed
sensor_msgs/msg/CompressedImage
```

### YOLO 검출 (`/vision/buoy_bbox`)

`std_msgs/msg/Float32MultiArray` — 검출 1개당 10개 값:

```text
[stamp_sec, detected, class_id, confidence,
 center_x, center_y, width, height, image_width, image_height]
```

- `detected >= 0.5` 이면 유효
- `publish_per_class:=true`(기본)일 때 클래스마다 메시지 1개씩 발행 (buoy + stick 동시 갱신)

### 수심 / Acoustic-Vision 핸드셰이크 / 상태

```text
/auv/depth                 std_msgs/msg/Float64  (양의 하방[m], 선택)
/depth/pose                geometry_msgs/msg/PoseWithCovarianceStamped (선택)
/homing/vision_search_active   std_msgs/msg/Bool  (Acoustic -> Vision)
/vision/target_confirmed       std_msgs/msg/Bool  (Vision -> Acoustic, 타깃 확정)
/homing/vision_control_granted std_msgs/msg/Bool  (Acoustic -> Vision)
/mission/state             std_msgs/msg/String   (latched)
```

`/depth/pose` 기본 변환: `depth = -pose.position.z`  
(`depth_pose_scale`, `depth_pose_offset_m`으로 조정. 빈 `depth_pose_topic:=` 으로 Pose 입력 비활성)

### 제어 출력

```text
/mavros/rc/override        mavros_msgs/msg/OverrideRCIn
/mission/rc_command        동일 내용 (웹 모니터용, 다른 override와 분리)
```

ArduSub 기본 채널 (이 노드가 건드리는 것):

```text
Ch3 Throttle/vertical
Ch4 Yaw
Ch5 Forward
```

미사용 채널은 `CHAN_NOCHANGE=65535`. PWM은 `1300~1700`으로 클램프.

## 설치

```bash
mkdir -p ~/auv_ws/src
cp -r auv_buoy_vision_control ~/auv_ws/src/
cd ~/auv_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

노트북 YOLO 의존성:

```bash
# 권장
pip install -r src/auv_buoy_vision_control/requirements-yolo.txt

# 또는 환경 점검/설치 스크립트
ros2 run auv_buoy_vision_control check_yolo_env.py
# setup_laptop_yolo_env.sh
```

## 노트북: YOLO 실행

`model_path`는 필수에 가깝습니다 (launch 기본은 빈 문자열, 노드 기본 예시는 `/home/auv/models/buoy.pt`).

```bash
ros2 launch auv_buoy_vision_control laptop_yolo_detection.launch.py \
  model_path:=/path/to/model.pt \
  image_topic:=/camera/camera/color/image_raw/compressed \
  bbox_topic:=/vision/buoy_bbox \
  device:=cuda:0 \
  publish_per_class:=true
```

클래스 필터 예:

```bash
# class id
target_class_id:=0 target_class_name:=

# 또는 class name (모델 names에 존재해야 함)
target_class_name:=buoy
```

미션이 buoy+stick을 쓰므로 **`target_class_id`로 한 클래스만 막지 마세요.**  
(둘 다 필요하면 `target_class_id:=-1`, `target_class_name:=` 유지)

Annotated JPEG:

```bash
ros2 topic hz /vision/yolo/annotated/compressed
```

## AUV NUC: 상태머신 실행

사전 점검: QGC/MAVLink에서 RC 채널·PWM 방향과 수심 부호/단위를 확인한다.
`work_depth_m`은 하강 목표가 아니라 Vision 제어 중 유지할 수심입니다. Acoustic/상위 제어가 넘겨준 실제 작업 수심과 맞춰 설정하세요.

```bash
ros2 launch auv_buoy_vision_control auv_bbox_controller.launch.py \
  bbox_topic:=/vision/buoy_bbox \
  depth_pose_topic:=/depth/pose \
  depth_pose_scale:=-1.0 \
  rc_override_topic:=/mavros/rc/override \
  work_depth_m:=9.5 \
  surface_depth_m:=0.4 \
  max_depth_m:=10.5 \
  buoy_class_id:=0 \
  stick_class_id:=1
```

핸드셰이크 상태 확인:

```bash
ros2 topic echo /mission/state
ros2 topic echo /homing/vision_search_active
ros2 topic echo /vision/target_confirmed
ros2 topic echo /homing/vision_control_granted
```

## 주요 파라미터 (노드/launch 기본값)

### 탐색 / 접근

| 파라미터 | 기본 | 설명 |
|----------|------|------|
| `search_yaw_pwm` | 1600 | SEARCH/AREA_VERIFY yaw 고정 PWM |
| `search_timeout_sec` | 40 | SEARCH 타임아웃 → AREA_VERIFY |
| `target_confirm_hits` | 4 | TARGET_CONFIRM/SEARCH에서 가까운 동일 buoy 확인에 필요한 연속 hit |
| `target_confirm_sec` | 0.3 | 위 연속 hit 이후 추가 유지 시간 |
| `min_detection_hits` | 5 | AREA_VERIFY 등 후속 재탐색의 연속 hit |
| `approach_area_ratio` | 0.30 | ALIGN 진입·전진 P 스케일용 면적비 |
| `approach_forward_pwm` (`forward_pwm` launch 인자) | 1700 | APPROACH 전진 최대 |
| `approach_forward_min_pwm` | 1560 | APPROACH 전진 최소 (가까울 때) |
| `approach_vision_throttle_weight` | 0.4 | APPROACH/ALIGN throttle 비전 비중 (나머지는 수심 P) |
| `max_yaw_delta` | 180 | 비전 yaw 최대 편차 (±) |
| `fork_target_x` / `fork_target_y` | 0.30 / 0.70 | ALIGN 포크 목표 (정규화, 왼쪽 아래 3사분면 쪽) |

### 수심 / 필터

| 파라미터 | 기본 | 설명 |
|----------|------|------|
| `work_depth_m` | 9.5 | Vision 제어 중 유지할 작업 수심. 하강 동작은 수행하지 않음 |
| `depth_kp_pwm_per_m` | 45 | 수심 P게인 (PWM/m) |
| `max_depth_delta_pwm` | 160 | 수심 PWM 편차 클램프 |
| `buoyancy_hold_delta_pwm` | 40 | 양성 부력 보정 (목표에서도 하강 바이어스). ASCEND에서는 꺼짐 |
| `lpf_tau_sec` | 0.3 | 수심·bbox 중심/크기 1차 LPF 시상수 (0=off) |

### 포크 펄스

실기 전 낮은 추력부터 조정:

```bash
insert_pwm:=1560 insert_duration_sec:=0.8
detach_pwm:=1620 detach_duration_sec:=0.3
backoff_pwm:=1420 backoff_duration_sec:=0.5
```

### PWM / 방향

```bash
min_pwm:=1300 max_pwm:=1700
yaw_invert:=false
vertical_positive_is_up:=true
```

`CHAN_RELEASE=0`, `CHAN_NOCHANGE=65535`는 PWM 클램프 대상이 아닙니다.

## 확인 명령

```bash
ros2 topic hz /camera/camera/color/image_raw/compressed
ros2 topic echo /vision/buoy_bbox
ros2 topic echo /mission/state
ros2 topic echo /mavros/rc/override
ros2 topic echo /mission/rc_command
```

`/mavros/rc/in`, `/mavros/rc/out`은 FCU가 인식한 RC 상태입니다. ROS에서 넣으려면 `/mavros/rc/override`를 사용합니다.

## 네트워크

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

테더에서 discovery가 안 되면 DDS static discovery 또는 동일 subnet/multicast를 확인하세요.

## 알려진 제한

- `VERIFY_RELEASE`는 buoy가 일정 시간 안 보이는 것을 **임시 성공**으로 봅니다. 실기체 최종 성공 판정용은 아닙니다.
- APPROACH에 별도 타임아웃은 없습니다 (면적·stick 조건 또는 buoy 유실로만 빠져나옴).
- Web GUI로 미션을 띄우면 GUI 입력값이 launch 기본보다 우선할 수 있으니, GUI 기본값이 이 README/노드와 같은지 확인하세요.
