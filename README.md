# AUV 부표 비전/제어 브리지

이 패키지는 아래 ROS 2 파이프라인을 위한 예시 구성입니다.

```text
AUV NUC
  /camera/camera/color/image_raw/compressed
      |
      | 테더 / ROS 2 DDS
      v
노트북 PC / RTX 4070
  yolo_buoy_detector (Python / Ultralytics / .pt)
  /vision/buoy_bbox publish
      |
      | 테더 / ROS 2 DDS
      v
AUV NUC
  bbox_controller_node (C++)
  /mavros/rc/override publish
```

## 토픽 구성

카메라 입력:

```text
/camera/camera/color/image_raw/compressed
sensor_msgs/msg/CompressedImage
```

YOLO 검출 결과:

```text
/vision/buoy_bbox
std_msgs/msg/Float32MultiArray
data = [
  stamp_sec,
  detected,      # 1.0: 검출됨, 0.0: 검출 안 됨
  class_id,
  confidence,
  center_x,
  center_y,
  width,
  height,
  image_width,
  image_height
]
```

제어 출력:

```text
/mavros/rc/override
mavros_msgs/msg/OverrideRCIn
```

기본 RC 채널 매핑은 ArduSub 기준입니다.

```text
Channel 1: Pitch
Channel 2: Roll
Channel 3: Throttle / vertical
Channel 4: Yaw
Channel 5: Forward
Channel 6: Lateral
```

컨트롤러는 18채널 override 배열을 publish합니다. 사용하지 않는 채널은 `CHAN_NOCHANGE=65535`로 둡니다.

## 설치

이 패키지를 ROS 2 워크스페이스에 복사합니다.

```bash
mkdir -p ~/auv_ws/src
cp -r auv_buoy_vision_control ~/auv_ws/src/
cd ~/auv_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

`rosdep`으로 설치되지 않는 경우, ROS 2 배포판/Ubuntu 패키지에서 `mavros_msgs`를 설치해야 합니다.

노트북에서는 ROS 2가 사용하는 Python 환경에 YOLO 추론 의존성을 설치합니다.

```bash
pip install ultralytics
```

이 패키지는 AUV 제어 노드가 C++이기 때문에 `ament_cmake` 패키지입니다. 다만 노트북에서 실행하는 YOLO 검출 노드는 Python 실행 파일로 설치됩니다.

## 노트북에서 YOLO 노드 실행

`model_path`에는 학습된 `.pt` 모델 경로를 넣습니다. 기본값은 임의로 `/home/auv/models/buoy.pt`로 넣어두었으니 실제 모델 위치에 맞게 수정하면 됩니다.

```bash
ros2 launch auv_buoy_vision_control laptop_yolo_detection.launch.py \
  model_path:=/home/auv/models/buoy.pt \
  image_topic:=/camera/camera/color/image_raw/compressed \
  bbox_topic:=/vision/buoy_bbox \
  device:=cuda:0 \
  target_class_name:=buoy
```

학습 모델을 class name이 아니라 class id 기준으로 쓰려면 다음처럼 실행합니다.

```bash
ros2 launch auv_buoy_vision_control laptop_yolo_detection.launch.py \
  model_path:=/home/auv/models/buoy.pt \
  target_class_id:=0 \
  target_class_name:=
```

노트북에서 CUDA를 사용할 수 없으면 `device:=cpu`로 바꾸면 됩니다.

## AUV NUC에서 상태머신 컨트롤러 실행

기본 launch는 `mission_state_machine_node`를 실행합니다. 이 노드는 다음 임무 흐름을 관리합니다.

```text
IDLE -> DIVE -> SEARCH -> APPROACH_BUOY -> ALIGN_STICK
     -> INSERT_FORK -> DETACH -> BACKOFF -> VERIFY_RELEASE
     -> SEARCH/AREA_VERIFY -> ASCEND -> COMPLETE
```

상태머신에는 다음 입력이 필요합니다.

```text
/vision/buoy_bbox          std_msgs/msg/Float32MultiArray
/auv/depth                 std_msgs/msg/Float64 (미터, 수면 0, 아래 방향 양수)
/depth/pose                geometry_msgs/msg/PoseWithCovarianceStamped (선택 입력)
/mission/control_enable    std_msgs/msg/Bool
```

`/depth/pose`를 사용할 때는 기본적으로 `depth = -pose.position.z`로 변환합니다. 좌표계가 다른 센서는 `depth_pose_scale`과 `depth_pose_offset_m`을 조정해야 합니다. `depth_pose_topic:=`처럼 빈 값으로 실행하면 Pose 입력을 끌 수 있습니다. 실제 센서의 부호와 단위를 확인하기 전에는 자동제어를 활성화하지 마십시오.

먼저 QGC 또는 MAVLink 콘솔에서 실제 RC 채널 매핑과 PWM 방향을 확인한 다음 실행합니다.

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

노드는 안전을 위해 `IDLE` 및 RC release 상태로 시작합니다. 카메라, 수심, MAVROS, PWM 방향을 확인한 후 별도 터미널에서 활성화합니다.

```bash
ros2 topic pub --once /mission/control_enable std_msgs/msg/Bool "{data: true}"
```

즉시 비활성화하고 조종권을 반환하려면 다음 명령을 사용합니다.

```bash
ros2 topic pub --once /mission/control_enable std_msgs/msg/Bool "{data: false}"
```

현재 상태 확인:

```bash
ros2 topic echo /mission/state
```

포크의 영상상 목표 위치는 화면 정규화 좌표입니다. `(0.5, 0.5)`는 화면 중앙이며 실제 카메라와 포크의 위치 차이를 수조 시험으로 보정해야 합니다.

```bash
fork_target_x:=0.5
fork_target_y:=0.55
```

삽입, 자석 분리 및 후진 동작은 시간 기반 PWM 펄스로 시작하도록 구현되어 있습니다. 아래 값은 실제 기체 시험 전 반드시 낮은 추력부터 조정해야 합니다.

```bash
insert_pwm:=1560 insert_duration_sec:=0.8
detach_pwm:=1620 detach_duration_sec:=0.3
backoff_pwm:=1420 backoff_duration_sec:=0.5
```

상태머신에서 buoy와 stick을 동시에 갱신하려면 YOLO launch에 `publish_per_class:=true`를 지정합니다.

```bash
ros2 launch auv_buoy_vision_control laptop_yolo_detection.launch.py \
  model_path:=/home/pc/Downloads/best.pt \
  device:=cpu \
  show_preview:=false \
  publish_per_class:=true
```

`publish_per_class`는 클래스별 최고 confidence bbox를 한 프레임에 각각 발행합니다. 여러 부표와 각 stick의 공간적 연결까지 처리하려면 이후 detection array 메시지로 확장해야 합니다. 현재 `VERIFY_RELEASE`는 부표가 일정 시간 보이지 않는 것을 임시 성공 조건으로 사용하므로 실기체용 최종 성공 판정은 아닙니다.

상태머신에서 자주 조정할 제어 파라미터:

```bash
min_pwm:=1300
max_pwm:=1700
yaw_invert:=true
vertical_positive_is_up:=false
max_yaw_delta:=180
forward_pwm:=1560
```

상태머신 노드는 계산된 모든 추진기 PWM을 `1300~1700` 범위로 제한합니다. launch에서 더 좁은 범위는 지정할 수 있지만 `1300~1700` 바깥으로 확장하면 노드가 실행을 거부합니다. `CHAN_RELEASE=0`과 `CHAN_NOCHANGE=65535`는 추진기 PWM이 아닌 MAVROS 제어용 특수값이므로 이 제한을 적용하지 않습니다.

## 확인 명령어

AUV NUC 카메라 토픽 확인:

```bash
ros2 topic hz /camera/camera/color/image_raw/compressed
ros2 topic echo /camera/camera/color/image_raw/compressed --once
```

노트북 YOLO 검출 결과 확인:

```bash
ros2 topic echo /vision/buoy_bbox
```

AUV NUC RC override 확인:

```bash
ros2 topic echo /mavros/rc/override
ros2 topic echo /mavros/rc/in
ros2 topic echo /mavros/rc/out
```

주의할 점: `/mavros/rc/in`과 `/mavros/rc/out`은 FCU가 현재 인식하고 있는 RC 입력/출력 상태입니다. ROS에서 RC 입력을 실제로 넣으려면 `/mavros/rc/override`를 publish해야 합니다.

## 네트워크 설정

두 장비는 같은 ROS domain을 사용해야 하고, localhost 전용 모드가 아니어야 합니다.

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

테더 네트워크에서 DDS discovery가 잘 되지 않으면 사용하는 DDS vendor의 static discovery 설정을 쓰거나, 두 장비를 multicast가 가능한 같은 subnet에 둡니다.
