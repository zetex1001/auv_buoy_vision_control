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

## AUV NUC에서 컨트롤러 실행

먼저 QGC 또는 MAVLink 콘솔에서 실제 RC 채널 매핑을 확인합니다. 그 다음 컨트롤러를 실행합니다.

```bash
ros2 launch auv_buoy_vision_control auv_bbox_controller.launch.py \
  bbox_topic:=/vision/buoy_bbox \
  rc_override_topic:=/mavros/rc/override \
  throttle_channel:=3 \
  yaw_channel:=4 \
  forward_channel:=5
```

자주 조정할 파라미터:

```bash
yaw_invert:=true
vertical_positive_is_up:=false
max_yaw_delta:=180
max_throttle_delta:=100
forward_pwm:=1560
lost_behavior:=neutral
```

`lost_behavior:=neutral`은 부표가 검출되지 않을 때 throttle/yaw/forward를 1500으로 유지합니다.
`lost_behavior:=release`는 해당 채널에 `CHAN_RELEASE=0`을 보냅니다.

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
