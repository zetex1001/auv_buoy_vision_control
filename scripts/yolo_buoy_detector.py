#!/usr/bin/env python3
import importlib
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray


BBOX_FORMAT = (
    "data = [stamp_sec, detected, class_id, confidence, center_x, center_y, "
    "width, height, image_width, image_height]"
)


class YoloBuoyDetector(Node):
    def __init__(self) -> None:
        super().__init__("yolo_buoy_detector")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw/compressed")
        self.declare_parameter("bbox_topic", "/vision/buoy_bbox")
        self.declare_parameter("model_path", "/home/auv/models/buoy.pt")
        self.declare_parameter("target_class_id", -1)
        self.declare_parameter("target_class_name", "buoy")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("imgsz", 640)

        self.image_topic = self.get_parameter("image_topic").value
        self.bbox_topic = self.get_parameter("bbox_topic").value
        self.model_path = self.get_parameter("model_path").value
        self.target_class_id = int(self.get_parameter("target_class_id").value)
        self.target_class_name = str(self.get_parameter("target_class_name").value).strip().lower()
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.device = str(self.get_parameter("device").value)
        self.imgsz = int(self.get_parameter("imgsz").value)

        self.model = self._load_model(self.model_path)
        self.class_names = getattr(self.model, "names", {}) or {}

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_sub = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.on_image,
            image_qos,
        )
        self.bbox_pub = self.create_publisher(Float32MultiArray, self.bbox_topic, 10)

        self.get_logger().info(f"Subscribing: {self.image_topic}")
        self.get_logger().info(f"Publishing: {self.bbox_topic} ({BBOX_FORMAT})")
        self.get_logger().info(
            f"YOLO PT model={self.model_path}, device={self.device}, imgsz={self.imgsz}, "
            f"target_class_id={self.target_class_id}, target_class_name='{self.target_class_name}'"
        )

    def _load_model(self, model_path: str):
        ultralytics = importlib.import_module("ultralytics")
        return ultralytics.YOLO(model_path)

    def on_image(self, msg: CompressedImage) -> None:
        image = self._decode_compressed_image(msg)
        if image is None:
            self.get_logger().warning("Failed to decode compressed image", throttle_duration_sec=2.0)
            return

        image_height, image_width = image.shape[:2]
        detection = self._detect_best_target(image)
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

        out = Float32MultiArray()
        if detection is None:
            out.data = [
                stamp_sec,
                0.0,
                -1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                float(image_width),
                float(image_height),
            ]
        else:
            class_id, confidence, center_x, center_y, width, height = detection
            out.data = [
                stamp_sec,
                1.0,
                float(class_id),
                float(confidence),
                float(center_x),
                float(center_y),
                float(width),
                float(height),
                float(image_width),
                float(image_height),
            ]

        self.bbox_pub.publish(out)

    def _decode_compressed_image(self, msg: CompressedImage) -> Optional[np.ndarray]:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        return cv2.imdecode(data, cv2.IMREAD_COLOR)

    def _detect_best_target(self, image: np.ndarray) -> Optional[Tuple[int, float, float, float, float, float]]:
        results = self.model.predict(
            source=image,
            conf=self.confidence_threshold,
            imgsz=self.imgsz,
            device=self.device,
            verbose=False,
        )
        if not results:
            return None

        boxes = getattr(results[0], "boxes", None)
        if boxes is None or len(boxes) == 0:
            return None

        xyxy = boxes.xyxy.detach().cpu().numpy()
        confidences = boxes.conf.detach().cpu().numpy()
        classes = boxes.cls.detach().cpu().numpy().astype(int)

        best = None
        best_confidence = -1.0
        for rect, confidence, class_id in zip(xyxy, confidences, classes):
            if not self._class_matches(class_id):
                continue
            if confidence < best_confidence:
                continue

            x1, y1, x2, y2 = rect
            width = max(0.0, float(x2 - x1))
            height = max(0.0, float(y2 - y1))
            center_x = float(x1 + width / 2.0)
            center_y = float(y1 + height / 2.0)
            best = (int(class_id), float(confidence), center_x, center_y, width, height)
            best_confidence = float(confidence)

        return best

    def _class_matches(self, class_id: int) -> bool:
        if self.target_class_id >= 0:
            return class_id == self.target_class_id
        if not self.target_class_name:
            return True
        class_name = str(self.class_names.get(class_id, "")).strip().lower()
        return class_name == self.target_class_name


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloBuoyDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
