#!/usr/bin/env python3
import re
from typing import Optional, Union

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

try:
    import cv2
    from cv_bridge import CvBridge
except ImportError:
    cv2 = None
    CvBridge = None


class UsbCameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__("usb_camera_publisher")
        self.declare_parameter("video_device", "/dev/video0")
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("frame_id", "right_wrist_camera_optical_frame")
        self.declare_parameter("camera_name", "right_wrist_camera")

        self.video_device = str(self.get_parameter("video_device").value)
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.fps = max(float(self.get_parameter("fps").value), 1.0)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.camera_name = str(self.get_parameter("camera_name").value)

        if cv2 is None or CvBridge is None:
            raise RuntimeError(
                "usb_camera_publisher requires OpenCV and cv_bridge. Install them or use v4l2_camera/usb_cam."
            )

        self.bridge = CvBridge()
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.image_pub = self.create_publisher(Image, "image_raw", sensor_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, "camera_info", sensor_qos)
        self.capture = self._open_capture(self.video_device)
        self.last_read_failed = False
        self.timer = self.create_timer(1.0 / self.fps, self._publish_frame)

        self.get_logger().info(
            f"Publishing wrist camera from {self.video_device} on {self.get_namespace()}/image_raw"
        )

    def _open_capture(self, device: str) -> "cv2.VideoCapture":
        source: Union[int, str] = device
        match = re.fullmatch(r"/dev/video(\d+)", device)
        if match:
            source = int(match.group(1))

        capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            capture = cv2.VideoCapture(source)

        if not capture.isOpened():
            self.get_logger().error(
                f"Failed to open required wrist camera device {device}. "
                "Check the cable, device path, permissions, and run `v4l2-ctl --list-devices`."
            )
            raise RuntimeError(f"Failed to open camera device {device}")

        capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.image_width))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.image_height))
        capture.set(cv2.CAP_PROP_FPS, float(self.fps))
        return capture

    def _publish_frame(self) -> None:
        ok, frame = self.capture.read()
        if not ok or frame is None:
            if not self.last_read_failed:
                self.get_logger().error(
                    f"Failed to read from required wrist camera {self.video_device}. Recording cannot use wrist images."
                )
                self.last_read_failed = True
            return

        self.last_read_failed = False
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.image_pub.publish(msg)

        camera_info = self._make_camera_info(msg.width, msg.height, msg.header.frame_id, msg.header.stamp)
        self.camera_info_pub.publish(camera_info)

    def _make_camera_info(self, width: int, height: int, frame_id: str, stamp) -> CameraInfo:
        camera_info = CameraInfo()
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = frame_id
        camera_info.width = width
        camera_info.height = height
        camera_info.distortion_model = "plumb_bob"
        camera_info.d = [0.0] * 5
        camera_info.k = [0.0] * 9
        camera_info.k[0] = 1.0
        camera_info.k[4] = 1.0
        camera_info.k[8] = 1.0
        camera_info.r = [0.0] * 9
        camera_info.r[0] = 1.0
        camera_info.r[4] = 1.0
        camera_info.r[8] = 1.0
        camera_info.p = [0.0] * 12
        camera_info.p[0] = 1.0
        camera_info.p[5] = 1.0
        camera_info.p[10] = 1.0
        return camera_info

    def destroy_node(self) -> bool:
        if hasattr(self, "capture") and self.capture is not None:
            self.capture.release()
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node: Optional[UsbCameraPublisher] = None
    try:
        node = UsbCameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if rclpy.ok():
            logger = node.get_logger() if node is not None else rclpy.logging.get_logger("usb_camera_publisher")
            logger.error(str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
