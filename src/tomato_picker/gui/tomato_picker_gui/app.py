from __future__ import annotations

import argparse
import sys
import threading
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from tomato_picker_interfaces.action import PickTarget
from tomato_picker_interfaces.msg import TargetObject

from tomato_picker_gui.depth import depth_to_point, valid_depth
from tomato_picker_gui.selection import select_pixel

try:
    from PySide6.QtCore import QPointF, QRectF, Qt, QTimer, Signal
    from PySide6.QtGui import QImage, QPainter, QPen, QPolygonF
    from PySide6.QtWidgets import (
        QApplication,
        QCheckBox,
        QDoubleSpinBox,
        QFormLayout,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QSpinBox,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exception:  # pragma: no cover - depends on optional desktop environment
    raise RuntimeError("PySide6 is required for tomato_picker_gui") from exception


class RosBridge(Node):
    def __init__(self, config: dict):
        super().__init__("wrist_target_gui")
        self._config = config
        self._cv = CvBridge()
        self._lock = threading.Lock()
        self._color: Optional[np.ndarray] = None
        self._color_frame = ""
        self._depth_mm: Optional[np.ndarray] = None
        self._depth_k: Optional[np.ndarray] = None
        self._target: Optional[TargetObject] = None
        self._status = "waiting for wrist RGB-D"

        topics = config["topics"]
        self.create_subscription(Image, topics["color_image"], self._color_cb, 10)
        self.create_subscription(Image, topics["depth_image"], self._depth_cb, 10)
        self.create_subscription(CameraInfo, topics["depth_info"], self._info_cb, 10)
        self._target_pub = self.create_publisher(TargetObject, topics["selected_target"], 10)
        self._pick_client = ActionClient(self, PickTarget, topics["pick_action"])
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _color_cb(self, msg: Image) -> None:
        image = self._cv.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        with self._lock:
            self._color = np.asarray(image).copy()
            self._color_frame = msg.header.frame_id

    def _depth_cb(self, msg: Image) -> None:
        image = np.asarray(self._cv.imgmsg_to_cv2(msg, desired_encoding="passthrough"))
        if msg.encoding in ("32FC1", "32FC"):
            depth_mm = image.astype(np.float32) * 1000.0
        else:
            depth_mm = image.astype(np.float32)
        with self._lock:
            self._depth_mm = depth_mm.copy()

    def _info_cb(self, msg: CameraInfo) -> None:
        with self._lock:
            self._depth_k = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)

    def snapshot(self) -> tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray], str]:
        with self._lock:
            color = None if self._color is None else self._color.copy()
            depth = None if self._depth_mm is None else self._depth_mm.copy()
            k = None if self._depth_k is None else self._depth_k.copy()
            frame = self._color_frame
        return color, depth, k, frame

    def status(self) -> str:
        with self._lock:
            return self._status

    def _set_status(self, text: str) -> None:
        with self._lock:
            self._status = text

    def make_target(self, u: int, v: int, depth_mm: float, k: np.ndarray, camera_frame: str) -> TargetObject:
        target_frame = self._config["frames"]["target_frame"]
        tool_frame = self._config["frames"]["tool_frame"]
        x, y, z = depth_to_point(u, v, depth_mm / 1000.0, k)

        point = PointStamped()
        point.header.frame_id = camera_frame
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x, point.point.y, point.point.z = x, y, z

        try:
            point_tf = self._tf_buffer.lookup_transform(
                target_frame, camera_frame, Time(), timeout=Duration(seconds=0.5)
            )
            point_base = do_transform_point(point, point_tf)
            tool_tf = self._tf_buffer.lookup_transform(
                target_frame, tool_frame, Time(), timeout=Duration(seconds=0.5)
            )
        except TransformException as exception:
            raise RuntimeError(f"TF unavailable: {exception}") from exception

        target = TargetObject()
        target.id = "gui_target"
        target.class_name = "manual_roi"
        target.confidence = 1.0
        target.pose.header.frame_id = target_frame
        target.pose.header.stamp = self.get_clock().now().to_msg()
        target.pose.pose.position = point_base.point
        target.pose.pose.orientation = tool_tf.transform.rotation
        self._target_pub.publish(target)
        with self._lock:
            self._target = target
        self._set_status(
            f"target: x={point_base.point.x:.3f} y={point_base.point.y:.3f} z={point_base.point.z:.3f} m"
        )
        return target

    def send_pick(self, pre_pick: float, approach: float, retreat: float, retry: int, use_eef: bool, go_home: bool) -> None:
        with self._lock:
            target = self._target
        if target is None:
            raise RuntimeError("select and confirm a target first")
        if not self._pick_client.wait_for_server(timeout_sec=1.0):
            raise RuntimeError("PickTarget action is unavailable")

        goal = PickTarget.Goal()
        goal.target_pose = target.pose
        goal.use_pre_pick_pose = False
        goal.use_approach_pose = False
        goal.use_retreat_pose = False
        goal.use_place_pose = False
        goal.use_eef = use_eef
        goal.go_home_after_finish = go_home
        goal.pre_pick_distance = pre_pick
        goal.approach_distance = approach
        goal.retreat_distance = retreat
        goal.retry_times = retry

        self._set_status("PickTarget goal submitted")
        future = self._pick_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, message) -> None:
        feedback = message.feedback
        self._set_status(
            f"{feedback.stage_text}: {feedback.completed_steps}/{feedback.total_steps}"
        )

    def _goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._set_status("PickTarget goal rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        result = future.result().result
        self._set_status(f"PickTarget: {result.message}")


class ImageCanvas(QWidget):
    polygon_changed = Signal()

    def __init__(self):
        super().__init__()
        self.setMinimumSize(900, 600)
        self._image: Optional[QImage] = None
        self._image_size = (0, 0)
        self._points: list[QPointF] = []
        self._selected: Optional[QPointF] = None

    @property
    def points(self) -> list[dict]:
        return [{"x": p.x(), "y": p.y()} for p in self._points]

    def set_frame(self, bgr: np.ndarray) -> None:
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])
        h, w = rgb.shape[:2]
        self._image = QImage(rgb.data, w, h, 3 * w, QImage.Format.Format_RGB888).copy()
        self._image_size = (w, h)
        self.update()

    def clear_selection(self) -> None:
        self._points.clear()
        self._selected = None
        self.polygon_changed.emit()
        self.update()

    def set_selected_pixel(self, x: float, y: float) -> None:
        self._selected = QPointF(x, y)
        self.update()

    def _draw_rect(self) -> QRectF:
        if self._image is None:
            return QRectF()
        iw, ih = self._image_size
        scale = min(self.width() / iw, self.height() / ih)
        w, h = iw * scale, ih * scale
        return QRectF((self.width() - w) / 2.0, (self.height() - h) / 2.0, w, h)

    def _to_image(self, pos: QPointF) -> Optional[QPointF]:
        rect = self._draw_rect()
        if not rect.contains(pos) or self._image is None:
            return None
        iw, ih = self._image_size
        x = (pos.x() - rect.x()) * iw / rect.width()
        y = (pos.y() - rect.y()) * ih / rect.height()
        return QPointF(max(0.0, min(iw - 1.0, x)), max(0.0, min(ih - 1.0, y)))

    def _to_widget(self, point: QPointF) -> QPointF:
        rect = self._draw_rect()
        iw, ih = self._image_size
        return QPointF(rect.x() + point.x() * rect.width() / iw, rect.y() + point.y() * rect.height() / ih)

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            point = self._to_image(event.position())
            if point is not None:
                self._points.append(point)
                self._selected = None
                self.polygon_changed.emit()
                self.update()
        elif event.button() == Qt.MouseButton.RightButton:
            self.clear_selection()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), Qt.GlobalColor.black)
        if self._image is None:
            return
        rect = self._draw_rect()
        painter.drawImage(rect, self._image)

        painter.setPen(QPen(Qt.GlobalColor.yellow, 2))
        if self._points:
            polygon = QPolygonF([self._to_widget(p) for p in self._points])
            painter.drawPolyline(polygon)
            if len(self._points) >= 3:
                painter.drawLine(self._to_widget(self._points[-1]), self._to_widget(self._points[0]))
            for point in self._points:
                p = self._to_widget(point)
                painter.drawEllipse(p, 3, 3)

        if self._selected is not None:
            painter.setPen(QPen(Qt.GlobalColor.red, 2))
            p = self._to_widget(self._selected)
            painter.drawLine(QPointF(p.x() - 8, p.y()), QPointF(p.x() + 8, p.y()))
            painter.drawLine(QPointF(p.x(), p.y() - 8), QPointF(p.x(), p.y() + 8))


class MainWindow(QMainWindow):
    def __init__(self, bridge: RosBridge, config: dict):
        super().__init__()
        self._bridge = bridge
        self._config = config
        self._canvas = ImageCanvas()
        self._status = QLabel("waiting")
        self._last_color: Optional[np.ndarray] = None

        self.setWindowTitle(config["gui"]["title"])
        self.resize(int(config["gui"]["width"]), int(config["gui"]["height"]))

        confirm = QPushButton("确认框选目标")
        confirm.clicked.connect(self._confirm_target)
        clear = QPushButton("清除 ROI")
        clear.clicked.connect(self._canvas.clear_selection)
        send = QPushButton("执行 PickTarget")
        send.clicked.connect(self._send_pick)

        pick = config["pick"]
        self._pre_pick = self._distance_box(float(pick["pre_pick_distance"]))
        self._approach = self._distance_box(float(pick["approach_distance"]))
        self._retreat = self._distance_box(float(pick["retreat_distance"]))
        self._retry = QSpinBox()
        self._retry.setRange(0, 20)
        self._retry.setValue(int(pick["retry_times"]))
        self._use_eef = QCheckBox()
        self._use_eef.setChecked(bool(pick["use_eef"]))
        self._go_home = QCheckBox()
        self._go_home.setChecked(bool(pick["go_home_after_finish"]))

        form = QFormLayout()
        form.addRow("Pre-pick [m]", self._pre_pick)
        form.addRow("Approach [m]", self._approach)
        form.addRow("Retreat [m]", self._retreat)
        form.addRow("Retry", self._retry)
        form.addRow("Use EEF", self._use_eef)
        form.addRow("Go home", self._go_home)

        buttons = QHBoxLayout()
        buttons.addWidget(confirm)
        buttons.addWidget(clear)
        buttons.addWidget(send)

        panel = QVBoxLayout()
        panel.addLayout(form)
        panel.addLayout(buttons)
        panel.addWidget(self._status)
        panel.addStretch(1)

        root = QHBoxLayout()
        root.addWidget(self._canvas, 1)
        root.addLayout(panel)
        widget = QWidget()
        widget.setLayout(root)
        self.setCentralWidget(widget)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(33)

    @staticmethod
    def _distance_box(value: float) -> QDoubleSpinBox:
        box = QDoubleSpinBox()
        box.setRange(0.001, 1.0)
        box.setDecimals(3)
        box.setSingleStep(0.01)
        box.setValue(value)
        return box

    def _refresh(self) -> None:
        color, _, _, _ = self._bridge.snapshot()
        if color is not None:
            self._last_color = color
            self._canvas.set_frame(color)
        self._status.setText(self._bridge.status())

    def _confirm_target(self) -> None:
        try:
            color, depth, k, frame = self._bridge.snapshot()
            if color is None or depth is None or k is None or not frame:
                raise RuntimeError("wrist RGB-D or CameraInfo is not ready")
            if len(self._canvas.points) < 3:
                raise RuntimeError("select at least three ROI vertices")
            h, w = color.shape[:2]
            if depth.shape[:2] != (h, w):
                raise RuntimeError("depth must be registered to the color image")

            select = self._config["selection"]
            pixel, mask = select_pixel(
                w,
                h,
                self._canvas.points,
                str(select["point_method"]),
                int(select["mask_close_kernel"]),
                int(select["mask_open_kernel"]),
            )
            u, v = int(round(pixel[0])), int(round(pixel[1]))
            depth_mm = valid_depth(
                depth,
                u,
                v,
                mask,
                int(select["depth_kernel_size"]),
                float(select["depth_trim_ratio"]),
                int(select["mask_close_kernel"]),
                int(select["mask_open_kernel"]),
            )
            if depth_mm <= 0.0:
                raise RuntimeError("no valid depth inside ROI")
            self._bridge.make_target(u, v, depth_mm, k, frame)
            self._canvas.set_selected_pixel(u, v)
        except Exception as exception:
            QMessageBox.warning(self, "Target selection failed", str(exception))

    def _send_pick(self) -> None:
        try:
            self._bridge.send_pick(
                self._pre_pick.value(),
                self._approach.value(),
                self._retreat.value(),
                self._retry.value(),
                self._use_eef.isChecked(),
                self._go_home.isChecked(),
            )
        except Exception as exception:
            QMessageBox.warning(self, "PickTarget failed", str(exception))


def load_config(path: str) -> dict:
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def main() -> int:
    parser = argparse.ArgumentParser(description="Tomato-Picker wrist RGB-D manual target GUI")
    default_config = Path(get_package_share_directory("tomato_picker_gui")) / "config" / "gui.yaml"
    parser.add_argument("--config", default=str(default_config))
    args, ros_args = parser.parse_known_args()

    config = load_config(args.config)
    rclpy.init(args=ros_args)
    bridge = RosBridge(config)
    spin_thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv[:1])
    window = MainWindow(bridge, config)
    window.show()
    exit_code = app.exec()

    rclpy.shutdown()
    spin_thread.join(timeout=1.0)
    bridge.destroy_node()
    return int(exit_code)


if __name__ == "__main__":
    raise SystemExit(main())
