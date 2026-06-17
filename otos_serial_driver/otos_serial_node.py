import math
import threading
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_srvs.srv import Empty
import serial
from tf2_ros import TransformBroadcaster


class OtosSerialNode(Node):
    SCALAR_MIN = 0.872
    SCALAR_MAX = 1.127

    def __init__(self) -> None:
        super().__init__("otos_serial_node")

        self.declare_parameter("serial_port", "/dev/arduino")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("linear_scalar", 1.0)
        self.declare_parameter("angular_scalar", 1.0)
        self.declare_parameter("use_sensor_stddev_covariance", True)
        self.declare_parameter(
            "pose_covariance_diagonal_fallback",
            [0.02, 0.02, 99999.0, 99999.0, 99999.0, 0.05],
        )
        self.declare_parameter(
            "twist_covariance_diagonal_fallback",
            [0.05, 0.05, 99999.0, 99999.0, 99999.0, 0.1],
        )

        self._serial_port_name = str(self.get_parameter("serial_port").value)
        self._baud_rate = int(self.get_parameter("baud_rate").value)
        self._linear_scalar = float(self.get_parameter("linear_scalar").value)
        self._angular_scalar = float(self.get_parameter("angular_scalar").value)

        self._serial_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._pending_ack: Optional[tuple[str, threading.Event, dict]] = None
        self._stop_event = threading.Event()
        self._serial_conn: Optional[serial.Serial] = None

        self._odom_pub = self.create_publisher(Odometry, "/otos_odom", 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_service(Empty, "/otos/reset", self._on_reset)
        self.create_service(Empty, "/otos/calibrate", self._on_calibrate)

        self.add_on_set_parameters_callback(self._on_parameters_set)

        self._reader_thread = threading.Thread(target=self._serial_loop, daemon=True)
        self._reader_thread.start()

        self.get_logger().info("otos_serial_node started")

    def _validate_cov_diag(self, values: List[float], param_name: str, default: List[float]) -> List[float]:
        if len(values) != 6:
            self.get_logger().warn(
                f"{param_name} must have 6 values; using default {default}."
            )
            return default
        return values

    def _diag6_to_cov36(self, diag6: List[float]) -> List[float]:
        cov = [0.0] * 36
        cov[0] = float(diag6[0])
        cov[7] = float(diag6[1])
        cov[14] = float(diag6[2])
        cov[21] = float(diag6[3])
        cov[28] = float(diag6[4])
        cov[35] = float(diag6[5])
        return cov

    def _make_covariances_from_sample(self, sample: List[float]) -> tuple[List[float], List[float]]:
        pose_fallback = list(self.get_parameter("pose_covariance_diagonal_fallback").value)
        twist_fallback = list(self.get_parameter("twist_covariance_diagonal_fallback").value)

        pose_fallback = self._validate_cov_diag(
            [float(v) for v in pose_fallback],
            "pose_covariance_diagonal_fallback",
            [0.02, 0.02, 99999.0, 99999.0, 99999.0, 0.05],
        )
        twist_fallback = self._validate_cov_diag(
            [float(v) for v in twist_fallback],
            "twist_covariance_diagonal_fallback",
            [0.05, 0.05, 99999.0, 99999.0, 99999.0, 0.1],
        )

        pose_cov = self._diag6_to_cov36(pose_fallback)
        twist_cov = self._diag6_to_cov36(twist_fallback)

        if bool(self.get_parameter("use_sensor_stddev_covariance").value):
            sx, sy, sh = sample[9], sample[10], sample[11]
            svx, svy, swz = sample[12], sample[13], sample[14]
            pose_cov[0] = sx * sx
            pose_cov[7] = sy * sy
            pose_cov[35] = sh * sh
            twist_cov[0] = svx * svx
            twist_cov[7] = svy * svy
            twist_cov[35] = swz * swz

        return pose_cov, twist_cov

    def _publish_sample(self, sample: List[float]) -> None:
        x, y, h, vx, vy, wz, _, _, _ = sample[:9]

        frame_id = str(self.get_parameter("frame_id").value)
        child_frame_id = str(self.get_parameter("child_frame_id").value)
        publish_tf = bool(self.get_parameter("publish_tf").value)

        pose_cov, twist_cov = self._make_covariances_from_sample(sample)

        now_msg = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = frame_id
        odom.child_frame_id = child_frame_id

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(h * 0.5)
        odom.pose.pose.orientation.w = math.cos(h * 0.5)
        odom.pose.covariance = pose_cov

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = wz
        odom.twist.covariance = twist_cov

        self._odom_pub.publish(odom)

        if publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now_msg
            tf_msg.header.frame_id = frame_id
            tf_msg.child_frame_id = child_frame_id
            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf_msg)

    def _parse_data_line(self, line: str) -> Optional[List[float]]:
        parts = line.split(",")
        if len(parts) != 18:
            return None
        try:
            values = [float(p) for p in parts]
        except ValueError:
            return None
        return values

    def _handle_status_line(self, line: str) -> None:
        with self._command_lock:
            if self._pending_ack is None:
                return

            expected_cmd, event, result = self._pending_ack
            if line.startswith(f"#OK,{expected_cmd}"):
                result["ok"] = True
                event.set()
            elif line.startswith(f"#ERR,{expected_cmd}"):
                result["ok"] = False
                event.set()

    def _serial_loop(self) -> None:
        reconnect_delay_sec = 1.0

        while not self._stop_event.is_set():
            if self._serial_conn is None:
                try:
                    conn = serial.Serial(self._serial_port_name, self._baud_rate, timeout=1.0)
                    with self._serial_lock:
                        self._serial_conn = conn
                    self.get_logger().info(
                        f"Connected to serial device {self._serial_port_name} @ {self._baud_rate}"
                    )
                    self._send_scalar_command_no_wait(self._linear_scalar, self._angular_scalar)
                except serial.SerialException as exc:
                    self.get_logger().warn(
                        f"Unable to open serial port {self._serial_port_name}: {exc}"
                    )
                    time.sleep(reconnect_delay_sec)
                    continue

            try:
                assert self._serial_conn is not None
                raw = self._serial_conn.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                if line.startswith("#"):
                    self._handle_status_line(line)
                    continue

                sample = self._parse_data_line(line)
                if sample is None:
                    self.get_logger().warn(f"Dropped malformed line: {line}")
                    continue

                self._publish_sample(sample)

            except (serial.SerialException, OSError) as exc:
                self.get_logger().warn(f"Serial connection lost: {exc}")
                self._close_serial_connection()
                time.sleep(reconnect_delay_sec)

    def _close_serial_connection(self) -> None:
        with self._serial_lock:
            if self._serial_conn is not None:
                try:
                    self._serial_conn.close()
                except Exception:
                    pass
                self._serial_conn = None

    def _write_line(self, text: str) -> bool:
        with self._serial_lock:
            if self._serial_conn is None:
                return False
            try:
                self._serial_conn.write(text.encode("utf-8"))
                self._serial_conn.flush()
                return True
            except serial.SerialException:
                self._serial_conn = None
                return False

    def _send_command_wait_ack(self, cmd_code: str, line: str, timeout_sec: float = 2.0) -> bool:
        with self._command_lock:
            event = threading.Event()
            result = {"ok": False}
            self._pending_ack = (cmd_code, event, result)

            if not self._write_line(line):
                self._pending_ack = None
                return False

            event.wait(timeout_sec)
            ok = bool(result["ok"])
            self._pending_ack = None
            return ok

    def _send_scalar_command_no_wait(self, linear_scalar: float, angular_scalar: float) -> None:
        command = f"S,{linear_scalar:.3f},{angular_scalar:.3f}\n"
        if not self._write_line(command):
            self.get_logger().warn("Failed to write scalar command (serial unavailable)")

    def _set_scalars(self, linear_scalar: float, angular_scalar: float) -> bool:
        command = f"S,{linear_scalar:.3f},{angular_scalar:.3f}\n"
        return self._send_command_wait_ack("S", command)

    def _validate_scalar_range(self, linear_scalar: float, angular_scalar: float) -> Optional[str]:
        if not (self.SCALAR_MIN <= linear_scalar <= self.SCALAR_MAX):
            return (
                f"linear_scalar out of range [{self.SCALAR_MIN}, {self.SCALAR_MAX}]: "
                f"{linear_scalar}"
            )
        if not (self.SCALAR_MIN <= angular_scalar <= self.SCALAR_MAX):
            return (
                f"angular_scalar out of range [{self.SCALAR_MIN}, {self.SCALAR_MAX}]: "
                f"{angular_scalar}"
            )
        return None

    def _on_parameters_set(self, params):
        proposed_linear = self._linear_scalar
        proposed_angular = self._angular_scalar
        serial_port_updated = False
        baud_rate_updated = False

        for param in params:
            if param.name == "linear_scalar":
                proposed_linear = float(param.value)
            elif param.name == "angular_scalar":
                proposed_angular = float(param.value)
            elif param.name == "serial_port":
                serial_port_updated = True
            elif param.name == "baud_rate":
                baud_rate_updated = True

        error = self._validate_scalar_range(proposed_linear, proposed_angular)
        if error is not None:
            return SetParametersResult(successful=False, reason=error)

        scalar_changed = (
            abs(proposed_linear - self._linear_scalar) > 1e-9
            or abs(proposed_angular - self._angular_scalar) > 1e-9
        )

        if scalar_changed:
            if not self._set_scalars(proposed_linear, proposed_angular):
                return SetParametersResult(
                    successful=False,
                    reason="Failed to apply scalars to Arduino (serial unavailable or NACK).",
                )
            self._linear_scalar = proposed_linear
            self._angular_scalar = proposed_angular
            self.get_logger().info(
                f"Applied scalars: linear={self._linear_scalar:.3f}, angular={self._angular_scalar:.3f}"
            )

        if serial_port_updated or baud_rate_updated:
            for param in params:
                if param.name == "serial_port":
                    self._serial_port_name = str(param.value)
                elif param.name == "baud_rate":
                    self._baud_rate = int(param.value)
            self._close_serial_connection()
            self.get_logger().info(
                f"Serial settings changed; reconnecting to {self._serial_port_name} @ {self._baud_rate}"
            )

        return SetParametersResult(successful=True)

    def _on_reset(self, _request, response):
        if self._send_command_wait_ack("R", "R\n"):
            return response
        self.get_logger().error("Reset command failed (serial unavailable or NACK)")
        return response

    def _on_calibrate(self, _request, response):
        if self._send_command_wait_ack("C", "C\n", timeout_sec=5.0):
            return response
        self.get_logger().error("Calibrate command failed (serial unavailable or NACK)")
        return response

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=2.0)
        self._close_serial_connection()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OtosSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
