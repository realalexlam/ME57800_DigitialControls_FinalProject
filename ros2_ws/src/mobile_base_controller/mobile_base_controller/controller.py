import glob
import math
import threading
from dataclasses import dataclass
from typing import Optional

from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def rad_s_to_rpm(rad_s: float) -> float:
    return rad_s * 60.0 / (2.0 * math.pi)


def rpm_to_rad_s(rpm: float) -> float:
    return rpm * (2.0 * math.pi) / 60.0


def find_tty_acm() -> str:
    devices = sorted(glob.glob("/dev/ttyACM*"))
    if not devices:
        raise RuntimeError("No /dev/ttyACM* device found")
    return devices[0]


@dataclass
class DiscretePI:
    kp: float
    ki: float
    kff: float = 0.0
    integral: float = 0.0

    def reset(self) -> None:
        self.integral = 0.0

    def update(
        self,
        error: float,
        ref: float,
        Ts: float,
        umin: float,
        umax: float,
    ) -> float:
        i_new = self.integral + Ts * error
        u_unsat = self.kp * error + self.ki * i_new + self.kff * ref
        u = clamp(u_unsat, umin, umax)

        if abs(u_unsat - u) < 1e-9:
            self.integral = i_new

        return u


class CascadeTagStraighteningController(Node):
    def __init__(self) -> None:
        super().__init__("cascade_tag_straightening_controller")

        self.declare_parameter("port", "")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("serial_timeout", 0.05)
        self.declare_parameter("control_hz", 30.0)
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("tag_timeout_sec", 0.3)
        self.declare_parameter("wheel_timeout_sec", 0.3)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("track_width", 0.28)
        self.declare_parameter("left_joint_name", "left_wheel_joint")
        self.declare_parameter("right_joint_name", "right_wheel_joint")
        self.declare_parameter("k_heading", 0.896413)
        self.declare_parameter("k_lateral", 1.000012)
        self.declare_parameter("heading_deadband", 0.015866)
        self.declare_parameter("lateral_deadband", 0.001035)
        self.declare_parameter("invert_tag_x", False)
        self.declare_parameter("invert_tag_yaw", False)
        self.declare_parameter("camera_filter_alpha", 0.25)
        self.declare_parameter("tag_fade_start_sec", 0.15)
        self.declare_parameter("k_rpm_balance", 1.346468)
        self.declare_parameter("rpm_balance_deadband", 0.005353)
        self.declare_parameter("rpm_ignore_threshold", 5.0)
        # Incoming /joint_states velocity values for this robot are RPM.
        self.declare_parameter("encoder_units", "rpm")
        # Inner-loop deadbands stay in rad/s because PI control uses rad/s.
        self.declare_parameter("encoder_deadband", 0.05)
        self.declare_parameter("wheel_error_deadband", 0.05)
        self.declare_parameter("motor_cmd_deadband", 20.0)
        self.declare_parameter("kp_left", 25.0)
        self.declare_parameter("ki_left", 6.0)
        self.declare_parameter("kff_left", 0.0)
        self.declare_parameter("kp_right", 25.0)
        self.declare_parameter("ki_right", 6.0)
        self.declare_parameter("kff_right", 0.0)
        self.declare_parameter("motor_cmd_limit", 3000.0)
        self.declare_parameter("invert_left_encoder", False)
        self.declare_parameter("invert_right_encoder", False)
        self.declare_parameter("max_steering_correction", 1.785322)
        self.declare_parameter("max_forward_speed", 1.0)
        self.declare_parameter("max_user_yaw_rate", 2.0)
        self.declare_parameter("straight_assist_min_speed", 0.05)
        self.declare_parameter("straight_assist_yaw_threshold", 0.15)

        port_param = str(self.get_parameter("port").value)
        self.port = port_param if port_param else find_tty_acm()
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.serial_timeout = float(self.get_parameter("serial_timeout").value)
        self.control_hz = float(self.get_parameter("control_hz").value)
        self.Ts = 1.0 / self.control_hz
        self.cmd_timeout_sec = float(self.get_parameter("cmd_timeout_sec").value)
        self.tag_timeout_sec = float(self.get_parameter("tag_timeout_sec").value)
        self.wheel_timeout_sec = float(self.get_parameter("wheel_timeout_sec").value)
        self.r = float(self.get_parameter("wheel_radius").value)
        self.L = float(self.get_parameter("track_width").value)
        self.left_joint_name = str(self.get_parameter("left_joint_name").value)
        self.right_joint_name = str(self.get_parameter("right_joint_name").value)
        self.k_heading = float(self.get_parameter("k_heading").value)
        self.k_lateral = float(self.get_parameter("k_lateral").value)
        self.heading_deadband = float(self.get_parameter("heading_deadband").value)
        self.lateral_deadband = float(self.get_parameter("lateral_deadband").value)
        self.invert_tag_x = bool(self.get_parameter("invert_tag_x").value)
        self.invert_tag_yaw = bool(self.get_parameter("invert_tag_yaw").value)
        self.camera_filter_alpha = float(self.get_parameter("camera_filter_alpha").value)
        self.tag_fade_start_sec = float(self.get_parameter("tag_fade_start_sec").value)
        self.k_rpm_balance = float(self.get_parameter("k_rpm_balance").value)
        self.rpm_balance_deadband = float(self.get_parameter("rpm_balance_deadband").value)
        self.rpm_ignore_threshold = float(self.get_parameter("rpm_ignore_threshold").value)
        self.encoder_units = str(self.get_parameter("encoder_units").value).strip().lower()
        self.encoder_deadband = float(self.get_parameter("encoder_deadband").value)
        self.wheel_error_deadband = float(self.get_parameter("wheel_error_deadband").value)
        self.motor_cmd_deadband = float(self.get_parameter("motor_cmd_deadband").value)
        kp_left = float(self.get_parameter("kp_left").value)
        ki_left = float(self.get_parameter("ki_left").value)
        kff_left = float(self.get_parameter("kff_left").value)
        kp_right = float(self.get_parameter("kp_right").value)
        ki_right = float(self.get_parameter("ki_right").value)
        kff_right = float(self.get_parameter("kff_right").value)
        self.motor_cmd_limit = float(self.get_parameter("motor_cmd_limit").value)
        self.invert_left_encoder = bool(self.get_parameter("invert_left_encoder").value)
        self.invert_right_encoder = bool(self.get_parameter("invert_right_encoder").value)
        self.max_steering_correction = float(self.get_parameter("max_steering_correction").value)
        self.max_forward_speed = float(self.get_parameter("max_forward_speed").value)
        self.max_user_yaw_rate = float(self.get_parameter("max_user_yaw_rate").value)
        self.straight_assist_min_speed = float(self.get_parameter("straight_assist_min_speed").value)
        self.straight_assist_yaw_threshold = float(self.get_parameter("straight_assist_yaw_threshold").value)

        if self.encoder_units not in ("rad_s", "rpm"):
            raise ValueError("encoder_units must be 'rad_s' or 'rpm'")
        if not 0.0 < self.camera_filter_alpha <= 1.0:
            raise ValueError("camera_filter_alpha must be in (0, 1]")
        if not 0.0 <= self.tag_fade_start_sec <= self.tag_timeout_sec:
            raise ValueError("tag_fade_start_sec must be between 0 and tag_timeout_sec")

        self.left_pi = DiscretePI(kp=kp_left, ki=ki_left, kff=kff_left)
        self.right_pi = DiscretePI(kp=kp_right, ki=ki_right, kff=kff_right)

        self.v_cmd = 0.0
        self.omega_cmd = 0.0
        self.tag_x = 0.0
        self.tag_z = 0.0
        self.tag_yaw = 0.0
        self.tag_x_ref = 0.0
        self.tag_yaw_ref = 0.0
        self.tag_ref_valid = False
        self.was_straight_assist_enabled = False
        self.w_left = 0.0
        self.w_right = 0.0
        self.left_rpm = 0.0
        self.right_rpm = 0.0
        self.last_cmd_time: Optional[float] = None
        self.last_tag_time: Optional[float] = None
        self.last_wheel_time: Optional[float] = None

        self.lock = threading.Lock()
        self.ser = None
        self.connect_serial()

        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)
        self.create_subscription(PoseStamped, "/tag_pose", self.tag_callback, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_callback, 20)
        self.timer = self.create_timer(self.Ts, self.control_step)

        self.get_logger().info(f"Controller started on port {self.port}")
        self.get_logger().info(
            f"Camera+RPM straight-line correction active: "
            f"k_heading={self.k_heading:.6f}, "
            f"k_lateral={self.k_lateral:.6f}, "
            f"k_rpm_balance={self.k_rpm_balance:.6f}, "
            f"heading_deadband={self.heading_deadband:.6f}, "
            f"lateral_deadband={self.lateral_deadband:.6f}, "
            f"rpm_balance_deadband={self.rpm_balance_deadband:.6f}, "
            f"rpm_ignore_threshold={self.rpm_ignore_threshold:.1f}, "
            f"tag_timeout_sec={self.tag_timeout_sec:.2f}, "
            f"tag_fade_start_sec={self.tag_fade_start_sec:.2f}, "
            f"straight_assist_min_speed={self.straight_assist_min_speed:.2f}, "
            f"straight_assist_yaw_threshold={self.straight_assist_yaw_threshold:.2f}, "
            f"max_steering_correction={self.max_steering_correction:.6f}, "
            f"encoder_units={self.encoder_units}"
        )
        self.get_logger().info(
            "Wheel feedback is converted to rad/s internally for PI control."
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def connect_serial(self) -> None:
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=self.serial_timeout,
            )
            self.get_logger().info(f"Connected to {self.port}")
            self.send_data("^RWD 30000\r")
        except Exception as e:
            self.get_logger().error(f"Could not open serial port {self.port}: {e}")
            self.ser = None

    def disconnect_serial(self) -> None:
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.close()

    def send_data(self, data: str) -> None:
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.write(data.encode())
            else:
                self.get_logger().error("Serial port not open")

    def vel_control(self, left_cmd: float, right_cmd: float) -> None:
        if abs(left_cmd) < self.motor_cmd_deadband:
            left_cmd = 0.0
        if abs(right_cmd) < self.motor_cmd_deadband:
            right_cmd = 0.0

        left_cmd = int(clamp(left_cmd, -self.motor_cmd_limit, self.motor_cmd_limit))
        right_cmd = int(clamp(right_cmd, -self.motor_cmd_limit, self.motor_cmd_limit))

        self.send_data(f"!S 1 {left_cmd}\r")
        self.send_data(f"!S 2 {right_cmd}\r")

    def cmd_callback(self, msg: Twist) -> None:
        self.v_cmd = clamp(msg.linear.x, -self.max_forward_speed, self.max_forward_speed)
        self.omega_cmd = clamp(msg.angular.z, -self.max_user_yaw_rate, self.max_user_yaw_rate)
        self.last_cmd_time = self.now_sec()

    def tag_callback(self, msg: PoseStamped) -> None:
        x = msg.pose.position.x
        z = msg.pose.position.z

        # For straight-driving correction, the camera frame is interpreted as:
        #   x = left/right offset from the marker
        #   z = forward depth to the marker
        # The heading surrogate is therefore the bearing angle atan2(x, z).
        z_safe = max(z, 1e-6)
        yaw = math.atan2(x, z_safe)

        if self.invert_tag_x:
            x = -x
        if self.invert_tag_yaw:
            yaw = -yaw

        yaw = wrap_to_pi(yaw)

        # Exponential smoothing keeps one noisy frame from producing a large
        # steering jump. The most recent filtered value is still held briefly
        # when the camera drops out.
        if self.last_tag_time is None:
            self.tag_x = x
            self.tag_z = z
            self.tag_yaw = yaw
        else:
            alpha = self.camera_filter_alpha
            self.tag_x = alpha * x + (1.0 - alpha) * self.tag_x
            self.tag_z = alpha * z + (1.0 - alpha) * self.tag_z
            yaw_delta = wrap_to_pi(yaw - self.tag_yaw)
            self.tag_yaw = wrap_to_pi(self.tag_yaw + alpha * yaw_delta)

        self.last_tag_time = self.now_sec()

    def joint_callback(self, msg: JointState) -> None:
        if not msg.name or not msg.velocity:
            return

        try:
            li = msg.name.index(self.left_joint_name)
            ri = msg.name.index(self.right_joint_name)
        except ValueError:
            self.get_logger().warn("Wheel joint names not found in /joint_states")
            return

        if li >= len(msg.velocity) or ri >= len(msg.velocity):
            return

        wl_raw = msg.velocity[li]
        wr_raw = msg.velocity[ri]

        if self.invert_left_encoder:
            wl_raw = -wl_raw
        if self.invert_right_encoder:
            wr_raw = -wr_raw

        if self.encoder_units == "rpm":
            self.left_rpm = wl_raw
            self.right_rpm = wr_raw
            wl = rpm_to_rad_s(wl_raw)
            wr = rpm_to_rad_s(wr_raw)
        else:
            wl = wl_raw
            wr = wr_raw
            self.left_rpm = rad_s_to_rpm(wl_raw)
            self.right_rpm = rad_s_to_rpm(wr_raw)

        if abs(wl) < self.encoder_deadband:
            wl = 0.0
        if abs(wr) < self.encoder_deadband:
            wr = 0.0

        self.w_left = wl
        self.w_right = wr
        self.last_wheel_time = self.now_sec()

    def reset_controllers(self) -> None:
        self.left_pi.reset()
        self.right_pi.reset()

    def stop_robot(self) -> None:
        self.reset_controllers()
        self.vel_control(0.0, 0.0)

    def straight_assist_enabled(self) -> bool:
        # Only apply "keep me straight" corrections when the operator is
        # effectively asking for straight-line travel. This prevents the
        # camera/RPM assist from fighting intentional turns.
        return (
            abs(self.v_cmd) >= self.straight_assist_min_speed
            and abs(self.omega_cmd) <= self.straight_assist_yaw_threshold
        )

    def fresh_tag_available(self, t_now: float) -> bool:
        return (
            self.last_tag_time is not None
            and (t_now - self.last_tag_time) <= self.tag_timeout_sec
        )

    def latch_tag_reference(self) -> None:
        # Store the visual relationship to the tag at the beginning of a
        # straight run. Future camera corrections are based on deviation from
        # this remembered reference, not on trying to point directly at the tag.
        self.tag_x_ref = self.tag_x
        self.tag_yaw_ref = self.tag_yaw
        self.tag_ref_valid = True

    def control_step(self) -> None:
        t_now = self.now_sec()

        if self.last_cmd_time is None or (t_now - self.last_cmd_time) > self.cmd_timeout_sec:
            self.stop_robot()
            return

        if self.last_wheel_time is None or (t_now - self.last_wheel_time) > self.wheel_timeout_sec:
            self.get_logger().warn("Wheel feedback stale; stopping robot")
            self.stop_robot()
            return

        camera_corr = 0.0
        rpm_corr = 0.0

        straight_assist_active = self.straight_assist_enabled()
        fresh_tag = self.fresh_tag_available(t_now)

        if straight_assist_active and not self.was_straight_assist_enabled:
            if fresh_tag:
                self.latch_tag_reference()
            else:
                self.tag_ref_valid = False
        elif not straight_assist_active:
            self.tag_ref_valid = False

        self.was_straight_assist_enabled = straight_assist_active

        if straight_assist_active:
            if fresh_tag and not self.tag_ref_valid:
                self.latch_tag_reference()

            if self.last_tag_time is not None and self.tag_ref_valid:
                tag_age = t_now - self.last_tag_time
                if tag_age <= self.tag_timeout_sec:
                    e_y = self.tag_x - self.tag_x_ref
                    e_psi = wrap_to_pi(self.tag_yaw - self.tag_yaw_ref)

                    if abs(e_y) < self.lateral_deadband:
                        e_y = 0.0
                    if abs(e_psi) < self.heading_deadband:
                        e_psi = 0.0

                    # The AprilTag acts as a straightness reference. We first
                    # remember the tag pose at the start of a straight run,
                    # then correct only deviations away from that remembered
                    # view rather than steering toward the raw tag position.
                    camera_corr = self.k_heading * e_psi + self.k_lateral * e_y

                    # Hold the last filtered camera estimate briefly, then fade
                    # it out to zero as the tag becomes stale. Once the tag is
                    # fully stale, straightness falls back to RPM balancing.
                    if tag_age > self.tag_fade_start_sec:
                        fade_window = max(self.tag_timeout_sec - self.tag_fade_start_sec, 1e-6)
                        fade = 1.0 - ((tag_age - self.tag_fade_start_sec) / fade_window)
                        camera_corr *= clamp(fade, 0.0, 1.0)

            if abs(self.left_rpm) > self.rpm_ignore_threshold and abs(self.right_rpm) > self.rpm_ignore_threshold:
                rpm_avg = max((abs(self.left_rpm) + abs(self.right_rpm)) / 2.0, 1.0)
                e_rpm = (self.right_rpm - self.left_rpm) / rpm_avg
                if abs(e_rpm) >= self.rpm_balance_deadband:
                    rpm_corr = self.k_rpm_balance * e_rpm

        steering_corr = camera_corr + rpm_corr
        steering_corr = clamp(
            steering_corr,
            -self.max_steering_correction,
            self.max_steering_correction,
        )
        omega_ref = self.omega_cmd - steering_corr

        w_left_ref = (self.v_cmd - 0.5 * self.L * omega_ref) / self.r
        w_right_ref = (self.v_cmd + 0.5 * self.L * omega_ref) / self.r

        e_left = w_left_ref - self.w_left
        e_right = w_right_ref - self.w_right

        if abs(e_left) < self.wheel_error_deadband:
            e_left = 0.0
        if abs(e_right) < self.wheel_error_deadband:
            e_right = 0.0

        u_left = self.left_pi.update(
            error=e_left,
            ref=w_left_ref,
            Ts=self.Ts,
            umin=-self.motor_cmd_limit,
            umax=self.motor_cmd_limit,
        )
        u_right = self.right_pi.update(
            error=e_right,
            ref=w_right_ref,
            Ts=self.Ts,
            umin=-self.motor_cmd_limit,
            umax=self.motor_cmd_limit,
        )

        self.vel_control(u_left, u_right)

    def destroy_node(self):
        try:
            self.stop_robot()
        except Exception:
            pass
        self.disconnect_serial()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CascadeTagStraighteningController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down controller")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
