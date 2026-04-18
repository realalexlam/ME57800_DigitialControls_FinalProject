import math
import glob
import threading
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
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


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def find_tty_acm() -> str:
    devices = sorted(glob.glob("/dev/ttyACM*"))
    if not devices:
        raise RuntimeError("No /dev/ttyACM* device found")
    return devices[0]


@dataclass
class DiscretePI:
    """
    Discrete PI controller:
        I[k] = I[k-1] + Ts * e[k]
        u[k] = Kp * e[k] + Ki * I[k] + Kff * ref[k]
    """
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

        # Simple anti-windup: only accept integral update if not saturated
        if abs(u_unsat - u) < 1e-9:
            self.integral = i_new

        return u


class CascadeTagStraighteningController(Node):
    """
    Cascaded discrete-time controller for a differential-drive robot.

    Outer loop:
        camera/tag feedback -> steering correction

    Reference generator:
        base motion + steering correction -> desired wheel speeds

    Inner loops:
        left/right wheel PI controllers

    Final output:
        serial motor driver commands
    """

    def __init__(self) -> None:
        super().__init__("cascade_tag_straightening_controller")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("port", "")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("serial_timeout", 0.05)

        self.declare_parameter("control_hz", 30.0)

        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("tag_timeout_sec", 0.3)
        self.declare_parameter("wheel_timeout_sec", 0.3)

        # Robot geometry
        self.declare_parameter("wheel_radius", 0.05)   # meters
        self.declare_parameter("track_width", 0.28)    # meters

        # Joint names from /joint_states
        self.declare_parameter("left_joint_name", "left_wheel_joint")
        self.declare_parameter("right_joint_name", "right_wheel_joint")

        # Outer-loop gains
        self.declare_parameter("k_heading", 1.5)
        self.declare_parameter("k_lateral", 1.0)

        # Deadbands
        self.declare_parameter("heading_deadband", 0.03)        # rad
        self.declare_parameter("lateral_deadband", 0.01)        # m
        self.declare_parameter("encoder_deadband", 0.05)        # rad/s
        self.declare_parameter("wheel_error_deadband", 0.05)    # rad/s
        self.declare_parameter("motor_cmd_deadband", 20.0)      # driver units

        # Inner-loop PI gains
        self.declare_parameter("kp_left", 25.0)
        self.declare_parameter("ki_left", 6.0)
        self.declare_parameter("kff_left", 0.0)

        self.declare_parameter("kp_right", 25.0)
        self.declare_parameter("ki_right", 6.0)
        self.declare_parameter("kff_right", 0.0)

        # Motor command limit
        self.declare_parameter("motor_cmd_limit", 3000.0)

        # Sign convention parameters
        self.declare_parameter("invert_tag_x", False)
        self.declare_parameter("invert_tag_yaw", False)
        self.declare_parameter("invert_left_encoder", False)
        self.declare_parameter("invert_right_encoder", False)

        # Optional command shaping
        self.declare_parameter("max_steering_correction", 2.0)  # rad/s
        self.declare_parameter("max_forward_speed", 1.0)        # m/s
        self.declare_parameter("max_user_yaw_rate", 2.0)        # rad/s

        # ------------------------------------------------------------------
        # Read parameters
        # ------------------------------------------------------------------
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

        self.invert_tag_x = bool(self.get_parameter("invert_tag_x").value)
        self.invert_tag_yaw = bool(self.get_parameter("invert_tag_yaw").value)
        self.invert_left_encoder = bool(self.get_parameter("invert_left_encoder").value)
        self.invert_right_encoder = bool(self.get_parameter("invert_right_encoder").value)

        self.max_steering_correction = float(self.get_parameter("max_steering_correction").value)
        self.max_forward_speed = float(self.get_parameter("max_forward_speed").value)
        self.max_user_yaw_rate = float(self.get_parameter("max_user_yaw_rate").value)

        # ------------------------------------------------------------------
        # Controllers
        # ------------------------------------------------------------------
        self.left_pi = DiscretePI(kp=kp_left, ki=ki_left, kff=kff_left)
        self.right_pi = DiscretePI(kp=kp_right, ki=ki_right, kff=kff_right)

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self.v_cmd = 0.0
        self.omega_cmd = 0.0

        self.tag_x = 0.0
        self.tag_yaw = 0.0

        self.w_left = 0.0
        self.w_right = 0.0

        self.last_cmd_time: Optional[float] = None
        self.last_tag_time: Optional[float] = None
        self.last_wheel_time: Optional[float] = None

        # ------------------------------------------------------------------
        # Serial interface
        # ------------------------------------------------------------------
        self.lock = threading.Lock()
        self.ser = None
        self.connect_serial()

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)
        self.create_subscription(PoseStamped, "/tag_pose", self.tag_callback, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_callback, 20)

        self.timer = self.create_timer(self.Ts, self.control_step)

        self.get_logger().info(f"Controller started on port {self.port}")

    # ----------------------------------------------------------------------
    # Utility
    # ----------------------------------------------------------------------
    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ----------------------------------------------------------------------
    # Serial I/O
    # ----------------------------------------------------------------------
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

            # Keeps the same startup pattern as your existing driver code
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
        # Motor command deadband
        if abs(left_cmd) < self.motor_cmd_deadband:
            left_cmd = 0.0
        if abs(right_cmd) < self.motor_cmd_deadband:
            right_cmd = 0.0

        left_cmd = int(clamp(left_cmd, -self.motor_cmd_limit, self.motor_cmd_limit))
        right_cmd = int(clamp(right_cmd, -self.motor_cmd_limit, self.motor_cmd_limit))

        self.send_data(f"!S 1 {left_cmd}\r")
        self.send_data(f"!S 2 {right_cmd}\r")

    # ----------------------------------------------------------------------
    # ROS callbacks
    # ----------------------------------------------------------------------
    def cmd_callback(self, msg: Twist) -> None:
        self.v_cmd = clamp(msg.linear.x, -self.max_forward_speed, self.max_forward_speed)
        self.omega_cmd = clamp(msg.angular.z, -self.max_user_yaw_rate, self.max_user_yaw_rate)
        self.last_cmd_time = self.now_sec()

    def tag_callback(self, msg: PoseStamped) -> None:
        x = msg.pose.position.x
        yaw = quat_to_yaw(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )

        if self.invert_tag_x:
            x = -x
        if self.invert_tag_yaw:
            yaw = -yaw

        self.tag_x = x
        self.tag_yaw = wrap_to_pi(yaw)
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

        wl = msg.velocity[li]
        wr = msg.velocity[ri]

        if self.invert_left_encoder:
            wl = -wl
        if self.invert_right_encoder:
            wr = -wr

        # Encoder deadband
        if abs(wl) < self.encoder_deadband:
            wl = 0.0
        if abs(wr) < self.encoder_deadband:
            wr = 0.0

        self.w_left = wl
        self.w_right = wr
        self.last_wheel_time = self.now_sec()

    # ----------------------------------------------------------------------
    # Safety
    # ----------------------------------------------------------------------
    def reset_controllers(self) -> None:
        self.left_pi.reset()
        self.right_pi.reset()

    def stop_robot(self) -> None:
        self.reset_controllers()
        self.vel_control(0.0, 0.0)

    # ----------------------------------------------------------------------
    # Main control loop
    # ----------------------------------------------------------------------
    def control_step(self) -> None:
        t_now = self.now_sec()

        # Fresh operator command required
        if self.last_cmd_time is None or (t_now - self.last_cmd_time) > self.cmd_timeout_sec:
            self.stop_robot()
            return

        # Fresh wheel feedback required
        if self.last_wheel_time is None or (t_now - self.last_wheel_time) > self.wheel_timeout_sec:
            self.get_logger().warn("Wheel feedback stale; stopping robot")
            self.stop_robot()
            return

        # ==============================================================
        # Outer loop: camera/tag correction
        # ==============================================================
        e_y = 0.0
        e_psi = 0.0

        if self.last_tag_time is not None and (t_now - self.last_tag_time) <= self.tag_timeout_sec:
            e_y = self.tag_x
            e_psi = self.tag_yaw

            # Camera deadband
            if abs(e_y) < self.lateral_deadband:
                e_y = 0.0
            if abs(e_psi) < self.heading_deadband:
                e_psi = 0.0

        steering_corr = self.k_heading * e_psi + self.k_lateral * e_y
        steering_corr = clamp(
            steering_corr,
            -self.max_steering_correction,
            self.max_steering_correction,
        )

        # Background correction added to user command
        omega_ref = self.omega_cmd - steering_corr

        # ==============================================================
        # Reference generation (diff-drive inverse kinematics)
        # ==============================================================
        w_left_ref = (self.v_cmd - 0.5 * self.L * omega_ref) / self.r
        w_right_ref = (self.v_cmd + 0.5 * self.L * omega_ref) / self.r

        # ==============================================================
        # Inner loops: wheel speed control
        # ==============================================================
        e_left = w_left_ref - self.w_left
        e_right = w_right_ref - self.w_right

        # Wheel-error deadband
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

        # ==============================================================
        # Final actuation
        # ==============================================================
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