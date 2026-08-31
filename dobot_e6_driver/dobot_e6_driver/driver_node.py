import math
import re
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import Int32
from std_msgs.msg import String

from sensor_msgs.msg import JointState

from dobot_e6_msgs.msg import JointPositionCommand
from dobot_e6_msgs.msg import JointVelocityCommand

from dobot_sdk import DobotRobot
from dobot_sdk import CoordinateType


# ============================================================
# Fixed driver constants
# ============================================================

JOINT_NAMES = [
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6'
]


# ============================================================
# Robot control modes
# ============================================================

CONTROL_MODE_IDLE = 'IDLE'

CONTROL_MODE_POSITION = 'POSITION'

CONTROL_MODE_VELOCITY = 'VELOCITY'


# ============================================================
# Dobot RobotMode
# ============================================================

ROBOT_MODE_NAMES = {
    1: 'INIT',
    2: 'BRAKE_OPEN',
    3: 'POWEROFF',
    4: 'DISABLED',
    5: 'ENABLED_IDLE',
    6: 'BACKDRIVE',
    7: 'RUNNING',
    8: 'SINGLE_MOVE',
    9: 'ERROR',
    10: 'PAUSE',
    11: 'COLLISION',
}


# ============================================================
# Driver
# ============================================================

class DobotE6Driver(Node):

    def __init__(self):

        super().__init__('dobot_e6_driver')

        # ====================================================
        # ROS parameters
        # ====================================================

        self._declare_parameters()
        self._load_parameters()
        self._validate_parameters()

        # Configuration parameters are intentionally startup-only.
        # This avoids changing timer frequencies or safety limits
        # while the robot is operating.
        self.add_on_set_parameters_callback(
            self._reject_runtime_parameter_changes
        )

        # ====================================================
        # Synchronization
        # ====================================================

        self.command_lock = threading.Lock()
        self.feedback_lock = threading.Lock()
        self.state_lock = threading.Lock()

        self.shutdown_event = threading.Event()

        # ====================================================
        # Robot connection
        # ====================================================

        self.robot = None
        self.robot_enabled_by_driver = False
        self.accept_commands = False

        # ====================================================
        # Control mode
        # ====================================================

        self.control_mode = CONTROL_MODE_IDLE

        # ====================================================
        # Position-control state
        # ====================================================

        self.position_target = None
        self.position_command_time = None
        self.position_motion_started = False

        # ====================================================
        # Velocity-control state
        # ====================================================

        self.qdot_desired = [0.0] * 6
        self.q_ref = None

        self.last_servo_time = None
        self.last_velocity_command_time = None
        self.last_velocity_status_log_time = None

        # Velocity-tracking diagnostic state.
        self.velocity_tracking_error_start_time = None
        self.last_velocity_tracking_warning_time = None

        # ====================================================
        # Safety-monitor state
        # ====================================================

        self.last_joint_limit_warning_time = [None] * 6

        # ====================================================
        # Recovery state
        # ====================================================

        self.recovery_in_progress = False
        self.recovery_thread = None

        # ====================================================
        # Realtime feedback state
        # ====================================================

        self.latest_joint_position = None
        self.latest_joint_velocity = None
        self.latest_robot_mode = None

        self.last_feedback_time = None

        self.feedback_valid = False
        self.feedback_received = False
        self.last_logged_robot_mode = None

        # ====================================================
        # Suction gripper state
        # ====================================================

        # Logical ROS state:
        #   True  -> suction ON / adsorb
        #   False -> suction OFF / release
        #
        # The monitor is only published after ToolDOInstant is
        # accepted or GetToolDO readback succeeds.
        self.gripper_state = None
        self.gripper_state_valid = False

        # ====================================================
        # ROS Publishers
        # ====================================================

        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.robot_mode_pub = self.create_publisher(
            Int32,
            '/dobot/robot_mode',
            10
        )

        # Driver-side control mode monitor.  TRANSIENT_LOCAL
        # allows a late subscriber to receive the latest mode.
        control_mode_qos = QoSProfile(depth=1)
        control_mode_qos.reliability = ReliabilityPolicy.RELIABLE
        control_mode_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.control_mode_monitor_pub = self.create_publisher(
            String,
            '/dobot/control_mode_monitor',
            control_mode_qos
        )

        # Suction-gripper output monitor. TRANSIENT_LOCAL lets
        # late subscribers receive the latest known state.
        gripper_monitor_qos = QoSProfile(depth=1)
        gripper_monitor_qos.reliability = ReliabilityPolicy.RELIABLE
        gripper_monitor_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.suction_gripper_monitor_pub = self.create_publisher(
            Bool,
            '/dobot/suction_gripper_monitor',
            gripper_monitor_qos
        )

        # ====================================================
        # ROS Subscribers
        # ====================================================

        self.enable_sub = self.create_subscription(
            Bool,
            '/dobot/enable',
            self.enable_callback,
            10
        )

        self.recover_sub = self.create_subscription(
            Empty,
            '/dobot/recover',
            self.recover_callback,
            10
        )

        self.joint_position_sub = self.create_subscription(
            JointPositionCommand,
            '/dobot/joint_position_cmd',
            self.joint_position_callback,
            10
        )

        self.joint_velocity_sub = self.create_subscription(
            JointVelocityCommand,
            '/dobot/joint_velocity_cmd',
            self.joint_velocity_callback,
            10
        )

        self.home_sub = self.create_subscription(
            Empty,
            '/dobot/home',
            self.home_callback,
            10
        )

        self.packing_sub = self.create_subscription(
            Empty,
            '/dobot/packing',
            self.packing_callback,
            10
        )

        self.stop_sub = self.create_subscription(
            Empty,
            '/dobot/stop',
            self.stop_callback,
            10
        )

        # Suction gripper command:
        #   True  -> suction ON / adsorb
        #   False -> suction OFF / release
        self.suction_gripper_sub = self.create_subscription(
            Bool,
            '/dobot/suction_gripper',
            self.suction_gripper_callback,
            10
        )

        # ====================================================
        # Initial robot connection
        # ====================================================

        self.connect_robot(
            start_feedback=True
        )

        # ====================================================
        # Timers
        # ====================================================

        self.feedback_timer = self.create_timer(
            self.feedback_publish_period_s,
            self.publish_feedback
        )

        # ServoJ velocity controller.  The timer remains idle
        # unless CONTROL_MODE_VELOCITY is active.
        self.velocity_control_timer = self.create_timer(
            self.servo_period_s,
            self.velocity_control_loop
        )

        # Independent motion/safety supervision.
        self.safety_timer = self.create_timer(
            self.safety_monitor_period_s,
            self.safety_monitor_loop
        )

        # GetToolDO readback is deliberately suspended during
        # VELOCITY mode to avoid adding IO-query jitter to the
        # ServoJ streaming loop. Gripper commands themselves are
        # still allowed during VELOCITY mode.
        self.gripper_monitor_timer = self.create_timer(
            self.gripper_monitor_period_s,
            self.monitor_suction_gripper
        )

        self.publish_control_mode_monitor(
            CONTROL_MODE_IDLE
        )

        self._log_configuration()


    # ========================================================
    # ROS parameter configuration
    # ========================================================

    def _declare_parameters(self):

        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_ip', '192.168.5.1'),

                ('feedback_publish_frequency_hz', 50.0),
                ('safety_monitor_frequency_hz', 50.0),
                ('servo_frequency_hz', 33.333333),

                ('servo_min_dt_s', 0.020),
                ('servo_max_dt_s', 0.060),

                ('max_feedback_age_s', 0.050),
                ('velocity_command_timeout_s', 0.150),

                (
                    'joint_limits_deg',
                    [360.0, 135.0, 154.0, 160.0, 173.0, 360.0]
                ),
                ('joint_limit_margin_deg', 2.0),
                ('joint_limit_warning_deg', 5.0),

                ('velocity_software_limit_rad_s', 1.00),
                ('zero_velocity_epsilon_rad_s', 1.0e-5),

                ('absolute_measured_velocity_limit_rad_s', 2.00),
                ('velocity_mode_measured_velocity_limit_rad_s', 1.20),

                ('velocity_tracking_error_warning_rad_s', 0.08),
                ('velocity_tracking_error_warning_time_s', 0.50),

                # ES01 suction gripper / Tool IO
                # ToolDO index range in the installed SDK: 0-1.
                ('gripper_tool_do_index', 1),
                ('gripper_active_high', True),
                ('gripper_monitor_frequency_hz', 2.0),

                ('position_completion_tolerance_deg', 0.50),
                ('position_min_active_time_s', 0.20),
                ('position_motion_timeout_s', 120.0),

                (
                    'home_position_rad',
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                ),
                ('home_velocity_ratio', 50.0),
                ('home_acceleration_ratio', 50.0),

                (
                    'packing_position_rad',
                    [
                        -1.4247983737273358,
                        -0.010110058577726934,
                        -2.452188131684251,
                        -0.7426624173849468,
                        0.09788714902695247,
                        0.26438398563707133
                    ]
                ),
                ('packing_velocity_ratio', 20.0),
                ('packing_acceleration_ratio', 20.0),

                ('clear_error_max_attempts', 5),
                ('clear_error_retry_period_s', 1.0),
                ('clear_error_mode_check_period_s', 0.5),
                ('clear_error_valid_mode_count', 2),

                ('power_on_timeout_s', 120.0),
                ('power_on_poll_period_s', 5.0),
                ('power_on_stable_count', 2)
            ]
        )

    def _reject_runtime_parameter_changes(
        self,
        parameters
    ):

        del parameters

        return SetParametersResult(
            successful=False,
            reason=(
                'Dobot E6 driver parameters are startup-only. '
                'Change the YAML/launch configuration and '
                'restart the node.'
            )
        )

    def _load_parameters(self):

        self.robot_ip = str(
            self.get_parameter('robot_ip').value
        )

        self.feedback_publish_frequency_hz = float(
            self.get_parameter(
                'feedback_publish_frequency_hz'
            ).value
        )

        self.safety_monitor_frequency_hz = float(
            self.get_parameter(
                'safety_monitor_frequency_hz'
            ).value
        )

        self.servo_frequency_hz = float(
            self.get_parameter(
                'servo_frequency_hz'
            ).value
        )

        self.servo_min_dt_s = float(
            self.get_parameter(
                'servo_min_dt_s'
            ).value
        )

        self.servo_max_dt_s = float(
            self.get_parameter(
                'servo_max_dt_s'
            ).value
        )

        self.max_feedback_age_s = float(
            self.get_parameter(
                'max_feedback_age_s'
            ).value
        )

        self.velocity_command_timeout_s = float(
            self.get_parameter(
                'velocity_command_timeout_s'
            ).value
        )

        self.joint_limits_deg = [
            float(v)
            for v in self.get_parameter(
                'joint_limits_deg'
            ).value
        ]

        self.joint_limit_margin_deg = float(
            self.get_parameter(
                'joint_limit_margin_deg'
            ).value
        )

        self.joint_limit_warning_deg = float(
            self.get_parameter(
                'joint_limit_warning_deg'
            ).value
        )

        self.velocity_software_limit_rad_s = float(
            self.get_parameter(
                'velocity_software_limit_rad_s'
            ).value
        )


        self.zero_velocity_epsilon_rad_s = float(
            self.get_parameter(
                'zero_velocity_epsilon_rad_s'
            ).value
        )

        self.absolute_measured_velocity_limit_rad_s = float(
            self.get_parameter(
                'absolute_measured_velocity_limit_rad_s'
            ).value
        )

        self.velocity_mode_measured_velocity_limit_rad_s = float(
            self.get_parameter(
                'velocity_mode_measured_velocity_limit_rad_s'
            ).value
        )

        self.velocity_tracking_error_warning_rad_s = float(
            self.get_parameter(
                'velocity_tracking_error_warning_rad_s'
            ).value
        )

        self.velocity_tracking_error_warning_time_s = float(
            self.get_parameter(
                'velocity_tracking_error_warning_time_s'
            ).value
        )

        self.gripper_tool_do_index = int(
            self.get_parameter(
                'gripper_tool_do_index'
            ).value
        )

        self.gripper_active_high = bool(
            self.get_parameter(
                'gripper_active_high'
            ).value
        )

        self.gripper_monitor_frequency_hz = float(
            self.get_parameter(
                'gripper_monitor_frequency_hz'
            ).value
        )

        self.position_completion_tolerance_rad = math.radians(
            float(
                self.get_parameter(
                    'position_completion_tolerance_deg'
                ).value
            )
        )

        self.position_min_active_time_s = float(
            self.get_parameter(
                'position_min_active_time_s'
            ).value
        )

        self.position_motion_timeout_s = float(
            self.get_parameter(
                'position_motion_timeout_s'
            ).value
        )

        self.home_position_rad = [
            float(v)
            for v in self.get_parameter(
                'home_position_rad'
            ).value
        ]

        self.home_velocity_ratio = float(
            self.get_parameter(
                'home_velocity_ratio'
            ).value
        )

        self.home_acceleration_ratio = float(
            self.get_parameter(
                'home_acceleration_ratio'
            ).value
        )

        self.packing_position_rad = [
            float(v)
            for v in self.get_parameter(
                'packing_position_rad'
            ).value
        ]

        self.packing_velocity_ratio = float(
            self.get_parameter(
                'packing_velocity_ratio'
            ).value
        )

        self.packing_acceleration_ratio = float(
            self.get_parameter(
                'packing_acceleration_ratio'
            ).value
        )

        self.clear_error_max_attempts = int(
            self.get_parameter(
                'clear_error_max_attempts'
            ).value
        )

        self.clear_error_retry_period_s = float(
            self.get_parameter(
                'clear_error_retry_period_s'
            ).value
        )

        self.clear_error_mode_check_period_s = float(
            self.get_parameter(
                'clear_error_mode_check_period_s'
            ).value
        )

        self.clear_error_valid_mode_count = int(
            self.get_parameter(
                'clear_error_valid_mode_count'
            ).value
        )

        self.power_on_timeout_s = float(
            self.get_parameter(
                'power_on_timeout_s'
            ).value
        )

        self.power_on_poll_period_s = float(
            self.get_parameter(
                'power_on_poll_period_s'
            ).value
        )

        self.power_on_stable_count = int(
            self.get_parameter(
                'power_on_stable_count'
            ).value
        )

        self.feedback_publish_period_s = (
            1.0 / self.feedback_publish_frequency_hz
        )

        self.safety_monitor_period_s = (
            1.0 / self.safety_monitor_frequency_hz
        )

        self.gripper_monitor_period_s = (
            1.0 / self.gripper_monitor_frequency_hz
        )

        self.servo_period_s = (
            1.0 / self.servo_frequency_hz
        )

        self.joint_hard_limits_rad = [
            math.radians(
                limit_deg
                - self.joint_limit_margin_deg
            )
            for limit_deg in self.joint_limits_deg
        ]

        self.joint_warning_limits_rad = [
            max(
                0.0,
                hard_limit
                - math.radians(
                    self.joint_limit_warning_deg
                )
            )
            for hard_limit in self.joint_hard_limits_rad
        ]

    def _validate_parameters(self):

        if not self.robot_ip:
            raise ValueError(
                'robot_ip must not be empty.'
            )

        if len(self.joint_limits_deg) != 6:
            raise ValueError(
                'joint_limits_deg must contain 6 values.'
            )

        if len(self.home_position_rad) != 6:
            raise ValueError(
                'home_position_rad must contain 6 values.'
            )

        if len(self.packing_position_rad) != 6:
            raise ValueError(
                'packing_position_rad must contain 6 values.'
            )

        if (
            self.feedback_publish_frequency_hz <= 0.0
            or self.safety_monitor_frequency_hz <= 0.0
            or self.servo_frequency_hz <= 0.0
        ):
            raise ValueError(
                'All configured frequencies must be > 0.'
            )

        if (
            self.servo_min_dt_s <= 0.0
            or self.servo_max_dt_s <= 0.0
            or self.servo_min_dt_s >= self.servo_max_dt_s
        ):
            raise ValueError(
                'ServoJ dt limits are invalid.'
            )

        if not (
            self.servo_min_dt_s
            <= self.servo_period_s
            <= self.servo_max_dt_s
        ):
            raise ValueError(
                'servo_frequency_hz produces a nominal '
                'period outside the configured ServoJ dt '
                'limits.'
            )

        if self.max_feedback_age_s <= 0.0:
            raise ValueError(
                'max_feedback_age_s must be > 0.'
            )

        if (
            self.velocity_command_timeout_s
            <= self.servo_period_s
        ):
            raise ValueError(
                'velocity_command_timeout_s must be larger '
                'than one ServoJ period.'
            )

        if self.joint_limit_margin_deg < 0.0:
            raise ValueError(
                'joint_limit_margin_deg must be >= 0.'
            )

        if self.joint_limit_warning_deg < 0.0:
            raise ValueError(
                'joint_limit_warning_deg must be >= 0.'
            )

        for i, limit_deg in enumerate(
            self.joint_limits_deg
        ):

            if (
                limit_deg
                <= (
                    self.joint_limit_margin_deg
                    + self.joint_limit_warning_deg
                )
            ):

                raise ValueError(
                    f'Joint {i + 1}: joint limit must be '
                    'larger than margin + warning zone.'
                )

        if self.velocity_software_limit_rad_s <= 0.0:
            raise ValueError(
                'velocity_software_limit_rad_s must be > 0.'
            )

        if (
            self.velocity_mode_measured_velocity_limit_rad_s
            <= self.velocity_software_limit_rad_s
        ):
            raise ValueError(
                'velocity_mode_measured_velocity_limit_rad_s '
                'must be larger than '
                'velocity_software_limit_rad_s.'
            )

        if (
            self.absolute_measured_velocity_limit_rad_s
            <= self.velocity_mode_measured_velocity_limit_rad_s
        ):
            raise ValueError(
                'absolute_measured_velocity_limit_rad_s '
                'must be larger than the velocity-mode '
                'measured velocity limit.'
            )


        if self.zero_velocity_epsilon_rad_s < 0.0:

            raise ValueError(
                'zero_velocity_epsilon_rad_s must be >= 0.'
            )

        if self.velocity_tracking_error_warning_rad_s < 0.0:

            raise ValueError(
                'velocity_tracking_error_warning_rad_s must be >= 0.'
            )

        if self.velocity_tracking_error_warning_time_s < 0.0:

            raise ValueError(
                'velocity_tracking_error_warning_time_s must be >= 0.'
            )

        if self.gripper_tool_do_index not in (0, 1):

            raise ValueError(
                'gripper_tool_do_index must be 0 or 1.'
            )

        if self.gripper_monitor_frequency_hz <= 0.0:

            raise ValueError(
                'gripper_monitor_frequency_hz must be > 0.'
            )

        if self.position_completion_tolerance_rad <= 0.0:

            raise ValueError(
                'position_completion_tolerance_deg must be > 0.'
            )

        if self.position_min_active_time_s < 0.0:

            raise ValueError(
                'position_min_active_time_s must be >= 0.'
            )

        for name, value in (
            ('home_velocity_ratio', self.home_velocity_ratio),
            (
                'home_acceleration_ratio',
                self.home_acceleration_ratio
            ),
            (
                'packing_velocity_ratio',
                self.packing_velocity_ratio
            ),
            (
                'packing_acceleration_ratio',
                self.packing_acceleration_ratio
            )
        ):

            if not 1.0 <= value <= 100.0:

                raise ValueError(
                    f'{name} must be in [1, 100].'
                )

        if self.position_motion_timeout_s <= 0.0:
            raise ValueError(
                'position_motion_timeout_s must be > 0.'
            )

        if self.clear_error_max_attempts < 1:
            raise ValueError(
                'clear_error_max_attempts must be >= 1.'
            )

        if self.clear_error_valid_mode_count < 1:
            raise ValueError(
                'clear_error_valid_mode_count must be >= 1.'
            )

        if self.power_on_stable_count < 1:
            raise ValueError(
                'power_on_stable_count must be >= 1.'
            )

        valid, invalid_joint = (
            self.positions_within_limits(
                self.home_position_rad
            )
        )

        if not valid:
            raise ValueError(
                'home_position_rad exceeds the software '
                f'limit of Joint {invalid_joint + 1}.'
            )

        valid, invalid_joint = (
            self.positions_within_limits(
                self.packing_position_rad
            )
        )

        if not valid:
            raise ValueError(
                'packing_position_rad exceeds the software '
                f'limit of Joint {invalid_joint + 1}.'
            )

    def _log_configuration(self):

        self.get_logger().info(
            '================================='
        )

        self.get_logger().info(
            'DOBOT E6 DRIVER CONFIGURATION'
        )

        self.get_logger().info(
            f'Robot IP: {self.robot_ip}'
        )

        self.get_logger().info(
            'Feedback publish frequency: '
            f'{self.feedback_publish_frequency_hz:.2f} Hz'
        )

        self.get_logger().info(
            'Safety monitor frequency: '
            f'{self.safety_monitor_frequency_hz:.2f} Hz'
        )

        self.get_logger().info(
            'ServoJ nominal frequency: '
            f'{self.servo_frequency_hz:.3f} Hz '
            f'({self.servo_period_s:.4f} s)'
        )

        self.get_logger().info(
            'Velocity command software limit: '
            f'+/-{self.velocity_software_limit_rad_s:.3f} '
            'rad/s'
        )

        self.get_logger().info(
            'Software hard joint limits [deg]: '
            + str([
                round(math.degrees(v), 3)
                for v in self.joint_hard_limits_rad
            ])
        )

        self.get_logger().info(
            'Suction gripper ToolDO index: '
            f'{self.gripper_tool_do_index}'
        )

        self.get_logger().info(
            'Suction gripper active-high: '
            f'{self.gripper_active_high}'
        )

        self.get_logger().info(
            'Suction gripper monitor frequency: '
            f'{self.gripper_monitor_frequency_hz:.2f} Hz'
        )

        self.get_logger().info(
            '================================='
        )

    def publish_control_mode_monitor(
        self,
        mode=None
    ):

        if mode is None:

            with self.state_lock:

                mode = self.control_mode

        msg = String()
        msg.data = str(mode)

        self.control_mode_monitor_pub.publish(
            msg
        )


    # ========================================================
    # Connect robot
    # ========================================================

    def connect_robot(self, start_feedback=True):

        self.get_logger().info(
            f'Connecting to Magician E6 at {self.robot_ip}...'
        )

        robot = DobotRobot(
            self.robot_ip
        )

        robot.Connect()

        self.get_logger().info(
            'TCP/IP connection established.'
        )

        response = (
            robot.robot_control.RequestControl()
        )

        self.get_logger().info(
            f'RequestControl: {response}'
        )

        if not response.startswith('0,'):

            raise RuntimeError(
                f'RequestControl failed: {response}'
            )

        if start_feedback:

            robot.StartFeedbackMonitor(
                callback=self.feedback_callback
            )

            self.get_logger().info(
                'Realtime feedback monitor started.'
            )

        with self.command_lock:

            self.robot = robot

        return robot

    # ========================================================
    # Realtime feedback callback
    # ========================================================

    def feedback_callback(self, status):

        try:

            if status is None:
                return

            if status.joint_state is None:
                return

            q_actual = (
                status.joint_state.q_actual
            )

            qd_actual = (
                status.joint_state.qd_actual
            )

            if len(q_actual) != 6:
                return

            if len(qd_actual) != 6:
                return

            # ------------------------------------------------
            # Dobot -> ROS units
            # ------------------------------------------------

            joint_position = [
                math.radians(float(q))
                for q in q_actual
            ]

            joint_velocity = [
                math.radians(float(qd))
                for qd in qd_actual
            ]

            # ------------------------------------------------
            # RobotMode
            # ------------------------------------------------

            if hasattr(
                status.robot_mode,
                'value'
            ):

                robot_mode = int(
                    status.robot_mode.value
                )

            else:

                robot_mode = int(
                    status.robot_mode
                )

            now = time.monotonic()

            # ------------------------------------------------
            # Store feedback
            # ------------------------------------------------

            with self.feedback_lock:

                self.latest_joint_position = (
                    joint_position
                )

                self.latest_joint_velocity = (
                    joint_velocity
                )

                self.latest_robot_mode = (
                    robot_mode
                )

                self.last_feedback_time = (
                    now
                )

                self.feedback_valid = True

            # ------------------------------------------------
            # Command availability
            #
            # New commands may start only in:
            #
            # 4 = DISABLED
            # 5 = ENABLED_IDLE
            #
            # Velocity control already running is handled
            # independently and may operate while mode = 7.
            # ------------------------------------------------

            if not self.recovery_in_progress:

                self.accept_commands = (
                    robot_mode in (4, 5)
                )

            else:

                self.accept_commands = False

            # ------------------------------------------------
            # Robot no longer enabled
            # ------------------------------------------------

            if robot_mode in (3, 4, 9):

                self.robot_enabled_by_driver = False

            # =================================================
            # POSITION control-mode state tracking
            # =================================================

            position_completed = False
            position_ended_before_target = False
            position_aborted = False
            publish_idle = False

            with self.state_lock:

                if (
                    self.control_mode
                    == CONTROL_MODE_POSITION
                ):

                    # ----------------------------------------
                    # Robot entered an active motion state.
                    # ----------------------------------------

                    if robot_mode in (7, 8):

                        self.position_motion_started = True

                    # ----------------------------------------
                    # Robot returned to ENABLED_IDLE.
                    # ----------------------------------------

                    elif robot_mode == 5:

                        elapsed = 0.0

                        if self.position_command_time is not None:

                            elapsed = (
                                now
                                - self.position_command_time
                            )

                        target_reached = False

                        if self.position_target is not None:

                            max_error = max(
                                abs(
                                    joint_position[i]
                                    - self.position_target[i]
                                )
                                for i in range(6)
                            )

                            target_reached = (
                                max_error
                                <= self.position_completion_tolerance_rad
                            )

                        # Normal completion:
                        # target reached and either the robot
                        # was observed moving or the command was
                        # a very short move.
                        if (
                            target_reached
                            and (
                                self.position_motion_started
                                or (
                                    elapsed
                                    >= self.position_min_active_time_s
                                )
                            )
                        ):

                            self._reset_position_state_locked()

                            self.control_mode = (
                                CONTROL_MODE_IDLE
                            )

                            position_completed = True
                            publish_idle = True

                        # The controller returned to IDLE after
                        # an observed motion, but measured target
                        # error is still above tolerance.
                        elif (
                            self.position_motion_started
                            and (
                                elapsed
                                >= self.position_min_active_time_s
                            )
                            and not target_reached
                        ):

                            self._reset_position_state_locked()

                            self.control_mode = (
                                CONTROL_MODE_IDLE
                            )

                            position_ended_before_target = True
                            publish_idle = True

                    # ----------------------------------------
                    # Position movement cannot continue.
                    # ----------------------------------------

                    elif robot_mode in (
                        3,
                        4,
                        9,
                        10,
                        11
                    ):

                        self._reset_position_state_locked()

                        self.control_mode = (
                            CONTROL_MODE_IDLE
                        )

                        position_aborted = True
                        publish_idle = True

            if publish_idle:

                self.publish_control_mode_monitor(
                    CONTROL_MODE_IDLE
                )

            if position_completed:

                self.get_logger().info(
                    'Position movement completed. '
                    'Control mode -> IDLE.'
                )

            if position_ended_before_target:

                self.get_logger().warning(
                    'Position movement ended while the '
                    'measured target error remained above '
                    'the configured tolerance. '
                    'Control mode -> IDLE.'
                )

            if position_aborted:

                self.get_logger().warning(
                    'Position movement interrupted by '
                    f'RobotMode={robot_mode}. '
                    'Control mode -> IDLE.'
                )

            # ------------------------------------------------
            # First packet
            # ------------------------------------------------

            if not self.feedback_received:

                self.feedback_received = True

                self.get_logger().info(
                    'First realtime feedback packet received.'
                )

        except Exception as e:

            self.get_logger().error(
                f'Feedback callback error: {e}'
            )

    # ========================================================
    # Publish feedback to ROS
    # ========================================================

    def publish_feedback(self):

        with self.feedback_lock:

            if not self.feedback_valid:
                return

            if self.latest_joint_position is None:
                return

            if self.latest_joint_velocity is None:
                return

            joint_position = (
                self.latest_joint_position.copy()
            )

            joint_velocity = (
                self.latest_joint_velocity.copy()
            )

            robot_mode = (
                self.latest_robot_mode
            )

        # ====================================================
        # JointState
        # ====================================================

        msg = JointState()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.name = JOINT_NAMES

        msg.position = (
            joint_position
        )

        msg.velocity = (
            joint_velocity
        )

        msg.effort = []

        self.joint_state_pub.publish(
            msg
        )

        # ====================================================
        # RobotMode
        # ====================================================

        if robot_mode is not None:

            mode_msg = Int32()

            mode_msg.data = (
                robot_mode
            )

            self.robot_mode_pub.publish(
                mode_msg
            )

            if (
                robot_mode
                != self.last_logged_robot_mode
            ):

                mode_name = (
                    ROBOT_MODE_NAMES.get(
                        robot_mode,
                        'UNKNOWN'
                    )
                )

                self.get_logger().info(
                    f'RobotMode changed: '
                    f'{robot_mode} ({mode_name})'
                )

                self.last_logged_robot_mode = (
                    robot_mode
                )


    # ========================================================
    # Independent safety monitor
    # ========================================================

    def safety_monitor_loop(self):

        now = time.monotonic()

        with self.state_lock:

            control_mode = self.control_mode
            position_command_time = self.position_command_time
            qdot_desired = self.qdot_desired.copy()

        # No motion currently owned by this driver.
        if control_mode == CONTROL_MODE_IDLE:

            self.velocity_tracking_error_start_time = None
            return

        feedback = self.get_feedback_snapshot()

        if (
            not feedback['valid']
            or feedback['position'] is None
            or feedback['velocity'] is None
            or feedback['last_feedback_time'] is None
        ):

            self.stop_robot_motion(
                reason='safety monitor: feedback unavailable'
            )

            return

        feedback_age = (
            now
            - feedback['last_feedback_time']
        )

        # ----------------------------------------------------
        # 1. Feedback watchdog for BOTH POSITION and VELOCITY.
        # ----------------------------------------------------

        if (
            feedback_age
            > self.max_feedback_age_s
        ):

            self.get_logger().error(
                'SAFETY: feedback timeout '
                f'({feedback_age:.3f} s).'
            )

            self.stop_robot_motion(
                reason='safety monitor: feedback timeout'
            )

            return

        # ----------------------------------------------------
        # 2. RobotMode compatibility.
        # ----------------------------------------------------

        if (
            control_mode
            == CONTROL_MODE_POSITION
        ):

            valid_robot_modes = (5, 7, 8)

        else:

            valid_robot_modes = (5, 7)

        if (
            feedback['robot_mode']
            not in valid_robot_modes
        ):

            mode_name = ROBOT_MODE_NAMES.get(
                feedback['robot_mode'],
                'UNKNOWN'
            )

            self.get_logger().error(
                'SAFETY: incompatible RobotMode '
                f'{feedback["robot_mode"]} ({mode_name}) '
                f'while control mode is {control_mode}.'
            )

            self.stop_robot_motion(
                reason='safety monitor: invalid RobotMode'
            )

            return

        # ----------------------------------------------------
        # 3. Actual measured joint-position hard limits.
        # ----------------------------------------------------

        valid, invalid_joint = (
            self.positions_within_limits(
                feedback['position']
            )
        )

        if not valid:

            joint_text = (
                'unknown'
                if invalid_joint is None
                else str(invalid_joint + 1)
            )

            self.get_logger().error(
                'SAFETY: measured Joint '
                f'{joint_text} position exceeded the '
                'software hard limit.'
            )

            self.stop_robot_motion(
                reason='safety monitor: actual joint limit'
            )

            return

        # ----------------------------------------------------
        # 4. Joint-limit warning zone.
        # ----------------------------------------------------

        for i in range(6):

            if (
                abs(feedback['position'][i])
                >= self.joint_warning_limits_rad[i]
            ):

                last_warning = (
                    self.last_joint_limit_warning_time[i]
                )

                if (
                    last_warning is None
                    or now - last_warning >= 1.0
                ):

                    self.get_logger().warning(
                        'SAFETY: Joint '
                        f'{i + 1} is inside the configured '
                        'joint-limit warning zone. '
                        f'q={feedback["position"][i]:.4f} rad.'
                    )

                    self.last_joint_limit_warning_time[i] = (
                        now
                    )

        # ----------------------------------------------------
        # 5. Absolute measured velocity limit.
        # ----------------------------------------------------

        max_actual_velocity = max(
            abs(v)
            for v in feedback['velocity']
        )

        if (
            max_actual_velocity
            > self.absolute_measured_velocity_limit_rad_s
        ):

            self.get_logger().error(
                'SAFETY: measured joint velocity exceeded '
                'the absolute configured limit. '
                f'Maximum measured='
                f'{max_actual_velocity:.4f} rad/s.'
            )

            self.stop_robot_motion(
                reason='safety monitor: absolute velocity limit'
            )

            return

        # ----------------------------------------------------
        # 6. Position movement timeout.
        # ----------------------------------------------------

        if (
            control_mode
            == CONTROL_MODE_POSITION
            and position_command_time is not None
        ):

            elapsed = (
                now
                - position_command_time
            )

            if (
                elapsed
                > self.position_motion_timeout_s
            ):

                self.get_logger().error(
                    'SAFETY: position movement timeout '
                    f'after {elapsed:.2f} s.'
                )

                self.stop_robot_motion(
                    reason='safety monitor: position timeout'
                )

                return

        # ----------------------------------------------------
        # 7. VELOCITY-mode measured-speed protection and
        #    tracking diagnostic.
        # ----------------------------------------------------

        if (
            control_mode
            == CONTROL_MODE_VELOCITY
        ):

            if (
                max_actual_velocity
                > self.velocity_mode_measured_velocity_limit_rad_s
            ):

                self.get_logger().error(
                    'SAFETY: measured speed in VELOCITY mode '
                    'exceeded the configured limit. '
                    f'Maximum measured='
                    f'{max_actual_velocity:.4f} rad/s.'
                )

                self.stop_robot_motion(
                    reason=(
                        'safety monitor: velocity-mode '
                        'measured velocity limit'
                    )
                )

                return

            max_tracking_error = max(
                abs(
                    qdot_desired[i]
                    - feedback['velocity'][i]
                )
                for i in range(6)
            )

            if (
                max_tracking_error
                > self.velocity_tracking_error_warning_rad_s
            ):

                if (
                    self.velocity_tracking_error_start_time
                    is None
                ):

                    self.velocity_tracking_error_start_time = (
                        now
                    )

                error_duration = (
                    now
                    - self.velocity_tracking_error_start_time
                )

                if (
                    error_duration
                    >= self.velocity_tracking_error_warning_time_s
                ):

                    last_warning = (
                        self.last_velocity_tracking_warning_time
                    )

                    if (
                        last_warning is None
                        or now - last_warning >= 1.0
                    ):

                        self.get_logger().warning(
                            'VELOCITY TRACKING: persistent '
                            'velocity error. '
                            f'Maximum error='
                            f'{max_tracking_error:.4f} rad/s.'
                        )

                        self.last_velocity_tracking_warning_time = (
                            now
                        )

            else:

                self.velocity_tracking_error_start_time = None

    # ========================================================
    # Helpers - control state
    # ========================================================

    def _reset_position_state_locked(self):

        self.position_target = None
        self.position_command_time = None
        self.position_motion_started = False

    def _reset_velocity_state_locked(self):

        self.qdot_desired = [0.0] * 6

        self.q_ref = None

        self.last_servo_time = None
        self.last_velocity_command_time = None
        self.last_velocity_status_log_time = None

        self.velocity_tracking_error_start_time = None
        self.last_velocity_tracking_warning_time = None

    def reset_all_motion_state(self):

        publish_idle = False

        with self.state_lock:

            previous_mode = self.control_mode

            self._reset_position_state_locked()
            self._reset_velocity_state_locked()

            self.control_mode = CONTROL_MODE_IDLE

            publish_idle = (
                previous_mode
                != CONTROL_MODE_IDLE
            )

        if publish_idle:

            self.publish_control_mode_monitor(
                CONTROL_MODE_IDLE
            )

    # ========================================================
    # Joint-position limit validation
    # ========================================================

    def positions_within_limits(
        self,
        positions_rad
    ):

        if len(positions_rad) != 6:

            return False, None

        for i in range(6):

            if not math.isfinite(
                positions_rad[i]
            ):

                return False, i

            if (
                abs(positions_rad[i])
                > self.joint_hard_limits_rad[i]
            ):

                return False, i

        return True, None

    def position_near_limit_joints(
        self,
        positions_rad
    ):

        joints = []

        for i in range(6):

            if (
                abs(positions_rad[i])
                >= self.joint_warning_limits_rad[i]
            ):

                joints.append(i)

        return joints

    def velocity_command_toward_limit(
        self,
        current_position,
        velocity
    ):

        for i in range(6):

            warning_limit = (
                self.joint_warning_limits_rad[i]
            )

            if (
                current_position[i]
                >= warning_limit
                and velocity[i] > 0.0
            ):

                return True, i

            if (
                current_position[i]
                <= -warning_limit
                and velocity[i] < 0.0
            ):

                return True, i

        return False, None

    def get_feedback_snapshot(self):

        with self.feedback_lock:

            return {
                'valid': self.feedback_valid,
                'position': (
                    None
                    if self.latest_joint_position is None
                    else self.latest_joint_position.copy()
                ),
                'velocity': (
                    None
                    if self.latest_joint_velocity is None
                    else self.latest_joint_velocity.copy()
                ),
                'robot_mode': self.latest_robot_mode,
                'last_feedback_time': self.last_feedback_time,
            }

    # ========================================================
    # Check whether a new position motion can start
    # ========================================================

    def motion_allowed(self, movement_name):

        if self.recovery_in_progress:

            self.get_logger().warning(
                f'{movement_name} rejected: '
                'robot recovery is in progress.'
            )

            return False

        # ----------------------------------------------------
        # No other control mode may be active.
        # ----------------------------------------------------

        with self.state_lock:

            control_mode = (
                self.control_mode
            )

        if control_mode != CONTROL_MODE_IDLE:

            self.get_logger().warning(
                f'{movement_name} rejected: '
                f'control mode is {control_mode}.'
            )

            return False

        # ----------------------------------------------------
        # Check current feedback.
        # ----------------------------------------------------

        feedback = self.get_feedback_snapshot()

        if not feedback['valid']:

            self.get_logger().warning(
                f'{movement_name} rejected: '
                'no valid robot feedback.'
            )

            return False

        if (
            feedback['position'] is None
            or feedback['last_feedback_time'] is None
        ):

            self.get_logger().warning(
                f'{movement_name} rejected: '
                'incomplete feedback state.'
            )

            return False

        feedback_age = (
            time.monotonic()
            - feedback['last_feedback_time']
        )

        if (
            feedback_age
            > self.max_feedback_age_s
        ):

            self.get_logger().warning(
                f'{movement_name} rejected: '
                f'feedback is stale '
                f'({feedback_age:.3f} s).'
            )

            return False

        # Current measured position must already be inside the
        # configured software limits before a new motion starts.
        valid, invalid_joint = (
            self.positions_within_limits(
                feedback['position']
            )
        )

        if not valid:

            joint_text = (
                'unknown'
                if invalid_joint is None
                else str(invalid_joint + 1)
            )

            self.get_logger().error(
                f'{movement_name} rejected: '
                f'measured Joint {joint_text} is outside '
                'the configured software limit.'
            )

            return False

        if not self.accept_commands:

            self.get_logger().warning(
                f'{movement_name} rejected: '
                'robot is not ready to receive commands.'
            )

            return False

        # Motion begins only from ENABLED_IDLE.
        if feedback['robot_mode'] != 5:

            mode_name = (
                ROBOT_MODE_NAMES.get(
                    feedback['robot_mode'],
                    'UNKNOWN'
                )
            )

            self.get_logger().warning(
                f'{movement_name} rejected: '
                f'RobotMode='
                f'{feedback["robot_mode"]} '
                f'({mode_name}). '
                'Expected ENABLED_IDLE (5).'
            )

            return False

        return True

    # ========================================================
    # Execute MovJ position movement
    # ========================================================

    def execute_joint_move(
        self,
        position_rad,
        velocity_ratio,
        acceleration_ratio,
        movement_name
    ):

        if not self.motion_allowed(
            movement_name
        ):

            return

        # ----------------------------------------------------
        # Validate target
        # ----------------------------------------------------

        if len(position_rad) != 6:

            self.get_logger().error(
                f'{movement_name} rejected: '
                'exactly 6 joint positions are required.'
            )

            return

        position_rad = [
            float(q)
            for q in position_rad
        ]

        if not all(
            math.isfinite(q)
            for q in position_rad
        ):

            self.get_logger().error(
                f'{movement_name} rejected: '
                'non-finite joint position.'
            )

            return

        valid, invalid_joint = (
            self.positions_within_limits(
                position_rad
            )
        )

        if not valid:

            joint_text = (
                'unknown'
                if invalid_joint is None
                else str(invalid_joint + 1)
            )

            self.get_logger().error(
                f'{movement_name} rejected: '
                f'Joint {joint_text} target '
                'exceeds software joint limits.'
            )

            return

        near_limit_joints = (
            self.position_near_limit_joints(
                position_rad
            )
        )

        if near_limit_joints:

            self.get_logger().warning(
                f'{movement_name}: target is inside the '
                'joint-limit warning zone for joint(s): '
                + ', '.join(
                    str(i + 1)
                    for i in near_limit_joints
                )
            )

        # ----------------------------------------------------
        # Validate percentages
        # ----------------------------------------------------

        velocity_ratio = float(
            velocity_ratio
        )

        acceleration_ratio = float(
            acceleration_ratio
        )

        if not (
            math.isfinite(velocity_ratio)
            and
            1.0 <= velocity_ratio <= 100.0
        ):

            self.get_logger().error(
                f'{movement_name} rejected: '
                'velocity ratio must be 1-100 %.'
            )

            return

        if not (
            math.isfinite(acceleration_ratio)
            and
            1.0 <= acceleration_ratio <= 100.0
        ):

            self.get_logger().error(
                f'{movement_name} rejected: '
                'acceleration ratio must be 1-100 %.'
            )

            return

        position_deg = [
            math.degrees(q)
            for q in position_rad
        ]

        velocity_ratio_cmd = int(
            round(velocity_ratio)
        )

        acceleration_ratio_cmd = int(
            round(acceleration_ratio)
        )

        # ----------------------------------------------------
        # Enter POSITION mode before sending MovJ.
        # ----------------------------------------------------

        with self.state_lock:

            if self.control_mode != CONTROL_MODE_IDLE:

                self.get_logger().warning(
                    f'{movement_name} rejected: '
                    'control mode changed before execution.'
                )

                return

            self.control_mode = (
                CONTROL_MODE_POSITION
            )

            self.position_target = (
                position_rad.copy()
            )

            self.position_command_time = (
                time.monotonic()
            )

            self.position_motion_started = False

        self.publish_control_mode_monitor(
            CONTROL_MODE_POSITION
        )

        self.get_logger().info(
            '================================='
        )

        self.get_logger().info(
            f'MOVING ROBOT: {movement_name}'
        )

        self.get_logger().info(
            'Target [rad]: '
            + str([
                round(q, 4)
                for q in position_rad
            ])
        )

        self.get_logger().info(
            f'Velocity ratio: '
            f'{velocity_ratio_cmd} %'
        )

        self.get_logger().info(
            f'Acceleration ratio: '
            f'{acceleration_ratio_cmd} %'
        )

        self.get_logger().info(
            'Control mode -> POSITION'
        )

        self.get_logger().info(
            '================================='
        )

        try:

            with self.command_lock:

                if self.robot is None:

                    raise RuntimeError(
                        'Robot connection unavailable.'
                    )

                response = (
                    self.robot.motion.MovJ(
                        position_deg,
                        CoordinateType.JOINT,
                        a=acceleration_ratio_cmd,
                        v=velocity_ratio_cmd,
                        cp=0
                    )
                )

            self.get_logger().info(
                f'MovJ ({movement_name}): '
                f'{response}'
            )

            if not response.startswith('0,'):

                self.reset_all_motion_state()

                self.get_logger().error(
                    f'{movement_name} rejected '
                    f'by robot controller.'
                )

        except Exception as e:

            self.reset_all_motion_state()

            self.get_logger().error(
                f'{movement_name} movement error: {e}'
            )

    # ========================================================
    # General position command callback
    # ========================================================

    def joint_position_callback(self, msg):

        self.execute_joint_move(
            position_rad=list(msg.position),
            velocity_ratio=msg.velocity_ratio,
            acceleration_ratio=msg.acceleration_ratio,
            movement_name='JOINT POSITION'
        )

    # ========================================================
    # HOME
    # ========================================================

    def home_callback(self, msg):

        del msg

        self.execute_joint_move(
            position_rad=self.home_position_rad,
            velocity_ratio=self.home_velocity_ratio,
            acceleration_ratio=self.home_acceleration_ratio,
            movement_name='HOME'
        )

    # ========================================================
    # PACKING
    # ========================================================

    def packing_callback(self, msg):

        del msg

        self.execute_joint_move(
            position_rad=self.packing_position_rad,
            velocity_ratio=self.packing_velocity_ratio,
            acceleration_ratio=self.packing_acceleration_ratio,
            movement_name='PACKING'
        )

    # ========================================================
    # Joint velocity command callback
    # ========================================================

    def joint_velocity_callback(self, msg):

        # ----------------------------------------------------
        # Validate command structure and values
        # ----------------------------------------------------

        if len(msg.velocity) != 6:

            self.get_logger().error(
                'Velocity command rejected: '
                'exactly 6 joint velocities are required.'
            )

            return

        velocity = [
            float(v)
            for v in msg.velocity
        ]

        if not all(
            math.isfinite(v)
            for v in velocity
        ):

            self.get_logger().error(
                'Velocity command rejected: '
                'all velocities must be finite.'
            )

            return

        for i in range(6):

            if (
                abs(velocity[i])
                > self.velocity_software_limit_rad_s
            ):

                self.get_logger().error(
                    'Velocity command rejected: '
                    f'Joint {i + 1} requested '
                    f'{velocity[i]:.3f} rad/s. '
                    f'Current software limit is '
                    f'+/-'
                    f'{self.velocity_software_limit_rad_s:.3f} '
                    'rad/s.'
                )

                return

        with self.state_lock:

            control_mode = (
                self.control_mode
            )

        # POSITION owns the robot until it completes or the
        # user explicitly stops it.
        if (
            control_mode
            == CONTROL_MODE_POSITION
        ):

            self.get_logger().warning(
                'Velocity command rejected: '
                'POSITION mode is currently active. '
                'Use /dobot/stop first.'
            )

            return

        # Zero velocity is an explicit exit from VELOCITY mode.
        zero_command = all(
            abs(v)
            <= self.zero_velocity_epsilon_rad_s
            for v in velocity
        )

        if zero_command:

            if (
                control_mode
                == CONTROL_MODE_VELOCITY
            ):

                self.get_logger().info(
                    'Zero velocity command received.'
                )

                self.stop_robot_motion(
                    reason='zero velocity command'
                )

            else:

                self.get_logger().info(
                    'Zero velocity command received '
                    'while already IDLE.'
                )

            return

        # Every nonzero command needs fresh feedback because
        # the warning-zone direction guard uses q_actual.
        feedback = (
            self.get_feedback_snapshot()
        )

        if (
            not feedback['valid']
            or feedback['position'] is None
            or feedback['velocity'] is None
            or feedback['last_feedback_time'] is None
        ):

            if (
                control_mode
                == CONTROL_MODE_VELOCITY
            ):

                self.stop_robot_motion(
                    reason='velocity command: feedback unavailable'
                )

            else:

                self.get_logger().warning(
                    'Velocity mode rejected: '
                    'incomplete feedback state.'
                )

            return

        now = time.monotonic()

        feedback_age = (
            now
            - feedback['last_feedback_time']
        )

        if (
            feedback_age
            > self.max_feedback_age_s
        ):

            if (
                control_mode
                == CONTROL_MODE_VELOCITY
            ):

                self.stop_robot_motion(
                    reason='velocity command: stale feedback'
                )

            else:

                self.get_logger().warning(
                    'Velocity mode rejected: '
                    f'feedback age is '
                    f'{feedback_age:.3f} s.'
                )

            return

        # ----------------------------------------------------
        # Directional joint-limit guard
        #
        # Once q_actual enters the warning zone, commands that
        # move farther toward that joint limit are blocked.
        # Commands that move away from the limit remain valid.
        # ----------------------------------------------------

        toward_limit, limit_joint = (
            self.velocity_command_toward_limit(
                feedback['position'],
                velocity
            )
        )

        if toward_limit:

            message = (
                'Velocity command blocked: '
                f'Joint {limit_joint + 1} is inside the '
                'joint-limit warning zone and the requested '
                'velocity moves it toward the limit.'
            )

            if (
                control_mode
                == CONTROL_MODE_VELOCITY
            ):

                self.get_logger().error(
                    message
                )

                self.stop_robot_motion(
                    reason='velocity command toward joint limit'
                )

            else:

                self.get_logger().warning(
                    message
                )

            return

        # ----------------------------------------------------
        # Existing VELOCITY mode:
        # update setpoint and watchdog only.
        # q_ref is not reinitialized.
        # ----------------------------------------------------

        if (
            control_mode
            == CONTROL_MODE_VELOCITY
        ):

            with self.state_lock:

                if (
                    self.control_mode
                    != CONTROL_MODE_VELOCITY
                ):

                    return

                self.qdot_desired = (
                    velocity.copy()
                )

                self.last_velocity_command_time = (
                    now
                )

            return

        # ====================================================
        # Start a new VELOCITY mode
        # ====================================================

        if self.recovery_in_progress:

            self.get_logger().warning(
                'Velocity mode rejected: '
                'robot recovery is in progress.'
            )

            return

        if (
            feedback['robot_mode']
            != 5
        ):

            mode_name = (
                ROBOT_MODE_NAMES.get(
                    feedback['robot_mode'],
                    'UNKNOWN'
                )
            )

            self.get_logger().warning(
                'Velocity mode rejected: '
                f'RobotMode='
                f'{feedback["robot_mode"]} '
                f'({mode_name}). '
                'Expected ENABLED_IDLE (5).'
            )

            return

        # qd_actual is intentionally NOT used as a startup
        # interlock. Small non-zero velocity feedback values
        # can be present while the robot is physically idle.
        # RobotMode == 5 (ENABLED_IDLE), fresh feedback and
        # valid joint positions are sufficient to start.

        valid, invalid_joint = (
            self.positions_within_limits(
                feedback['position']
            )
        )

        if not valid:

            joint_text = (
                'unknown'
                if invalid_joint is None
                else str(invalid_joint + 1)
            )

            self.get_logger().error(
                'Velocity mode rejected: '
                f'Joint {joint_text} is outside '
                'the configured software limits.'
            )

            return

        # Atomic transition IDLE -> VELOCITY.
        with self.state_lock:

            if (
                self.control_mode
                != CONTROL_MODE_IDLE
            ):

                self.get_logger().warning(
                    'Velocity mode rejected: '
                    'control mode changed during startup.'
                )

                return

            self.q_ref = (
                feedback['position'].copy()
            )

            self.qdot_desired = (
                velocity.copy()
            )

            self.last_servo_time = (
                now
            )

            self.last_velocity_command_time = (
                now
            )

            self.last_velocity_status_log_time = (
                now
            )

            self.velocity_tracking_error_start_time = None
            self.last_velocity_tracking_warning_time = None

            self.control_mode = (
                CONTROL_MODE_VELOCITY
            )

        self.publish_control_mode_monitor(
            CONTROL_MODE_VELOCITY
        )

        self.get_logger().info(
            '================================='
        )

        self.get_logger().info(
            'VELOCITY CONTROL STARTED'
        )

        self.get_logger().info(
            'Control mode: IDLE -> VELOCITY'
        )

        self.get_logger().info(
            'Initial q_ref obtained from q_actual.'
        )

        self.get_logger().info(
            'Desired velocity [rad/s]: '
            + str([
                round(v, 4)
                for v in velocity
            ])
        )

        self.get_logger().info(
            '================================='
        )

    # ========================================================
    # ServoJ velocity-control loop
    # ========================================================

    def velocity_control_loop(self):

        # ----------------------------------------------------
        # Read velocity-control state
        # ----------------------------------------------------

        with self.state_lock:

            if (
                self.control_mode
                != CONTROL_MODE_VELOCITY
            ):

                return

            qdot_desired = (
                self.qdot_desired.copy()
            )

            q_ref = (
                None
                if self.q_ref is None
                else self.q_ref.copy()
            )

            last_servo_time = (
                self.last_servo_time
            )

            last_command_time = (
                self.last_velocity_command_time
            )

            last_status_log_time = (
                self.last_velocity_status_log_time
            )

        if (
            q_ref is None
            or last_servo_time is None
            or last_command_time is None
        ):

            self.stop_robot_motion(
                reason='invalid velocity-control state'
            )

            return

        now = time.monotonic()

        # ====================================================
        # 1. Velocity-command watchdog
        # ====================================================

        command_age = (
            now
            - last_command_time
        )

        if (
            command_age
            > self.velocity_command_timeout_s
        ):

            self.get_logger().warning(
                'VELOCITY WATCHDOG: '
                f'no new command for '
                f'{command_age:.3f} s.'
            )

            self.stop_robot_motion(
                reason='velocity command timeout'
            )

            return

        # ====================================================
        # 2. Feedback watchdog
        # ====================================================

        with self.feedback_lock:

            feedback_valid = (
                self.feedback_valid
            )

            last_feedback_time = (
                self.last_feedback_time
            )

            current_robot_mode = (
                self.latest_robot_mode
            )

            current_velocity = (
                None
                if self.latest_joint_velocity is None
                else self.latest_joint_velocity.copy()
            )

        if (
            not feedback_valid
            or last_feedback_time is None
        ):

            self.stop_robot_motion(
                reason='realtime feedback unavailable'
            )

            return

        feedback_age = (
            now
            - last_feedback_time
        )

        if feedback_age > self.max_feedback_age_s:

            self.get_logger().error(
                'VELOCITY CONTROL: '
                f'feedback timeout '
                f'({feedback_age:.3f} s).'
            )

            self.stop_robot_motion(
                reason='feedback timeout'
            )

            return

        # ====================================================
        # 3. RobotMode validation
        #
        # Start is only allowed in mode 5.
        #
        # Once ServoJ starts, mode 7 is also valid.
        # ====================================================

        if current_robot_mode not in (5, 7):

            mode_name = (
                ROBOT_MODE_NAMES.get(
                    current_robot_mode,
                    'UNKNOWN'
                )
            )

            self.get_logger().error(
                'VELOCITY CONTROL: '
                f'invalid RobotMode='
                f'{current_robot_mode} '
                f'({mode_name}).'
            )

            self.stop_robot_motion(
                reason='invalid RobotMode'
            )

            return

        # ====================================================
        # 4. Calculate actual dt
        #
        # monotonic time is used rather than assuming exactly
        # self.servo_period_s seconds have elapsed.
        # ====================================================

        dt = (
            now
            - last_servo_time
        )

        # ----------------------------------------------------
        # Early execution:
        #
        # do not integrate and do NOT update last_servo_time.
        # The elapsed time will accumulate until a valid dt.
        # ----------------------------------------------------

        if dt < self.servo_min_dt_s:

            return

        # ----------------------------------------------------
        # Excessive scheduler delay:
        #
        # do not generate a large position jump.
        # ----------------------------------------------------

        if dt > self.servo_max_dt_s:

            self.get_logger().error(
                'VELOCITY CONTROL TIMING FAULT: '
                f'dt={dt:.4f} s, '
                f'maximum allowed='
                f'{self.servo_max_dt_s:.4f} s.'
            )

            self.stop_robot_motion(
                reason='control-loop timing fault'
            )

            return

        # ====================================================
        # 5. Integrate velocity -> position reference
        #
        # q_ref(k+1) =
        # q_ref(k) + qdot_desired * dt
        # ====================================================

        new_q_ref = [
            q_ref[i]
            + qdot_desired[i] * dt
            for i in range(6)
        ]

        # ====================================================
        # 6. Joint-limit protection
        # ====================================================

        valid, invalid_joint = (
            self.positions_within_limits(
                new_q_ref
            )
        )

        if not valid:

            self.get_logger().error(
                'VELOCITY CONTROL: '
                f'Joint {invalid_joint + 1} '
                'reached the software position limit.'
            )

            self.stop_robot_motion(
                reason='joint position limit'
            )

            return

        # ====================================================
        # 7. ROS radians -> Dobot degrees
        # ====================================================

        q_ref_deg = [
            math.degrees(q)
            for q in new_q_ref
        ]

        # ====================================================
        # 8. Send ServoJ
        #
        # The measured dt is also passed as ServoJ t.
        # ====================================================

        try:

            with self.command_lock:

                if self.robot is None:

                    raise RuntimeError(
                        'Robot connection unavailable.'
                    )

                response = (
                    self.robot.motion.ServoJ(
                        q_ref_deg,
                        t=dt
                    )
                )

        except Exception as e:

            self.get_logger().error(
                f'ServoJ communication error: {e}'
            )

            self.stop_robot_motion(
                reason='ServoJ communication error'
            )

            return

        # ====================================================
        # 9. Check ServoJ acceptance
        # ====================================================

        if not response.startswith('0,'):

            self.get_logger().error(
                'ServoJ rejected by robot: '
                f'{response}'
            )

            self.stop_robot_motion(
                reason='ServoJ rejected'
            )

            return

        # ====================================================
        # 10. Commit new trajectory state only after the
        #     ServoJ command was accepted.
        # ====================================================

        with self.state_lock:

            if (
                self.control_mode
                != CONTROL_MODE_VELOCITY
            ):

                return

            self.q_ref = (
                new_q_ref
            )

            self.last_servo_time = (
                now
            )

        # ====================================================
        # 11. Periodic diagnostics
        #
        # Avoid printing at 33 Hz.
        # ====================================================

        if (
            last_status_log_time is None
            or (
                now
                - last_status_log_time
                >= 1.0
            )
        ):

            actual_text = 'N/A'

            if current_velocity is not None:

                actual_text = str([
                    round(v, 4)
                    for v in current_velocity
                ])

            self.get_logger().info(
                'VELOCITY CONTROL | '
                f'dt={dt:.4f} s | '
                f'qdot_des={str([round(v, 4) for v in qdot_desired])} | '
                f'qdot_actual={actual_text}'
            )

            with self.state_lock:

                if (
                    self.control_mode
                    == CONTROL_MODE_VELOCITY
                ):

                    self.last_velocity_status_log_time = (
                        now
                    )

    # ========================================================
    # ES01 suction gripper
    # ========================================================

    def gripper_logical_to_raw(self, suction_on):

        logical = bool(suction_on)

        if self.gripper_active_high:
            return 1 if logical else 0

        return 0 if logical else 1

    def gripper_raw_to_logical(self, raw_status):

        raw_on = int(raw_status) == 1

        if self.gripper_active_high:
            return raw_on

        return not raw_on

    @staticmethod
    def parse_single_io_status(response):

        if not isinstance(response, str):
            return None

        match = re.search(
            r'^\s*0\s*,\s*\{\s*([01])\s*\}',
            response
        )

        if match:
            return int(match.group(1))

        return None

    def publish_suction_gripper_monitor(self, suction_on):

        msg = Bool()
        msg.data = bool(suction_on)

        self.suction_gripper_monitor_pub.publish(
            msg
        )

    def suction_gripper_callback(self, msg):

        # Gripper actuation is independent from arm motion mode.
        # It may be commanded during IDLE, POSITION or VELOCITY.
        if self.recovery_in_progress:

            self.get_logger().warning(
                'Suction gripper command rejected: '
                'robot recovery is in progress.'
            )

            return

        feedback = self.get_feedback_snapshot()

        if (
            not feedback['valid']
            or feedback['last_feedback_time'] is None
        ):

            self.get_logger().warning(
                'Suction gripper command rejected: '
                'no valid robot feedback.'
            )

            return

        feedback_age = (
            time.monotonic()
            - feedback['last_feedback_time']
        )

        if feedback_age > self.max_feedback_age_s:

            self.get_logger().warning(
                'Suction gripper command rejected: '
                f'feedback is stale ({feedback_age:.3f} s).'
            )

            return

        # Tool IO is allowed while DISABLED, ENABLED_IDLE,
        # RUNNING or SINGLE_MOVE.
        if feedback['robot_mode'] not in (4, 5, 7, 8):

            mode_name = ROBOT_MODE_NAMES.get(
                feedback['robot_mode'],
                'UNKNOWN'
            )

            self.get_logger().warning(
                'Suction gripper command rejected: '
                f'RobotMode={feedback["robot_mode"]} '
                f'({mode_name}).'
            )

            return

        suction_on = bool(msg.data)
        raw_status = self.gripper_logical_to_raw(
            suction_on
        )

        try:

            with self.command_lock:

                if self.robot is None:

                    raise RuntimeError(
                        'Robot connection unavailable.'
                    )

                response = (
                    self.robot.io.ToolDOInstant(
                        self.gripper_tool_do_index,
                        raw_status
                    )
                )

        except Exception as e:

            self.get_logger().error(
                'Suction gripper command error: '
                f'{e}'
            )

            return

        self.get_logger().info(
            'ToolDOInstant(gripper): '
            f'{response}'
        )

        if not response.startswith('0,'):

            self.get_logger().error(
                'Suction gripper command rejected by '
                f'robot controller: {response}'
            )

            return

        # The controller accepted the ToolDOInstant command.
        # Publish immediately. A GetToolDO readback will verify
        # the electrical output whenever VELOCITY mode is inactive.
        self.gripper_state = suction_on
        self.gripper_state_valid = True

        self.publish_suction_gripper_monitor(
            suction_on
        )

        action = (
            'ON / ADSORB'
            if suction_on
            else 'OFF / RELEASE'
        )

        self.get_logger().info(
            'Suction gripper -> '
            f'{action}'
        )

    def monitor_suction_gripper(self):

        # Avoid GetToolDO polling while ServoJ is streaming.
        # This prevents periodic dashboard IO queries from adding
        # timing jitter to the velocity-control loop.
        if self.recovery_in_progress:
            return

        with self.state_lock:
            control_mode = self.control_mode

        if control_mode == CONTROL_MODE_VELOCITY:
            return

        feedback = self.get_feedback_snapshot()

        if (
            not feedback['valid']
            or feedback['last_feedback_time'] is None
        ):
            return

        feedback_age = (
            time.monotonic()
            - feedback['last_feedback_time']
        )

        if feedback_age > self.max_feedback_age_s:
            return

        if feedback['robot_mode'] not in (4, 5, 7, 8):
            return

        try:

            with self.command_lock:

                if self.robot is None:
                    return

                response = (
                    self.robot.io.GetToolDO(
                        self.gripper_tool_do_index
                    )
                )

        except Exception as e:

            self.get_logger().warning(
                'Suction gripper monitor readback error: '
                f'{e}'
            )

            return

        raw_status = self.parse_single_io_status(
            response
        )

        if raw_status is None:

            self.get_logger().warning(
                'Unable to parse GetToolDO response: '
                f'{response}'
            )

            return

        suction_on = self.gripper_raw_to_logical(
            raw_status
        )

        state_changed = (
            not self.gripper_state_valid
            or self.gripper_state != suction_on
        )

        self.gripper_state = suction_on
        self.gripper_state_valid = True

        self.publish_suction_gripper_monitor(
            suction_on
        )

        if state_changed:

            action = (
                'ON / ADSORB'
                if suction_on
                else 'OFF / RELEASE'
            )

            self.get_logger().info(
                'Suction gripper readback -> '
                f'{action}'
            )

    # ========================================================
    # STOP any current motion
    # ========================================================

    def stop_robot_motion(self, reason='user request'):

        # ----------------------------------------------------
        # Immediately prevent further ServoJ / MovJ activity.
        # ----------------------------------------------------

        with self.state_lock:

            previous_mode = (
                self.control_mode
            )

            self._reset_position_state_locked()

            self._reset_velocity_state_locked()

            self.control_mode = (
                CONTROL_MODE_IDLE
            )

        if (
            previous_mode
            != CONTROL_MODE_IDLE
        ):

            self.publish_control_mode_monitor(
                CONTROL_MODE_IDLE
            )

        self.get_logger().info(
            '================================='
        )

        self.get_logger().info(
            f'STOPPING ROBOT MOTION: {reason}'
        )

        self.get_logger().info(
            f'Previous control mode: '
            f'{previous_mode}'
        )

        # ----------------------------------------------------
        # Stop DOBOT motion queue / current movement.
        # ----------------------------------------------------

        try:

            with self.command_lock:

                if self.robot is None:

                    raise RuntimeError(
                        'Robot connection unavailable.'
                    )

                response = (
                    self.robot.robot_control.Stop()
                )

            self.get_logger().info(
                f'Stop: {response}'
            )

            if response.startswith('0,'):

                self.get_logger().info(
                    'Robot Stop command accepted.'
                )

            else:

                self.get_logger().warning(
                    'Robot Stop command returned: '
                    f'{response}'
                )

        except Exception as e:

            self.get_logger().warning(
                f'Robot Stop command error: {e}'
            )

        self.get_logger().info(
            'Control mode -> IDLE'
        )

        self.get_logger().info(
            '================================='
        )

    # ========================================================
    # /dobot/stop callback
    # ========================================================

    def stop_callback(self, msg):

        del msg

        self.stop_robot_motion(
            reason='/dobot/stop'
        )

    # ========================================================
    # Enable / Disable
    # ========================================================

    def enable_callback(self, msg):

        if self.recovery_in_progress:

            self.get_logger().warning(
                'Enable/Disable rejected: '
                'recovery is in progress.'
            )

            return

        with self.feedback_lock:

            current_mode = (
                self.latest_robot_mode
            )

        with self.state_lock:

            control_mode = (
                self.control_mode
            )

        # ====================================================
        # ENABLE
        # ====================================================

        if msg.data:

            if control_mode != CONTROL_MODE_IDLE:

                self.get_logger().warning(
                    'Enable rejected: '
                    f'control mode is {control_mode}.'
                )

                return

            if not self.accept_commands:

                self.get_logger().warning(
                    'Enable rejected: '
                    'robot is not ready.'
                )

                return

            if current_mode == 3:

                self.get_logger().error(
                    'Robot is POWEROFF. '
                    'Use /dobot/recover.'
                )

                return

            if current_mode == 9:

                self.get_logger().error(
                    'Robot is in ERROR. '
                    'Use /dobot/recover.'
                )

                return

            if current_mode != 4:

                self.get_logger().warning(
                    'Enable ignored: '
                    f'RobotMode={current_mode}. '
                    'Expected DISABLED (4).'
                )

                return

            try:

                with self.command_lock:

                    if self.robot is None:

                        raise RuntimeError(
                            'Robot connection unavailable.'
                        )

                    response = (
                        self.robot
                        .robot_control
                        .EnableRobot()
                    )

                self.get_logger().info(
                    f'EnableRobot: {response}'
                )

                if response.startswith('0,'):

                    self.robot_enabled_by_driver = True

                    self.get_logger().info(
                        'Robot enabled successfully.'
                    )

            except Exception as e:

                self.get_logger().error(
                    f'EnableRobot error: {e}'
                )

        # ====================================================
        # DISABLE
        # ====================================================

        else:

            # ------------------------------------------------
            # Stop any active motion first.
            # ------------------------------------------------

            if control_mode != CONTROL_MODE_IDLE:

                self.stop_robot_motion(
                    reason='disable requested'
                )

            try:

                with self.command_lock:

                    if self.robot is None:

                        raise RuntimeError(
                            'Robot connection unavailable.'
                        )

                    response = (
                        self.robot
                        .robot_control
                        .DisableRobot()
                    )

                self.get_logger().info(
                    f'DisableRobot: {response}'
                )

                if response.startswith('0,'):

                    self.robot_enabled_by_driver = False

                    self.get_logger().info(
                        'Robot disabled successfully.'
                    )

            except Exception as e:

                self.get_logger().error(
                    f'DisableRobot error: {e}'
                )

    # ========================================================
    # Recovery callback
    # ========================================================

    def recover_callback(self, msg):

        del msg

        if self.recovery_in_progress:

            self.get_logger().warning(
                'Recovery already in progress.'
            )

            return

        with self.feedback_lock:

            current_mode = (
                self.latest_robot_mode
            )

        if current_mode == 5:

            self.get_logger().info(
                'Recovery not required: '
                'robot is already enabled.'
            )

            return

        if current_mode == 4:

            self.accept_commands = True

            self.get_logger().info(
                'Recovery not required: '
                'robot is DISABLED and ready.'
            )

            return

        # ----------------------------------------------------
        # Clear local motion state.
        #
        # Do not depend on Stop() here because the robot may
        # already be in an E-stop/error state.
        # ----------------------------------------------------

        self.reset_all_motion_state()

        self.recovery_in_progress = True

        self.accept_commands = False

        self.recovery_thread = threading.Thread(
            target=self.recovery_worker,
            daemon=True
        )

        self.recovery_thread.start()

    # ========================================================
    # Clear errors for recovery
    # ========================================================

    def clear_errors_for_recovery(self):

        for attempt in range(
            1,
            self.clear_error_max_attempts + 1
        ):

            if self.shutdown_event.is_set():

                return None

            self.get_logger().info(
                f'RECOVERY: ClearError attempt '
                f'{attempt}/'
                f'{self.clear_error_max_attempts}...'
            )

            try:

                with self.command_lock:

                    if self.robot is None:

                        raise RuntimeError(
                            'Robot connection unavailable.'
                        )

                    response = (
                        self.robot
                        .robot_control
                        .ClearError()
                    )

                self.get_logger().info(
                    f'ClearError: {response}'
                )

                if not response.startswith('0,'):

                    self.shutdown_event.wait(
                        self.clear_error_retry_period_s
                    )

                    continue

                self.get_logger().info(
                    'RECOVERY: ClearError accepted.'
                )

                valid_mode = None

                valid_count = 0

                for _ in range(6):

                    if self.shutdown_event.is_set():

                        return None

                    self.shutdown_event.wait(
                        self.clear_error_mode_check_period_s
                    )

                    with self.command_lock:

                        if self.robot is None:

                            raise RuntimeError(
                                'Robot connection unavailable.'
                            )

                        mode_response = (
                            self.robot
                            .robot_control
                            .RobotMode()
                        )

                    mode = self.parse_robot_mode(
                        mode_response
                    )

                    if mode is None:

                        valid_mode = None

                        valid_count = 0

                        continue

                    mode_name = (
                        ROBOT_MODE_NAMES.get(
                            mode,
                            'UNKNOWN'
                        )
                    )

                    self.get_logger().info(
                        'RECOVERY: RobotMode after '
                        f'ClearError = '
                        f'{mode} ({mode_name})'
                    )

                    if mode in (3, 4):

                        if mode == valid_mode:

                            valid_count += 1

                        else:

                            valid_mode = mode

                            valid_count = 1

                        if (
                            valid_count
                            >= self.clear_error_valid_mode_count
                        ):

                            return mode

                    elif mode == 9:

                        valid_mode = None

                        valid_count = 0

                        self.get_logger().warning(
                            'RECOVERY: robot returned '
                            'to ERROR.'
                        )

                        break

                    else:

                        valid_mode = None

                        valid_count = 0

                self.shutdown_event.wait(
                    self.clear_error_retry_period_s
                )

            except Exception as e:

                self.get_logger().warning(
                    f'RECOVERY: ClearError failed: {e}'
                )

                self.shutdown_event.wait(
                    self.clear_error_retry_period_s
                )

        return None

    # ========================================================
    # Complete recovery worker
    # ========================================================

    def recovery_worker(self):

        recovery_success = False

        try:

            self.robot_enabled_by_driver = False

            self.get_logger().info(
                '================================='
            )

            self.get_logger().info(
                'STARTING ROBOT RECOVERY'
            )

            self.get_logger().info(
                'Normal commands are blocked.'
            )

            self.get_logger().info(
                '================================='
            )

            post_clear_mode = (
                self.clear_errors_for_recovery()
            )

            if post_clear_mode is None:

                raise RuntimeError(
                    'Robot remained in ERROR after '
                    'ClearError attempts.'
                )

            mode_name = (
                ROBOT_MODE_NAMES.get(
                    post_clear_mode,
                    'UNKNOWN'
                )
            )

            self.get_logger().info(
                f'RECOVERY: post-ClearError mode='
                f'{post_clear_mode} ({mode_name})'
            )

            # =================================================
            # Already powered
            # =================================================

            if post_clear_mode == 4:

                self.accept_commands = True

                recovery_success = True

                self.get_logger().info(
                    'RECOVERY COMPLETE: '
                    'robot is DISABLED and ready.'
                )

                return

            # =================================================
            # Must be POWEROFF for PowerOn
            # =================================================

            if post_clear_mode != 3:

                raise RuntimeError(
                    f'Unexpected recovery state: '
                    f'{post_clear_mode}'
                )

            with self.command_lock:

                robot = self.robot

            if robot is None:

                raise RuntimeError(
                    'Robot connection unavailable.'
                )

            # =================================================
            # Stop realtime feedback
            # =================================================

            try:

                robot.StopFeedbackMonitor()

                self.get_logger().info(
                    'RECOVERY: realtime feedback stopped.'
                )

            except Exception as e:

                self.get_logger().warning(
                    f'Could not stop feedback: {e}'
                )

            self.invalidate_feedback()

            # =================================================
            # PowerOn
            # =================================================

            self.get_logger().info(
                'RECOVERY: sending PowerOn...'
            )

            try:

                response = (
                    robot
                    .robot_control
                    .PowerOn()
                )

                self.get_logger().info(
                    f'PowerOn: {response}'
                )

            except Exception as e:

                self.get_logger().warning(
                    'RECOVERY: Dashboard connection '
                    'interrupted during PowerOn.'
                )

                self.get_logger().warning(
                    'This is expected during controller '
                    'restart.'
                )

                self.get_logger().warning(
                    f'PowerOn exception: {e}'
                )

            # =================================================
            # Discard old connection
            # =================================================

            try:

                robot.Disconnect()

            except Exception:

                pass

            with self.command_lock:

                if self.robot is robot:

                    self.robot = None

            # =================================================
            # Initialization phase
            # =================================================

            self.get_logger().info(
                '================================='
            )

            self.get_logger().info(
                'INITIALIZING ROBOT...'
            )

            self.get_logger().info(
                'New robot commands remain blocked.'
            )

            self.get_logger().info(
                '================================='
            )

            start_time = (
                time.monotonic()
            )

            deadline = (
                start_time
                + self.power_on_timeout_s
            )

            stable_disabled_count = 0

            while (
                time.monotonic() < deadline
                and not self.shutdown_event.is_set()
            ):

                elapsed = (
                    time.monotonic()
                    - start_time
                )

                probe_robot = None

                try:

                    self.get_logger().info(
                        'INITIALIZING: checking controller '
                        f'({elapsed:.0f} s elapsed)...'
                    )

                    probe_robot = DobotRobot(
                        self.robot_ip
                    )

                    probe_robot.Connect()

                    response = (
                        probe_robot
                        .robot_control
                        .RequestControl()
                    )

                    if not response.startswith('0,'):

                        raise RuntimeError(
                            f'RequestControl failed: '
                            f'{response}'
                        )

                    mode_response = (
                        probe_robot
                        .robot_control
                        .RobotMode()
                    )

                    mode = self.parse_robot_mode(
                        mode_response
                    )

                    if mode is None:

                        raise RuntimeError(
                            'Unable to parse RobotMode.'
                        )

                    mode_name = (
                        ROBOT_MODE_NAMES.get(
                            mode,
                            'UNKNOWN'
                        )
                    )

                    self.get_logger().info(
                        f'INITIALIZING: RobotMode='
                        f'{mode} ({mode_name})'
                    )

                    if mode == 4:

                        stable_disabled_count += 1

                    else:

                        stable_disabled_count = 0

                except Exception as e:

                    stable_disabled_count = 0

                    self.get_logger().info(
                        'INITIALIZING: controller '
                        f'not ready yet: {e}'
                    )

                finally:

                    if probe_robot is not None:

                        try:

                            probe_robot.Disconnect()

                        except Exception:

                            pass

                # =================================================
                # Stable DISABLED state
                # =================================================

                if (
                    stable_disabled_count
                    >= self.power_on_stable_count
                ):

                    self.get_logger().info(
                        'INITIALIZING: controller stable.'
                    )

                    new_robot = None

                    try:

                        new_robot = DobotRobot(
                            self.robot_ip
                        )

                        new_robot.Connect()

                        response = (
                            new_robot
                            .robot_control
                            .RequestControl()
                        )

                        if not response.startswith('0,'):

                            raise RuntimeError(
                                f'RequestControl failed: '
                                f'{response}'
                            )

                        mode_response = (
                            new_robot
                            .robot_control
                            .RobotMode()
                        )

                        mode = self.parse_robot_mode(
                            mode_response
                        )

                        if mode != 4:

                            raise RuntimeError(
                                'Robot changed state during '
                                'reconnection.'
                            )

                        new_robot.StartFeedbackMonitor(
                            callback=self.feedback_callback
                        )

                        with self.command_lock:

                            self.robot = new_robot

                        self.accept_commands = True

                        recovery_success = True

                        self.get_logger().info(
                            '================================='
                        )

                        self.get_logger().info(
                            'ROBOT INITIALIZATION COMPLETE'
                        )

                        self.get_logger().info(
                            'RobotMode = 4 (DISABLED)'
                        )

                        self.get_logger().info(
                            'Realtime feedback restored.'
                        )

                        self.get_logger().info(
                            'Use /dobot/enable.'
                        )

                        self.get_logger().info(
                            '================================='
                        )

                        return

                    except Exception as e:

                        stable_disabled_count = 0

                        self.get_logger().warning(
                            'Permanent connection failed: '
                            f'{e}'
                        )

                        if new_robot is not None:

                            try:

                                new_robot.Disconnect()

                            except Exception:

                                pass

                self.shutdown_event.wait(
                    self.power_on_poll_period_s
                )

            if not self.shutdown_event.is_set():

                raise RuntimeError(
                    'PowerOn initialization timeout.'
                )

        except Exception as e:

            self.accept_commands = False

            self.get_logger().error(
                '================================='
            )

            self.get_logger().error(
                f'RECOVERY FAILED: {e}'
            )

            self.get_logger().error(
                'Robot commands remain blocked.'
            )

            self.get_logger().error(
                '================================='
            )

        finally:

            self.recovery_in_progress = False

            if not recovery_success:

                self.robot_enabled_by_driver = False

    # ========================================================
    # Invalidate feedback
    # ========================================================

    def invalidate_feedback(self):

        with self.feedback_lock:

            self.latest_joint_position = None

            self.latest_joint_velocity = None

            self.latest_robot_mode = None

            self.last_feedback_time = None

            self.feedback_valid = False

            self.feedback_received = False

            self.last_logged_robot_mode = None

        # ToolDO state will be read again after reconnection.
        self.gripper_state = None
        self.gripper_state_valid = False

    # ========================================================
    # Parse RobotMode
    # ========================================================

    @staticmethod
    def parse_robot_mode(response):

        match = re.search(
            r'\{(\d+)\}',
            response
        )

        if match:

            return int(
                match.group(1)
            )

        return None

    # ========================================================
    # Shutdown
    # ========================================================

    def destroy_node(self):

        self.get_logger().info(
            'Shutting down Dobot E6 driver...'
        )

        self.accept_commands = False

        self.shutdown_event.set()

        # ----------------------------------------------------
        # Stop active robot motion first.
        # ----------------------------------------------------

        with self.state_lock:

            active_mode = (
                self.control_mode
            )

        if active_mode != CONTROL_MODE_IDLE:

            self.stop_robot_motion(
                reason='driver shutdown'
            )

        # ----------------------------------------------------
        # Recovery thread
        # ----------------------------------------------------

        if (
            self.recovery_thread is not None
            and self.recovery_thread.is_alive()
        ):

            self.recovery_thread.join(
                timeout=2.0
            )

        # ----------------------------------------------------
        # Robot connection
        # ----------------------------------------------------

        with self.command_lock:

            robot = self.robot

        if robot is not None:

            # ------------------------------------------------
            # Release suction gripper before disabling the arm.
            #
            # If the driver knows that suction is currently ON,
            # explicitly command the configured logical OFF state
            # while the TCP/IP connection is still available.
            #
            # The raw ToolDO value is generated through
            # gripper_logical_to_raw(False), so this also works
            # correctly when gripper_active_high is False.
            # ------------------------------------------------

            if (
                self.gripper_state_valid
                and self.gripper_state
            ):

                self.get_logger().info(
                    'Suction gripper is active during shutdown. '
                    'Releasing gripper...'
                )

                try:

                    raw_off_status = (
                        self.gripper_logical_to_raw(False)
                    )

                    with self.command_lock:

                        if self.robot is None:

                            raise RuntimeError(
                                'Robot connection unavailable.'
                            )

                        response = (
                            self.robot.io.ToolDOInstant(
                                self.gripper_tool_do_index,
                                raw_off_status
                            )
                        )

                    self.get_logger().info(
                        'ToolDOInstant(gripper OFF) on shutdown: '
                        f'{response}'
                    )

                    if response.startswith('0,'):

                        self.gripper_state = False
                        self.gripper_state_valid = True

                        self.publish_suction_gripper_monitor(
                            False
                        )

                        self.get_logger().info(
                            'Suction gripper released successfully '
                            'before shutdown.'
                        )

                    else:

                        self.get_logger().warning(
                            'Suction gripper release was rejected '
                            'during shutdown: '
                            f'{response}'
                        )

                except Exception as e:

                    self.get_logger().warning(
                        'Could not release suction gripper '
                        'during shutdown: '
                        f'{e}'
                    )

            # ------------------------------------------------
            # Disable if this node enabled the robot.
            # ------------------------------------------------

            if self.robot_enabled_by_driver:

                try:

                    response = (
                        robot
                        .robot_control
                        .DisableRobot()
                    )

                    self.get_logger().info(
                        'DisableRobot on shutdown: '
                        f'{response}'
                    )

                except Exception as e:

                    self.get_logger().warning(
                        'Could not disable robot: '
                        f'{e}'
                    )

            # ------------------------------------------------
            # Stop feedback
            # ------------------------------------------------

            try:

                robot.StopFeedbackMonitor()

                self.get_logger().info(
                    'Realtime feedback monitor stopped.'
                )

            except Exception as e:

                self.get_logger().warning(
                    'Could not stop feedback: '
                    f'{e}'
                )

            # ------------------------------------------------
            # Disconnect
            # ------------------------------------------------

            try:

                robot.Disconnect()

                self.get_logger().info(
                    'Robot disconnected.'
                )

            except Exception as e:

                self.get_logger().warning(
                    f'Disconnect error: {e}'
                )

        super().destroy_node()


# ============================================================
# Main
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = DobotE6Driver()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()