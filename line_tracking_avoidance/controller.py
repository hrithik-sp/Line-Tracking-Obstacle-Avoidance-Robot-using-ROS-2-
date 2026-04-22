#!/usr/bin/env python3
"""
Line-following robot controller with obstacle avoidance.

Error is normalised to [-1, 1] by the line_detector, so PD gains are
resolution-independent.

Avoidance uses symmetric timed turns (same speed & duration in opposite
directions → rotation errors cancel).

State machine:
  FOLLOW_LINE  →  obstacle detected
  AVOID_BACKUP →  reverse for clearance
  AVOID_TURN1  →  turn ~90° away  (timed)
  AVOID_DRIVE1 →  drive sideways   (timed)
  AVOID_TURN2  →  turn ~90° back   (timed, symmetric)
  AVOID_DRIVE2 →  drive past       (timed)
  AVOID_RETURN →  arc toward line   (camera-guided)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool, String
from enum import Enum


class State(Enum):
    FOLLOW_LINE  = 1
    AVOID_BACKUP = 2
    AVOID_TURN1  = 3
    AVOID_DRIVE1 = 4
    AVOID_TURN2  = 5
    AVOID_DRIVE2 = 6
    AVOID_RETURN = 7


class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # ── Line-following parameters ──
        # Error is normalised [-1, 1], so Kp≈1.0 gives sensible angular vel.
        self.declare_parameter('Kp',                     1.0)
        self.declare_parameter('Kd',                     0.3)
        self.declare_parameter('base_speed',             0.18)
        self.declare_parameter('max_angular',            0.5)
        self.declare_parameter('search_turn_speed',      0.35)
        self.declare_parameter('max_no_line_sec',        0.8)
        self.declare_parameter('obstacle_trigger_count', 3)

        # ── Avoidance parameters ──
        self.declare_parameter('avoid_speed',            0.18)
        self.declare_parameter('turn_speed',             0.5)
        self.declare_parameter('turn_duration',          3.0)
        self.declare_parameter('strafe_time',            2.5)
        self.declare_parameter('pass_time',              5.5)
        self.declare_parameter('return_linear',          0.10)
        self.declare_parameter('return_angular',         0.35)

        self.Kp              = self.get_parameter('Kp').value
        self.Kd              = self.get_parameter('Kd').value
        self.base_speed      = self.get_parameter('base_speed').value
        self.max_ang         = self.get_parameter('max_angular').value
        self.search_turn_spd = self.get_parameter('search_turn_speed').value
        self.max_no_line     = self.get_parameter('max_no_line_sec').value
        self.obs_trigger     = int(self.get_parameter('obstacle_trigger_count').value)

        self.avoid_speed     = self.get_parameter('avoid_speed').value
        self.turn_speed      = self.get_parameter('turn_speed').value
        self.turn_dur        = self.get_parameter('turn_duration').value
        self.strafe_time     = self.get_parameter('strafe_time').value
        self.pass_time       = self.get_parameter('pass_time').value
        self.ret_lin         = self.get_parameter('return_linear').value
        self.ret_ang         = self.get_parameter('return_angular').value

        # ── Runtime state ──
        self.state          = State.FOLLOW_LINE
        self.line_error     = 0.0
        self.line_det       = False
        self.line_side      = 'none'
        self.obs_det        = False
        self.front_dist     = 10.0
        self.left_dist      = 10.0
        self.right_dist     = 10.0
        self.prev_err       = 0.0
        self.obs_count      = 0
        self.turn_dir       = 1.0

        # Timing
        self.phase_start    = 0.0
        self._clock_init    = False
        self.last_line_time = 0.0
        self.last_line_side = 'center'

        # ── Pub / Sub ──
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Float32, '/line_error',        self._cb_lerr,  10)
        self.create_subscription(Bool,    '/line_detected',     self._cb_ldet,  10)
        self.create_subscription(String,  '/line_side',         self._cb_lside, 10)
        self.create_subscription(Bool,    '/obstacle_detected', self._cb_odet,  10)
        self.create_subscription(Float32, '/front_distance',    self._cb_front, 10)
        self.create_subscription(Float32, '/left_distance',     self._cb_left,  10)
        self.create_subscription(Float32, '/right_distance',    self._cb_right, 10)

        self.create_timer(0.05, self._loop)
        self.get_logger().info('Controller ready  (normalised error, Kp={self.Kp})')

    # ── Callbacks ────────────────────────────────────────────────────────
    def _cb_lerr(self,  m): self.line_error = m.data
    def _cb_ldet(self,  m): self.line_det   = m.data
    def _cb_lside(self, m): self.line_side  = m.data
    def _cb_odet(self,  m): self.obs_det    = m.data
    def _cb_front(self, m): self.front_dist = m.data
    def _cb_left(self,  m): self.left_dist  = m.data
    def _cb_right(self, m): self.right_dist = m.data

    # ── Utilities ────────────────────────────────────────────────────────
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _move(self, lin, ang):
        t = Twist()
        t.linear.x  = float(lin)
        t.angular.z = float(ang)
        self.cmd_pub.publish(t)

    def _go(self, new_state):
        self.get_logger().info(f'{self.state.name} -> {new_state.name}')
        self.state       = new_state
        self.phase_start = self._now()

    # ── Main loop ────────────────────────────────────────────────────────
    def _loop(self):
        if not self._clock_init:
            t = self._now()
            self.phase_start    = t
            self.last_line_time = t
            self._clock_init    = True

        now     = self._now()
        elapsed = now - self.phase_start

        # ==============================================================
        #  FOLLOW_LINE — PD tracking (error is normalised [-1, 1])
        # ==============================================================
        if self.state == State.FOLLOW_LINE:
            if self.line_det:
                self.last_line_time = now
                if self.line_side in ('left', 'right', 'center'):
                    self.last_line_side = self.line_side

            # obstacle debounce
            if self.obs_det:
                self.obs_count += 1
            else:
                self.obs_count = 0

            if self.obs_count >= self.obs_trigger:
                self.turn_dir = (1.0 if self.left_dist > self.right_dist
                                 else -1.0)
                self.obs_count = 0
                self.get_logger().info(
                    f'Obstacle!  L={self.left_dist:.2f}  R={self.right_dist:.2f}'
                    f'  → avoid {"LEFT" if self.turn_dir > 0 else "RIGHT"}')
                self._go(State.AVOID_BACKUP)

            elif not self.line_det:
                gap = now - self.last_line_time
                if gap < self.max_no_line:
                    self._move(self.base_speed * 0.5, 0.0)
                else:
                    d = 1.0 if self.last_line_side == 'left' else -1.0
                    self._move(0.0, self.search_turn_spd * d)
            else:
                # ── PD control ──
                err   = self.line_error                         # [-1, 1]
                d_err = err - self.prev_err
                ang   = -(self.Kp * err + self.Kd * d_err)
                ang   = max(-self.max_ang, min(self.max_ang, ang))
                # Slow down when line is far from centre
                speed = self.base_speed * max(0.5, 1.0 - abs(err))
                self._move(speed, ang)
                self.prev_err = err

        # ==============================================================
        #  AVOID_BACKUP
        # ==============================================================
        elif self.state == State.AVOID_BACKUP:
            if self.front_dist > 0.60 or elapsed > 2.5:
                self._go(State.AVOID_TURN1)
            else:
                self._move(-0.10, 0.0)
                self.get_logger().info(
                    f'BACKUP  front={self.front_dist:.2f}',
                    throttle_duration_sec=0.5)

        # ==============================================================
        #  AVOID_TURN1 — turn ~90° away (timed)
        # ==============================================================
        elif self.state == State.AVOID_TURN1:
            if elapsed >= self.turn_dur:
                self._move(0.0, 0.0)
                self._go(State.AVOID_DRIVE1)
            else:
                self._move(0.0, self.turn_dir * self.turn_speed)
                self.get_logger().info(
                    f'TURN1  {elapsed:.1f}/{self.turn_dur:.1f}s',
                    throttle_duration_sec=0.5)

        # ==============================================================
        #  AVOID_DRIVE1 — drive laterally
        # ==============================================================
        elif self.state == State.AVOID_DRIVE1:
            if elapsed >= self.strafe_time:
                self._go(State.AVOID_TURN2)
            elif self.front_dist < 0.25:
                self._move(0.0, 0.0)
                self.get_logger().warn('DRIVE1: front blocked')
            else:
                self._move(self.avoid_speed, 0.0)
                self.get_logger().info(
                    f'DRIVE1  {elapsed:.1f}/{self.strafe_time:.1f}s',
                    throttle_duration_sec=0.5)

        # ==============================================================
        #  AVOID_TURN2 — turn back (symmetric with TURN1)
        # ==============================================================
        elif self.state == State.AVOID_TURN2:
            if elapsed >= self.turn_dur:
                self._move(0.0, 0.0)
                self._go(State.AVOID_DRIVE2)
            else:
                self._move(0.0, -self.turn_dir * self.turn_speed)
                self.get_logger().info(
                    f'TURN2  {elapsed:.1f}/{self.turn_dur:.1f}s',
                    throttle_duration_sec=0.5)

        # ==============================================================
        #  AVOID_DRIVE2 — drive past obstacle
        # ==============================================================
        elif self.state == State.AVOID_DRIVE2:
            if self.front_dist < 0.30:
                self.get_logger().warn('DRIVE2: obstacle ahead — restarting')
                self.turn_dir = (1.0 if self.left_dist > self.right_dist
                                 else -1.0)
                self._go(State.AVOID_BACKUP)
            elif elapsed >= self.pass_time:
                self._go(State.AVOID_RETURN)
            else:
                self._move(self.avoid_speed, 0.0)
                self.get_logger().info(
                    f'DRIVE2  {elapsed:.1f}/{self.pass_time:.1f}s',
                    throttle_duration_sec=0.5)

        # ==============================================================
        #  AVOID_RETURN — arc toward line + PD rejoin
        # ==============================================================
        elif self.state == State.AVOID_RETURN:
            if self.line_det and self.line_side == 'center' and elapsed > 1.5:
                self.get_logger().info('Line centred → FOLLOW_LINE')
                self.prev_err = 0.0
                self.last_line_time = now
                self._go(State.FOLLOW_LINE)

            elif self.line_det:
                self.last_line_time = now
                self.last_line_side = self.line_side
                err = self.line_error
                ang = -(self.Kp * 1.2 * err)
                ang = max(-self.max_ang, min(self.max_ang, ang))
                self._move(self.base_speed * 0.7, ang)
                self.get_logger().info(
                    f'RETURN align  side={self.line_side}  err={err:.2f}',
                    throttle_duration_sec=0.5)

            elif elapsed > 15.0:
                self.get_logger().warn('RETURN timeout → FOLLOW_LINE')
                self.last_line_time = now
                self._go(State.FOLLOW_LINE)

            else:
                self._move(self.ret_lin, -self.turn_dir * self.ret_ang)
                self.get_logger().info(
                    f'RETURN arc  {elapsed:.1f}s',
                    throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Controller())
    rclpy.shutdown()


if __name__ == '__main__':
    main()