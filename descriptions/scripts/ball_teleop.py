#!/usr/bin/env python3
"""Keyboard teleop for the two thruster ball robot.

The robot (see ../robots/ball.xacro) carries two thrusters on the sides of the
sphere: thruster id 0 at y = +0.6 m (left) and thruster id 1 at y = -0.6 m
(right), both pushing along the body +x axis. The thruster system plugin
(libthruster.so) takes a rotor speed in rad/s on

    /<name>/thrusters/id_0/input      (left)
    /<name>/thrusters/id_1/input      (right)

and converts it to thrust = rotorConstant * |w| * w, so a positive command
pushes the robot forward. Those topics are bridged to ROS 2 by the spawn launch
file as std_msgs/Float64.

Key mapping (w/a/s/d or the arrow keys, space stops, q quits): every key event
adds one step (step_rpm, default 100 rpm) to the signed rotor speed of the
involved thrusters, so the robot keeps its throttle after the key is released
and the opposite key undoes a press exactly:

    forward (w/up)       both thrusters += step
    back (s/down)        both thrusters -= step
    left (a/left)        left -= step, right += step
    right (d/right)      left += step, right -= step
    space                both thrusters reset to 0

A key never sets an absolute throttle: presses add up, in any order, and the
speed of each thruster is clamped at +-max_rpm. The terminal sends auto-repeat
events while a key is held, so holding a key keeps adding steps until the clamp
is reached, while a single tap gives a single step.
"""

import math
import os
import select
import signal
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64

# Full throttle of one thruster, in revolutions per minute.
MAX_RPM = 1500.0
# Rotor speed added by one key event (and removed by the opposite key), rpm.
STEP_RPM = 100.0

RPM_TO_RAD_PER_S = 2.0 * math.pi / 60.0


def rpm_to_rad_per_s(rpm: float) -> float:
    """Converts a rotor speed from rpm into the rad/s expected by the plugin."""
    return rpm * RPM_TO_RAD_PER_S


class RpmState:
    """Signed rotor speed of both thrusters, changed one step at a time.

    Every key event shifts the speeds by step_rpm; the opposite key shifts them
    back. Nothing is reset when a key is released, so the robot keeps whatever
    throttle the presses have added up to. Speeds are clamped to +-max_rpm.
    """

    # (left, right) direction of the step taken by every key.
    DELTAS = {
        'forward': (1, 1),
        'back': (-1, -1),
        'left': (-1, 1),
        'right': (1, -1),
    }

    def __init__(self, max_rpm: float = MAX_RPM, step_rpm: float = STEP_RPM):
        self._max_rpm = max_rpm
        self._step_rpm = step_rpm
        self._left_rpm = 0.0
        self._right_rpm = 0.0

    @property
    def left_rpm(self) -> float:
        return self._left_rpm

    @property
    def right_rpm(self) -> float:
        return self._right_rpm

    def press(self, name: str):
        """Adds one step of the named key to the rotor speeds."""
        left_delta, right_delta = self.DELTAS[name]
        self._left_rpm = self._clamp(
            self._left_rpm + left_delta * self._step_rpm)
        self._right_rpm = self._clamp(
            self._right_rpm + right_delta * self._step_rpm)

    def reset(self):
        """Stops both thrusters (the stop key)."""
        self._left_rpm = 0.0
        self._right_rpm = 0.0

    def _clamp(self, rpm: float) -> float:
        return max(-self._max_rpm, min(self._max_rpm, rpm))


class BallTeleop(Node):
    """Publishes the accumulated rotor speeds of a ball robot."""

    FORWARD_KEYS = ('w', 'W', '\x1b[A')
    BACK_KEYS = ('s', 'S', '\x1b[B')
    LEFT_KEYS = ('a', 'A', '\x1b[D')
    RIGHT_KEYS = ('d', 'D', '\x1b[C')
    QUIT_KEYS = ('q', 'Q', '\x03')
    STOP_KEY = ' '

    def __init__(self):
        super().__init__('ball_teleop')

        self.declare_parameter('name', 'ball')
        self.declare_parameter('max_rpm', MAX_RPM)
        self.declare_parameter('step_rpm', STEP_RPM)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('left_thruster_id', 0)
        self.declare_parameter('right_thruster_id', 1)

        self._name = self.get_parameter('name').value
        self._max_rpm = float(self.get_parameter('max_rpm').value)
        self._step_rpm = float(self.get_parameter('step_rpm').value)
        self._left_id = int(self.get_parameter('left_thruster_id').value)
        self._right_id = int(self.get_parameter('right_thruster_id').value)

        # Every key event steps the rotor speeds; nothing is zeroed on release.
        self._rpm_state = RpmState(self._max_rpm, self._step_rpm)

        self._left_publisher = self.create_publisher(
            Float64, self._topic(self._left_id), 10)
        self._right_publisher = self.create_publisher(
            Float64, self._topic(self._right_id), 10)

        self._buffer = ''
        self._quit = False
        self._last_command = None

        # Keyboard input needs a terminal; without one the node stays idle
        # instead of failing, so it can be started from a launch file.
        self._tty = sys.stdin.isatty()
        self._saved_termios = None
        if self._tty:
            self._fd = sys.stdin.fileno()
            self._saved_termios = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
            self._print_help()
        else:
            self.get_logger().warning(
                'stdin is not a terminal, keyboard input is disabled')

        rate_hz = float(self.get_parameter('rate_hz').value)
        self._timer = self.create_timer(1.0 / max(rate_hz, 1.0), self._on_timer)

    def request_quit(self):
        """Asks the node to stop (quit key or a termination signal)."""
        self._quit = True

    def quit_requested(self) -> bool:
        return self._quit

    def _topic(self, thruster_id: int) -> str:
        return '/%s/thrusters/id_%d/input' % (self._name, thruster_id)

    def _print_help(self):
        print('ball teleop: w/s drive forward/back, a/d turn left/right, '
              'space stops, q quits')
        print('  every press adds %.0f rpm to the involved thrusters; holding '
              'a key keeps adding' % self._step_rpm)
        print('  one thruster is clamped at +-%.0f rpm and keeps its speed '
              'when the keys are released' % self._max_rpm)
        sys.stdout.flush()

    # -- keyboard -----------------------------------------------------------

    def _read_keys(self):
        """Reads whatever is available on stdin; never blocks."""
        if not self._tty:
            return
        while select.select([sys.stdin], [], [], 0)[0]:
            chunk = os.read(self._fd, 32)
            if not chunk:
                return
            self._buffer += chunk.decode(errors='ignore')
            self._drain_buffer()

    def _drain_buffer(self):
        while self._buffer:
            if self._buffer[0] == '\x1b':
                if len(self._buffer) < 3:
                    return  # wait for the rest of the escape sequence
                token, self._buffer = self._buffer[:3], self._buffer[3:]
            else:
                token, self._buffer = self._buffer[0], self._buffer[1:]
            self._handle_token(token)

    def _handle_token(self, token: str):
        if token in self.FORWARD_KEYS:
            self._rpm_state.press('forward')
        elif token in self.BACK_KEYS:
            self._rpm_state.press('back')
        elif token in self.LEFT_KEYS:
            self._rpm_state.press('left')
        elif token in self.RIGHT_KEYS:
            self._rpm_state.press('right')
        elif token == self.STOP_KEY:
            self._rpm_state.reset()
        elif token in self.QUIT_KEYS:
            self._quit = True

    # -- publishing ---------------------------------------------------------

    def _on_timer(self):
        self._read_keys()
        if self._quit:
            return  # main() sends the final zero command and shuts down

        rpm_left = self._rpm_state.left_rpm
        rpm_right = self._rpm_state.right_rpm
        self.publish(rpm_left, rpm_right)

        if (rpm_left, rpm_right) != self._last_command:
            self._last_command = (rpm_left, rpm_right)
            self.get_logger().info(
                'left %.0f rpm (%.1f rad/s), right %.0f rpm (%.1f rad/s)'
                % (rpm_left, rpm_to_rad_per_s(rpm_left),
                   rpm_right, rpm_to_rad_per_s(rpm_right)))

    def publish(self, left_rpm: float, right_rpm: float):
        left = Float64()
        left.data = rpm_to_rad_per_s(left_rpm)
        right = Float64()
        right.data = rpm_to_rad_per_s(right_rpm)
        self._left_publisher.publish(left)
        self._right_publisher.publish(right)

    def stop(self):
        """Sends a zero command to both thrusters."""
        self.publish(0.0, 0.0)

    def restore_terminal(self):
        if self._saved_termios is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved_termios)
            self._saved_termios = None


def _spin(node):
    """Spins the node in a worker thread."""
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


def main(args=None):
    # The termination signals are handled here instead of by rclpy: rclpy would
    # kill the context first, and then the final zero command could not be sent
    # any more. The node is spun in a thread so that this thread stays free to
    # run the signal handlers.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = BallTeleop()
    signal.signal(signal.SIGINT, lambda signum, frame: node.request_quit())
    signal.signal(signal.SIGTERM, lambda signum, frame: node.request_quit())

    spinner = threading.Thread(target=_spin, args=(node,), daemon=True)
    spinner.start()
    try:
        while rclpy.ok() and not node.quit_requested():
            time.sleep(0.1)
    finally:
        # Always leave the thrusters stopped and the terminal usable.
        print('ball teleop: stopping the thrusters', flush=True)
        node.restore_terminal()
        if rclpy.ok():
            node.stop()
            node.destroy_node()
            rclpy.shutdown()
        spinner.join(timeout=2.0)
        print('ball teleop: stopped', flush=True)


if __name__ == '__main__':
    main()
