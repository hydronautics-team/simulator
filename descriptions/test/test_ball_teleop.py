#!/usr/bin/env python3
# Copyright (c) 2026 buoyancy_test authors.
#
# Unit tests for the ball keyboard teleop: the rpm accumulator and the
# consistency between the teleop throttle and the thruster limits configured in
# the robot description.

import math
import os
import re
import sys

import pytest
import rclpy

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.abspath(os.path.join(HERE, '..', 'scripts'))
XACRO = os.path.abspath(
    os.path.join(HERE, '..', 'robots', 'ball.xacro'))
sys.path.insert(0, SCRIPTS)

import ball_teleop as teleop  # noqa: E402  (needs the sys.path entry above)


# -- rpm accumulator ---------------------------------------------------------

# Expectations follow the module defaults, so changing the step or the throttle
# in ball_teleop.py does not require touching the tests below.
STEP = teleop.STEP_RPM
MAX = teleop.MAX_RPM


def test_starts_stopped():
    state = teleop.RpmState()
    assert (state.left_rpm, state.right_rpm) == (0.0, 0.0)


def test_forward_adds_and_back_removes_speed():
    state = teleop.RpmState()
    state.press('forward')
    assert (state.left_rpm, state.right_rpm) == (STEP, STEP)
    state.press('forward')
    assert (state.left_rpm, state.right_rpm) == (2 * STEP, 2 * STEP)

    state.press('back')
    assert (state.left_rpm, state.right_rpm) == (STEP, STEP)
    state.press('back')
    assert (state.left_rpm, state.right_rpm) == (0.0, 0.0)
    # pressing on past zero reverses both thrusters
    state.press('back')
    assert (state.left_rpm, state.right_rpm) == (-STEP, -STEP)


def test_left_and_right_turn_the_thrusters_against_each_other():
    state = teleop.RpmState()
    state.press('left')
    assert (state.left_rpm, state.right_rpm) == (-STEP, STEP)

    # the opposite key undoes the turn exactly ...
    state.press('right')
    assert (state.left_rpm, state.right_rpm) == (0.0, 0.0)
    # ... and keeps turning the other way
    state.press('right')
    assert (state.left_rpm, state.right_rpm) == (STEP, -STEP)


def test_presses_add_up_in_any_order():
    state = teleop.RpmState()
    for name in ('forward', 'left', 'forward', 'right'):
        state.press(name)
    assert (state.left_rpm, state.right_rpm) == (2 * STEP, 2 * STEP)

    for name in ('back', 'right', 'back', 'left'):
        state.press(name)
    assert (state.left_rpm, state.right_rpm) == (0.0, 0.0)


def test_speeds_are_clamped_to_the_configured_throttle():
    state = teleop.RpmState()
    for _ in range(100):
        state.press('forward')
    assert (state.left_rpm, state.right_rpm) == (MAX, MAX)

    state.press('back')
    assert (state.left_rpm, state.right_rpm) == (MAX - STEP, MAX - STEP)


def test_reset_stops_both_thrusters():
    state = teleop.RpmState()
    state.press('forward')
    state.press('left')
    state.reset()
    assert (state.left_rpm, state.right_rpm) == (0.0, 0.0)


def test_parameters_change_the_step_and_the_clamp():
    state = teleop.RpmState(max_rpm=1000.0, step_rpm=250.0)
    state.press('forward')
    assert (state.left_rpm, state.right_rpm) == (250.0, 250.0)
    for _ in range(10):
        state.press('forward')
    assert (state.left_rpm, state.right_rpm) == (1000.0, 1000.0)


def test_rpm_to_rad_per_s():
    assert teleop.rpm_to_rad_per_s(0.0) == 0.0
    assert teleop.rpm_to_rad_per_s(60.0) == pytest.approx(2.0 * math.pi)
    assert teleop.rpm_to_rad_per_s(teleop.MAX_RPM) == pytest.approx(
        teleop.MAX_RPM * 2.0 * math.pi / 60.0)


def test_node_starts_and_can_be_stopped():
    """Smoke test: constructing the node must not fail (parameter mismatch,
    missing publishers, ...) even without a terminal."""
    rclpy.init()
    try:
        node = teleop.BallTeleop()
        node.stop()
        node.restore_terminal()
        node.destroy_node()
    finally:
        rclpy.shutdown()


# -- robot description consistency -------------------------------------------

def _xacro_property(name):
    with open(XACRO) as handle:
        text = handle.read()
    match = re.search(
        r'<xacro:property\s+name="%s"\s+value="\$\{([^}]+)\}"' % name, text)
    if not match:
        match = re.search(
            r'<xacro:property\s+name="%s"\s+value="([^"]+)"' % name, text)
    assert match, 'property %s not found in %s' % (name, XACRO)
    expression = match.group(1)
    # The values used here are plain numbers, never expressions with names.
    return float(expression)


def test_full_throttle_is_not_clipped_by_the_thruster_limits():
    max_rad_per_s = teleop.rpm_to_rad_per_s(teleop.MAX_RPM)
    clamp_max = _xacro_property('thruster_clamp_max')
    thrust_max = _xacro_property('thruster_thrust_max')
    rotor_constant = _xacro_property('thruster_rotor_constant')

    assert max_rad_per_s <= clamp_max, (
        'teleop asks for %.1f rad/s but the thruster clamps at %.1f rad/s'
        % (max_rad_per_s, clamp_max))

    thrust = rotor_constant * max_rad_per_s * max_rad_per_s
    assert thrust <= thrust_max, (
        'full throttle produces %.1f N but the thruster is limited to %.1f N'
        % (thrust, thrust_max))
