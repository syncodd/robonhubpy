"""Offline validation of mock_drone.py against the real robonhubpy gates.

No GUI, no server: MockDrone runs in a thread sending straight to robonhubpy's
udpin:14551 listener — the same socket path the GUI proxy would feed. Exercises
every blocking gate (gstat bits, COMMAND_ACK, altitude watchdog) end-to-end:

    constructor-timeout (dead port) → arm → takeoff(1) → move front 2s
        → MANUAL_CONTROL abort of AUTO_MOVE + ALT_HOLD stick descent
        → yaw cw 90 → set_altitude(1.5) → unknown PARAM_SET ignored
        → POSHOLD est_v2_en gate → hold(POSHOLD) → land → DISARMED
        → land from 5 m (relative watchdog: no false abort)
        → emergency_land force-disarm when land() watchdog aborts

Exit code 0 = pass.
"""

import sys
import math
import time
import socket
import functools
import threading

from mock_drone import (MockDrone, PARAM_TABLE, MODE_STABILIZE, MODE_ALT_HOLD,
                        MODE_AUTO_LAND, MODE_AUTO_MOVE, MODE_POSHOLD)
from robonhub import Robonhub, CommandRejected

FAILURES = []


def check(name, cond, detail=''):
    status = 'PASS' if cond else 'FAIL'
    print(f'[{status}] {name}' + (f' — {detail}' if detail else ''), flush=True)
    if not cond:
        FAILURES.append(name)


def wait_for(cond, timeout, poll=0.02):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if cond():
            return True
        time.sleep(poll)
    return cond()


def test_constructor_timeout():
    """No drone anywhere: constructor must raise, not hang forever.

    A plain (non-SO_REUSEADDR) socket holds 14551 so Robonhub falls back to
    14555 where nothing sends — the heartbeat wait must time out with a
    ConnectionError naming the ports.
    """
    blocker = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    blocker.bind(('0.0.0.0', 14551))
    try:
        t0 = time.time()
        try:
            Robonhub(wait_connection=True, connect_timeout=1.0)
            check('constructor timeout raises', False, 'no exception')
        except ConnectionError as e:
            elapsed = time.time() - t0
            check('constructor timeout raises ConnectionError', True,
                  f'{elapsed:.1f}s')
            check('error names tried + fallback ports',
                  '14555' in str(e) and '14551' in str(e), str(e)[:120])
            check('constructor timeout bounded', elapsed < 5.0,
                  f'{elapsed:.1f}s')
    finally:
        blocker.close()


def force_airborne(rh, drone, alt):
    """Teleport the mock to a hover at `alt` (armed, ALT_HOLD, sticks mid)."""
    rh.set_controls(0, 0, 0, 0.5)
    wait_for(lambda: drone.throttle_stick > 0.4, 2.0)
    drone.target_alt = alt
    drone.alt = alt
    drone.vz = 0.0
    drone.mode = MODE_ALT_HOLD
    drone.armed = True
    return wait_for(lambda: rh.gstat['armed'] is True
                    and abs(rh.altitude - alt) < 0.3, 3.0)


def main():
    test_constructor_timeout()

    drone = MockDrone(target='127.0.0.1:14551', rate=50.0, watchdog=30.0)
    t = threading.Thread(target=drone.run, daemon=True)
    t.start()

    t0 = time.time()
    rh = Robonhub(wait_connection=True)
    check('connect', rh.connected, f'{time.time() - t0:.1f}s')

    rh.arm()
    check('arm', rh.gstat['armed'] is True, f"gstat={rh.gstat}")

    rh.takeoff(1.0)
    check('takeoff → ALT_HOLD', rh.gstat['mode'] == 'ALT_HOLD',
          f"mode={rh.gstat['mode']} alt={rh.altitude:.2f}")
    check('takeoff altitude ≈ 1.0', abs(rh.altitude - 1.0) < 0.2,
          f'alt={rh.altitude:.2f}')

    pos_before = list(drone.pos)
    rh.move('front', 2.0)
    check('move front → back to ALT_HOLD',
          rh.gstat['mode'] == 'ALT_HOLD' and rh.gstat['move_phase'] == 0,
          f"mode={rh.gstat['mode']} phase={rh.gstat['move_phase']}")
    check('move front displaced +x', drone.pos[0] > pos_before[0] + 0.05,
          f'x {pos_before[0]:.2f} → {drone.pos[0]:.2f}')

    # MANUAL_CONTROL pilot-override: a stick deflection must abort AUTO_MOVE
    # (mirrors firmware mode_auto_move.cpp check_pilot_abort).
    rh.move('front', 2.0, wait=False)
    check('AUTO_MOVE leg started (wait=False)',
          wait_for(lambda: drone.mode == MODE_AUTO_MOVE, 1.0),
          f'mode={drone.mode}')
    t_abort = time.time()
    rh.set_controls(0, 0, 0, 0.0)  # throttle to 0 = pilot override
    aborted = wait_for(lambda: drone.mode == MODE_ALT_HOLD
                       and drone.move is None, 1.5)
    check('MANUAL_CONTROL throttle aborts AUTO_MOVE', aborted,
          f'mode={drone.mode} in {time.time() - t_abort:.2f}s')
    check('abort happened before leg end', time.time() - t_abort < 2.0,
          f'{time.time() - t_abort:.2f}s')
    # ALT_HOLD with low throttle stick must descend (firmware climb stick).
    alt_before = drone.alt
    time.sleep(0.8)
    check('ALT_HOLD low throttle descends', drone.alt < alt_before - 0.05,
          f'alt {alt_before:.2f} → {drone.alt:.2f}')
    rh.set_controls(0, 0, 0, 0.5)  # re-center, hold here
    time.sleep(0.3)

    yaw_before = drone.yaw
    rh.yaw('cw', 90)
    check('yaw done', not rh.gstat['yaw_active'])
    check('yaw rotated −90° (cw)',
          abs(math.degrees(drone.yaw - yaw_before) + 90) < 2,
          f'Δyaw={math.degrees(drone.yaw - yaw_before):.1f}°')

    rh.set_altitude(1.5)
    check('set_altitude settled', rh.gstat['alt_settled'],
          f'alt={rh.altitude:.2f}')
    check('altitude ≈ 1.5', abs(rh.altitude - 1.5) < 0.2,
          f'alt={rh.altitude:.2f}')

    # Unknown PARAM_SET must be ignored like firmware (no echo → set_param
    # raises after retries) and must not create a table entry.
    unknown_raised = False
    try:
        rh.set_param('no_such_param', 1.0)
    except RuntimeError:
        unknown_raised = True
    check('unknown PARAM_SET ignored (set_param raises)', unknown_raised)
    check('unknown param not created', 'no_such_param' not in PARAM_TABLE)

    # POSHOLD gate: firmware refuses POSHOLD unless the ESKF is enabled.
    rh.set_param('est_v2_en', 0.0)
    poshold_refused = False
    try:
        rh.set_mode('POSHOLD')
    except CommandRejected:
        poshold_refused = True
    check('POSHOLD refused when est_v2_en=0',
          poshold_refused and drone.mode != MODE_POSHOLD,
          f'mode={drone.mode}')
    rh.set_param('est_v2_en', 1.0)

    rh.hold(duration=0.5)
    check('hold → POSHOLD entered then held',
          rh.gstat['mode'] == 'POSHOLD', f"mode={rh.gstat['mode']}")

    t_land = time.time()
    rh.land()
    check('land → DISARMED', rh.gstat['armed'] is False,
          f'{time.time() - t_land:.1f}s')
    check('final altitude 0', drone.alt < 0.05, f'alt={drone.alt:.2f}')
    check('final mode STABILIZE', drone.mode == 0, f'mode={drone.mode}')

    # emergency_land from disarmed state must no-op cleanly
    rh.emergency_land()
    check('emergency_land no-op when disarmed', drone.armed is False)

    # Relative not-descending watchdog: a landing started ABOVE the old
    # absolute 0.40 m floor (5 m here) must complete, not false-abort —
    # even with a descend_window shorter than the full descent.
    check('teleport to 5 m hover', force_airborne(rh, drone, 5.0),
          f'alt={rh.altitude:.2f} gstat={rh.gstat}')
    t_land5 = time.time()
    rh.land(timeout=30.0, descend_window=3.0)
    check('land from 5 m → DISARMED (no false abort)',
          rh.gstat['armed'] is False, f'{time.time() - t_land5:.1f}s')
    check('land from 5 m touched down', drone.alt < 0.05 and
          drone.mode == MODE_STABILIZE,
          f'alt={drone.alt:.2f} mode={drone.mode}')

    # emergency_land force-disarm: when land()'s watchdog aborts (descent
    # stalls, e.g. ground-effect), emergency_land must NOT report done with
    # the drone still armed — it must force a disarm.
    check('teleport to 2 m hover', force_airborne(rh, drone, 2.0),
          f'alt={rh.altitude:.2f} gstat={rh.gstat}')
    orig_step = MockDrone.step
    def stuck_step(dt):
        if drone.mode == MODE_AUTO_LAND:
            drone.vz = 0.0
            return  # descent stalls: altitude pinned, stays armed
        orig_step(drone, dt)
    drone.step = stuck_step
    # shrink the watchdog window so the abort fires fast in-test
    rh.land = functools.partial(Robonhub.land, rh, descend_window=2.0)
    try:
        t_em = time.time()
        rh.emergency_land()
        forced = wait_for(lambda: drone.armed is False, 3.0)
        check('emergency_land force-disarms on watchdog abort', forced,
              f'armed={drone.armed} in {time.time() - t_em:.1f}s')
        check('watchdog abort really happened (never descended)',
              drone.alt > 1.5, f'alt={drone.alt:.2f}')
    finally:
        del drone.step
        del rh.land

    print()
    if FAILURES:
        print(f'FAILED: {len(FAILURES)} check(s): {FAILURES}', flush=True)
        return 1
    print('ALL CHECKS PASSED', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
