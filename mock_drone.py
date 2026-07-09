"""Mock drone — MAVLink emulator of the Robonhub firmware for GUI "Test in Sim".

Speaks the same MAVLink contract as firmware mavlink_handler.c (sysid 1,
compid 1, MAVLink 2) so the whole existing pipeline — GUI server UDP 14550,
robonhubpy gstat gates, 3D view telemetry — works unchanged against it.

Motion is kinematic, smooth and deterministic: ideal accel/decel ramps, clean
takeoff/land curves, exact mode transitions. It validates command SEQUENCING
and logic, not flight physics.

Usage:
    python3 mock_drone.py [--target 127.0.0.1:14550] [--rate 50] [--watchdog 10]

Exits on its own when no GCS heartbeat is heard for --watchdog seconds
(orphan protection if the parent process dies without killing us).
"""

import os
import sys
import math
import time
import argparse

os.environ['MAVLINK20'] = '1'  # must precede mavutil connection creation
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink

# Mirrors firmware flight_mode_t (shared_state.h) — same order robonhubpy uses.
MODE_STABILIZE    = 0
MODE_ALT_HOLD     = 1
MODE_AUTO_TAKEOFF = 2
MODE_AUTO_LAND    = 3
MODE_AUTO_MOVE    = 4
MODE_POSHOLD      = 5

# AUTO_MOVE phases as reported in gstat (mode_auto_move.cpp convention).
PHASE_NONE, PHASE_WARMUP, PHASE_RAMP, PHASE_HOLD, PHASE_EXIT = 0, 1, 2, 3, 4

G = 9.80665

# Kinematic limits — chosen so every robonhubpy timeout passes with margin:
# takeoff(1 m) ≈ 3 s vs timeout 7*alt+10; move leg ≈ dur+1.3 s vs 2*dur+6;
# land descends ≥0.3 m/s immediately (robonhubpy land() aborts if not clearly
# descending within 10 s).
VZ_MAX    = 0.5    # m/s climb/descend
AZ_MAX    = 1.5    # m/s² vertical accel
ALT_KP    = 1.2    # vz_cmd = clamp(ALT_KP * err, ±VZ_MAX)
TILT_TAU  = 0.15   # s, 1st-order lag on tilt commands (smooth attitude)
WARMUP_S  = 0.3
RAMP_S    = 0.4
EXIT_S    = 0.6
# alt_settled thresholds — must match firmware pos_z_alt_settled(0.12, 0.20)
SETTLE_ERR, SETTLE_VZ, SETTLE_SUSTAIN = 0.12, 0.20, 0.3

PARAM_TABLE = {
    'cal_status': 7.0,   # gyro+accel+mag calibrated → StatusPill green
    'est_v2_en':  1.0,
    'fs_alt_max': 10.0,
}


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def euler_to_quat_frd(roll, pitch, yaw):
    """ZYX euler (rad, FRD) → [w, x, y, z]."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            cy * sp * cr + sy * cp * sr,
            sy * cp * cr - cy * sp * sr]


class MockDrone:
    def __init__(self, target='127.0.0.1:14550', rate=50.0, watchdog=10.0):
        self.conn = mavutil.mavlink_connection(
            f'udpout:{target}', source_system=1, source_component=1)
        self.mav = self.conn.mav
        self.dt = 1.0 / rate
        self.watchdog = watchdog

        self.mode = MODE_STABILIZE
        self.armed = False
        # World frame: FLU (x fwd/north, y left, z up). Converted to NED/FRD
        # only at the emit boundary, mirroring how the firmware works in FLU
        # internally and publishes FRD.
        self.alt = 0.0
        self.vz = 0.0
        self.pos = [0.0, 0.0]     # world x (north), y (left/west)
        self.vel = [0.0, 0.0]
        self.yaw = 0.0            # FLU yaw, + = CCW
        self.target_alt = 0.0
        self.tilt_roll = 0.0      # firmware convention: + = bank right, deg
        self.tilt_pitch = 0.0     # + = nose-down = forward, deg
        self.cmd_roll = 0.0
        self.cmd_pitch = 0.0
        # Last MANUAL_CONTROL sticks (firmware keeps the last value too).
        self.throttle_stick = 0.0
        self.roll_stick = 0.0
        self.pitch_stick = 0.0
        self.yaw_stick = 0.0
        self.battery_v = 4.10

        self.move = None          # {'roll','pitch','dur','t'} active leg
        self.move_phase = PHASE_NONE
        self.yaw_job = None       # {'remaining' (FLU deg), 'rate' (dps)}
        self.settle_t = 0.0       # continuous time alt_settled condition held

        self.last_gcs_heartbeat = None
        self.boot = time.monotonic()
        self.running = True

    # ── helpers ──────────────────────────────────────────────────────────

    def _ms(self):
        return int((time.monotonic() - self.boot) * 1000) & 0xFFFFFFFF

    def alt_settled(self):
        return self.settle_t >= SETTLE_SUSTAIN

    def yaw_active(self):
        return self.yaw_job is not None

    def in_alt_mode(self):
        return self.mode in (MODE_ALT_HOLD, MODE_AUTO_MOVE, MODE_POSHOLD)

    # ── kinematic step (fixed dt, deterministic) ─────────────────────────

    def step(self, dt):
        # Tilt command lag → smooth visual attitude
        a = 1.0 - math.exp(-dt / TILT_TAU)
        self.tilt_roll += (self.cmd_roll - self.tilt_roll) * a
        self.tilt_pitch += (self.cmd_pitch - self.tilt_pitch) * a

        if not self.armed:
            self.vz = 0.0
            self.vel = [0.0, 0.0]
            self.cmd_roll = self.cmd_pitch = 0.0
            if self.battery_v < 4.10:
                pass  # battery does not recover
            return
        self.battery_v = max(3.50, self.battery_v - 0.0004 * dt)

        # ── pilot override (mirrors mode_auto_move.cpp check_pilot_abort):
        # significant stick deflection aborts an AUTO_MOVE leg to ALT_HOLD ──
        if self.mode == MODE_AUTO_MOVE and self.move is not None:
            if (self.throttle_stick > 0.70 or self.throttle_stick < 0.30
                    or abs(self.roll_stick) > 0.30
                    or abs(self.pitch_stick) > 0.30
                    or abs(self.yaw_stick) > 0.50):
                self.move = None
                self.move_phase = PHASE_NONE
                self.mode = MODE_ALT_HOLD
                self.cmd_roll = self.cmd_pitch = 0.0
                self.target_alt = self.alt  # firmware re-anchors at current alt

        # ── vertical: trapezoidal vz toward target ──
        err = self.target_alt - self.alt
        vz_cmd = clamp(ALT_KP * err, -VZ_MAX, VZ_MAX)
        if self.mode == MODE_ALT_HOLD:
            # firmware stick_to_climb_cmd: ALT_HOLD throttle is a climb-rate
            # stick (0.5 = hold, low = descend, high = climb, mid deadband) —
            # a script that leaves the stick at 0 descends like on HW.
            st = self.throttle_stick
            if st < 0.40:
                vz_cmd = -VZ_MAX * (0.40 - st) / 0.40
                self.target_alt = self.alt  # target tracks the stick descent
            elif st > 0.60:
                vz_cmd = VZ_MAX * (st - 0.60) / 0.40
                self.target_alt = self.alt
        if self.mode == MODE_AUTO_LAND:
            # descend decisively from the first tick (robonhubpy watchdog)
            vz_cmd = clamp(vz_cmd, -VZ_MAX, -0.3 if self.alt > 0.05 else 0.0)
        dvz = clamp(vz_cmd - self.vz, -AZ_MAX * dt, AZ_MAX * dt)
        self.vz += dvz
        self.alt += self.vz * dt
        if self.alt <= 0.0:
            self.alt, self.vz = 0.0, 0.0

        # settle tracker (matches firmware pos_z_alt_settled semantics)
        if abs(err) < SETTLE_ERR and abs(self.vz) < SETTLE_VZ:
            self.settle_t += dt
        else:
            self.settle_t = 0.0

        # ── mode state machines ──
        if self.mode == MODE_AUTO_TAKEOFF and self.alt_settled():
            self.mode = MODE_ALT_HOLD

        if self.mode == MODE_AUTO_LAND and self.alt < 0.06 and abs(self.vz) < 0.2:
            self.armed = False
            self.mode = MODE_STABILIZE
            self.alt = self.vz = 0.0
            self.vel = [0.0, 0.0]
            self.cmd_roll = self.cmd_pitch = 0.0
            self.move = None
            self.move_phase = PHASE_NONE
            self.yaw_job = None
            return

        if self.mode == MODE_AUTO_MOVE and self.move is not None:
            self.move['t'] += dt
            t = self.move['t']
            if t < WARMUP_S:
                self.move_phase = PHASE_WARMUP
                ramp = 0.0
            elif t < WARMUP_S + RAMP_S:
                self.move_phase = PHASE_RAMP
                ramp = (t - WARMUP_S) / RAMP_S
            elif t < WARMUP_S + RAMP_S + self.move['dur']:
                self.move_phase = PHASE_HOLD
                ramp = 1.0
            elif t < WARMUP_S + RAMP_S + self.move['dur'] + EXIT_S:
                self.move_phase = PHASE_EXIT
                # reverse-tilt brake: ramp back through zero to decelerate
                ramp = -((t - WARMUP_S - RAMP_S - self.move['dur']) / EXIT_S)
            else:
                # leg done → hand back to ALT_HOLD (firmware behavior)
                self.move = None
                self.move_phase = PHASE_NONE
                self.mode = MODE_ALT_HOLD
                self.cmd_roll = self.cmd_pitch = 0.0
                self.vel = [0.0, 0.0]
                ramp = 0.0
            if self.move is not None:
                self.cmd_roll = self.move['roll'] * ramp
                self.cmd_pitch = self.move['pitch'] * ramp

        # ── yaw job ──
        if self.yaw_job is not None:
            j = self.yaw_job
            step_deg = math.copysign(min(j['rate'] * dt, abs(j['remaining'])),
                                     j['remaining'])
            self.yaw += math.radians(step_deg)
            j['remaining'] -= step_deg
            if abs(j['remaining']) < 1e-6:
                self.yaw_job = None

        # ── horizontal integration from tilt (a = g·tan(tilt), body→world) ──
        a_fwd = G * math.tan(math.radians(self.tilt_pitch))   # +x body
        a_left = -G * math.tan(math.radians(self.tilt_roll))  # roll>0 = right
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        ax = c * a_fwd - s * a_left
        ay = s * a_fwd + c * a_left
        if self.move_phase in (PHASE_NONE,) and self.mode != MODE_AUTO_MOVE:
            # outside a leg: ideal hold — bleed residual velocity quickly
            self.vel[0] *= math.exp(-3.0 * dt)
            self.vel[1] *= math.exp(-3.0 * dt)
        self.vel[0] += ax * dt
        self.vel[1] += ay * dt
        self.pos[0] += self.vel[0] * dt
        self.pos[1] += self.vel[1] * dt

        # zero residual drift when a leg just ended and velocity is tiny
        if self.mode != MODE_AUTO_MOVE and math.hypot(*self.vel) < 0.02:
            self.vel = [0.0, 0.0]

    # ── command handling (gates mirror firmware mavlink_handler.c) ───────

    def handle_command_long(self, m):
        cmd = m.command
        accepted = False

        if cmd == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            arm_req = m.param1 > 0.5
            throttle_safe = (self.throttle_stick <= 0.05
                             if self.mode == MODE_STABILIZE else True)
            if arm_req and not throttle_safe:
                pass  # reject — throttle not idle in STABILIZE
            else:
                self.armed = arm_req
                if not arm_req:
                    self.mode = MODE_STABILIZE
                accepted = True

        elif cmd == mavlink.MAV_CMD_DO_SET_MODE:
            new_mode = int(m.param2)
            if new_mode == MODE_STABILIZE:
                self.mode, accepted = MODE_STABILIZE, True
            elif new_mode == MODE_ALT_HOLD:
                # firmware: accepted while disarmed, or inflight stable hover
                # (alt_est_can_engage_alt_hold_inflight: airborne, |vz|<=1.5)
                if not self.armed or (self.alt > 0.05 and abs(self.vz) <= 1.5):
                    self.mode, accepted = MODE_ALT_HOLD, True
                    if self.armed:
                        self.target_alt = self.alt
            elif new_mode == MODE_AUTO_TAKEOFF:
                # firmware alt_est_can_engage_auto_takeoff: on ground, quasi-static
                if self.alt <= 0.05 and abs(self.vz) < 0.2:
                    self.target_alt = max(self.target_alt, 1.0)
                    self.mode, accepted = MODE_AUTO_TAKEOFF, True
            elif new_mode == MODE_AUTO_LAND:
                # firmware alt_est_can_engage_auto_land: ARMED + airborne
                if self.armed and self.alt > 0.05:
                    self.mode, accepted = MODE_AUTO_LAND, True
                    self.target_alt = -0.2
            elif new_mode == MODE_POSHOLD:
                # firmware eskf_can_engage_poshold_inflight: ESKF enabled
                # (est_v2_en) + airborne above 0.30 m
                if (PARAM_TABLE.get('est_v2_en', 0.0) >= 0.5
                        and self.armed and self.alt > 0.30):
                    self.mode, accepted = MODE_POSHOLD, True

        elif cmd == mavlink.MAV_CMD_NAV_TAKEOFF:
            # firmware gate (alt_est_can_engage_auto_takeoff): on ground +
            # quasi-static; auto-arms when disarmed (deadman not modeled).
            if self.alt <= 0.05 and abs(self.vz) < 0.2:
                alt = m.param7
                if math.isnan(alt) or alt <= 0:
                    alt = 1.0
                if not self.armed:
                    self.armed = True
                self.target_alt = alt
                self.settle_t = 0.0
                self.mode = MODE_AUTO_TAKEOFF
                accepted = True

        elif cmd == mavlink.MAV_CMD_NAV_LAND:
            # firmware gate (alt_est_can_engage_auto_land): ARMED + airborne
            if self.armed and self.alt > 0.05:
                self.mode = MODE_AUTO_LAND
                self.target_alt = -0.2
                accepted = True

        elif cmd == mavlink.MAV_CMD_USER_1:  # AUTO_MOVE leg
            if self.armed and self.in_alt_mode():
                dur = m.param3 if m.param3 > 0 else 1.0
                self.move = {'roll': clamp(m.param1, -15, 15),
                             'pitch': clamp(m.param2, -15, 15),
                             'dur': dur, 't': 0.0}
                self.move_phase = PHASE_WARMUP
                self.mode = MODE_AUTO_MOVE
                accepted = True

        elif cmd in (mavlink.MAV_CMD_USER_2, mavlink.MAV_CMD_USER_3):
            if self.armed and self.in_alt_mode():
                tgt = (m.param1 if cmd == mavlink.MAV_CMD_USER_3
                       else self.target_alt + m.param1 * 0.01)
                self.target_alt = clamp(tgt, 0.1, PARAM_TABLE['fs_alt_max'])
                self.settle_t = 0.0
                accepted = True

        elif cmd == mavlink.MAV_CMD_USER_4:  # timed yaw, + = CCW (FLU)
            if self.armed and self.in_alt_mode():
                rate = m.param2 if m.param2 > 0 else 60.0
                self.yaw_job = {'remaining': m.param1,
                                'rate': clamp(rate, 10.0, 200.0)}
                accepted = True

        else:
            return  # unknown command — firmware sends no ACK path we rely on

        self.mav.command_ack_send(
            cmd, mavlink.MAV_RESULT_ACCEPTED if accepted
            else mavlink.MAV_RESULT_UNSUPPORTED)

    def handle_msg(self, msg):
        t = msg.get_type()
        if t == 'HEARTBEAT':
            if msg.get_srcSystem() != 1:  # GCS / robonhubpy / GUI server
                self.last_gcs_heartbeat = time.monotonic()
        elif t == 'COMMAND_LONG':
            if msg.target_system in (0, 1):
                self.handle_command_long(msg)
        elif t == 'MANUAL_CONTROL':
            # x=pitch, y=roll, z=throttle, r=yaw — robonhub.set_controls packing
            self.pitch_stick = clamp(msg.x / 1000.0, -1.0, 1.0)
            self.roll_stick = clamp(msg.y / 1000.0, -1.0, 1.0)
            self.throttle_stick = clamp(msg.z / 1000.0, 0.0, 1.0)
            self.yaw_stick = clamp(msg.r / 1000.0, -1.0, 1.0)
        elif t == 'PARAM_REQUEST_LIST':
            items = sorted(PARAM_TABLE.items())
            for i, (name, val) in enumerate(items):
                self.mav.param_value_send(
                    name.encode('ascii'), val,
                    mavlink.MAV_PARAM_TYPE_REAL32, len(items), i)
        elif t == 'PARAM_REQUEST_READ':
            name = msg.param_id
            if isinstance(name, bytes):
                name = name.decode('ascii', errors='ignore')
            name = name.rstrip('\x00')
            if name in PARAM_TABLE:
                self.mav.param_value_send(
                    name.encode('ascii'), PARAM_TABLE[name],
                    mavlink.MAV_PARAM_TYPE_REAL32, len(PARAM_TABLE),
                    sorted(PARAM_TABLE).index(name))
        elif t == 'PARAM_SET':
            name = msg.param_id
            if isinstance(name, bytes):
                name = name.decode('ascii', errors='ignore')
            name = name.rstrip('\x00')
            # firmware handle_param_set: unknown parameter is silently
            # ignored — no PARAM_VALUE echo, no new table entry — so a
            # typo'd set_param() times out in sim exactly like on HW.
            if name in PARAM_TABLE:
                PARAM_TABLE[name] = msg.param_value
                self.mav.param_value_send(
                    name.encode('ascii'), msg.param_value,
                    mavlink.MAV_PARAM_TYPE_REAL32, len(PARAM_TABLE), 0)

    # ── telemetry emit (rate-gated) ──────────────────────────────────────

    def send_telemetry(self, now, timers):
        ms = self._ms()

        def due(name, period):
            if now - timers.get(name, 0.0) >= period:
                timers[name] = now
                return True
            return False

        if due('hb', 0.5):  # 2 Hz
            base_mode = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            if self.armed:
                base_mode |= mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            self.mav.heartbeat_send(
                mavlink.MAV_TYPE_QUADROTOR, mavlink.MAV_AUTOPILOT_GENERIC,
                base_mode, self.mode,
                mavlink.MAV_STATE_ACTIVE if self.armed
                else mavlink.MAV_STATE_STANDBY)
            self.mav.extended_sys_state_send(
                mavlink.MAV_VTOL_STATE_UNDEFINED,
                mavlink.MAV_LANDED_STATE_IN_AIR if (self.armed and self.alt > 0.05)
                else mavlink.MAV_LANDED_STATE_ON_GROUND)

        if due('gstat', 0.1):  # 10 Hz — bit layout MUST match firmware
            busy = (self.mode in (MODE_AUTO_TAKEOFF, MODE_AUTO_MOVE,
                                  MODE_AUTO_LAND) or self.yaw_active())
            gstat = ((self.mode & 0x0F)
                     | ((1 if self.armed else 0) << 4)
                     | ((1 if self.yaw_active() else 0) << 5)
                     | ((1 if self.alt_settled() else 0) << 6)
                     | ((1 if busy else 0) << 7)
                     | ((self.move_phase & 0x07) << 8))
            self.mav.named_value_int_send(ms, b'gstat', gstat)
            self.mav.altitude_send(ms * 1000, self.alt, self.alt, self.alt,
                                   self.alt, 0.0, 0.0)
            # body specific force FRD ×1000: hover = (0, 0, −9806)
            self.mav.scaled_imu_send(ms, 0, 0, -9806, 0, 0, 0, 0, 0, 0)
            hover = 0.45 if (self.armed and self.alt > 0.02) else (
                0.15 if self.armed else 0.0)
            self.mav.actuator_output_status_send(
                ms * 1000, 4, [hover] * 4 + [0.0] * 28)

        if due('att', 0.05):  # 20 Hz
            # FRD euler: roll + = bank right (= firmware tilt_roll), pitch
            # nose-down = negative FRD pitch (firmware cmd pitch>0 = nose-down),
            # yaw FRD = −yaw FLU.
            q = euler_to_quat_frd(math.radians(self.tilt_roll),
                                  -math.radians(self.tilt_pitch),
                                  -self.yaw)
            self.mav.attitude_quaternion_send(ms, q[0], q[1], q[2], q[3],
                                              0.0, 0.0, 0.0)
            # LOCAL_POSITION_NED: N = x_flu, E = −y_flu, D = −alt
            self.mav.local_position_ned_send(
                ms, self.pos[0], -self.pos[1], -self.alt,
                self.vel[0], -self.vel[1], -self.vz)

        if due('batt', 1.0):  # 1 Hz
            mv = int(self.battery_v * 1000)
            remaining = int(clamp((self.battery_v - 3.50) / (4.10 - 3.50)
                                  * 100, 0, 100))
            self.mav.battery_status_send(
                0, mavlink.MAV_BATTERY_FUNCTION_ALL,
                mavlink.MAV_BATTERY_TYPE_LIPO, 2500,
                [mv] + [0xFFFF] * 9, -1, -1, -1, remaining)

    # ── main loop ────────────────────────────────────────────────────────

    def run(self):
        self.mav.statustext_send(mavlink.MAV_SEVERITY_INFO,
                                 b'SIM: mock drone ready')
        timers = {}
        next_tick = time.monotonic()
        while self.running:
            now = time.monotonic()
            # drain inbound
            while True:
                msg = self.conn.recv_match(blocking=False)
                if msg is None:
                    break
                self.handle_msg(msg)
            self.step(self.dt)
            self.send_telemetry(now, timers)
            # orphan watchdog: exit when GCS heartbeats stop
            if self.last_gcs_heartbeat is not None:
                if now - self.last_gcs_heartbeat > self.watchdog:
                    print('mock_drone: GCS heartbeat lost, exiting',
                          flush=True)
                    return
            elif now - self.boot > 60.0:
                print('mock_drone: no GCS ever connected, exiting', flush=True)
                return
            next_tick += self.dt
            sleep = next_tick - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_tick = time.monotonic()  # fell behind — resync


def main():
    ap = argparse.ArgumentParser(description='Robonhub mock drone (MAVLink emulator)')
    ap.add_argument('--target', default='127.0.0.1:14550',
                    help='UDP host:port to send telemetry to (GUI server)')
    ap.add_argument('--rate', type=float, default=50.0, help='sim step Hz')
    ap.add_argument('--watchdog', type=float, default=10.0,
                    help='exit after this many s without a GCS heartbeat')
    args = ap.parse_args()
    drone = MockDrone(args.target, args.rate, args.watchdog)
    print(f'mock_drone: emulating firmware on udpout:{args.target}', flush=True)
    try:
        drone.run()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
