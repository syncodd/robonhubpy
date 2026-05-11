"""Python API for Robonhub drone."""

import os
import time
from queue import Queue, Empty
from typing import Optional, Callable, List, Dict, Any, Union, Sequence
import logging
import errno
from threading import Thread, Timer
from pymavlink import mavutil
from pymavlink.quaternion import Quaternion
from pymavlink.dialects.v20 import common as mavlink

logger = logging.getLogger('robonhub')
if not logger.hasHandlers():
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter('%(name)s: %(message)s'))
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)

class Robonhub:
    connected: bool = False
    mode: str = ''
    armed: bool = False
    landed: bool = False
    attitude: List[float]
    attitude_euler: List[float]  # roll, pitch, yaw
    rates: List[float]
    channels: List[int]
    motors: List[float]
    acc: List[float]
    gyro: List[float]

    system_id: int
    messages: Dict[str, Dict[str, Any]]  # MAVLink messages storage
    values: Dict[Union[str, int], Union[float, List[float]]]  # named values

    _connection_timeout = 3
    _print_buffer: str = ''
    # Matches firmware flight_mode_t in shared_state.h:
    # 0 = MODE_STABILIZE, 1 = MODE_ALT_HOLD, 2 = MODE_GUIDED.
    _modes = ['STABILIZE', 'ALT_HOLD', 'GUIDED']

    def __init__(self, system_id: int=1, wait_connection: bool=True):
        if not (0 <= system_id < 256):
            raise ValueError('system_id must be in range [0, 255]')
        self._setup_mavlink()
        self.system_id = system_id
        self._init_state()
        try:
            # Direct connection
            logger.debug('Listening on port 14550')
            self.connection: mavutil.mavfile = mavutil.mavlink_connection('udpin:0.0.0.0:14551', source_system=255)  # type: ignore
        except OSError as e:
            if e.errno != errno.EADDRINUSE:
                raise
            # Port busy - using proxy
            logger.debug('Listening on port 14555 (proxy)')
            self.connection: mavutil.mavfile = mavutil.mavlink_connection('udpin:0.0.0.0:14555', source_system=254)  # type: ignore
        self.connection.target_system = system_id
        self.mavlink: mavlink.MAVLink = self.connection.mav
        self._event_listeners: Dict[str, List[Callable[..., Any]]] = {}
        self._disconnected_timer = Timer(0, self._disconnected)
        self._reader_thread = Thread(target=self._read_mavlink, daemon=True)
        self._reader_thread.start()
        self._heartbeat_thread = Thread(target=self._send_heartbeat, daemon=True)
        self._heartbeat_thread.start()
        if wait_connection:
            self.wait('mavlink.HEARTBEAT')
            time.sleep(0.2) # give some time to receive initial state

    def _init_state(self):
        self.attitude = [1, 0, 0, 0]
        self.attitude_euler = [0, 0, 0]
        self.rates = [0, 0, 0]
        self.channels = [0, 0, 0, 0, 0, 0, 0, 0]
        self.motors = [0, 0, 0, 0]
        self.acc = [0, 0, 0]
        self.gyro = [0, 0, 0]
        # Telemetry from firmware (mavlink_handler.c):
        #   ALTITUDE @ 25 Hz   → altitude_relative (m)
        #   BATTERY_STATUS @ 1 Hz → voltages[0] (mV), battery_remaining (%)
        self.altitude: float = 0.0
        self.battery_voltage: float = 0.0    # volts
        self.battery_remaining: int = -1     # %, -1 = unknown
        self.messages = {}
        self.values = {}

    def on(self, event: str, callback: Callable):
        event = event.lower()
        if event not in self._event_listeners:
            self._event_listeners[event] = []
        self._event_listeners[event].append(callback)

    def off(self, event_or_callback: Union[str, Callable]):
        if isinstance(event_or_callback, str):
            event = event_or_callback.lower()
            if event in self._event_listeners:
                del self._event_listeners[event]
        else:
            for event in self._event_listeners:
                if event_or_callback in self._event_listeners[event]:
                    self._event_listeners[event].remove(event_or_callback)

    def _trigger(self, event: str, *args):
        event = event.lower()
        for callback in self._event_listeners.get(event, []):
            try:
                callback(*args)
            except Exception as e:
                logger.error(f'Error in event listener for event {event}: {e}')

    def wait(self, event: str, value: Union[Any, Callable[..., bool]] = lambda *args: True, timeout=None) -> Any:
        """Wait for an event"""
        event = event.lower()
        q = Queue()
        def callback(*args):
            if len(args) == 0:
                result = None
            elif len(args) == 1:
                result = args[0]
            else:
                result = args
            if callable(value) and value(*args):
                q.put_nowait(result)
            elif value == result:
                q.put_nowait(result)
        self.on(event, callback)
        try:
            return q.get(timeout=timeout)
        except Empty:
            raise TimeoutError
        finally:
            self.off(callback)

    @staticmethod
    def _setup_mavlink():
        # otherwise it will use MAVLink 1.0 until connected
        os.environ['MAVLINK20'] = '1'
        mavutil.set_dialect('common')

    def _read_mavlink(self):
        while True:
            try:
                msg: Optional[mavlink.MAVLink_message] = self.connection.recv_match(blocking=True)
                if msg is None or msg.get_srcSystem() != self.system_id:
                    continue
                self._connected()
                msg_dict = msg.to_dict()
                msg_dict['_timestamp'] = time.time()  # add timestamp
                self.messages[msg.get_type()] = msg_dict
                self._trigger('mavlink', msg)
                self._trigger(f'mavlink.{msg.get_type()}', msg)  # trigger mavlink.<message_type>
                self._trigger(f'mavlink.{msg.get_msgId()}', msg)  # trigger mavlink.<message_id>
                self._handle_mavlink_message(msg)

            except Exception as e:
                logger.error(f'Error reading MAVLink message: {e}')

    def _handle_mavlink_message(self, msg: mavlink.MAVLink_message):
        if isinstance(msg, mavlink.MAVLink_heartbeat_message):
            self.mode = self._modes[msg.custom_mode] if msg.custom_mode < len(self._modes) else f'UNKNOWN({msg.custom_mode})'
            self.armed = msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED != 0
            self._trigger('mode', self.mode)
            self._trigger('armed', self.armed)

        if isinstance(msg, mavlink.MAVLink_extended_sys_state_message):
            self.landed = msg.landed_state == mavlink.MAV_LANDED_STATE_ON_GROUND
            self._trigger('landed', self.landed)

        if isinstance(msg, mavlink.MAVLink_attitude_quaternion_message):
            self.attitude = self._mavlink_to_flu([msg.q1, msg.q2, msg.q3, msg.q4])
            self.rates = self._mavlink_to_flu([msg.rollspeed, msg.pitchspeed, msg.yawspeed])
            self.attitude_euler = list(Quaternion(self.attitude).euler)  # type: ignore
            self._trigger('attitude', self.attitude)
            self._trigger('attitude_euler', self.attitude_euler)

        if isinstance(msg, mavlink.MAVLink_rc_channels_raw_message):
            self.channels = [msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
                             msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw]
            self._trigger('channels', self.channels)

        if isinstance(msg, mavlink.MAVLink_actuator_control_target_message):
            self.motors = msg.controls[:4]  # type: ignore
            self._trigger('motors', self.motors)

        # TODO: to be removed: the old way of passing motor outputs
        if isinstance(msg, mavlink.MAVLink_actuator_output_status_message):
            self.motors = msg.actuator[:4]  # type: ignore
            self._trigger('motors', self.motors)

        if isinstance(msg, mavlink.MAVLink_scaled_imu_message):
            self.acc = self._mavlink_to_flu([msg.xacc / 1000, msg.yacc / 1000, msg.zacc / 1000])
            self.gyro = self._mavlink_to_flu([msg.xgyro / 1000, msg.ygyro / 1000, msg.zgyro / 1000])
            self._trigger('acc', self.acc)
            self._trigger('gyro', self.gyro)

        if isinstance(msg, mavlink.MAVLink_altitude_message):
            self.altitude = float(msg.altitude_relative)
            self._trigger('altitude', self.altitude)

        if isinstance(msg, mavlink.MAVLink_battery_status_message):
            v = msg.voltages[0] if msg.voltages else 0
            # 0xFFFF means "unknown" per the MAVLink spec; treat as 0.
            self.battery_voltage = (v / 1000.0) if v != 0xFFFF else 0.0
            self.battery_remaining = int(msg.battery_remaining)
            self._trigger('battery', self.battery_voltage, self.battery_remaining)

        if isinstance(msg, mavlink.MAVLink_serial_control_message):
            # new chunk of data
            text = bytes(msg.data)[:msg.count].decode('utf-8', errors='ignore')
            logger.debug(f'Console: {repr(text)}')
            self._trigger('print', text)
            self._print_buffer += text
            if msg.flags & mavlink.SERIAL_CONTROL_FLAG_MULTI == 0:
                # last chunk
                self._trigger('print_full', self._print_buffer)
                self._print_buffer = ''

        if isinstance(msg, mavlink.MAVLink_statustext_message):
            logger.info(f'Robonhub #{msg.get_srcSystem()}: {msg.text}')
            self._trigger('status', msg.text)

        if isinstance(msg, (mavlink.MAVLink_named_value_float_message, mavlink.MAVLink_named_value_int_message)):
            self.values[msg.name] = msg.value
            self._trigger('value', msg.name, msg.value)
            self._trigger(f'value.{msg.name}', msg.value)

        if isinstance(msg, mavlink.MAVLink_debug_message):
            self.values[msg.ind] = msg.value
            self._trigger('value', msg.ind, msg.value)
            self._trigger(f'value.{msg.ind}', msg.value)

        if isinstance(msg, mavlink.MAVLink_debug_vect_message):
            self.values[msg.name] = [msg.x, msg.y, msg.z]
            self._trigger('value', msg.name, self.values[msg.name])
            self._trigger(f'value.{msg.name}', self.values[msg.name])

        if isinstance(msg, mavlink.MAVLink_debug_float_array_message):
            self.values[msg.name] = list(msg.data)
            self._trigger('value', msg.name, self.values[msg.name])
            self._trigger(f'value.{msg.name}', self.values[msg.name])

    def _send_heartbeat(self):
        while True:
            self.mavlink.heartbeat_send(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            time.sleep(1)

    @staticmethod
    def _mavlink_to_flu(v: List[float]) -> List[float]:
        if len(v) == 3:  # vector
            return [v[0], -v[1], -v[2]]
        elif len(v) == 4:  # quaternion
            return [v[0], v[1], -v[2], -v[3]]
        else:
            raise ValueError(f'List must have 3 (vector) or 4 (quaternion) elements')

    @staticmethod
    def _flu_to_mavlink(v: List[float]) -> List[float]:
        return Robonhub._mavlink_to_flu(v)

    def _command_send(self, command: int, params: Sequence[float]):
        if len(params) != 7:
            raise ValueError('Command must have 7 parameters')
        for attempt in range(3):
            try:
                logger.debug(f'Send command {command} with params {params} (attempt #{attempt + 1})')
                self.mavlink.command_long_send(self.system_id, 0, command, 0, *params)  # type: ignore
                self.wait('mavlink.COMMAND_ACK', value=lambda msg: msg.command == command and msg.result == mavlink.MAV_RESULT_ACCEPTED, timeout=0.1)
                return
            except TimeoutError:
                continue
        raise RuntimeError(f'Failed to send command {command} after 3 attempts')

    def _connected(self):
        # Reset disconnection timer
        self._disconnected_timer.cancel()
        self._disconnected_timer = Timer(self._connection_timeout, self._disconnected)
        self._disconnected_timer.start()

        if not self.connected:
            logger.info('Connection is established')
            self.connected = True
            self._trigger('connected')

    def _disconnected(self):
        logger.info('Connection is lost')
        self.connected = False
        self._trigger('disconnected')

    def get_param(self, name: str) -> float:
        if len(name.encode('ascii')) > 16:
            raise ValueError('Parameter name must be 16 characters or less')
        for attempt in range(3):
            try:
                logger.debug(f'Get param {name} (attempt #{attempt + 1})')
                self.mavlink.param_request_read_send(self.system_id, 0, name.encode('ascii'), -1)
                msg: mavlink.MAVLink_param_value_message = \
                    self.wait('mavlink.PARAM_VALUE', value=lambda msg: msg.param_id == name, timeout=0.1)
                return msg.param_value
            except TimeoutError:
                continue
        raise RuntimeError(f'Failed to get parameter {name} after 3 attempts')

    def set_param(self, name: str, value: float):
        if len(name.encode('ascii')) > 16:
            raise ValueError('Parameter name must be 16 characters or less')
        for attempt in range(3):
            try:
                logger.debug(f'Set param {name} to {value} (attempt #{attempt + 1})')
                self.mavlink.param_set_send(self.system_id, 0, name.encode('ascii'), value, mavlink.MAV_PARAM_TYPE_REAL32)
                self.wait('mavlink.PARAM_VALUE', value=lambda msg: msg.param_id == name, timeout=0.1)
                return
            except TimeoutError:
                # on timeout try again
                continue
        raise RuntimeError(f'Failed to set parameter {name} to {value} after 3 attempts')

    def set_mode(self, mode: Union[str, int]):
        if isinstance(mode, str):
            mode = self._modes.index(mode.upper())
        self._command_send(mavlink.MAV_CMD_DO_SET_MODE, (0, mode, 0, 0, 0, 0, 0))

    def set_armed(self, armed: bool):
        self._command_send(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, (1 if armed else 0, 0, 0, 0, 0, 0, 0))

    def arm(self):
        """Arm the motors via MAV_CMD_COMPONENT_ARM_DISARM.

        Firmware (mavlink_handler.c handle_command_long) rejects the arm
        request if `pilot.throttle > 0.05`, so push throttle to zero first
        and give MANUAL_CONTROL a moment to land before sending the command.
        """
        self.set_controls(0, 0, 0, 0)
        time.sleep(0.05)
        self.set_armed(True)

    def disarm(self):
        """Disarm the motors via MAV_CMD_COMPONENT_ARM_DISARM."""
        self.set_controls(0, 0, 0, 0)
        time.sleep(0.05)
        self.set_armed(False)

    def get_attitude(self) -> List[float]:
        return self.attitude

    def get_attitude_euler(self) -> List[float]:
        return self.attitude_euler

    def get_roll(self) -> float:
        return self.attitude_euler[0]

    def get_pitch(self) -> float:
        return self.attitude_euler[1]

    def get_yaw(self) -> float:
        return self.attitude_euler[2]

    def get_altitude(self) -> float:
        return self.altitude

    def get_battery_voltage(self) -> float:
        return self.battery_voltage

    def get_battery_remaining(self) -> int:
        return self.battery_remaining

    def get_acc(self) -> List[float]:
        return self.acc

    def get_gyro(self) -> List[float]:
        return self.gyro

    def get_channels(self) -> List[int]:
        return self.channels

    def get_mode(self) -> str:
        return self.mode

    def is_connected(self, timeout: float = 0.5) -> bool:
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.connected:
                return True
            time.sleep(0.01)
        return False

    def wait_until_connected(self, timeout: Optional[float] = None):
        """Bloğa kadar bağlanmayı bekler (Wait until connected)"""
        logger.info('Waiting for connection...')
        start_time = time.time()
        while True:
            if self.connected:
                logger.info('Connected!')
                return True
            if isinstance(timeout, (int, float)):
                if time.time() - start_time > timeout:
                    raise TimeoutError('Connection timeout')
            time.sleep(0.1)

    # ──────────────────────────────────────────────────────────────────
    # GUIDED-mode commands — wrap the firmware's MAVLink command IDs.
    # Direction codes (USER_1) match firmware mavlink_handler.c:
    #   0 = front, 1 = back, 2 = right, 3 = left
    # Yaw sign (USER_4): + = CCW per FLU, - = CW.
    # ──────────────────────────────────────────────────────────────────
    _MOVE_DIRS = {
        'front': 0, 'forward': 0, '+x': 0,
        'back':  1, 'backward': 1, '-x': 1,
        'right': 2, '-y': 2,
        'left':  3, '+y': 3,
    }

    def move(self, direction: str, duration: float, tilt_rad: float = 0.0):
        """Body-frame timed move via firmware GUIDED MAV_CMD_USER_1.

        direction: 'front'|'back'|'left'|'right' (also accepts
                   forward/backward and FLU axis names).
        duration : seconds (firmware clamps 0.05..10).
        tilt_rad : optional tilt magnitude (rad). 0 = use firmware
                   default `gd_move_tilt`. Clamped < TILT_MAX.

        Drone must be in GUIDED HOVER/IDLE for the firmware to accept
        the command. Returns immediately — drone executes MOVE then
        auto-transitions to BRAKE then HOVER per the state machine.
        """
        d = direction.lower()
        if d not in self._MOVE_DIRS:
            raise ValueError(f"direction must be one of {sorted(set(self._MOVE_DIRS))}")
        dir_code = self._MOVE_DIRS[d]
        if not (0.05 <= duration <= 10.0):
            raise ValueError("duration must be in [0.05, 10] seconds")
        self._command_send(mavlink.MAV_CMD_USER_1,
                           (float(dir_code), float(duration), float(tilt_rad),
                            0, 0, 0, 0))

    def hold(self, duration: float = 0.0):
        """Switch to GUIDED so pos_control_z holds current altitude.

        If duration > 0, blocks for that many seconds before returning.
        Use after manual control to park the drone autonomously.
        """
        self.set_mode('GUIDED')
        if duration > 0:
            time.sleep(duration)

    def takeoff(self, altitude: float = 1.0):
        """GUIDED auto-takeoff to `altitude` (m) via MAV_CMD_NAV_TAKEOFF.

        Firmware runs the SPOOL → re-anchor → TAKEOFF → HOVER state
        machine. Caller must arm first. Returns immediately; the climb
        completes asynchronously (~7 s for 1 m at TARGET_SLEW=0.15 m/s).
        """
        if not (0.1 <= altitude <= 10.0):
            raise ValueError("altitude must be in [0.1, 10] m")
        self._command_send(mavlink.MAV_CMD_NAV_TAKEOFF,
                           (0, 0, 0, 0, 0, 0, float(altitude)))

    def land(self):
        """GUIDED auto-land via MAV_CMD_NAV_LAND.

        Firmware descends via TGT_HOLD_ALT(0) then runs the TOUCHDOWN
        thrust ramp and disarms. Returns immediately.
        """
        self._command_send(mavlink.MAV_CMD_NAV_LAND, (0, 0, 0, 0, 0, 0, 0))

    def set_altitude(self, altitude: float):
        """Set absolute hover altitude target via MAV_CMD_USER_3.

        Drone must already be in GUIDED HOVER/TAKEOFF. Replaces the
        current target without re-spooling. Clamped server-side to
        [0.1, fs_alt_max].
        """
        if not (0.1 <= altitude <= 10.0):
            raise ValueError("altitude must be in [0.1, 10] m")
        self._command_send(mavlink.MAV_CMD_USER_3,
                           (float(altitude), 0, 0, 0, 0, 0, 0))

    def change_altitude(self, delta_cm: float):
        """Adjust hover altitude by `delta_cm` (signed) via MAV_CMD_USER_2.

        Positive = up, negative = down. Sum clamped to [0.1, fs_alt_max].
        """
        self._command_send(mavlink.MAV_CMD_USER_2,
                           (float(delta_cm), 0, 0, 0, 0, 0, 0))

    def yaw(self, direction: str, degrees: float, rate_dps: float = 60.0):
        """Rotate yaw by `degrees` via MAV_CMD_USER_4.

        direction : 'cw' (clockwise per pilot view) or 'ccw'.
        degrees   : magnitude in degrees (e.g. 5).
        rate_dps  : optional rotation rate (default 60 deg/s, clamped
                    10..200 server-side).

        Firmware computes duration = |degrees|/rate, applies a yaw-rate
        override for that interval, then returns control to heading
        hold. Sign convention: CCW is positive in FLU, so CCW maps to
        +deg, CW to -deg over MAVLink.
        """
        d = direction.lower()
        if d not in ('cw', 'ccw'):
            raise ValueError("direction must be 'cw' or 'ccw'")
        if degrees < 0:
            raise ValueError("degrees must be non-negative; use direction for sign")
        signed = (+degrees) if d == 'ccw' else (-degrees)
        self._command_send(mavlink.MAV_CMD_USER_4,
                           (float(signed), float(rate_dps), 0, 0, 0, 0, 0))

    def set_position(self, position: List[float], yaw: Optional[float] = None, wait: bool = False, tolerance: float = 0.1):
        raise NotImplementedError('Position control is not implemented yet')

    def set_velocity(self, velocity: List[float], yaw: Optional[float] = None):
        raise NotImplementedError('Velocity control is not implemented yet')

    def set_attitude(self, attitude: List[float], thrust: float):
        if len(attitude) == 3:
            attitude = Quaternion([attitude[0], attitude[1], attitude[2]]).q  # type: ignore
        elif len(attitude) != 4:
            raise ValueError('Attitude must be [roll, pitch, yaw] or [w, x, y, z] quaternion')
        if not (0 <= thrust <= 1):
            raise ValueError('Thrust must be in range [0, 1]')
        attitude = self._flu_to_mavlink(attitude)
        for _ in range(2): # duplicate to ensure delivery
            self.mavlink.set_attitude_target_send(0, self.system_id, 0, 0,
                [attitude[0], attitude[1], attitude[2], attitude[3]],
                0, 0, 0, thrust)

    def set_rates(self, rates: List[float], thrust: float):
        if len(rates) != 3:
            raise ValueError('Rates must be [roll_rate, pitch_rate, yaw_rate]')
        if not (0 <= thrust <= 1):
            raise ValueError('Thrust must be in range [0, 1]')
        rates = self._flu_to_mavlink(rates)
        for _ in range(2):  # duplicate to ensure delivery
            self.mavlink.set_attitude_target_send(0, self.system_id, 0,
                mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE,
                [1, 0, 0, 0],
                rates[0], rates[1], rates[2], thrust)

    def set_motors(self, motors: List[float]):
        if len(motors) != 4:
            raise ValueError('motors must have 4 values')
        if not all(0 <= m <= 1 for m in motors):
            raise ValueError('motors must be in range [0, 1]')
        for _ in range(2):  # duplicate to ensure delivery
            self.mavlink.set_actuator_control_target_send(int(time.time() * 1000000), 0, self.system_id, 0, motors + [0] * 4)  # type: ignore

    def set_controls(self, roll: float, pitch: float, yaw: float, throttle: float):
        """Send pilot's controls. Warning: not intended for automatic control"""
        if not (-1 <= roll <= 1 and -1 <= pitch <= 1 and -1 <= yaw <= 1):
            raise ValueError('roll, pitch, yaw must be in range [-1, 1]')
        if not 0 <= throttle <= 1:
            raise ValueError('throttle must be in range [0, 1]')
        self.mavlink.manual_control_send(self.system_id, int(pitch * 1000), int(roll * 1000), int(throttle * 1000), int(yaw * 1000), 0)  # type: ignore

    def led(self, idx: int, state: Union[str, bool, int]):
        """Control a user LED. Firmware exposes `led <1-5> <on|off>`.

        1-based to match the published robonhubpy library convention.
        Physical placement (verified empirically by lighting them in
        numeric order on the drone):
            1 = motor FR,  2 = motor RR,  3 = motor RL,  4 = motor FL,
            5 = top.
        """
        if idx not in (1, 2, 3, 4, 5):
            raise ValueError('LED index must be 1..5')
        if isinstance(state, str):
            state = state.lower()
            if state not in ('on', 'off'):
                raise ValueError("LED state must be 'on' or 'off'")
        else:
            state = 'on' if state else 'off'
        self.cli(f'led {idx} {state}', wait_response=False)

    def motor(self, idx: int, percentage: float):
        """Spin a single motor for testing.

        Wraps the firmware `motor <id 0-3> <duty 0-511>` CLI which sets
        `motor_test_mode = true` and writes the PWM directly. Use 0% to
        stop. Disarm or call `arm()` again to re-engage normal flight.
        """
        if idx not in (0, 1, 2, 3):
            raise ValueError('Motor index must be 0..3')
        if not (0 <= percentage <= 100):
            raise ValueError('Motor percentage must be in range [0, 100]')
        duty = int(round(percentage / 100.0 * 511))
        self.cli(f'motor {idx} {duty}', wait_response=False)

    def cli(self, cmd: str, wait_response: bool = True, timeout: float = 1.0) -> str:
        """Run a CLI command on the firmware over MAVLink SERIAL_CONTROL.

        The firmware (mavlink_handler.c handle_serial_control) does NOT
        echo the command back; it only sends the command's printf output
        followed by a `robonhub> ` prompt. This client used to require an
        echo prefix to match — that timed out on every call. Now we
        accept any output and strip the trailing prompt.
        """
        cmd = cmd.strip()
        if cmd == 'reboot' or cmd.startswith('calibrate_accel'):
            # reboot has no response; calibrate_accel (no-arg) blocks on
            # stdin and never returns over MAVLink — fire-and-forget.
            wait_response = False

        raw = (cmd + '\n').encode('utf-8')
        if len(raw) > 70:
            raise ValueError(f'Command is too long: {len(raw)} > 70')
        cmd_bytes = raw.ljust(70, b'\0')
        sent_len = len(raw)

        # Drop any half-collected output from a previous command so we
        # don't return stale text.
        self._print_buffer = ''

        self.mavlink.serial_control_send(0, 0, 0, 0, sent_len, cmd_bytes)
        if not wait_response:
            return ''

        try:
            response = self.wait('print_full', timeout=timeout)
        except TimeoutError:
            return ''
        # Strip the prompt the firmware appends after every command.
        if response.endswith('robonhub> '):
            response = response[:-len('robonhub> ')]
        return response.strip()
