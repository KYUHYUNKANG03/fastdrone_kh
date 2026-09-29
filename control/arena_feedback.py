"""Closed-loop sensor-to-state adapter used by ``validation_suite``."""
import numpy as np
from scipy.spatial.transform import Rotation

from control.arena_estimator import NavigationFilter
from control.arena_sensors import SensorSuite, load_sensor_profile


class RotorObserver:
    def __init__(self, x0, profile, dt):
        spec = load_sensor_profile(profile)
        self.source = spec["rotor_observer"].get("source", "telemetry")
        self.dt = float(dt)
        self.tau_s = max(float(spec["rotor_observer"].get("tau_s", .02)), 1e-6)
        # Rotor speed is a measured/observed state.  Do not seed it from the
        # plant truth unless a profile explicitly requests that initial value.
        self.n = np.asarray(spec["rotor_observer"].get("initial_rpm", [0., 0., 0., 0.]), dtype=float).copy()
        self.last_command = self.n.copy()
        self._pending = []
        self._last_sample_time = None
        self._time = 0.0

    @property
    def ready(self):
        """Availability, not accuracy: telemetry must have arrived at least once."""
        return self.source == "predictor" or self._last_sample_time is not None

    @property
    def sample_time(self):
        return self._last_sample_time

    def update(self, command, measurements, *, now):
        """Use only arrived RPM packets, or propagate the applied command.

        The command supplied at an endpoint acted during the preceding interval.
        Predictor mode deliberately ignores all telemetry.
        """
        command = np.asarray(command, dtype=float)
        elapsed = float(now) - self._time
        if elapsed < -1e-9:
            raise ValueError("rotor observer time must be nondecreasing")
        self._time = float(now)
        if self.source == "predictor":
            self.n += (1.0-np.exp(-max(0., elapsed)/self.tau_s))*(command-self.n)
        else:
            self._pending.extend(m for m in measurements if m.kind == "rpm")
            arrived = [m for m in self._pending if m.arrival_time <= now+1e-9]
            self._pending = [m for m in self._pending if m.arrival_time > now+1e-9]
            for m in sorted(arrived, key=lambda m: (m.sample_time, m.sequence)):
                if self._last_sample_time is None:
                    # The first available reading initializes the measured state.
                    self.n = m.value.copy()
                elif m.sample_time <= self._last_sample_time:
                    continue
                else:
                    alpha = 1.0-np.exp(-(m.sample_time-self._last_sample_time)/self.tau_s)
                    self.n += alpha*(m.value-self.n)
                self._last_sample_time = m.sample_time
        self.last_command = command.copy()
        return self.n.copy()


class ArenaSensorFeedback:
    """Owns sensor generation, navigation filtering, and rotor observation."""
    def __init__(self, x0, profile, dt, seed=0):
        self.profile = load_sensor_profile(profile)
        self.dt = float(dt)
        self.sensors = SensorSuite(self.profile, dt, seed=seed)
        e = self.profile['estimator']
        initial = np.asarray(x0,dtype=float).copy()
        for key,section in (('initial_position_error_m',slice(0,3)),('initial_velocity_error_m_s',slice(3,6))):
            if key in e: initial[section] += np.asarray(e[key])
        if 'initial_attitude_error_deg' in e:
            initial[6:10] = (Rotation.from_quat(initial[6:10]) *
                            Rotation.from_rotvec(np.deg2rad(e['initial_attitude_error_deg']))).as_quat()
        if e.get('kind','legacy15') == 'joint_baro':
            from control.arena_joint_estimator import JointNavigationFilter
            self.filter = JointNavigationFilter(initial,self.profile)
        else:
            self.filter = NavigationFilter(initial, self.profile)
        preflight_samples = int(self.profile["estimator"].get("preflight_baro_samples", 0))
        reference = x0[2]+float(e['preflight_reference_error_m']) if 'preflight_reference_error_m' in e else None
        preflight_bias = self.sensors.preflight_barometer_bias(x0[2], preflight_samples,reference)
        if preflight_bias is not None:
            self.filter.set_initial_baro_bias(preflight_bias)
        self.rotors = RotorObserver(x0, self.profile, dt)
        self._state = np.asarray(x0, dtype=float).copy()
        self._last_packets = []

    @property
    def state(self):
        return self._state.copy()

    @property
    def diagnostics(self):
        return dict(self.filter.diagnostics)

    def initial_packets(self, t, x_true, xdot_true):
        packets = self.sensors.sample(t, x_true, xdot_true)
        self._last_packets = packets
        self.filter.advance(t, [m for m in packets if m.kind != "rpm"])
        self._state[:13] = self.filter.state[:13]
        self._state[13:17] = self.rotors.update(self.rotors.n, packets, now=t)
        return packets

    def step(self, t, x_true, xdot_true, command):
        packets = self.sensors.sample(t, x_true, xdot_true)
        self.filter.advance(t, [m for m in packets if m.kind != "rpm"])
        self._state[:13] = self.filter.state[:13]
        self._state[13:17] = self.rotors.update(command, packets, now=t)
        self._last_packets = packets
        return self._state.copy()
