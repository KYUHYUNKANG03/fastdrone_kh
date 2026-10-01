"""Configurable measurement models for the arena plant.

The plant state remains the source of truth.  This module deliberately sits
between the plant and the estimator: it produces timestamped packets with
noise, bias, dropout and transport latency, so a controller can be evaluated
with the same aircraft dynamics under different sensor specifications.
"""
from dataclasses import dataclass
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class Measurement:
    kind: str
    sample_time: float
    arrival_time: float
    value: np.ndarray
    sequence: int

    def __post_init__(self):
        object.__setattr__(self, "value", np.asarray(self.value, dtype=float).copy())


DEFAULT_SENSOR_PROFILE = {
    "schema": "sensor/1",
    "name": "nominal",
    "imu": {"rate_hz": 500.0, "accel_noise_density": 0.03,
            "gyro_noise_density": 0.002, "accel_bias": [0, 0, 0],
            "gyro_bias": [0, 0, 0], "accel_bias_rw": 0.0002,
            "gyro_bias_rw": 0.00002, "latency_s": 0.0,
            "dropout_prob": 0.0, "accel_clip": 100.0, "gyro_clip": 50.0},
    "gnss": {"rate_hz": 10.0, "pos_sigma": 0.8, "vel_sigma": 0.15,
             "pos_bias": [0, 0, 0], "vel_bias": [0, 0, 0],
             "latency_s": 0.12, "dropout_prob": 0.0, "outage_windows": []},
    "barometer": {"enabled": True, "rate_hz": 50.0, "sigma": 0.25,
                   "bias": 0.0, "latency_s": 0.02, "dropout_prob": 0.0},
    "magnetometer": {"enabled": True, "rate_hz": 100.0, "sigma": 0.04,
                      "world_field": [20.0, 0.0, 40.0], "bias": [0, 0, 0],
                      "latency_s": 0.01, "dropout_prob": 0.0},
    "rpm": {"enabled": True, "rate_hz": 500.0, "sigma": 8.0,
            "bias": [0, 0, 0, 0], "latency_s": 0.004, "dropout_prob": 0.0},
    "estimator": {"history_s": 2.0, "gnss_pos_sigma": 1.5,
                   "gnss_vel_sigma": 0.5, "baro_sigma": 0.6,
                   "mag_sigma": 0.15, "initial_position_sigma": 0.5,
                   "initial_velocity_sigma": 0.2, "initial_attitude_deg": 1.0,
                   "estimate_baro_bias": True, "baro_bias_tau_s": 1.0,
                   "baro_gnss_max_age_s": 0.25,
                   "initialize_baro_bias": True,
                   "preflight_baro_samples": 0},
    "rotor_observer": {"source": "telemetry", "tau_s": 0.02}
}


def load_sensor_profile(profile):
    """Load and validate a profile from a mapping or JSON path."""
    if isinstance(profile, (str, Path)):
        profile = json.loads(Path(profile).read_text(encoding="utf-8"))
    if not isinstance(profile, dict) or profile.get("schema") != "sensor/1":
        raise ValueError("sensor profile must have schema 'sensor/1'")
    merged = deepcopy(DEFAULT_SENSOR_PROFILE)
    for key, value in profile.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key].update(deepcopy(value))
        else:
            merged[key] = deepcopy(value)
    for name in ("imu", "gnss", "barometer", "magnetometer", "rpm"):
        spec = merged[name]
        if not np.isfinite(spec.get('rate_hz', 0)) or float(spec.get("rate_hz", 0)) <= 0:
            raise ValueError(f"{name}.rate_hz must be finite and positive")
        if not np.isfinite(spec.get('latency_s', 0)) or float(spec.get("latency_s", 0)) < 0:
            raise ValueError(f"{name}.latency_s must be finite and nonnegative")
        if not 0 <= float(spec.get("dropout_prob", 0)) < 1:
            raise ValueError(f"{name}.dropout_prob must be in [0,1)")
        for key in ('sigma', 'pos_sigma', 'vel_sigma', 'accel_noise_density',
                    'gyro_noise_density', 'accel_bias_rw', 'gyro_bias_rw'):
            if key in spec and (not np.isfinite(spec[key]) or spec[key] < 0):
                raise ValueError(f'{name}.{key} must be finite and nonnegative')
    for key in ('accel_clip', 'gyro_clip'):
        if not np.isfinite(merged['imu'][key]) or merged['imu'][key] <= 0:
            raise ValueError(f'imu.{key} must be finite and positive')
    for tone in merged['imu'].get('gyro_vibration', []):
        if (not np.isfinite(tone['frequency_hz']) or tone['frequency_hz'] <= 0
                or tone['frequency_hz'] >= merged['imu']['rate_hz']/2.):
            raise ValueError('gyro vibration frequency must be positive and below IMU Nyquist')
        for key in ('amplitude_rad_s', 'phase_rad'):
            value = np.asarray(tone.get(key, [0., 0., 0.]))
            if value.shape != (3,) or not np.all(np.isfinite(value)):
                raise ValueError(f'gyro vibration {key} must be a finite 3-vector')
    rotor = merged['rotor_observer']
    if rotor.get('source') not in ('telemetry', 'predictor', 'telemetry_predictor'):
        raise ValueError('rotor_observer.source must be telemetry, predictor or telemetry_predictor')
    duration = rotor.get('prehistory_s', 0.)
    if not np.isfinite(duration) or duration < 0:
        raise ValueError('rotor_observer.prehistory_s must be finite and nonnegative')
    if duration and (not merged['rpm']['enabled'] or rotor['source'] == 'predictor'):
        raise ValueError('rotor prehistory requires an enabled telemetry observer')
    if rotor['source'] == 'telemetry_predictor':
        for key in ('motor_tau_s', 'history_s', 'max_age_s'):
            if key not in rotor or not np.isfinite(rotor[key]) or rotor[key] <= 0:
                raise ValueError(f'rotor_observer.{key} must be explicitly finite and positive')
        if rotor['history_s'] < max(rotor['max_age_s'], merged['rpm']['latency_s']):
            raise ValueError('rotor history must cover nominal latency and maximum ready age')
        initial = np.asarray(rotor.get('initial_rpm', [0.]*4))
        if initial.shape != (4,) or not np.all(np.isfinite(initial)):
            raise ValueError('initial_rpm must be a finite 4-vector (mechanical rad/s)')
    if not isinstance(merged['estimator'].get('trace_updates', False), bool):
        raise ValueError('estimator.trace_updates must be boolean')
    if merged['estimator'].get('preflight_baro_rng', 'shared') not in ('shared', 'independent'):
        raise ValueError('estimator.preflight_baro_rng must be shared or independent')
    e = merged['estimator']
    for key in ('gnss_pos_sigma', 'gnss_vel_sigma', 'baro_sigma', 'mag_sigma', 'history_s'):
        if not np.isfinite(e[key]) or e[key] <= 0:
            raise ValueError(f'estimator.{key} must be finite and positive')
    count = e['preflight_baro_samples']
    if isinstance(count, bool) or not isinstance(count, int) or count < 0:
        raise ValueError('preflight_baro_samples must be a nonnegative integer')
    # D5 navigation prehistory: optional and checked only when present. No
    # default is merged in, so profiles without it keep their exact hash.
    if 'navigation_prehistory_s' in e:
        value = e['navigation_prehistory_s']
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not np.isfinite(value) or value < 0):
            raise ValueError('estimator.navigation_prehistory_s must be finite and nonnegative')
    if e.get('kind','legacy15') not in ('legacy15','joint_baro'):
        raise ValueError('estimator.kind must be legacy15 or joint_baro')
    for key in ('initial_baro_bias_sigma_m','preflight_reference_sigma_m','preflight_baro_sigma_m',
                'baro_bias_rw_density_m_sqrt_s','accel_bias_rw_density','gyro_bias_rw_density',
                'accel_noise_density','gyro_noise_density'):
        if key in e and (not np.isfinite(e[key]) or float(e[key]) < 0):
            raise ValueError(f'estimator.{key} must be finite and nonnegative')
    for key in ('initial_position_error_m','initial_velocity_error_m_s','initial_attitude_error_deg'):
        if key in e and (np.asarray(e[key]).shape != (3,) or not np.all(np.isfinite(e[key]))):
            raise ValueError(f'estimator.{key} must be a finite 3-vector')
    if not np.isfinite(e.get('preflight_reference_error_m',0.)):
        raise ValueError('preflight_reference_error_m must be finite')
    for key,value in e.get('nis_gates',{}).items():
        if key not in ('gnss','barometer','magnetometer') or not np.isfinite(value) or value <= 0:
            raise ValueError('nis_gates must contain positive finite sensor thresholds')
    if not np.isfinite(merged['barometer'].get('bias_rate_m_s',0.)):
        raise ValueError('barometer.bias_rate_m_s must be finite')
    for sensor in ('gnss','barometer'):
        for window in merged[sensor].get('outlier_windows',[]):
            start,end = window['start_s'],window['end_s']
            if not np.isfinite(start) or not np.isfinite(end) or start < 0 or end <= start:
                raise ValueError('outlier window requires finite 0 <= start_s < end_s')
            for key,width in ((('pos_offset_m',3),('vel_offset_m_s',3)) if sensor=='gnss' else (('height_offset_m',1),)):
                value = np.asarray(window.get(key,[0.]*width if width>1 else 0.))
                if value.shape != ((width,) if width>1 else ()) or not np.all(np.isfinite(value)):
                    raise ValueError(f'invalid outlier {key}')
    return merged


class SensorSuite:
    """Generate packets at the configured rates from a true 17-state sample."""

    def __init__(self, profile, dt, seed=0, gravity=9.81):
        self.profile = load_sensor_profile(profile)
        self.dt = float(dt)
        self.gravity = float(gravity)
        names = ("imu", "gnss", "barometer", "magnetometer", "rpm")
        # Appending a child preserves the existing five flight streams exactly.
        # Children are keyed by index, so the eighth (navigation prehistory)
        # leaves the first seven unchanged.
        streams = np.random.SeedSequence(int(seed)).spawn(len(names)+3)
        self._rng = {name: np.random.default_rng(stream) for name, stream in zip(names, streams)}
        self._preflight_rng = np.random.default_rng(streams[len(names)])
        self._rotor_prehistory_rng = np.random.default_rng(streams[len(names)+1])
        self._navigation_prehistory_rng = {name: np.random.default_rng(stream) for name, stream in zip(
            ('imu', 'imu_bias', 'gnss', 'barometer', 'magnetometer'), streams[len(names)+2].spawn(5))}
        self._bias = {"accel": np.asarray(self.profile["imu"].get("accel_bias", [0]*3), dtype=float),
                      "gyro": np.asarray(self.profile["imu"].get("gyro_bias", [0]*3), dtype=float)}
        self._next = {name: 0 for name in ("imu", "gnss", "barometer", "magnetometer", "rpm")}
        self._seq = 0
        self._last_imu_time = None

    def rotor_prehistory(self, rotor_truth, duration):
        """Sample a declared steady-trim rotor history before flight time zero.

        This models past measurements, not full aircraft preflight dynamics.
        A separate RNG preserves all five flight streams and baro calibration.
        """
        spec, rng = self.profile['rpm'], self._rotor_prehistory_rng
        period = 1./spec['rate_hz']
        for index in range(int(np.ceil(duration/period))):
            t = -duration+index*period
            if t >= 0.:
                break
            dropped = rng.random() < spec['dropout_prob'] or any(
                start <= t < end for start, end in spec.get('outage_windows', []))
            value = np.asarray(rotor_truth)+np.asarray(spec['bias'])+spec['sigma']*rng.normal(size=4)
            yield t, ([] if dropped else [self._packet('rpm', t, value, spec)])

    def navigation_prehistory(self, x_now, xdot_now, duration):
        """Sample declared steady-trim navigation packets before flight time zero (D5).

        Truth over [-duration, 0): p(t) = p0 + v0*t; attitude, rates, velocity
        and the specific force from xdot_now stay at their time-zero values,
        the same steady-trim assumption as the rotor prehistory. The error
        models repeat sample() term by term. Each sensor continues its flight
        nominal grid k/rate to negative k on the plant-step ticks.

        Flight sampling is untouched: the eighth seeded child (one grandchild
        per error source) supplies every draw, and negative sequence numbers
        leave _seq, and so every flight packet, exactly as without a prehistory.
        The IMU bias random walk is generated backward from the configured bias,
        so it ends where the first flight IMU sample starts.
        Yields (tick time, packets sampled at that tick) in time order.
        """
        ticks = int(round(duration/self.dt))
        if duration <= 0 or abs(duration/self.dt-ticks) > 1e-7:
            raise ValueError('navigation prehistory must be a positive multiple of the plant step')
        x = np.asarray(x_now, dtype=float)
        xd = np.asarray(xdot_now, dtype=float)
        R = Rotation.from_quat(x[6:10]).as_matrix()
        rng = self._navigation_prehistory_rng
        times = [-(ticks-j)*self.dt for j in range(ticks)]
        names = ['imu', 'gnss'] + [name for name in ('barometer', 'magnetometer')
                                   if self.profile[name].get('enabled', False)]
        due = {}
        for name in names:
            # Same tolerance as _due: sample at the first tick on or after -m/rate.
            interval = 1.0/float(self.profile[name]['rate_hz'])
            m = int(np.floor(duration/interval+1e-9))
            due[name] = set()
            for t in times:
                if m >= 1 and t+1e-9 >= -m*interval:
                    due[name].add(t)
                    m -= 1

        def dropped(spec, stream, t):
            return stream.random() < float(spec.get('dropout_prob', 0.0)) or any(
                len(window) == 2 and float(window[0]) <= t < float(window[1])
                for window in spec.get('outage_windows', []))

        imu = self.profile['imu']
        accel_bias = {}
        gyro_bias = {}
        later = 0.0
        ba = np.asarray(imu.get('accel_bias', [0]*3), dtype=float).copy()
        bg = np.asarray(imu.get('gyro_bias', [0]*3), dtype=float).copy()
        for t in sorted(due['imu'], reverse=True):
            step = np.sqrt(later-t)
            ba = ba-float(imu.get('accel_bias_rw', 0.0))*step*rng['imu_bias'].normal(size=3)
            bg = bg-float(imu.get('gyro_bias_rw', 0.0))*step*rng['imu_bias'].normal(size=3)
            accel_bias[t], gyro_bias[t] = ba, bg
            later = t
        g_w = np.array([0., 0., -self.gravity])
        f_b = R.T @ (xd[3:6]-g_w)
        period = 1.0/float(imu['rate_hz'])
        a_sigma = float(imu.get('accel_noise_density', 0.0))/np.sqrt(period)
        g_sigma = float(imu.get('gyro_noise_density', 0.0))/np.sqrt(period)
        gnss, baro, mag = self.profile['gnss'], self.profile['barometer'], self.profile['magnetometer']
        raw = []
        for t in times:
            position = x[0:3]+x[3:6]*t
            if t in due['imu']:
                lost = dropped(imu, rng['imu'], t)
                a = f_b+accel_bias[t]+a_sigma*rng['imu'].normal(size=3)
                w = x[10:13]+gyro_bias[t]+g_sigma*rng['imu'].normal(size=3)
                for tone in imu.get('gyro_vibration', []):
                    w += np.asarray(tone['amplitude_rad_s'])*np.sin(
                        2.*np.pi*float(tone['frequency_hz'])*t + np.asarray(tone.get('phase_rad', [0.]*3)))
                a = np.clip(a, -float(imu.get('accel_clip', 1e9)), float(imu.get('accel_clip', 1e9)))
                w = np.clip(w, -float(imu.get('gyro_clip', 1e9)), float(imu.get('gyro_clip', 1e9)))
                if not lost:
                    raw.append((t, 'imu', np.r_[a, w], imu))
            if t in due['gnss']:
                lost = dropped(gnss, rng['gnss'], t)
                value = np.r_[position+np.asarray(gnss.get('pos_bias', [0]*3))
                              + float(gnss.get('pos_sigma', 0))*rng['gnss'].normal(size=3),
                              x[3:6]+np.asarray(gnss.get('vel_bias', [0]*3))
                              + float(gnss.get('vel_sigma', 0))*rng['gnss'].normal(size=3)]
                for window in gnss.get('outlier_windows', []):
                    if window['start_s'] <= t < window['end_s']:
                        value += np.r_[window.get('pos_offset_m', [0.]*3), window.get('vel_offset_m_s', [0.]*3)]
                if not lost:
                    raw.append((t, 'gnss', value, gnss))
            if t in due.get('barometer', ()):
                lost = dropped(baro, rng['barometer'], t)
                value = [position[2]+float(baro.get('bias', 0.0))+float(baro.get('sigma', 0))*rng['barometer'].normal()]
                if baro.get('bias_rate_m_s', 0.): value[0] += float(baro['bias_rate_m_s'])*t
                for window in baro.get('outlier_windows', []):
                    if window['start_s'] <= t < window['end_s']:
                        value[0] += float(window.get('height_offset_m', 0.))
                if not lost:
                    raw.append((t, 'barometer', value, baro))
            if t in due.get('magnetometer', ()):
                lost = dropped(mag, rng['magnetometer'], t)
                field = np.asarray(mag.get('world_field', [20., 0., 40.]), dtype=float)
                value = (R.T @ field+np.asarray(mag.get('bias', [0]*3))
                         + float(mag.get('sigma', 0))*rng['magnetometer'].normal(size=3))
                if not lost:
                    raw.append((t, 'magnetometer', value, mag))
        packets = {t: [] for t in times}
        for index, (t, kind, value, spec) in enumerate(raw):
            packets[t].append(Measurement(kind, float(t), float(t+spec.get('latency_s', 0.0)),
                                          value, index-len(raw)))
        for t in times:
            yield t, packets[t]

    def _due(self, name, t):
        rate = float(self.profile[name].get("rate_hz", 0.0))
        interval = 1.0/rate
        k = self._next[name]
        if t + 1e-9 < k*interval:
            return False
        self._next[name] = k + 1
        return True

    def _drop(self, name, spec, sample_time):
        if self._rng[name].random() < float(spec.get("dropout_prob", 0.0)):
            return True
        for window in spec.get("outage_windows", []):
            if len(window) == 2 and float(window[0]) <= sample_time < float(window[1]):
                return True
        return False

    def _packet(self, kind, sample_time, value, spec):
        self._seq += 1
        return Measurement(kind, float(sample_time),
                           float(sample_time + spec.get("latency_s", 0.0)), value, self._seq)

    def preflight_barometer_bias(self, altitude, samples, reference_altitude=None):
        """Average stationary samples; optionally preserve the in-flight noise stream.

        The legacy shared stream remains the default. Independent mode uses the
        sixth seeded stream, enabling paired flight-noise comparisons across
        averaging counts. Samples assume a constant, known reference altitude.
        """
        spec = self.profile["barometer"]
        count = int(samples)
        if not spec.get("enabled", False) or count <= 0:
            return None
        rng = (self._preflight_rng if self.profile['estimator'].get('preflight_baro_rng', 'shared')
               == 'independent' else self._rng['barometer'])
        readings = (float(altitude) + float(spec.get("bias", 0.0))
                    + float(spec.get("sigma", 0.0))*rng.normal(size=count))
        reference = altitude if reference_altitude is None else reference_altitude
        return float(np.mean(readings)-float(reference))

    def sample(self, t, x_true, xdot_true=None):
        """Return packets sampled at *t*; packets become available at arrival_time."""
        t = float(t)
        x = np.asarray(x_true, dtype=float)
        xd = np.zeros(17) if xdot_true is None else np.asarray(xdot_true, dtype=float)
        R = Rotation.from_quat(x[6:10]).as_matrix()
        out = []
        imu = self.profile["imu"]
        if self._due("imu", t):
            dropped = self._drop("imu", imu, t)
            rng = self._rng["imu"]
            g_w = np.array([0., 0., -self.gravity])
            f_b = R.T @ (xd[3:6] - g_w)
            # Noise density follows the sensor's nominal averaging period;
            # physical bias drift follows elapsed sample time, including drops.
            period = 1.0/float(imu["rate_hz"])
            elapsed = 0.0 if self._last_imu_time is None else t-self._last_imu_time
            self._last_imu_time = t
            self._bias["accel"] += float(imu.get("accel_bias_rw", 0.0))*np.sqrt(elapsed)*rng.normal(size=3)
            self._bias["gyro"] += float(imu.get("gyro_bias_rw", 0.0))*np.sqrt(elapsed)*rng.normal(size=3)
            a_sigma = float(imu.get("accel_noise_density", 0.0))/np.sqrt(period)
            g_sigma = float(imu.get("gyro_noise_density", 0.0))/np.sqrt(period)
            a = f_b + self._bias["accel"] + a_sigma*rng.normal(size=3)
            w = x[10:13] + self._bias["gyro"] + g_sigma*rng.normal(size=3)
            # A resolved measurement-error tone, not a physical body torque.
            # No additional random draws: other errors remain paired by seed.
            for tone in imu.get('gyro_vibration', []):
                w += np.asarray(tone['amplitude_rad_s'])*np.sin(
                    2.*np.pi*float(tone['frequency_hz'])*t + np.asarray(tone.get('phase_rad', [0.]*3)))
            a = np.clip(a, -float(imu.get("accel_clip", 1e9)), float(imu.get("accel_clip", 1e9)))
            w = np.clip(w, -float(imu.get("gyro_clip", 1e9)), float(imu.get("gyro_clip", 1e9)))
            if not dropped:
                out.append(self._packet("imu", t, np.r_[a, w], imu))
        gnss = self.profile["gnss"]
        if self._due("gnss", t):
            dropped = self._drop("gnss", gnss, t)
            rng = self._rng["gnss"]
            value = np.r_[x[0:3] + np.asarray(gnss.get("pos_bias", [0]*3)) + float(gnss.get("pos_sigma", 0))*rng.normal(size=3),
                          x[3:6] + np.asarray(gnss.get("vel_bias", [0]*3)) + float(gnss.get("vel_sigma", 0))*rng.normal(size=3)]
            for window in gnss.get('outlier_windows',[]):
                if window['start_s'] <= t < window['end_s']:
                    value += np.r_[window.get('pos_offset_m',[0.]*3),window.get('vel_offset_m_s',[0.]*3)]
            if not dropped:
                out.append(self._packet("gnss", t, value, gnss))
        baro = self.profile["barometer"]
        if baro.get("enabled", False) and self._due("barometer", t):
            dropped = self._drop("barometer", baro, t)
            value = [x[2] + float(baro.get("bias", 0.0)) + float(baro.get("sigma", 0))*self._rng["barometer"].normal()]
            if baro.get('bias_rate_m_s',0.): value[0] += float(baro['bias_rate_m_s'])*t
            for window in baro.get('outlier_windows',[]):
                if window['start_s'] <= t < window['end_s']:
                    value[0] += float(window.get('height_offset_m',0.))
            if not dropped:
                out.append(self._packet("barometer", t, value, baro))
        mag = self.profile["magnetometer"]
        if mag.get("enabled", False) and self._due("magnetometer", t):
            dropped = self._drop("magnetometer", mag, t)
            field = np.asarray(mag.get("world_field", [20., 0., 40.]), dtype=float)
            value = R.T @ field + np.asarray(mag.get("bias", [0]*3)) + float(mag.get("sigma", 0))*self._rng["magnetometer"].normal(size=3)
            if not dropped:
                out.append(self._packet("magnetometer", t, value, mag))
        rpm = self.profile["rpm"]
        if rpm.get("enabled", False) and self._due("rpm", t):
            dropped = self._drop("rpm", rpm, t)
            value = x[13:17] + np.asarray(rpm.get("bias", [0]*4)) + float(rpm.get("sigma", 0))*self._rng["rpm"].normal(size=4)
            if not dropped:
                out.append(self._packet("rpm", t, value, rpm))
        return out
