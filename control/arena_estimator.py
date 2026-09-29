"""Fixed-lag 15-state error-state navigation filter for the arena.

Every accepted packet is retained in a bounded event log. When a delayed
measurement arrives, the filter restores the state immediately before that
packet's timestamp, applies the correction there, and replays later events.
"""
import numpy as np
from bisect import bisect_left
from scipy.spatial.transform import Rotation

from control.arena_sensors import load_sensor_profile


def _skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]], dtype=float)


def _qmul(q, r):
    x1, y1, z1, w1 = q
    x2, y2, z2, w2 = r
    return np.array([w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2,
                     w1*w2 - x1*x2 - y1*y2 - z1*z2])


class NavigationFilter:
    """Timestamp-aware fixed-lag ESKF used by one arena trial."""

    def __init__(self, x0_17, profile, gravity=9.81):
        self.profile = load_sensor_profile(profile)
        x0 = np.asarray(x0_17, dtype=float)
        self.gravity = float(gravity)
        self.p = x0[0:3].copy(); self.v = x0[3:6].copy(); self.q = x0[6:10].copy()
        self.q /= max(np.linalg.norm(self.q), 1e-12)
        self.ba = np.zeros(3); self.bg = np.zeros(3)
        e = self.profile["estimator"]
        self.P = np.diag(np.r_[np.full(3, float(e.get("initial_position_sigma", .5))**2),
                               np.full(3, float(e.get("initial_velocity_sigma", .2))**2),
                               np.full(3, np.deg2rad(float(e.get("initial_attitude_deg", 1.0)))**2),
                               np.full(6, .05**2)])
        self.time = None; self.last_imu = None; self._now = 0.0
        self.baro_bias = 0.0; self._latest_baro = None; self._baro_bias_time = None
        self._pending = []; self._events = []; self._seen = set()
        self._history_s = float(e.get("history_s", 2.0)); self._replay_count = 0
        self._initial_snapshot = self._snapshot()
        self.diagnostics = self._empty_diagnostics()
        self.trace_updates = e.get('trace_updates', False)
        self.update_trace = []
        self.arrival_trace = []
        self._last_update_detail = None

    @staticmethod
    def _empty_diagnostics():
        return {"updates": 0, "replays": 0, "delayed_updates": 0,
                "rejected_updates": 0, "stale_measurements": 0, "nis": {}}

    @property
    def x17(self):
        omega = self.last_imu[3:6] - self.bg if self.last_imu is not None else np.zeros(3)
        return np.r_[self.p, self.v, self.q, omega, np.zeros(4)]

    @property
    def state(self):
        return self.x17.copy()

    def set_initial_baro_bias(self, value):
        """Install preflight calibration in the replay baseline."""
        if self._events:
            raise RuntimeError("preflight calibration must precede filter events")
        self.baro_bias = float(value)
        self._baro_bias_time = 0.0
        self._initial_snapshot = self._snapshot()

    def _snapshot(self):
        return dict(p=self.p.copy(), v=self.v.copy(), q=self.q.copy(), ba=self.ba.copy(),
                    bg=self.bg.copy(), P=self.P.copy(), time=self.time,
                    last_imu=None if self.last_imu is None else self.last_imu.copy(),
                    baro_bias=self.baro_bias,
                    latest_baro=None if self._latest_baro is None else tuple(self._latest_baro),
                    baro_bias_time=self._baro_bias_time)

    def _restore(self, s):
        self.p = s["p"].copy(); self.v = s["v"].copy(); self.q = s["q"].copy()
        self.ba = s["ba"].copy(); self.bg = s["bg"].copy(); self.P = s["P"].copy()
        self.time = s["time"]; self.last_imu = None if s["last_imu"] is None else s["last_imu"].copy()
        self.baro_bias = float(s["baro_bias"])
        self._latest_baro = s["latest_baro"]
        self._baro_bias_time = s["baro_bias_time"]

    @staticmethod
    def _event_key(packet):
        return (float(packet.sample_time), 0 if packet.kind == "imu" else 1, int(packet.sequence))

    def _propagate(self, z, dt):
        if dt <= 0.: return
        f = np.asarray(z[:3], dtype=float) - self.ba
        w = np.asarray(z[3:6], dtype=float) - self.bg
        R = Rotation.from_quat(self.q).as_matrix()
        a_w = R @ f + np.array([0., 0., -self.gravity])
        self.p += self.v*dt + .5*a_w*dt*dt; self.v += a_w*dt
        self.q = _qmul(self.q, Rotation.from_rotvec(w*dt).as_quat())
        self.q /= max(np.linalg.norm(self.q), 1e-12)
        F = np.eye(15); F[0:3, 3:6] = np.eye(3)*dt
        F[3:6, 6:9] = -R @ _skew(f)*dt; F[3:6, 9:12] = -R*dt
        F[6:9, 6:9] = np.eye(3)-_skew(w)*dt; F[6:9, 12:15] = -np.eye(3)*dt
        qa = float(self.profile["imu"].get("accel_noise_density", .03))**2
        qg = float(self.profile["imu"].get("gyro_noise_density", .002))**2
        Q = np.zeros((15, 15)); Q[3:6, 3:6] = np.eye(3)*qa*dt
        Q[6:9, 6:9] = np.eye(3)*qg*dt; Q[9:12, 9:12] = np.eye(3)*1e-7*dt
        Q[12:15, 12:15] = np.eye(3)*1e-9*dt
        self.P = (F @ self.P @ F.T + Q); self.P = (self.P+self.P.T)*.5

    def _inject(self, d):
        self.p += d[:3]; self.v += d[3:6]
        self.q = _qmul(self.q, Rotation.from_rotvec(d[6:9]).as_quat()); self.q /= max(np.linalg.norm(self.q), 1e-12)
        self.ba += d[9:12]; self.bg += d[12:15]

    def _update(self, measurement, h, H, Rm):
        innovation = np.asarray(measurement, dtype=float)-np.asarray(h, dtype=float)
        S = H @ self.P @ H.T + np.asarray(Rm, dtype=float)
        if self.trace_updates:
            self._last_update_detail = dict(
                measurement=np.asarray(measurement).tolist(), predicted=np.asarray(h).tolist(),
                innovation=innovation.tolist(), innovation_covariance=S.tolist(),
                measurement_covariance=np.asarray(Rm).tolist(),
                covariance_diagonal_before=np.diag(self.P).tolist(),
                state_before=self.state[:13].tolist(), correction_error_state=[0.]*15)
        try:
            nis = float(innovation @ np.linalg.solve(S, innovation)); K = np.linalg.solve(S, H @ self.P).T
        except np.linalg.LinAlgError:
            return False, np.inf
        gate = 100. if len(innovation) <= 3 else 200.
        if not np.isfinite(nis) or nis > gate: return False, nis
        I = np.eye(15); correction = K @ innovation; self._inject(correction)
        if self.trace_updates:
            self._last_update_detail['correction_error_state'] = correction.tolist()
        Rm = np.asarray(Rm, dtype=float)
        self.P = (I-K@H)@self.P@(I-K@H).T + K@Rm@K.T; self.P = (self.P+self.P.T)*.5
        return True, nis

    def _apply_event(self, packet):
        if self.time is None:
            self.time = float(packet.sample_time)
        dt = float(packet.sample_time)-float(self.time)
        if dt < -1e-9:
            raise ValueError("events must be applied in timestamp order")
        if self.last_imu is not None:
            # A measurement update can fall between IMU samples. Move the
            # nominal state/covariance to its timestamp using the last input.
            self._propagate(self.last_imu, max(0., dt))
        self.time = max(self.time, float(packet.sample_time))
        if packet.kind == "imu":
            self.last_imu = packet.value.copy(); return None, None
        e = self.profile["estimator"]
        if packet.kind == "gnss":
            if e.get("estimate_baro_bias", True) and self._latest_baro is not None:
                baro_time, baro_value = self._latest_baro
                if abs(float(packet.sample_time)-baro_time) <= float(e.get("baro_gnss_max_age_s", .25)):
                    tau = max(float(e.get("baro_bias_tau_s", 1.0)), 1e-6)
                    elapsed = (1.0/float(self.profile["gnss"]["rate_hz"]) if self._baro_bias_time is None
                               else max(0., float(packet.sample_time)-float(self._baro_bias_time)))
                    alpha = 1.0-np.exp(-elapsed/tau)
                    observed_bias = float(baro_value)-float(packet.value[2])
                    self.baro_bias += alpha*(observed_bias-self.baro_bias)
                    self._baro_bias_time = float(packet.sample_time)
            H = np.zeros((6,15)); H[:3,:3] = np.eye(3); H[3:6,3:6] = np.eye(3)
            Rm = np.diag(np.r_[np.full(3,float(e.get("gnss_pos_sigma",1.5))**2), np.full(3,float(e.get("gnss_vel_sigma",.5))**2)])
            return self._update(packet.value, np.r_[self.p,self.v], H, Rm)
        if packet.kind == "barometer":
            self._latest_baro = (float(packet.sample_time), float(packet.value[0]))
            if e.get("initialize_baro_bias", True) and self._baro_bias_time is None:
                # The arena starts from a known trim altitude, matching a
                # pre-flight barometer zero against launch/GNSS altitude.
                self.baro_bias = float(packet.value[0])-float(self.p[2])
                self._baro_bias_time = float(packet.sample_time)
            H = np.zeros((1,15)); H[0,2] = 1.
            corrected = np.asarray(packet.value, dtype=float)-self.baro_bias
            return self._update(corrected, [self.p[2]], H, [[float(e.get("baro_sigma",.6))**2]])
        if packet.kind == "magnetometer":
            field = np.asarray(self.profile["magnetometer"].get("world_field",[20.,0.,40.]), dtype=float)
            h = Rotation.from_quat(self.q).as_matrix().T @ field; H = np.zeros((3,15)); H[:,6:9] = _skew(h)
            return self._update(packet.value, h, H, np.eye(3)*float(e.get("mag_sigma",.15))**2)
        return None, None

    def _rebuild_diagnostics(self):
        d = self._empty_diagnostics()
        for r in self._events:
            if r["packet"].kind in ("imu", "rpm"): continue
            d["updates"] += int(r["accepted"]); d["rejected_updates"] += int(not r["accepted"])
            if r["accepted"] and self._now-r["packet"].sample_time > 1e-9: d["delayed_updates"] += 1
            d["nis"][r["packet"].kind] = r["nis"]
        d["replays"] = self._replay_count; d["stale_measurements"] = self.diagnostics.get("stale_measurements",0)
        d["barometer_bias_estimate"] = float(self.baro_bias)
        self.diagnostics = d

    def _replay_from(self, index):
        self._restore(self._initial_snapshot if index == 0 else self._events[index-1]["after"])
        for r in self._events[index:]:
            self._last_update_detail = None
            r["before"] = self._snapshot(); accepted, nis = self._apply_event(r["packet"])
            r["accepted"] = True if accepted is None else bool(accepted); r["nis"] = None if nis is None else float(nis)
            r["after"] = self._snapshot()
            if self.trace_updates and r['packet'].kind not in ('imu', 'rpm'):
                detail = dict(self._last_update_detail or {})
                detail.update(state_after=self.state[:13].tolist(),
                              covariance_diagonal_after=np.diag(self.P).tolist(),
                              baro_bias_before=float(r['before']['baro_bias']),
                              baro_bias_after=float(self.baro_bias))
                r['detail'] = detail
        self._replay_count += 1; self._rebuild_diagnostics()

    def _prune(self):
        cutoff = self._now-self._history_s
        while self._events and self._events[0]["packet"].sample_time < cutoff:
            self._initial_snapshot = self._events.pop(0)["after"]

    def advance(self, now, measurements=()):
        previous_state = self.state[:13] if self.trace_updates else None
        previous_time = self.time
        previous_baro_bias = self.baro_bias
        self._now = float(now); candidates = self._pending+list(measurements); self._pending=[]; available=[]
        for packet in candidates:
            (self._pending if packet.arrival_time > self._now+1e-9 else available).append(packet)
        earliest = None
        new_records, stale = [], []
        for packet in sorted(available, key=self._event_key):
            if packet.sequence in self._seen: continue
            self._seen.add(packet.sequence)
            if self._events and packet.sample_time < self._now-self._history_s:
                self.diagnostics["stale_measurements"] += 1
                if self.trace_updates and packet.kind not in ('imu', 'rpm'):
                    stale.append(packet)
                continue
            keys = [self._event_key(r["packet"]) for r in self._events]
            index = bisect_left(keys, self._event_key(packet)) if keys else 0
            record = {"packet":packet,"before":None,"after":None,"accepted":False,"nis":None}
            self._events.insert(index, record)
            new_records.append(record)
            earliest = index if earliest is None else min(earliest,index)
        if earliest is not None:
            self._replay_from(earliest)
        if self.time is None: self.time = self._now
        if self.trace_updates:
            updates = [r for r in new_records if r['packet'].kind not in ('imu', 'rpm')]
            if updates or stale:
                batch_id = len(self.arrival_trace)
                for record in updates:
                    packet = record['packet']
                    self.update_trace.append(dict(
                        batch_id=batch_id, sequence=int(packet.sequence), sensor=packet.kind,
                        sample_time_s=float(packet.sample_time), arrival_time_s=float(packet.arrival_time),
                        processed_at_s=self._now, raw_measurement=packet.value.tolist(),
                        status='accepted' if record['accepted'] else 'rejected',
                        nis=record['nis'], **record.get('detail', {})))
                for packet in stale:
                    self.update_trace.append(dict(
                        batch_id=batch_id, sequence=int(packet.sequence), sensor=packet.kind,
                        sample_time_s=float(packet.sample_time), arrival_time_s=float(packet.arrival_time),
                        processed_at_s=self._now, raw_measurement=packet.value.tolist(), status='stale'))
                new_sequences = {r['packet'].sequence for r in new_records}
                replay_tail = self._events[earliest:] if earliest is not None else ()
                replayed = [r for r in replay_tail if r['packet'].sequence not in new_sequences
                            and r['packet'].kind not in ('imu', 'rpm')]
                self.arrival_trace.append(dict(
                    batch_id=batch_id, processed_at_s=self._now,
                    sequences=[r['packet'].sequence for r in updates]+[p.sequence for p in stale],
                    state_time_before_s=previous_time, state_time_after_s=self.time,
                    state_before=previous_state.tolist(), state_after=self.state[:13].tolist(),
                    baro_bias_before=float(previous_baro_bias), baro_bias_after=float(self.baro_bias),
                    replayed_existing_measurements=len(replayed)))
        if earliest is not None:
            self._prune()
        return self.state
