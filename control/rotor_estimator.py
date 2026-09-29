"""Causal rotor telemetry projection using recorded, applied motor commands.

The newest usable measurement anchors the state at its sample timestamp.
Piecewise-constant command history projects that anchor to the present. This
is a model-based predictor, not a Kalman filter or a claim of exact rotor state.
"""
from collections import deque
import numpy as np


class TimestampedRotorEstimator:
    def __init__(self, initial, *, motor_tau_s, history_s, max_age_s, start_time=0.):
        self.n = np.asarray(initial, dtype=float).copy()
        if self.n.shape != (4,) or not np.all(np.isfinite(self.n)):
            raise ValueError('initial rotor estimate must be a finite 4-vector')
        if any(not np.isfinite(v) or v <= 0 for v in (motor_tau_s, history_s, max_age_s)):
            raise ValueError('rotor model time constants and history must be finite and positive')
        if max_age_s > history_s or not np.isfinite(start_time):
            raise ValueError('rotor ready age must fit history and start time must be finite')
        self.motor_tau_s, self.history_s, self.max_age_s = motor_tau_s, history_s, max_age_s
        self.time = float(start_time)
        self.sample_time = None
        self._history = deque()
        self._pending = []
        self.counts = dict(accepted=0, old_samples=0, outside_history=0, invalid_packets=0)

    @property
    def ready(self):
        return self.sample_time is not None and self.time-self.sample_time <= self.max_age_s+1e-9

    @property
    def diagnostics(self):
        return dict(self.counts, ready=self.ready,
                    sample_age_s=None if self.sample_time is None else self.time-self.sample_time,
                    history_intervals=len(self._history), pending_packets=len(self._pending),
                    motor_tau_s=self.motor_tau_s)

    def _flow(self, n, command, dt):
        return command+(n-command)*np.exp(-dt/self.motor_tau_s)

    def _project(self, measurement, sample_time):
        intervals = []
        for start, end, command in reversed(self._history):
            if end <= sample_time:
                break
            intervals.append((max(start, sample_time), end, command))
        n = measurement.copy()
        for start, end, command in reversed(intervals):
            n = self._flow(n, command, end-start)
        return n

    def update(self, command, measurements, *, now):
        command, now = np.asarray(command, dtype=float), float(now)
        if command.shape != (4,) or not np.all(np.isfinite(command)):
            raise ValueError('rotor command must be a finite 4-vector')
        if not np.isfinite(now) or now < self.time:
            raise ValueError('rotor observer time must be finite and nondecreasing')
        if now > self.time:
            self._history.append((self.time, now, command.copy()))
            self.n = self._flow(self.n, command, now-self.time)
        self.time = now
        cutoff = now-self.history_s
        while self._history and self._history[0][1] <= cutoff:
            self._history.popleft()
        for m in measurements:
            if m.kind != 'rpm':
                continue
            if (not np.isfinite(m.sample_time) or not np.isfinite(m.arrival_time)
                    or m.arrival_time < m.sample_time or m.sample_time > now+1e-9
                    or m.value.shape != (4,) or not np.all(np.isfinite(m.value))):
                self.counts['invalid_packets'] += 1
            elif m.arrival_time-m.sample_time > self.history_s+1e-9:
                # It cannot fit in retained history when delivered; do not queue it.
                self.counts['outside_history'] += 1
            else:
                self._pending.append(m)
        arrived = [m for m in self._pending if m.arrival_time <= now+1e-9]
        self._pending = [m for m in self._pending if m.arrival_time > now+1e-9]
        earliest = self._history[0][0] if self._history else now
        anchor = None
        for m in sorted(arrived, key=lambda m: (m.sample_time, m.sequence)):
            if self.sample_time is not None and m.sample_time <= self.sample_time:
                self.counts['old_samples'] += 1
            elif m.sample_time < max(cutoff, earliest)-1e-9:
                self.counts['outside_history'] += 1
            else:
                anchor = m
                self.sample_time = m.sample_time
                self.counts['accepted'] += 1
        if anchor is not None:
            self.n = self._project(anchor.value, anchor.sample_time)
        return self.n.copy()
