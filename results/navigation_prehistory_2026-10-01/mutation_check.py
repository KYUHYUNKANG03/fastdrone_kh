"""Mutation check for control/test_navigation_prehistory.py (D5).

Each deliberate bug is applied, one at a time, to a temporary copy of the tree; the named tests must
fail. Two writes of truth into the filter state after the prehistory are undone by the fixed-lag
replay at t = 0 (equivalent mutants) and are expected to pass. Nothing in the repository is modified.

python results/navigation_prehistory_2026-10-01/mutation_check.py > results/navigation_prehistory_2026-10-01/mutation_check.txt
"""
import shutil, subprocess, sys, tempfile
from pathlib import Path

SRC = Path(__file__).resolve().parents[2]
MUTATIONS = [
    ('shared _seq counter', 'control/arena_sensors.py',
     "packets[t].append(Measurement(kind, float(t), float(t+spec.get('latency_s', 0.0)),\n                                          value, index-len(raw)))",
     "packets[t].append(self._packet(kind, t, value, spec))"),
    ('flight rng in prehistory', 'control/arena_sensors.py',
     "rng = self._navigation_prehistory_rng\n",
     "rng = dict(self._rng, imu_bias=self._rng['imu'])\n"),
    ('reset to truth at zero', 'control/arena_feedback.py',
     "        self._navigation_packets = dict(counts=counts, in_transit_at_zero=int(in_transit))",
     "        self._navigation_packets = dict(counts=counts, in_transit_at_zero=int(in_transit))\n        self.filter.p = self._x0[0:3].copy(); self.filter.v = self._x0[3:6].copy()"),
    ('wrong start sign', 'control/arena_feedback.py',
     "start[0:3] -= self.navigation_prehistory_s*start[3:6]",
     "start[0:3] += self.navigation_prehistory_s*start[3:6]"),
    ('default merged into profile', 'control/arena_sensors.py',
     '"preflight_baro_samples": 0},',
     '"preflight_baro_samples": 0, "navigation_prehistory_s": 0.0},'),
    ('forward bias walk from b0 at -T', 'control/arena_sensors.py',
     "for t in sorted(due['imu'], reverse=True):\n            step = np.sqrt(later-t)\n            ba = ba-",
     "for t in sorted(due['imu']):\n            step = np.sqrt(abs(later-t)) if later else 0.\n            ba = ba+"),
    ('calibration time left at zero', 'control/arena_feedback.py',
     "preflight_bias, time=-self.navigation_prehistory_s if self.navigation_prehistory_s else 0.0)",
     "preflight_bias, time=0.0)"),
    ('drop in-transit packets at zero', 'control/arena_feedback.py',
     "            self.filter.advance(tick, packets)\n",
     "            self.filter.advance(tick, [m for m in packets if m.arrival_time <= 1e-9])\n"),

    ('fresh filter at zero (P0 again)', 'control/arena_feedback.py',
     "        self._navigation_packets = dict(counts=counts, in_transit_at_zero=int(in_transit))",
     "        self._navigation_packets = dict(counts=counts, in_transit_at_zero=int(in_transit))\n        self.filter = type(self.filter)(self._x0, self.profile)"),
    ('estimate-only reset in the replay baseline', 'control/arena_feedback.py',
     "        self._navigation_packets = dict(counts=counts, in_transit_at_zero=int(in_transit))",
     "        self._navigation_packets = dict(counts=counts, in_transit_at_zero=int(in_transit))\n        last = self.filter._events[-1]['after']; last['p'] = self._x0[0:3]-self.dt*self._x0[3:6]; last['v'] = self._x0[3:6].copy(); self.filter._restore(last)"),

    ('controller state reset to truth at zero', 'control/arena_feedback.py',
     "            # Handover covariance: what the controller's first estimate carries.",
     "            self._state[:6] = self._x0[:6]\n            # Handover covariance: what the controller's first estimate carries."),
    ('persistent estimate-only reset (P, biases, queue kept)', 'control/arena_feedback.py',
     "        self._navigation_packets = dict(counts=counts, in_transit_at_zero=int(in_transit))",
     "        self._navigation_packets = dict(counts=counts, in_transit_at_zero=int(in_transit))\n"
     "        old = self.filter; new = type(old)(self._x0, self.profile)\n"
     "        new.p = self._x0[0:3]-self.dt*self._x0[3:6]; new.P = old.P.copy(); new.ba = old.ba.copy(); new.bg = old.bg.copy(); new.baro_bias = old.baro_bias\n"
     "        new.time = old.time; new.last_imu = old.last_imu; new._pending = old._pending; new._seen = old._seen; new._baro_bias_time = old._baro_bias_time\n"
     "        new._initial_snapshot = new._snapshot(); self.filter = new"),
]
EQUIVALENT = {'reset to truth at zero', 'estimate-only reset in the replay baseline'}
work_root = Path(tempfile.mkdtemp(prefix='d5_mutation_'))
bad = []
for name, rel, old, new in MUTATIONS:
    work = work_root/'tree'
    if work.exists(): shutil.rmtree(work)
    shutil.copytree(SRC, work, ignore=shutil.ignore_patterns('results', '.git', '__pycache__'))
    path = work/rel
    text = path.read_text()
    assert text.count(old) == 1, (name, text.count(old))
    path.write_text(text.replace(old, new))
    run = subprocess.run([sys.executable, '-m', 'pytest', 'control/test_navigation_prehistory.py', '-q', '-p', 'no:cacheprovider'],
                         cwd=work, capture_output=True, text=True)
    failed = sorted({l.split('::')[1].split(' ')[0].split('[')[0] for l in run.stdout.splitlines() if l.startswith('FAILED')})
    caught = bool(failed)
    expected = name not in EQUIVALENT
    verdict = 'ok' if caught == expected else 'UNEXPECTED'
    if verdict != 'ok': bad.append(name)
    print(f'{verdict:10s} {name}: {run.stdout.strip().splitlines()[-1]}  caught by {failed}', flush=True)
shutil.rmtree(work_root)
print('ALL AS EXPECTED' if not bad else f'UNEXPECTED: {bad}')
sys.exit(1 if bad else 0)
