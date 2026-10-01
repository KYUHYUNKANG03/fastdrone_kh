"""Cross-check: prehistory covariance after T s equals the covariance after T s of ordinary
flight from a cold filter (same profile, seed, start, steady-trim truth). No controller."""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT/'scripts'))
import numpy as np
import navigation_prehistory_select as s
from control.arena import load_config, build_scenarios
from control.arena_factory import ArenaFactory
from control.arena_feedback import ArenaSensorFeedback
from control.sensor_binding import resolve_feedback

config = load_config(s.ROOT/s.CONFIG); factory = ArenaFactory(config)
base = resolve_feedback(config).profile
for name, sid in s.STATES:
    scenario = build_scenarios(config, factory.cp, factory.p, only=[sid])[0]
    x, xd, u = s.start_state(factory, scenario)
    for T in (8., 30.):
        pre, _ = s.handover_sigma(s.navigation_only(base, T), x, xd, u, factory.dt)
        start = x.copy(); start[0:3] -= T*x[3:6]
        fb = ArenaSensorFeedback(start, s.navigation_only(base, 0.), factory.dt, seed=s.SEED)
        fb.initial_packets(0., start, xd, initial_command=u)
        for k in range(1, int(round(T/factory.dt))+1):
            truth = start.copy(); truth[0:3] = start[0:3]+start[3:6]*k*factory.dt
            fb.step(k*factory.dt, truth, xd, u)
        flight = np.sqrt(np.diag(fb.filter.P)[:9])
        print(f'{name} T={T:g}: max |sigma_prehistory/sigma_flight - 1| = {np.max(np.abs(pre/flight-1))*100:.3f}%', flush=True)
