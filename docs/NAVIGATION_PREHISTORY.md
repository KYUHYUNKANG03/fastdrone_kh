# Common navigation prehistory (decision D5)

Optional `estimator.navigation_prehistory_s` starts the navigation filter at
`-T` instead of at flight time zero. Every controller receives the same
procedure. Without the field nothing changes (bit-identical, including the
sensor profile hash).

## Why

The isolation study (decision table D1/D6, E1/E3) traced the low-speed
propeller-model exits to the first barometer (0.02 s) and GNSS (0.12 s)
updates. The filter starts at the true state but declares
`initial_position_sigma = 0.5 m`, so the first measurement is taken almost at
face value and the altitude estimate steps by about 0.4 m. Shrinking the
declared sigma removes the step only by trusting the first measurement less.
Instead, decision D5 lets the filter converge on packets from an
already-flying aircraft and hands the result over unchanged.

## Behaviour

1. At `t = -T` the filter starts from the true state of that time (plus any
   configured initial error) with the configured `P0`, unchanged.
2. Over `[-T, 0)` the truth is a steady-trim cruise: `p(t) = p0 + v0 t`;
   velocity, attitude, body rates, rotor speeds and the specific force
   (`plant.evaluate_xdot(x0, trim command, no wind)`, the same value the
   rotor prehistory uses) stay at their `t = 0` values. The measured trim
   residual is 2.6e-15 m/s^2 at 20 m/s, so the constant-velocity truth and
   the IMU specific force agree.
3. IMU, GNSS, barometer and magnetometer packets are generated over that
   interval with the same noise, bias, dropout, latency, clipping and outlier
   models as in flight. Each sensor continues its flight sampling grid `k/rate`
   to negative `k` on the plant-step ticks. No controller or plant runs.
4. At `t = 0` the estimate, IMU and barometer biases, covariance, event log
   and the queue of packets still in transit are kept. Nothing is reset to
   truth. A GNSS fix sampled at -0.1 s still arrives at 0.02 s during flight.
5. Order: preflight barometer calibration (now dated `-T`) -> navigation
   prehistory -> rotor prehistory -> flight.

## What is unchanged

- Flight sampling (`sample`, `_due`, `_drop`, `_packet`) has no edits.
- The five flight streams, the preflight stream and the rotor-prehistory
  stream are the first seven `SeedSequence` children, as before. The
  prehistory uses an eighth child with one grandchild per error source (IMU
  noise/dropout, IMU bias walk, GNSS, barometer, magnetometer).
- Prehistory packets carry negative sequence numbers (`-M ... -1`). The
  shared counter `_seq`, and so every flight packet including its sequence
  number, is identical with and without a prehistory.
- The IMU bias random walk is generated backward from the configured bias, so
  it ends exactly where the first flight IMU sample begins. A walk run
  backward from a fixed point has the same law as a forward one.
- `DEFAULT_SENSOR_PROFILE` has no entry for the field, so profiles without it
  keep their `sensor_profile_sha256`. A profile with the field, including an
  explicit `0`, hashes differently.

`set_initial_baro_bias(value, time=0.0)` gained the calibration instant. The
legacy 15-state filter blends GNSS into its barometer bias from that instant;
left at zero, the first prehistory GNSS fix would have contributed nothing.
The joint 16-state filter does not use it.

## Choosing T (covariance only)

Setting: v9 sensor profile reduced to `navigation_only`
(`sensor_group_profile`), sensor seed 3. Three start states are built exactly
as `run_trial` builds them: hover (`ref_accel_0_VH_rho1`), 20 m/s
(`gust_lateral_p10_VL`), 85 m/s (`gust_lateral_p10_VH`). The measure is
`sqrt(diag P)` for position, velocity and attitude right after
`initial_packets(0)`. No controller, plant integration or tracking metric is
involved (`scripts/navigation_prehistory_select.py`).

- **Pre-registered rule, failed.** Candidates {1, 2, 3, 5, 8} s; a candidate
  passes when every component is within 5 % of the next candidate. Nothing
  passed. The attitude terms (gyro-bias learning: the filter's initial
  gyro-bias sigma is 0.05 rad/s) and the velocity terms still moved 25-32 %
  per step. Altitude sigma grows with T (0.052 m at 1 s, 0.085 m at
  steady state) because the preflight calibration starts the barometer bias
  well known and its random walk then widens it.
- **Revised rule (kj, 2026-10-01).** Set after seeing this covariance table and
  before any performance result: the steady state is `sigma(120 s)`. Pick the
  smallest candidate in {1, 2, 3, 5, 8, 13, 20, 30, 45, 60} s with every
  component within 5 % of it, then the largest over the three states.
  **T = 30 s** for all three states. The largest remaining distance is
  3.4-3.5 %.

At 8 s and at 30 s, for all three states, the prehistory covariance equals
the covariance after the same time of ordinary flight from a cold filter to
within 0.15 % (`results/navigation_prehistory_2026-10-01/covariance_vs_flight.*`;
it is not bit-identical because different noise realizations move the
linearization point). The slow convergence belongs to the filter, not to the
prehistory.

Cost: about 0.4 s of single-process wall time per simulated prehistory second,
so about 12 s per trial at 30 s. This is roughly +100 % for GSLQR and CPID
(about 13 s per trial), +12 % for V13, +4 % for F13 and +2 % for M17.

Full table: `results/navigation_prehistory_2026-10-01/t_pre_selection.md`.

## Effect check (reported, not used to choose T)

Pre-registered check 3 compared 30 s on against off.
- Low speed: `gust_lateral_p10_VL`, navigation-only and full sensors, five
  controllers, seeds 3/4/5. The off runs are the E1 records, reused because
  five on-branch samples (one per controller) were bit-identical to them.
- 85 m/s: both gust cases, full sensors, seed 3, on and off on this branch.
  CPID is excluded because its design region is 0-20 m/s.

Full tables are in `results/navigation_prehistory_2026-10-01/effect_summary.md`
and `effect_table.md`.

- **Navigation-only:** domain-exit seeds go from V13 3, F13 2, M17 3, GSLQR 1,
  CPID 0 to zero for every controller. The first-update altitude step (worst
  seed) shrinks from 0.405 m to 0.075 m.
- **Full sensors:** exits remain for V13 (3/3 seeds, one plant step each: at
  0.036, 5.534 and 6.848 s) and for F13 (1/3, one step at 5.534 s). Without
  the prehistory the totals over three seeds were V13 149, F13 111, M17 115
  and GSLQR 1 steps. With it, M17, GSLQR and CPID have none. With the prehistory, V13's minimum rotor speed in the first 0.5 s is
  9,944-10,121 RPM; the domain floor is 10,000 RPM and trim is 11,348 RPM.
  The first-update step (worst seed) goes from 0.405 m to 0.096 m.
- **85 m/s:** no exits either way. M17 failed `pre_gust_not_settled` in both
  gust cases without the prehistory and passed with it. The altitude step goes
  from 0.196 m to 0.082 m.
- **The handover is realistic, not perfect.** Without the prehistory the
  attitude estimate starts exactly true and is then disturbed by the first
  updates: the maximum within 0.5 s is 0.22-0.72 deg in the low-speed V13 runs
  and 0.60 deg at 85 m/s. With it, the start carries a steady-state estimation
  error. In the V13 runs that is 0.03-1.1 deg in attitude; M17 shows 0.1-0.2 m
  in position. V13's early exit for seed 5 coincides with a 1.1 deg handover
  attitude error and a gyro-noise spike. That is consistent with the E3
  finding that IMU and navigation noise together drive residual exits.
- Cost per trial (process wall time, 85 m/s pairs): GSLQR 14-15 s -> 27 s,
  V13 +13 s, F13 +10-19 s; M17 run-to-run variation hides it. The recorded
  `wall_seconds` starts after sensor initialization, so tuning records do not
  show this cost.

These are three seeds at one low speed and one seed at 85 m/s. They describe
the mechanism, not failure rates.

Verdict (independent session, 2026-10-01): pass. That session traced the V13
seed-5 exit at 0.036 s on its own. A 1.1 deg handover roll error on the weakly
observed axis, plus gyro noise, made V13's INDI differential command swing; the
same seed with navigation-only sensors is smooth. It is recorded as a V13
characteristic, not an implementation defect.

## Limitations to state in the paper

- The prehistory is a declared steady-trim cruise. It is not a takeoff or
  climb, and it does not include measured flight data.
- With the field set, `estimator_diagnostics.updates` and related counters
  include prehistory events still inside the 2 s fixed-lag window.
- The rotor prehistory (0.1 s) is a separate, unchanged mechanism.
  `ROTOR_OBSERVER.md` says the exact initial navigation state "remains a
  separate limitation". That remains true for configurations without this
  field. With it, the navigation state at zero is a converged estimate, not
  the truth.

## Tests

`control/test_navigation_prehistory.py` covers the pre-registered checks:

- (a) flight packets are bit-identical with and without a prehistory,
  including sequence numbers, for v9 and for a stress profile (dropouts, bias
  walks, vibration, barometer drift, shared preflight stream).
- (b) the handover estimate is not reset to truth.
- (c) a separate filter fed only the generated packets reproduces the
  feedback filter bit for bit.
- (d) runs are deterministic for a given seed.
- (e) the pinned v9 and navigation-only profile hashes are unchanged.
- (f) packets in transit at zero stay queued and apply at their sample time.
- (g) replacing one prehistory stream changes the handover but no flight
  packet.

Further checks cover the sampling grid, the noise-free formulas and the sign
of `p0 + v0 t`, the backward bias walk, input validation, guards, an explicit
zero and the legacy calibration time.

A mutation check (`results/navigation_prehistory_2026-10-01/mutation_check.py`,
output beside it) made twelve deliberate bugs, one at a time, on a copy of
the tree. Ten were caught: a shared sequence counter, flight random streams,
a wrong start sign, a default merged into the profile, a forward bias walk,
calibration left at zero, dropped in-transit packets, a fresh filter at
zero, a truth reset of the controller-facing state, and a persistent
estimate-only reset that keeps covariance and queue. The other two wrote
truth into the filter state, or into its last replay snapshot, after the
prehistory. The fixed-lag replay undid them at `t = 0`, when the barometer
and magnetometer packets sampled just before zero arrive, so the result was
identical to the original code.
