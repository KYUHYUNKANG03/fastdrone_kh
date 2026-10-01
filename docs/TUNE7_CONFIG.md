# Final tuning configuration `configs/arena_tune7.json` (draft)

Status: the D5 effect check was judged a pass by an independent session
(2026-10-01). This file is the final tuning configuration candidate, frozen
before `tune-final-7`. Tuning results go to `tune7`. The sensor-included
reference record (V13 evaluation 0) and the reproduction references are
recorded from the committed tree.

## Derivation (fixed before any tuning result)

The file is `configs/arena_rotor_projected_development_v9.json` with three
decided fields: two added (items 1 and 3) and one kept at its v9 value
(item 2). Nothing else changes; a JSON diff against v9 must show exactly the two
additions:

1. `sensor_feedback.tuning_seeds` (decision D4 (c)). The 18 scenarios of
   `tuning.scenarios` get seeds **2001, 2002, ..., 2018, in the order they
   appear in `tuning.scenarios`**: the first scenario gets 2001 and the
   eighteenth gets 2018. The rule is written here before the file is
   generated. The seeds lie in `seeds.tuning` (2000-2999) and are all
   distinct.
2. `sensor_feedback.seed` stays **3**, the development seed for smoke runs and
   design checks. It is outside every tuning seed, so those runs never reuse a
   tuning noise sequence. The field is unchanged from v9; it is listed only to
   record the decision.
3. `sensor_feedback.profile.estimator.navigation_prehistory_s = 30` (decision
   D5, `docs/NAVIGATION_PREHISTORY.md`). T = 30 s comes from the covariance-only
   rule, not from tracking results.

V13 and F13 keep the defaults: no `indi_cutoff_hz` or `rotor_startup_guard`
field (decision D3). The sensor values are the v9 engineering assumptions
(decision D5). The profile name stays `rotor_projected_warm`.

Decision D1 = A (kj): propeller-model domain exits are not a tuning penalty.
There is no code change; they are reported separately in the main experiment.

## Hashes

Recorded in `results/navigation_prehistory_2026-10-01/tune7_config_check.json`.
That file holds the `config_sha256` (canonical JSON, `control.arena.config_sha256`),
the raw-file sha256, the `sensor_profile_sha256` and the field-by-field diff
against v9.

The V13 evaluation-0 reference record for this configuration
(`results/arena/tuning/env_check_tune7/`) and the active reproduction
references are described in `docs/REPRODUCTION_REFERENCES_TUNE7.md`.
