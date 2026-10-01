# Reproduction references for tune7 (2026-10-01)

The runtime code changed after `scripts/data/sensor_*reproduction_v13.json` were
recorded: decision D4 and roadmap item 7 in `8bc6b4e`, D5 in `598d462`. Those
files pin the runtime source hashes, so `sensor_reproduce.py` and the
`active_reference_compatible` preflight check refuse them on the new code, by
design. Two new references were recorded and are now the active ones. **The old
`*_v13.json` files are kept unchanged.**

## New references

| file | case | config | trajectory sha256 |
|---|---|---|---|
| `scripts/data/sensor_reproduction_tune7.json` (new default) | V13, `gust_lateral_p10_VH`, seed 3 (same case as the old default) | `configs/arena_tune7.json` (final tuning config, 30 s navigation prehistory) | `505c9750…` |
| `scripts/data/sensor_legacy_reproduction_tune7.json` | V13, `gust_lateral_p10_VL`, seed 4 (same as before) | `configs/arena_sensor_candidate_v6.json` (unchanged) | `313d81f8…` |

Recording used `scripts/sensor_reproduction_record.py` (committed in `598d462`).
It follows the replay path of `sensor_reproduce.py` and of the original
`results/source_portability_v13/capture_reference.py`: the config's sensor
profile written to `sensors.json`, then `run_campaign`. It never overwrites a
reference. One difference from the original is that the `environment` string
records the actual machine (Apple M4, Python, NumPy, SciPy, CasADi versions).

### Provenance

A reference file has no commit field (the `*_v13.json` files do not either). It
pins the runtime source by 115 file hashes, which is the stronger binding.

- The commit and clean-tree flag are in each run's `manifest.json`. All six runs
  (two recordings, two replays, two old-reference checks) report `598d462`
  with `git_dirty: false`.
- Those manifests contain absolute local paths, so they are not committed.
  `results/reproduction_tune7_2026-10-01/provenance.json` keeps the provenance
  fields (commit, dirty flag, environment, config and profile hashes, verdicts,
  trajectory hashes) with repository-relative paths. It also holds the sha256 of
  each reference, the recording script and the V13 evaluation-0 record. This
  follows how `results/source_portability_v13` commits small receipts and not
  raw run folders.

### Checks before and after recording

1. **Before recording**
   (`results/reproduction_tune7_2026-10-01/old_v13_numeric_check.json`): the
   two old references were replayed on `598d462` without the hash gate.
   - The differing runtime files are exactly the nine D4/7 and D5 files.
   - Verdicts and numbers agree within rtol 1e-3, and `compare` catches a 1 %
     change.
   - The trajectories equal the ones `a44c718` produces on this Mac. They
     differ from the stored ones, which were made on another machine.
2. **After recording:** `sensor_reproduce.py` replayed both new references,
   including the runtime-hash gate. Both PASS, with no problems and
   bit-identical trajectories.

## Edits to existing files (the sensor team's code; recorded here as agreed)

| file | change | why |
|---|---|---|
| `scripts/sensor_preflight.py:92` | fixture path `sensor_reproduction_v13.json` -> `sensor_reproduction_tune7.json` | the active-fixture check must pin the current runtime |
| `scripts/sensor_preflight.py:95` | message "active v13 reproduction fixture" -> "active reproduction fixture" | the fixture is no longer v13 |
| `scripts/sensor_reproduce.py:91` | default `--reference` -> `sensor_reproduction_tune7.json` | the default replay is the active fixture |
| `scripts/test_sensor_envelope_screen.py:79` | provenance test reads `sensor_reproduction_tune7.json` | same reason |
| `.github/workflows/windows-sensor-validation.yml:49` | legacy step uses `sensor_legacy_reproduction_tune7.json` | legacy replay on the current runtime |
| `README.md:101-108` | the reproduction paragraph names the `*_tune7.json` fixtures as active (recorded at 598d462) and says the `*_v13.json` fixtures are kept; the v13-versus-v9 trajectory remark moved out (it remains in `docs/SOURCE_PORTABILITY.md`) | the first page after merging to main must name the active fixture |

These files are under `scripts/` and `.github/`, outside
`runtime_source_hashes()` (`control/` and `models/team_light/control/` `.py`,
115 files). The 115 hashes are identical before and after the edits and equal
the pin in `sensor_reproduction_tune7.json`.

Not edited yet (text still names the old files as default or legacy):
`docs/SENSOR_WINDOWS_DEVELOPMENT.md:69,73` and `docs/SOURCE_PORTABILITY.md:24-25`.
They describe the v13 transition and stay true as history, but they no longer
name the active default.

## V13 evaluation-0 reference record (school recheck A.4b)

`results/arena/tuning/env_check_tune7/` (`V13.record.json` `f8b933df…`,
`V13.jsonl` `3469e8b7…`) was recorded from `598d462` with `git_dirty: false`,
`configs/arena_tune7.json`, one scenario worker, budget 1 (evaluation 0 only).
The objective is 0.4258410417691406 and none of the 18 scenarios fails.
Wall time was 1,064 s; the recorded `wall_seconds` exclude the 30 s prehistory.

An earlier record of the same evaluation, made on the uncommitted tree, is kept
locally (not committed) for comparison
(`results/reproduction_tune7_2026-10-01/compare_env_check_records.json`):

- `V13.jsonl` is byte-identical.
- The record differs only in `git_revision`, `git_dirty` and `finished_utc`.
- All 18 scenario trajectories are bit-identical.
