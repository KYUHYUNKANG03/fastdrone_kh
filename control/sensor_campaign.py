"""Run reproducible sensor-specification campaigns around the arena runner.

Each point is executed in a fresh Python process so CasADi memory and
controller state cannot leak between profiles. The plant/scenario/controller
remain fixed while one sensor specification changes at a time.
"""
from copy import deepcopy
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys

from control.arena_sensors import load_sensor_profile


ROOT = Path(__file__).resolve().parents[1]


def _safe_name(value):
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value)).strip("_")


def make_variant(base, sweep, value):
    """Return a profile with one controlled sensor specification changed."""
    profile = deepcopy(base)
    if sweep == "gnss_latency":
        profile["gnss"]["latency_s"] = float(value)
    elif sweep == "gnss_position_noise":
        profile["gnss"]["pos_sigma"] = float(value)
    elif sweep == "gnss_outage_duration":
        duration = float(value)
        if duration < 0.0:
            raise ValueError("GNSS outage duration must be nonnegative")
        start = 5.0
        profile["gnss"]["outage_windows"] = [] if duration == 0.0 else [[start, start + duration]]
    elif sweep == "imu_noise_scale":
        scale = float(value)
        profile["imu"]["accel_noise_density"] *= scale
        profile["imu"]["gyro_noise_density"] *= scale
    elif sweep == "imu_rate":
        profile["imu"]["rate_hz"] = float(value)
    elif sweep == "rpm_noise":
        profile["rpm"]["sigma"] = float(value)
    elif sweep == "barometer_bias":
        profile["barometer"]["bias"] = float(value)
    elif sweep == "barometer_noise":
        profile["barometer"]["sigma"] = float(value)
    elif sweep == "barometer_latency":
        profile["barometer"]["latency_s"] = float(value)
    elif sweep == "barometer_rate":
        profile["barometer"]["rate_hz"] = float(value)
    elif sweep == "barometer_bias_tau":
        profile["estimator"]["baro_bias_tau_s"] = float(value)
    elif sweep == "barometer_bias_tracking":
        profile["estimator"]["estimate_baro_bias"] = bool(value)
    elif sweep == "dropout":
        probability = float(value)
        profile["imu"]["dropout_prob"] = probability
        profile["gnss"]["dropout_prob"] = probability
        profile["rpm"]["dropout_prob"] = probability
    else:
        raise ValueError(f"unknown sensor sweep {sweep!r}")
    profile["name"] = f"{base.get('name', 'profile')}_{sweep}_{value:g}"
    return load_sensor_profile(profile)


def make_subsystem_variant(base, comparison, subsystem):
    """Replace exactly one sensor/observer block with another profile's block."""
    if subsystem not in ("imu", "gnss", "barometer", "magnetometer", "rpm", "rotor_observer", "estimator"):
        raise ValueError(f"unknown sensor subsystem {subsystem!r}")
    profile = deepcopy(base)
    profile[subsystem] = deepcopy(comparison[subsystem])
    profile["name"] = f"{base.get('name', 'base')}_with_{comparison.get('name', 'comparison')}_{subsystem}"
    return load_sensor_profile(profile)


def make_subsystem_group_variant(base, comparison, subsystems):
    profile = deepcopy(base)
    for subsystem in subsystems:
        if subsystem not in ("imu", "gnss", "barometer", "magnetometer", "rpm", "rotor_observer", "estimator"):
            raise ValueError(f"unknown sensor subsystem {subsystem!r}")
        profile[subsystem] = deepcopy(comparison[subsystem])
    label = "+".join(subsystems)
    profile["name"] = f"{base.get('name', 'base')}_with_{comparison.get('name', 'comparison')}_{label}"
    return load_sensor_profile(profile)


def _read_trials(path):
    trials = []
    trial_file = path / "trials.jsonl"
    if not trial_file.exists():
        return trials
    for line in trial_file.read_text(encoding="utf-8").splitlines():
        if line.strip():
            trials.append(json.loads(line))
    return trials


def run_campaign(config, base_profile, output, sweep=None, values=None,
                 cases=None, controllers=None, seed=0, python=None, timeout_s=360.0,
                 comparison_profile=None, subsystems=None, subsystem_groups=None,
                 feedback="sensors"):
    """Run a campaign and return its JSON-serializable summary."""
    config = Path(config).resolve()
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    if feedback not in ("truth", "sensors"):
        raise ValueError("feedback must be truth or sensors")
    if feedback == "truth" and (sweep is not None or comparison_profile is not None):
        raise ValueError("sensor variants require sensor feedback")
    base = load_sensor_profile(base_profile) if feedback == "sensors" else {"name": "truth"}
    if feedback == "truth":
        variants = [("truth", base)]
    elif comparison_profile is not None:
        if sweep is not None:
            raise ValueError("use either a numeric sweep or a subsystem comparison")
        comparison = load_sensor_profile(comparison_profile)
        if subsystem_groups:
            groups = [entry.split("+") for entry in subsystem_groups]
            variants = [("base", base)] + [("stress_"+"+".join(group),
                                              make_subsystem_group_variant(base, comparison, group))
                                             for group in groups]
        else:
            selected = list(subsystems or ("imu", "gnss", "barometer", "magnetometer", "rpm", "rotor_observer"))
            variants = [("base", base)] + [(f"stress_{name}", make_subsystem_variant(base, comparison, name))
                                            for name in selected]
    elif sweep is None:
        variants = [(base.get("name", "base"), base)]
    else:
        if not values:
            raise ValueError("sensor sweep requires at least one value")
        variants = [(f"{sweep}_{value:g}", make_variant(base, sweep, value)) for value in values]
    profiles_dir = output / "profiles"
    runs_dir = output / "runs"
    profiles_dir.mkdir(exist_ok=True); runs_dir.mkdir(exist_ok=True)
    python = python or sys.executable
    cases = list(cases or ["ref_accel_0_VH_rho1"])
    controllers = list(controllers or ["GSLQR"])
    records = []
    for name, profile in variants:
        profile_path = profiles_dir / f"{_safe_name(name)}.json"
        profile_path.write_text(json.dumps(profile, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        for controller in controllers:
            run_output = runs_dir / _safe_name(name) / _safe_name(controller)
            run_output.mkdir(parents=True, exist_ok=True)
            previous_runs = set(run_output.glob("arena_*"))
            command = [python, "-m", "control.validation_suite", "--config", str(config), "--smoke",
                       "--feedback", feedback,
                       "--sensor-seed", str(int(seed)), "--output", str(run_output),
                       "--only-cases", *cases, "--only-controllers", controller]
            if feedback == "sensors":
                command += ["--sensor-profile", str(profile_path)]
            timed_out = False
            try:
                completed = subprocess.run(command, cwd=ROOT, text=True, capture_output=True,
                                           check=False, timeout=float(timeout_s))
                stdout, stderr, returncode = completed.stdout, completed.stderr, completed.returncode
            except subprocess.TimeoutExpired as exc:
                timed_out = True
                stdout = exc.stdout or ""; stderr = exc.stderr or ""
                if isinstance(stdout, bytes): stdout = stdout.decode(errors="replace")
                if isinstance(stderr, bytes): stderr = stderr.decode(errors="replace")
                stderr += f"\ncampaign timeout after {float(timeout_s):g} s\n"
                returncode = 124
            (run_output / "campaign.log").write_text(stdout + "\n" + stderr, encoding="utf-8")
            arena_runs = sorted(set(run_output.glob("arena_*"))-previous_runs,
                                key=lambda p: p.stat().st_mtime)
            latest = arena_runs[-1] if arena_runs else None
            trials = _read_trials(latest) if latest else []
            for trial in trials:
                records.append({"profile": profile.get("name", name), "sensor_seed": int(seed),
                                "feedback": feedback, "sweep": sweep,
                                "sweep_value": None if sweep is None else float(name.split("_")[-1]),
                                "returncode": returncode, "campaign_timed_out": timed_out,
                                "run_dir": str(latest) if latest else None, **trial})
            if not trials:
                records.append({"profile": profile.get("name", name), "sensor_seed": int(seed),
                                "feedback": feedback, "sweep": sweep,
                                "sweep_value": None if sweep is None else float(name.split("_")[-1]),
                                "controller": controller, "returncode": returncode,
                                "campaign_timed_out": timed_out,
                                "run_dir": str(latest) if latest else None, "passed": False,
                                "tracking_pass": False, "model_domain_valid": None,
                                "failure_reasons": ["campaign_timeout" if timed_out else "no_trial_output"]})
    summary = {"config": str(config), "base_profile": (str(Path(base_profile).resolve())
                                                       if feedback == "sensors" else None),
               "comparison_profile": None if comparison_profile is None else str(Path(comparison_profile).resolve()),
               "subsystems": subsystems,
               "subsystem_groups": subsystem_groups,
               "feedback": feedback, "seed": int(seed), "sweep": sweep,
               "values": values, "cases": cases, "controllers": controllers,
               "timeout_s": float(timeout_s), "records": records}
    (output / "campaign.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs" / "arena.json")
    parser.add_argument("--base-profile", type=Path, default=ROOT / "configs" / "sensors" / "nominal.json")
    parser.add_argument("--feedback", choices=["truth", "sensors"], default="sensors")
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "sensor_campaign")
    parser.add_argument("--sweep", choices=["gnss_latency", "gnss_position_noise", "gnss_outage_duration", "imu_noise_scale",
                                              "imu_rate", "rpm_noise", "dropout", "barometer_bias",
                                              "barometer_noise", "barometer_latency", "barometer_rate",
                                              "barometer_bias_tau", "barometer_bias_tracking"])
    parser.add_argument("--values", type=float, nargs="+", help="values for --sweep")
    parser.add_argument("--only-cases", nargs="+", default=["ref_accel_0_VH_rho1"])
    parser.add_argument("--only-controllers", nargs="+", default=["GSLQR"])
    parser.add_argument("--sensor-seed", type=int, default=0)
    parser.add_argument("--comparison-profile", type=Path,
                        help="replace one subsystem at a time from this profile")
    parser.add_argument("--subsystems", nargs="+",
                        choices=["imu", "gnss", "barometer", "magnetometer", "rpm", "rotor_observer", "estimator"])
    parser.add_argument("--subsystem-groups", nargs="+",
                        help="plus-separated blocks, for example imu+gnss+estimator")
    parser.add_argument("--timeout-s", type=float, default=360.0,
                        help="wall-clock limit for each profile/controller run")
    args = parser.parse_args(argv)
    result = run_campaign(args.config, args.base_profile, args.output, args.sweep, args.values,
                           args.only_cases, args.only_controllers, args.sensor_seed,
                           timeout_s=args.timeout_s, comparison_profile=args.comparison_profile,
                           subsystems=args.subsystems, subsystem_groups=args.subsystem_groups,
                           feedback=args.feedback)
    for record in result["records"]:
        print(f"{record.get('profile')}: passed={record.get('passed')} "
              f"simulated={record.get('simulated_seconds')} "
              f"reasons={record.get('failure_reasons', [])}")


if __name__ == "__main__":
    main()
