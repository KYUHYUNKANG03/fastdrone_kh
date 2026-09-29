"""One-factor sensor-feedback ablations around a matched GNSS profile.

Zero-noise/zero-latency variants are diagnostic controls, not hardware proposals.
Controller parameters and acceptance rules remain unchanged.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path

from control.navigation_grid_campaign import navigation_profile
from control.sensor_campaign import ROOT, run_campaign


CHANGES = {
    "gyro_noise_zero": ("imu", "gyro_noise_density", 0.),
    "gyro_noise_quarter": ("imu", "gyro_noise_density", .0005),
    "rpm_noise_zero": ("rpm", "sigma", 0.),
    "rpm_latency_zero": ("rpm", "latency_s", 0.),
    "rotor_filter_fast": ("rotor_observer", "tau_s", .002),
}


def variant(base, name):
    result = deepcopy(base)
    if name != "base":
        block, key, value = CHANGES[name]
        result[block][key] = value
    result["name"] = base["name"]+"_"+name
    return result


def run_ablation(output, controllers, seeds, variants, level, case, timeout_s):
    output = Path(output).resolve()
    profiles = output/"profiles"
    profiles.mkdir(parents=True, exist_ok=True)
    base = navigation_profile(ROOT/"configs/sensors/nominal.json",
                              ROOT/"configs/sensors/stress.json", 0., level)
    result = {"case": case, "controllers": controllers, "seeds": seeds,
              "gnss_level": level, "variants": variants, "changes": CHANGES,
              "records": [], "complete": False}
    for name in variants:
        profile = variant(base, name)
        path = profiles/f"{profile['name']}.json"
        path.write_text(json.dumps(profile, indent=2)+"\n")
        for seed in seeds:
            for controller in controllers:
                print(f"START {controller} / {name} / seed {seed}", flush=True)
                summary = run_campaign(ROOT/"configs/arena.json", path,
                                       output/"runs"/name/f"seed_{seed}"/controller,
                                       cases=[case], controllers=[controller], seed=seed,
                                       timeout_s=timeout_s)
                for r in summary["records"]:
                    result["records"].append({"variant": name, **r})
                    print(f"DONE {controller} / {name} / seed {seed}: "
                          f"track={r.get('tracking_pass')} domain={r.get('model_domain_valid')} "
                          f"outside={r.get('prop_domain_outside_fraction')} "
                          f"reasons={r.get('failure_reasons')}", flush=True)
                (output/"ablation.json").write_text(json.dumps(result, indent=2)+"\n")
    result["complete"] = True
    (output/"ablation.json").write_text(json.dumps(result, indent=2)+"\n")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"results/sensor_feedback_ablation")
    parser.add_argument("--controllers", nargs="+", default=["V13"])
    parser.add_argument("--sensor-seeds", type=int, nargs="+", default=[1])
    parser.add_argument("--variants", nargs="+", choices=["base", *CHANGES],
                        default=["base", *CHANGES])
    parser.add_argument("--gnss-level", type=float, default=0.)
    parser.add_argument("--case", default="gust_lateral_p10_VL")
    parser.add_argument("--timeout-s", type=float, default=180.)
    args = parser.parse_args(argv)
    run_ablation(args.output, args.controllers, args.sensor_seeds, args.variants,
                 args.gnss_level, args.case, args.timeout_s)


if __name__ == "__main__":
    main()
