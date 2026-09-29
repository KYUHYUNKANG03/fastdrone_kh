"""Bounded truth/ideal/matched-GNSS comparison on one arena flight case."""
import argparse
import json
from pathlib import Path

from control.navigation_grid_campaign import navigation_profile
from control.sensor_campaign import ROOT, run_campaign


def run_screen(output, controllers, seeds, timeout_s, case, levels):
    output = Path(output).resolve()
    profiles = output/"profiles"
    profiles.mkdir(parents=True, exist_ok=True)
    jobs = [("truth", None, "truth", 0),
            ("ideal", ROOT/"configs/sensors/ideal.json", "sensors", 0)]
    for level in levels:
        profile = navigation_profile(ROOT/"configs/sensors/nominal.json",
                                     ROOT/"configs/sensors/stress.json", 0., level)
        path = profiles/f"{profile['name']}.json"
        path.write_text(json.dumps(profile, indent=2)+"\n")
        jobs.extend((profile["name"], path, "sensors", seed) for seed in seeds)
    result = {"case": case, "controllers": controllers, "seeds": seeds,
              "gnss_levels": levels, "imu_level": 0., "timeout_s": timeout_s,
              "note": "GNSS covariance matched at every grid level; no GNSS outage.",
              "records": [], "complete": False}
    checkpoint = output/"controller_screen.json"
    for name, profile_path, feedback, seed in jobs:
        for controller in controllers:
            print(f"START {controller} / {name} / seed {seed}", flush=True)
            campaign = run_campaign(ROOT/"configs/arena.json", profile_path,
                                    output/"runs"/name/f"seed_{seed}"/controller,
                                    cases=[case], controllers=[controller], seed=seed,
                                    timeout_s=timeout_s, feedback=feedback)
            for record in campaign["records"]:
                result["records"].append(record)
                print(f"DONE {controller} / {name} / seed {seed}: "
                      f"tracking={record.get('tracking_pass')} "
                      f"domain={record.get('model_domain_valid')} "
                      f"timeout={record.get('campaign_timed_out')} "
                      f"reasons={record.get('failure_reasons')}", flush=True)
            checkpoint.write_text(json.dumps(result, indent=2)+"\n")
    result["complete"] = True
    checkpoint.write_text(json.dumps(result, indent=2)+"\n")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"results/sensor_timing_v2")
    parser.add_argument("--controllers", nargs="+", default=["GSLQR", "V13", "F13"])
    parser.add_argument("--sensor-seeds", type=int, nargs="+", default=[1, 2])
    parser.add_argument("--gnss-levels", type=float, nargs="+", default=[0., .25])
    parser.add_argument("--case", default="gust_lateral_p10_VL")
    parser.add_argument("--timeout-s", type=float, default=180.)
    args = parser.parse_args(argv)
    run_screen(args.output, args.controllers, args.sensor_seeds, args.timeout_s,
               args.case, args.gnss_levels)


if __name__ == "__main__":
    main()
