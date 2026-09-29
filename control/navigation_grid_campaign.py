"""Two-dimensional IMU/GNSS sensor-severity campaign.

The grid interpolates each sensor block from a nominal profile (level 0) to a
stress profile (level 1), while leaving barometer, magnetometer, RPM, and rotor
observer settings nominal. Each grid point and seed is run independently using
the bounded campaign runner.
"""
from copy import deepcopy
import argparse
import json
from pathlib import Path

import numpy as np

from control.arena_sensors import load_sensor_profile
from control.sensor_campaign import run_campaign


ROOT = Path(__file__).resolve().parents[1]


def _blend(a, b, level):
    aa = np.asarray(a, dtype=float); bb = np.asarray(b, dtype=float)
    value = aa + float(level)*(bb-aa)
    return float(value) if value.ndim == 0 else value.tolist()


def navigation_profile(nominal, stress, imu_level, gnss_level, include_outage=False):
    nominal = load_sensor_profile(nominal)
    stress = load_sensor_profile(stress)
    profile = deepcopy(nominal)
    for key in ("rate_hz", "accel_noise_density", "gyro_noise_density",
                "accel_bias_rw", "gyro_bias_rw", "latency_s", "dropout_prob"):
        profile["imu"][key] = _blend(nominal["imu"][key], stress["imu"][key], imu_level)
    for key in ("accel_bias", "gyro_bias"):
        profile["imu"][key] = _blend(nominal["imu"][key], stress["imu"][key], imu_level)
    for key in ("rate_hz", "pos_sigma", "vel_sigma", "latency_s", "dropout_prob"):
        profile["gnss"][key] = _blend(nominal["gnss"][key], stress["gnss"][key], gnss_level)
    for key in ("pos_bias", "vel_bias"):
        profile["gnss"][key] = _blend(nominal["gnss"][key], stress["gnss"][key], gnss_level)
    profile["gnss"]["outage_windows"] = (deepcopy(stress["gnss"].get("outage_windows", []))
                                                   if include_outage and gnss_level >= 1.0 else [])
    # Match the GNSS update covariance to the generated measurement quality.
    profile["estimator"]["gnss_pos_sigma"] = profile["gnss"]["pos_sigma"]
    profile["estimator"]["gnss_vel_sigma"] = profile["gnss"]["vel_sigma"]
    profile["name"] = f"nav_imu{float(imu_level):g}_gnss{float(gnss_level):g}"
    return load_sensor_profile(profile)


def run_grid(config, nominal, stress, output, imu_levels, gnss_levels, seeds,
             cases, controllers, timeout_s=60., include_outage=False, python=None):
    output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=True)
    profiles_dir = output/"profiles"; profiles_dir.mkdir(exist_ok=True)
    records = []
    for imu_level in imu_levels:
        for gnss_level in gnss_levels:
            profile = navigation_profile(nominal, stress, imu_level, gnss_level, include_outage)
            profile_path = profiles_dir/f"{profile['name']}.json"
            profile_path.write_text(json.dumps(profile, indent=2)+"\n", encoding="utf-8")
            for seed in seeds:
                point = output/"runs"/profile["name"]/f"seed_{int(seed)}"
                summary = run_campaign(config, profile_path, point, cases=cases,
                                       controllers=controllers, seed=int(seed), python=python,
                                       timeout_s=timeout_s)
                for record in summary["records"]:
                    records.append({"imu_level": float(imu_level), "gnss_level": float(gnss_level),
                                    "sensor_seed": int(seed), **record})
    result = {"config": str(Path(config).resolve()), "nominal": str(Path(nominal).resolve()),
              "stress": str(Path(stress).resolve()), "imu_levels": list(map(float, imu_levels)),
              "gnss_levels": list(map(float, gnss_levels)), "seeds": list(map(int, seeds)),
              "include_outage": bool(include_outage), "cases": list(cases),
              "controllers": list(controllers), "records": records}
    (output/"navigation_grid.json").write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT/"configs"/"arena.json")
    parser.add_argument("--nominal", type=Path, default=ROOT/"configs"/"sensors"/"nominal.json")
    parser.add_argument("--stress", type=Path, default=ROOT/"configs"/"sensors"/"stress.json")
    parser.add_argument("--output", type=Path, default=ROOT/"results"/"navigation_grid")
    parser.add_argument("--imu-levels", type=float, nargs="+", default=[0., .5, 1.])
    parser.add_argument("--gnss-levels", type=float, nargs="+", default=[0., .5, 1.])
    parser.add_argument("--sensor-seeds", type=int, nargs="+", default=[1, 2])
    parser.add_argument("--only-cases", nargs="+", default=["gust_lateral_p10_VL"])
    parser.add_argument("--only-controllers", nargs="+", default=["GSLQR"])
    parser.add_argument("--timeout-s", type=float, default=60.)
    parser.add_argument("--include-outage-at-max", action="store_true")
    args = parser.parse_args(argv)
    result = run_grid(args.config, args.nominal, args.stress, args.output,
                      args.imu_levels, args.gnss_levels, args.sensor_seeds,
                      args.only_cases, args.only_controllers, args.timeout_s,
                      args.include_outage_at_max)
    for record in result["records"]:
        print(f"imu={record['imu_level']:g} gnss={record['gnss_level']:g} "
              f"seed={record['sensor_seed']} controller={record.get('controller')} "
              f"tracking={record.get('tracking_pass')} domain={record.get('model_domain_valid')}")


if __name__ == "__main__":
    main()
