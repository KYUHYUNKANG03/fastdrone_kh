"""Report one explicit controller screen without mixing historical campaigns."""
import argparse
from collections import defaultdict
import json
from pathlib import Path

import numpy as np


def estimate_errors(record, source):
    if record.get("feedback") != "sensors" or not record.get("run_dir"):
        return None
    run_dir = Path(record["run_dir"])
    # Prefer artifacts alongside a relocated report, such as the Desktop copy.
    if source.parent.name in run_dir.parts:
        index = run_dir.parts.index(source.parent.name)
        local = source.parent.joinpath(*run_dir.parts[index+1:])
        if local.exists():
            run_dir = local
    files = list(run_dir.glob("*.npz"))
    if len(files) != 1:
        return None
    with np.load(files[0], allow_pickle=False) as trial:
        if "xs_est" not in trial.files:
            return None
        truth, estimate = trial["xs"], trial["xs_est"]
        if truth.shape != estimate.shape:
            raise ValueError(f"unaligned truth/estimate traces: {files[0]}")
        difference = estimate-truth
        out = {}
        for name, columns in (("position_m", slice(0, 3)), ("velocity_m_s", slice(3, 6)),
                              ("body_rate_rad_s", slice(10, 13)), ("rotor_rad_s", slice(13, 17))):
            out[name] = float(np.sqrt(np.mean(np.sum(difference[:, columns]**2, axis=1))))
        dots = np.sum(truth[:, 6:10]*estimate[:, 6:10], axis=1)
        norms = np.linalg.norm(truth[:, 6:10], axis=1)*np.linalg.norm(estimate[:, 6:10], axis=1)
        angles = 2.*np.arccos(np.clip(np.abs(dots)/norms, 0., 1.))
        out["attitude_deg"] = float(np.rad2deg(np.sqrt(np.mean(angles**2))))
    return out


def write_report(source):
    source = Path(source).resolve()
    data = json.loads(source.read_text())
    groups = defaultdict(list)
    for record in data["records"]:
        groups[(record["profile"], record["controller"])].append(record)
    lines = [
        "# Sensor timing verification and controller comparison", "",
        f"Case: `{data['case']}`. Complete: `{data['complete']}`.",
        "This is a screening experiment, not a hardware qualification or a controller ranking.",
        "", "GNSS levels change rate, position/velocity noise, latency, and dropout together. "
        "The IMU stays nominal and GNSS measurement covariance matches each generated profile. "
        "The bundled nominal profile has different estimator defaults; it is not interchangeable "
        "with the matched level-0 profile below.",
        "", "All counts use the existing arena acceptance rules. Domain validity is an independent "
        "propulsion-model check. A timeout is a runtime limit, not evidence of physical instability.",
        "", "| Profile | Controller | Trials | Tracking | Domain | Full pass | Mean RMSE v [m/s] | "
        "Mean RMSE z [m] | Solver failures | Timeouts |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    def average(records, key):
        values = [r[key] for r in records if isinstance(r.get(key), (int, float))]
        return f"{np.mean(values):.3f}" if values else "—"
    for (profile, controller), records in groups.items():
        n = len(records)
        counts = [sum(r.get(key) is True for r in records)
                  for key in ("tracking_pass", "model_domain_valid", "passed")]
        lines.append(
            f"| {profile} | {controller} | {n} | {counts[0]}/{n} | {counts[1]}/{n} | "
            f"{counts[2]}/{n} | {average(records, 'rmse_velocity')} | "
            f"{average(records, 'rmse_z')} | "
            f"{sum(r.get('optimizer_failures', 0) for r in records)} | "
            f"{sum(bool(r.get('campaign_timed_out')) for r in records)} |")
    lines += ["", "## Individual outcomes", "",
              "| Profile | Controller | Seed | Tracking | Domain | Reasons |",
              "|---|---|---:|---|---|---|"]
    for r in data["records"]:
        lines.append(f"| {r['profile']} | {r['controller']} | {r.get('sensor_seed', '—')} | "
                     f"{r.get('tracking_pass')} | {r.get('model_domain_valid')} | "
                     f"{', '.join(r.get('failure_reasons', [])) or 'none'} |")
    error_rows = []
    lines += ["", "## State-estimation error against plant truth", "",
              "These are RMS vector-error norms over the whole trial, including startup. "
              "They measure estimation error rather than trajectory-tracking error.", "",
              "| Profile | Controller | Seed | Position [m] | Velocity [m/s] | "
              "Attitude [deg] | Body rate [rad/s] | Rotor state [rad/s] |",
              "|---|---|---:|---:|---:|---:|---:|---:|"]
    for r in data["records"]:
        errors = estimate_errors(r, source)
        if errors is None:
            continue
        error_rows.append({"profile": r["profile"], "controller": r["controller"],
                           "sensor_seed": r["sensor_seed"], **errors})
        values = " | ".join(f"{errors[k]:.3f}" for k in
                            ("position_m", "velocity_m_s", "attitude_deg",
                             "body_rate_rad_s", "rotor_rad_s"))
        lines.append(f"| {r['profile']} | {r['controller']} | {r['sensor_seed']} | {values} |")
    (source.parent/"state_estimation_errors.json").write_text(json.dumps(error_rows, indent=2)+"\n")
    lines += ["", "## Scope and reproducibility", "",
              "- Navigation starts from known trim position, velocity, and attitude; "
              "uncertain initialization is not tested.",
              "- RPM is initialized only from configured values or arrived telemetry. "
              "The ideal-sensor path still includes discrete estimation and rotor filtering.",
              "- Noise uses per-sensor streams; this pairs noise realizations, not true "
              "trajectories or measured values after the controllers diverge.",
              "- At most one sensor sample is generated per plant tick; configured rates "
              "above the simulation frequency are not supported by this study.",
              "- One flight case and two sensor seeds do not establish failure probabilities.",
              "- Each trial directory contains its profile, source hashes, trajectory, "
              "solver log, and manifest.", "",
              "The accompanying plot shows per-profile means and observed seed ranges; "
              "these ranges are not confidence intervals.", ""]
    report = source.parent/"controller_screen.md"
    report.write_text("\n".join(lines), encoding="utf-8")
    return data, groups, report


def plot_report(data, groups, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    profiles = list(dict.fromkeys(r["profile"] for r in data["records"]))
    controllers = data["controllers"]
    colors = ["#2563eb", "#0d9488", "#dc6b28", "#9b51e0", "#64748b"]
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), layout="constrained")
    width = .8/len(controllers)
    for ax, key, label in zip(axes, ("rmse_velocity", "rmse_z"),
                              ("Velocity RMSE [m/s]", "Altitude RMSE [m]")):
        for j, controller in enumerate(controllers):
            for i, profile in enumerate(profiles):
                vals = [r[key] for r in groups.get((profile, controller), [])
                        if isinstance(r.get(key), (float, int))]
                if not vals:
                    continue
                mean = float(np.mean(vals))
                x = i-.4+(j+.5)*width
                ax.bar(x, mean, width*.9, color=colors[j % len(colors)],
                       label=controller if i == 0 else None)
                ax.errorbar(x, mean, yerr=[[mean-min(vals)], [max(vals)-mean]],
                            color="#1f2937", capsize=3, linewidth=1)
        ax.set_xticks(range(len(profiles)), profiles, rotation=15, ha="right", fontsize=9)
        ax.set_ylabel(label)
        ax.grid(axis="y", alpha=.2)
        ax.set_axisbelow(True)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, ncol=len(controllers), loc="outside lower center", frameon=False)
    fig.suptitle("Sensor timing v2 · " + data["case"] +
                 "\nMeans and observed seed ranges; one-case screening", fontsize=12)
    fig.savefig(output/"controller_screen.png", dpi=170)
    fig.savefig(output/"controller_screen.svg")
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    args = parser.parse_args(argv)
    data, groups, report = write_report(args.source)
    plot_report(data, groups, report.parent)
    print(report)


if __name__ == "__main__":
    main()
