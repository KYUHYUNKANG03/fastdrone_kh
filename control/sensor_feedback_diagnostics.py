"""Post-process saved trajectories; never alters controller feedback or verdicts."""
import argparse
import json
from pathlib import Path

import numpy as np


def angular_acceleration_error(times, truth_rate, estimated_rate, alignment, cutoff_hz=50., active_steps=None):
    """Error in the INDI finite-difference + LPF path, in rad/s².

    Both paths use the same true trajectory; this is not a comparison between
    different closed-loop runs. The initial filtered derivative is zero.
    """
    if alignment not in ("S0", "S1"):
        raise ValueError("alignment must be S0 or S1")
    dt = np.diff(np.asarray(times, dtype=float))
    if np.any(~np.isfinite(dt)) or np.any(dt <= 0.) or not np.isfinite(cutoff_hz) or cutoff_hz <= 0.:
        raise ValueError("timestamps and cutoff must be positive/increasing")
    error = np.asarray(estimated_rate)-np.asarray(truth_rate)
    periods = np.clip(dt, 1e-4, .2)
    raw = np.diff(error, axis=0)/periods[:, None]
    state = np.zeros(3)
    filtered = []
    active = np.ones(len(dt)+1, dtype=bool) if active_steps is None else np.asarray(active_steps, dtype=bool)
    if active.shape != (len(dt)+1,):
        raise ValueError('active_steps must match timestamps')
    for index, (period, sample) in enumerate(zip(periods, raw)):
        if not active[index+1]:
            state = np.zeros(3)
            filtered.append(state.copy())
            continue
        alpha = (1.-np.exp(-2.*np.pi*cutoff_hz*period) if alignment == "S1"
                 else period/(period+1./(2.*np.pi*cutoff_hz)))
        state += alpha*(sample-state)
        filtered.append(state.copy())
    return raw, np.asarray(filtered).reshape(-1, 3)


def trace_path(record, source):
    if not record.get("run_dir"):
        return None
    directory = Path(record["run_dir"])
    if source.parent.name in directory.parts:
        index = directory.parts.index(source.parent.name)
        local = source.parent.joinpath(*directory.parts[index+1:])
        if local.exists():
            directory = local
    files = list(directory.glob("*.npz"))
    return files[0] if len(files) == 1 else None


def diagnose(record, source, settle_s=.5):
    from models.team_light.control.baseline_v2 import baseline_params
    from control.validation_suite import plant_truth
    from models.team_light.control.propeller_curve import positive_thrust_j_limit
    from scipy.spatial.transform import Rotation
    path = trace_path(record, source)
    if path is None:
        return None
    result = {k: record.get(k) for k in
              ("scenario_id", "profile", "variant", "controller", "sensor_seed", "tracking_pass",
               "model_domain_valid", "passed", "failure_reasons", "rmse_velocity", "rmse_z",
               "prop_domain_outside_fraction", "optimizer_failures", "campaign_timed_out")}
    result.update(source=str(source), trace=str(path), diagnostic_start_s=settle_s)
    with np.load(path, allow_pickle=False) as trace:
        ts, xs, commands = trace["ts"], trace["xs"], trace["us"]
        plant = plant_truth(baseline_params(), record)
        rpm = xs[1:, 13:]*60./(2.*np.pi)
        low, high = plant["prop_curve"]["assumed_rpm_working_range"]
        vb = Rotation.from_quat(xs[1:, 6:10]).inv().apply(xs[1:, 3:6]-trace["wind"])
        ratio = np.maximum(vb[:, :1], 0.)/(xs[1:, 13:]*plant["D_prop"]/(2.*np.pi)+1e-8)
        low_rpm = np.any(rpm < low-1e-6, axis=1)
        high_rpm = np.any(rpm > high+1e-6, axis=1)
        high_ratio = np.any(ratio > positive_thrust_j_limit(plant)+1e-8, axis=1)
        reverse = vb[:, 0] < -1e-8
        outside = low_rpm | high_rpm | high_ratio | reverse
        if abs(outside.mean()-record["prop_domain_outside_fraction"]) > 1e-12:
            raise ValueError(f"domain reconstruction disagrees with saved verdict: {path}")
        result.update(minimum_rotor_rpm=float(rpm.min()), assumed_minimum_rpm=low,
                      below_minimum_rpm_samples=int(low_rpm.sum()),
                      above_maximum_rpm_samples=int(high_rpm.sum()),
                      above_advance_ratio_limit_samples=int(high_ratio.sum()),
                      reverse_flow_samples=int(reverse.sum()),
                      below_minimum_rpm_after_settle_samples=int(np.sum(low_rpm & (ts[1:] >= settle_s))),
                      domain_first_s=float(ts[1:][outside][0]) if outside.any() else None,
                      domain_last_s=float(ts[1:][outside][-1]) if outside.any() else None)
        control_times = ts[:len(commands)]
        command_dt = np.diff(control_times)
        command_slew = np.diff(commands, axis=0)/command_dt[:, None]
        keep_command = control_times[1:] >= settle_s
        rms = lambda a: float(np.sqrt(np.mean(np.sum(np.asarray(a)**2, axis=1))))
        result["command_slew_rms_rad_s2"] = rms(command_slew[keep_command]) if keep_command.any() else None
        if "xs_est" in trace.files and record["controller"] in ("V13", "F13"):
            estimate = trace["xs_est"]
            # A divergence stop may omit the final endpoint estimate, but
            # every executed command must still have its input feedback state.
            if estimate.shape[1:] != xs.shape[1:] or len(estimate) < len(commands):
                raise ValueError(f"unaligned estimate trace: {path}")
            manifest_path = path.parent/"manifest.json"
            settings = (json.loads(manifest_path.read_text()).get('controller_settings', {})
                        .get(record['controller'], {})) if manifest_path.exists() else {}
            alignment = settings.get('time_align', "S1" if record["controller"] == "V13" else "S0")
            cutoff = float(settings.get('indi_cutoff_hz', 50.))
            active = (np.all(np.isfinite(trace['indi_omega_dot_filtered']), axis=1)
                      if 'indi_omega_dot_filtered' in trace.files else None)
            raw, filtered = angular_acceleration_error(
                control_times, xs[:len(commands), 10:13], estimate[:len(commands), 10:13], alignment,
                cutoff, active_steps=active)
            result.update(indi_cutoff_hz=cutoff, time_align=alignment,
                          rotor_startup_guard=settings.get('rotor_startup_guard', False))
            result["angular_accel_error_raw_rms_rad_s2"] = rms(raw[keep_command]) if keep_command.any() else None
            result["angular_accel_error_filtered_rms_rad_s2"] = rms(filtered[keep_command]) if keep_command.any() else None
    return result


def report(sources, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    rows = []
    for source in map(lambda p: Path(p).resolve(), sources):
        data = json.loads(source.read_text())
        if data.get("complete") is not True:
            raise ValueError(f"campaign is incomplete: {source}")
        for record in data["records"]:
            row = diagnose(record, source)
            if row is not None:
                rows.append(row)
    (output/"feedback_diagnostics.json").write_text(json.dumps(rows, indent=2)+"\n")
    lines = ["# Sensor-feedback diagnostics", "",
             "Post-processing only: no feedback, controller settings, or acceptance thresholds were changed.",
             "Angular-acceleration errors are RMS vector norms comparing differentiated estimated "
             "and true body rates on the same trajectory. Filter settings come from each trial manifest "
             "(historical defaults: V13 S1 / F13 S0, 50 Hz). Error and command-slew RMS exclude the first 0.5 s; "
             "the original acceptance verdict always includes startup.", "",
             "| Case | Profile | Controller | Seed | Track | Domain | Outside [%] | "
             "Min RPM | Below-min samples after 0.5 s | Filtered angular-accel error [rad/s²] | "
             "Command slew [rad/s²] |",
             "|---|---|---|---:|---|---|---:|---:|---:|---:|---:|"]
    for r in rows:
        accel = r.get("angular_accel_error_filtered_rms_rad_s2")
        slew = r.get("command_slew_rms_rad_s2")
        accel_text = "—" if accel is None else f"{accel:.3f}"
        slew_text = "—" if slew is None else f"{slew:.1f}"
        lines.append(f"| {r['scenario_id']} | {r['profile']} | {r['controller']} | "
                     f"{r['sensor_seed']} | {r['tracking_pass']} | {r['model_domain_valid']} | "
                     f"{100*r['prop_domain_outside_fraction']:.3f} | {r['minimum_rotor_rpm']:.1f} | "
                     f"{r['below_minimum_rpm_after_settle_samples']} | {accel_text} | {slew_text} |")
    (output/"feedback_diagnostics.md").write_text("\n".join(lines)+"\n")
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sources", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    rows = report(args.sources, args.output)
    print(f"Wrote diagnostics for {len(rows)} completed traces to {args.output}")


if __name__ == "__main__":
    main()
