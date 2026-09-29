"""Aggregate sensor campaign JSON files into a threshold-oriented report."""
import argparse
import json
from pathlib import Path


def classify(record):
    if record.get("campaign_timed_out"):
        return "campaign_timeout"
    if record.get("passed") is True:
        return "pass"
    if record.get("tracking_pass") is False:
        return "tracking_failure"
    if record.get("model_domain_valid") is False:
        return "model_domain_exclusion"
    if record.get("optimizer_failures", 0):
        return "optimizer_failure"
    return "other_failure"


def collect(root):
    root = Path(root)
    records = []
    for path in sorted(root.glob("sensor_campaign*/campaign.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        for record in data.get("records", []):
            records.append({"campaign": path.parent.name, "classification": classify(record), **record})
    return records


def write_report(records, output):
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    lines = ["# Sensor campaign report", "",
             "Pass/fail is the arena result. `model_domain_exclusion` means tracking passed "
             "but the existing propulsion-model validity guard failed; it should not be counted "
             "as a sensor-estimator failure.", "",
             "| Campaign | Profile | Sweep value | Controller | Class | Tracking | Domain | "
             "RMSE v | RMSE z | Replays | Delayed updates | Reasons |",
             "|---|---|---:|---|---|---|---|---:|---:|---:|---:|---|"]
    for r in records:
        diag = r.get("estimator_diagnostics") or {}
        reasons = ", ".join(r.get("failure_reasons", []))
        lines.append(f"| {r.get('campaign','')} | {r.get('profile','')} | "
                     f"{'' if r.get('sweep_value') is None else r.get('sweep_value')} | "
                     f"{r.get('controller','')} | {r.get('classification')} | "
                     f"{r.get('tracking_pass')} | {r.get('model_domain_valid')} | "
                     f"{r.get('rmse_velocity','—')} | {r.get('rmse_z','—')} | "
                     f"{diag.get('replays','—')} | {diag.get('delayed_updates','—')} | {reasons} |")
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path("results"))
    parser.add_argument("--output", type=Path, default=Path("results/sensor_campaign_report.md"))
    args = parser.parse_args(argv)
    records = collect(args.root)
    write_report(records, args.output)
    print(f"wrote {args.output} ({len(records)} records)")


if __name__ == "__main__":
    main()
