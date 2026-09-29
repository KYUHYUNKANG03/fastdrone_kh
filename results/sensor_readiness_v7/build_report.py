"""Regenerate the v7 report from recorded trials; no controller or verdict edits."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
import numpy as np
from scipy.spatial.transform import Rotation
from control.sensor_feedback_diagnostics import diagnose, trace_path
from control.sensor_filter_report import startup
from control.sensor_binding import runtime_source_hashes
from control.arena import load_config
OUT = Path(__file__).resolve().parent
sources = ['matching_v13_v7', 'matching_f13_v7', 'matching_f13_alignment_v7',
           'matching_v13_cutoff_v7', 'matching_faults_v7']
rows = []
for name in sources:
    path = ROOT/'results'/name/'experiment.json'
    doc = json.loads(path.read_text())
    assert doc['complete'] and len(doc['records']) == doc['expected_trials'], name
    assert doc['contract']['runtime_source_sha256'] == runtime_source_hashes(), name
    for record in doc['records']:
        row = dict(record, source=str(path.relative_to(ROOT)))
        detail = diagnose(record, path)
        if detail is not None:
            row['feedback_diagnostics'] = detail
            row['startup'] = startup(record, path)
            trace = trace_path(record, path)
            with np.load(trace, allow_pickle=False) as data:
                k = min(len(data['xs']), len(data['xs_est']))
                truth, estimate = data['xs'][:k], data['xs_est'][:k]
                err = estimate-truth
                att = (Rotation.from_quat(truth[:,6:10]).inv()*Rotation.from_quat(estimate[:,6:10])).magnitude()
                rms = lambda x: float(np.sqrt(np.mean(np.sum(x*x,axis=1))))
                row['estimation_rms'] = dict(position_m=rms(err[:,:3]),velocity_m_s=rms(err[:,3:6]),
                    attitude_deg=float(np.rad2deg(np.sqrt(np.mean(att**2)))),
                    body_rate_rad_s=rms(err[:,10:13]),rotor_rad_s=rms(err[:,13:]))
        rows.append(row)
ideal_path=ROOT/'results/matching_ideal_v7/campaign.json'
ideal=json.loads(ideal_path.read_text())
assert len(ideal['records'])==2
rows += [dict(r, variant='ideal_sampled_legacy15',fault='nominal',source=str(ideal_path.relative_to(ROOT))) for r in ideal['records']]
truth_path=next((ROOT/'results/matching_truth_v7').glob('*/trials.jsonl'))
rows += [dict(json.loads(line),variant='truth',fault='nominal',run_dir=str(truth_path.parent),source=str(truth_path.relative_to(ROOT))) for line in truth_path.read_text().splitlines() if line]
assert len(rows)==19
for row in rows:
    if row.get('run_dir'):
        row['run_dir_relative']=str(Path(row['run_dir']).relative_to(ROOT))
(OUT/'outcomes.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
xmls=['locked_environment_tests.xml','portable_tools_tests.xml']
counts=dict(passed=0,skipped=0,failed=0)
for name in xmls:
    for case in ET.parse(OUT/name).getroot().iter('testcase'):
        if case.find('skipped') is not None: counts['skipped']+=1
        elif case.find('failure') is not None or case.find('error') is not None: counts['failed']+=1
        else: counts['passed']+=1
base=load_config('configs/arena_v2.json');candidate=load_config('configs/arena_sensor_candidate_v6.json');candidate.pop('sensor_feedback')
verification=dict(tests=counts,candidate_only_adds_sensor_binding=(candidate==base),runtime_source_sha256=runtime_source_hashes(),
    trials=len(rows),heldout_seeds_used=[],final_tuning_performed=False,
    limitations=['One development seed per comparison; no failure-rate estimate.',
                 'Sensor specifications are engineering assumptions, not calibrated hardware limits.',
                 'Windows execution and worker checks remain unverified.',
                 'Git metadata unavailable because local Git requests Xcode license acceptance.'])
assert verification['candidate_only_adds_sensor_binding']
verification['reproduction']={}
for name in ['sensor_reproduction_local_v7','sensor_reproduction_packaged_v7']:
    r=json.loads((ROOT/'results'/name/'comparison.json').read_text())
    verification['reproduction'][name]={k:r[k] for k in ['reproduction_pass','trajectory_bit_identical','problems']}
    assert r['reproduction_pass'] and r['trajectory_bit_identical']
verification['extracted_package_check_passed']=json.loads((OUT/'extracted_workspace_check.json').read_text())['passed']
assert verification['extracted_package_check_passed']
(OUT/'verification.json').write_text(json.dumps(verification,indent=2)+'\n')
fmt=lambda value: '—' if value is None else f'{value:.3f}'
lines=['# Sensor readiness and controller matching — v7','',
'**Development results only. The sensor candidate is not ready for final paper experiments.**','',
'All v7 runs use Python 3.13.7, the five exact pinned dependency versions, and one thread per numeric library. '
'The fitted controller-model coefficients and candidate configuration hash match the team reference. '
'The original truth configuration, aircraft, controller gains, and controller defaults were not changed.','',
f"Software verification: **{counts['passed']} tests passed, {counts['skipped']} long legacy tests skipped, {counts['failed']} failed**. "
'Code tests establish implementation behavior, not control robustness. The full Git/tag setup check is not claimed.','',
'## Matched high-speed comparison','',
'85 m/s lateral-gust case, development seed 3 for sensor runs. Variants change one specified parameter; '
'`aligned50` changes F13 S0 to S1. The ideal sampled reference uses the existing ideal profile and legacy 15D filter; '
'it is a separate interface reference, not a one-factor joint-filter ablation.','',
'| Controller | Feedback / variant | Duration [s] | Tracking | Model domain | z RMSE [m] | v RMSE [m/s] | Solver failures |',
'|---|---|---:|---|---|---:|---:|---:|']
for r in rows:
    if r.get('scenario_id')!='gust_lateral_p10_VH': continue
    lines.append(f"| {r['controller']} | {r['variant']} | {fmt(r.get('simulated_seconds'))} | {r.get('tracking_pass')} | {r.get('model_domain_valid')} | {fmt(r.get('rmse_z'))} | {fmt(r.get('rmse_velocity'))} | {r.get('optimizer_failures')} |")
lines += ['','RMSE from an early stop covers only the recorded prefix. A smaller number on an incomplete run does not imply better control. '
'All failures remain in `outcomes.json`, including stop reasons, domain excursions, startup availability, and estimation-error diagnostics.','',
'## Low-speed fault checks','',
'V13, 20 m/s lateral gust, development seed 4, same gains and sensor draws. Barometer drift is +0.05 m/s; '
'GNSS outlier adds +20 m to height during 5–5.2 s; GNSS outage is 5–6 s.','',
'| Fault | Duration [s] | Tracking | Model domain | z RMSE [m] | v RMSE [m/s] | Solver failures |',
'|---|---:|---|---|---:|---:|---:|']
for r in rows:
    if r.get('scenario_id')!='gust_lateral_p10_VL': continue
    lines.append(f"| {r['fault']} | {fmt(r.get('simulated_seconds'))} | {r.get('tracking_pass')} | {r.get('model_domain_valid')} | {fmt(r.get('rmse_z'))} | {fmt(r.get('rmse_velocity'))} | {r.get('optimizer_failures')} |")
lines += ['','These four runs test fault handling at one development seed. Their domain failures prevent treating them as validated propulsion results. '
'They do not establish a general outlier-rejection or outage-tolerance guarantee.','',
'## Reproduction and school-computer handoff','',
'The recorded nominal low-speed V13 case is embedded as a sensor-inclusive development reproduction reference. '
'`scripts/sensor_reproduce.py` compares verdicts, sensor identity, solver counts, and selected numeric metrics at rtol 1e-3, atol 1e-9; '
'bit identity is recorded separately. Reproducing its known domain failure is required for a reproduction pass.','',
'`scripts/sensor_workspace.py` packages allowlisted source, configuration, data, and documentation with byte checksums. '
'Follow [the Windows development instructions](../../docs/SENSOR_WINDOWS_DEVELOPMENT.md). '
'This does not bypass final Git/tag, full tuning reproduction, or worker-lifecycle checks.','',
'Both local replays passed and were trajectory-bit-identical: one in the working repository and one in a freshly extracted portable package. The 269-file package passed byte, dependency, thread, configuration, and model-coefficient checks on this Mac. Native Windows execution remains unverified.','',
'## Decision','',
'Keep the candidate provisional. The high-speed failures persist across several sensor/interface variants, '
'so neither reduced gyro noise nor reduced telemetry smoothing alone establishes a usable configuration. '
'Truth and ideal-feedback checks distinguish the reference architecture from the noisy closed loop. '
'The next tuning campaign must match sensors, estimator policy, and controller filtering under equal budgets, '
'and keep validation seeds independent. No final tuning records or paper performance claims were created.','',
'V6 results used a different Python/dependency environment. In unstable high-speed runs, the numeric trajectories and stop times '
'differ from v7; do not combine them into one repeated-trial sample. The locked v7 environment is the current development reference.','',
'![Recorded outcomes](comparison.png)','']
(OUT/'report.md').write_text('\n'.join(lines))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(1,2,figsize=(12,4.6),layout='constrained')
high=[r for r in rows if r.get('scenario_id')=='gust_lateral_p10_VH']
labels=[r['controller']+' '+r['variant'] for r in high]
axes[0].barh(labels,[r.get('simulated_seconds',0.) for r in high],color=['#087f5b' if r.get('passed') else '#b94b36' for r in high])
axes[0].axvline(12,color='#333',linestyle='--',lw=1)
axes[0].set_xlabel('Recorded simulation duration [s]')
axes[0].set_title('85 m/s: green = full pass, red = failure')
low=[r for r in rows if r.get('scenario_id')=='gust_lateral_p10_VL']
axes[1].bar([r['fault'] for r in low],[r['rmse_z'] for r in low],color='#32679e')
axes[1].tick_params(axis='x',rotation=25)
axes[1].set_ylabel('Altitude tracking RMSE [m]')
axes[1].set_title('20 m/s: four full-duration runs; all domain-invalid')
fig.suptitle('Development screen · fixed team gains · one seed per comparison')
fig.savefig(OUT/'comparison.png',dpi=170)
plt.close(fig)
print(json.dumps(dict(rows=len(rows),tests=counts),indent=2))
