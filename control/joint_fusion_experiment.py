"""Development screen for joint barometer-bias fusion; never labels runs as paper evidence."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
from control.navigation_grid_campaign import navigation_profile
from control.sensor_campaign import ROOT,run_campaign

POLICIES=('baseline','frozen100','joint_fixed_R','joint_matched_R')
ERRORS=('nominal','baro_drift','gnss_outlier','baro_outlier','reference_error')


def fusion_profile(policy,error='nominal'):
    p=navigation_profile(ROOT/'configs/sensors/nominal.json',ROOT/'configs/sensors/stress.json',0.,0.)
    if policy not in POLICIES or error not in ERRORS:raise ValueError('unknown policy/error')
    e=p['estimator'];e['trace_updates']=True
    if policy!='baseline':
        e.update(preflight_baro_samples=100,preflight_baro_rng='independent')
    if policy=='frozen100':e['estimate_baro_bias']=False
    if policy.startswith('joint'):
        e.update(kind='joint_baro',baro_bias_rw_density_m_sqrt_s=.03)
    if policy=='joint_matched_R':
        e.update(baro_sigma=p['barometer']['sigma'],mag_sigma=p['magnetometer']['sigma'])
    if error=='baro_drift':p['barometer']['bias_rate_m_s']=.05
    if error=='gnss_outlier':
        p['gnss']['outlier_windows']=[dict(start_s=2.,end_s=2.2,pos_offset_m=[0.,0.,20.])]
    if error=='baro_outlier':
        p['barometer']['outlier_windows']=[dict(start_s=2.,end_s=2.1,height_offset_m=5.)]
    if error=='reference_error':
        e.update(preflight_reference_error_m=.3,preflight_reference_sigma_m=.3,
                 initial_position_error_m=[0.,0.,.3])
    if policy!='baseline' or error!='nominal':p['name']=policy+'_'+error
    return p


def run(output,controllers,seeds,policies,errors,cases,timeout):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    result=dict(stage='DEVELOPMENT',controllers=controllers,seeds=seeds,policies=policies,
                errors=errors,cases=cases,records=[],complete=False)
    for policy in policies:
        for error in errors:
            profile=fusion_profile(policy,error);path=output/(policy+'_'+error+'.json')
            path.write_text(json.dumps(profile,indent=2)+'\n')
            for controller in controllers:
                for seed in seeds:
                    for case in cases:
                        print(f'START {controller}/{policy}/{error}/{case}/seed{seed}',flush=True)
                        summary=run_campaign(ROOT/'configs/arena.json',path,
                            output/'runs'/policy/error/controller/case/f'seed_{seed}',
                            cases=[case],controllers=[controller],seed=seed,timeout_s=timeout)
                        for r in summary['records']:
                            result['records'].append(dict(r,policy=policy,error=error))
                            print(f"DONE track={r.get('tracking_pass')} domain={r.get('model_domain_valid')} "
                                  f"outside={r.get('prop_domain_outside_fraction')} reasons={r.get('failure_reasons')}",flush=True)
                        (output/'experiment.json').write_text(json.dumps(result,indent=2)+'\n')
    result['complete']=True
    (output/'experiment.json').write_text(json.dumps(result,indent=2)+'\n')


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--controllers',nargs='+',default=['V13','F13'])
    p.add_argument('--seeds',nargs='+',type=int,default=[1])
    p.add_argument('--policies',nargs='+',choices=POLICIES,default=list(POLICIES))
    p.add_argument('--errors',nargs='+',choices=ERRORS,default=['nominal'])
    p.add_argument('--cases',nargs='+',default=['gust_lateral_p10_VL','gust_lateral_p10_VH'])
    p.add_argument('--timeout-s',type=float,default=300.)
    a=p.parse_args(argv);run(a.output,a.controllers,a.seeds,a.policies,a.errors,a.cases,a.timeout_s)

if __name__=='__main__':main()
