"""Known-model stationary Monte Carlo check; not a closed-loop flight reliability claim."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import chi2
from control.arena_joint_estimator import JointNavigationFilter
from control.arena_sensors import SensorSuite,load_sensor_profile


def run(count=128,seconds=2.,seed_start=40000):
    profile=load_sensor_profile({'schema':'sensor/1','imu':{'rate_hz':100.},
        'barometer':{'rate_hz':25.,'bias':.4},'magnetometer':{'rate_hz':20.},
        'rpm':{'enabled':False},'estimator':{'kind':'joint_baro','baro_sigma':.25,'mag_sigma':.04,
            'gnss_pos_sigma':.8,'gnss_vel_sigma':.15,'preflight_baro_samples':100,
            'preflight_baro_rng':'independent','baro_bias_rw_density_m_sqrt_s':0.}})
    rows=[]
    for seed in range(seed_start,seed_start+count):
        rng=np.random.default_rng(np.random.SeedSequence(seed,spawn_key=(99,)))
        x=np.zeros(17);x[2]=20.;x[9]=1.
        initial=x.copy();initial[:3]+=rng.normal(0.,.5,3);initial[3:6]+=rng.normal(0.,.2,3)
        initial[6:10]=Rotation.from_rotvec(rng.normal(0.,np.deg2rad(1.),3)).as_quat()
        p=load_sensor_profile(profile);p['imu']['accel_bias']=rng.normal(0.,.05,3).tolist()
        p['imu']['gyro_bias']=rng.normal(0.,.05,3).tolist()
        sensors=SensorSuite(p,.01,seed=seed);f=JointNavigationFilter(initial,p)
        f.set_initial_baro_bias(sensors.preflight_barometer_bias(20.,100))
        for k in range(round(seconds/.01)+1):
            t=k*.01;f.advance(t,sensors.sample(t,x))
        error=np.r_[x[:3]-f.p,x[3:6]-f.v,
                    (Rotation.from_quat(f.q).inv()*Rotation.from_quat(x[6:10])).as_rotvec(),
                    sensors._bias['accel']-f.ba,sensors._bias['gyro']-f.bg,.4-f.baro_bias]
        nees=float(error@np.linalg.solve(f.P,error))
        rows.append(dict(seed=seed,nees=nees,height_error_m=float(error[2]),
                         height_sigma_m=float(np.sqrt(f.P[2,2])),bias_error_m=float(error[15]),
                         bias_sigma_m=float(np.sqrt(f.P[15,15])),minimum_eigenvalue=float(np.linalg.eigvalsh(f.P).min())))
    lower,upper=chi2.ppf([.025,.975],16*count)/count
    mean=float(np.mean([r['nees'] for r in rows]))
    return dict(kind='stationary known-model consistency screen',count=count,duration_s=seconds,
                seed_start=seed_start,state_dimension=16,mean_nees=mean,
                mean_nees_95pct_reference_interval=[float(lower),float(upper)],
                within_reference_interval=bool(lower<=mean<=upper),rows=rows,
                scope='Correct Gaussian initial-error model, constant barometer bias, 100 Hz IMU, exact preflight altitude. '
                      'Gating, nonlinear attitude updates, and discrete sampled noise make chi-square bounds diagnostic. '
                      'Not a proof of consistency during aggressive flight or a controller success probability.')


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--count',type=int,default=128);p.add_argument('--seconds',type=float,default=2.)
    a=p.parse_args(argv);result=run(a.count,a.seconds)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
if __name__=='__main__':main()
