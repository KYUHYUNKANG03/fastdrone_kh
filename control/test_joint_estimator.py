from copy import deepcopy
import numpy as np
import pytest
from scipy.linalg import expm
from scipy.spatial.transform import Rotation
from control.arena_joint_estimator import JointNavigationFilter, discretize_error_model, right_jacobian
from control.arena_sensors import Measurement, SensorSuite, load_sensor_profile
from control.arena_feedback import ArenaSensorFeedback


def state():
    x=np.zeros(17);x[2]=20.;x[9]=1.;return x


def profile(**fields):
    return load_sensor_profile({'schema':'sensor/1','estimator':{'kind':'joint_baro','trace_updates':True,**fields}})


def test_baro_update_matches_analytic_joint_gaussian_and_correlates_height_bias():
    f=JointNavigationFilter(state(),profile())
    prior=f.P.copy(); H=np.zeros((1,16));H[0,2]=H[0,15]=1.
    K=prior@H.T/(.25+1.+.36)
    f.advance(0.,[Measurement('barometer',0.,0.,[21.],1)])
    assert f.p[2]==pytest.approx(20.+K[2,0])
    assert f.baro_bias==pytest.approx(K[15,0])
    np.testing.assert_allclose(f.P,prior-K@H@prior,atol=1e-14)
    assert f.P[2,15]<0


def test_rejected_gnss_cannot_mutate_correlated_baro_bias_or_covariance():
    f=JointNavigationFilter(state(),profile())
    f.advance(0.,[Measurement('barometer',0.,0.,[20.4],1)])
    before=f._snapshot()
    f.advance(0.,[Measurement('gnss',0.,0.,[0.,0.,1000.,0.,0.,0.],2)])
    assert f.update_trace[-1]['status']=='rejected'
    assert f.baro_bias==before['baro_bias']
    np.testing.assert_array_equal(f.P,before['P'])
    np.testing.assert_array_equal(f.p,before['p'])


def test_delayed_replay_matches_chronological_joint_posterior():
    p=profile(preflight_baro_samples=100,preflight_reference_sigma_m=.2)
    a,b=JointNavigationFilter(state(),p),JointNavigationFilter(state(),p)
    for f in (a,b):f.set_initial_baro_bias(.1)
    events=[Measurement('imu',0.,0.,[0.,0.,9.81,0.,0.,0.],1),
            Measurement('gnss',0.,.12,[0.,0.,20.2,0.,0.,0.],2),
            Measurement('barometer',0.,.02,[20.3],3),
            Measurement('imu',.1,.1,[0.,0.,9.81,0.,0.,0.],4),
            Measurement('barometer',.1,.12,[20.2],5)]
    for t in (0.,.02,.1,.12): a.advance(t,[e for e in events if e.arrival_time==t])
    b.advance(.12,events)
    np.testing.assert_array_equal(a.state,b.state)
    np.testing.assert_array_equal(a.P,b.P)
    assert a.baro_bias==b.baro_bias


def test_preflight_covariance_includes_reference_uncertainty():
    f=JointNavigationFilter(state(),profile(preflight_baro_samples=100,preflight_reference_sigma_m=.2))
    f.set_initial_baro_bias(.1)
    assert f.P[15,15]==pytest.approx(.25**2/100+.2**2)
    assert f._initial_snapshot['P'][15,15]==f.P[15,15]


def test_discrete_covariance_matches_white_acceleration_integral():
    A=np.array([[0.,1.],[0.,0.]])
    F,Q=discretize_error_model(A,np.diag([0.,2.]),.2)
    np.testing.assert_allclose(F,[[1.,.2],[0.,1.]])
    np.testing.assert_allclose(Q,2*np.array([[.2**3/3,.2**2/2],[.2**2/2,.2]]))


def test_second_order_discretization_converges_to_van_loan_solution():
    A=np.array([[0.,1.,0.],[0.,-.2,-1.],[0.,0.,0.]])
    L=np.diag([0.,.1,.01]); errors=[]
    for dt in (.02,.01):
        M=np.block([[A,L],[np.zeros_like(A),-A.T]])
        exact=expm(M*dt); exactF=exact[:3,:3];exactQ=exact[:3,3:]@exactF.T
        F,Q=discretize_error_model(A,L,dt)
        errors.append(np.linalg.norm(F-exactF)+np.linalg.norm(Q-exactQ))
        assert np.linalg.eigvalsh(Q).min()>=-1e-14
    assert errors[1]<errors[0]/6.


def test_right_reset_jacobian_matches_rotation_finite_difference():
    d=np.array([.12,-.08,.2]);R=Rotation.from_rotvec(d);eps=1e-6
    columns=[]
    for axis in np.eye(3):
        plus=(R.inv()*Rotation.from_rotvec(d+eps*axis)).as_rotvec()
        minus=(R.inv()*Rotation.from_rotvec(d-eps*axis)).as_rotvec()
        columns.append((plus-minus)/(2*eps))
    np.testing.assert_allclose(right_jacobian(d),np.array(columns).T,atol=1e-9)


def test_joint_covariance_stays_psd_after_delayed_measurements():
    p=profile(preflight_baro_samples=25,preflight_baro_rng='independent')
    f=JointNavigationFilter(state(),p);f.set_initial_baro_bias(.03)
    sensors=SensorSuite(p,.002,seed=3)
    for k in range(151):
        f.advance(k*.002,[m for m in sensors.sample(k*.002,state()) if m.kind!='rpm'])
        assert np.linalg.eigvalsh(f.P).min()>=-1e-12
        assert np.linalg.norm(f.q)==pytest.approx(1.)


def test_injected_drift_outliers_preserve_noise_draws_and_sample_time():
    base=load_sensor_profile({'schema':'sensor/1'})
    altered=deepcopy(base);altered['barometer']['bias_rate_m_s']=.2
    altered['gnss']['outlier_windows']=[{'start_s':.1,'end_s':.2,'pos_offset_m':[0.,0.,20.]}]
    a,b=SensorSuite(base,.002,seed=4),SensorSuite(altered,.002,seed=4)
    for k in range(151):
        t=k*.002
        for left,right in zip(a.sample(t,state()),b.sample(t,state())):
            expected=np.zeros(len(left.value))
            if left.kind=='barometer':expected[0]=.2*t
            if left.kind=='gnss' and .1<=t<.2:expected[2]=20.
            np.testing.assert_allclose(right.value-left.value,expected,atol=1e-14)
            assert left.arrival_time==right.arrival_time


def test_reference_error_is_not_a_truth_altitude_change():
    p=profile(preflight_baro_samples=25,preflight_reference_error_m=.3,
              initial_position_error_m=[0.,0.,.5])
    p['barometer'].update(sigma=0.,bias=1.)
    x=state();f=ArenaSensorFeedback(x,p,.002)
    assert f.filter.p[2]==20.5
    assert f.filter.baro_bias==pytest.approx(.7)
    np.testing.assert_array_equal(x,state())


@pytest.mark.parametrize('bad', [dict(kind='unknown'),dict(baro_bias_rw_density_m_sqrt_s=-1.),
                               dict(initial_position_error_m=[0.,float('nan'),0.]),dict(nis_gates={'gnss':-1.})])
def test_invalid_estimator_options_fail_early(bad):
    with pytest.raises(ValueError):profile(**bad)
