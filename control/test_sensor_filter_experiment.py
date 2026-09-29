from copy import deepcopy
import json
import numpy as np
import pytest

from control.arena import validate_config
from control.arena_feedback import RotorObserver
from control.arena_sensors import Measurement
from control.hybrid_comparison import ProperHybrid
from control.sensor_feedback_diagnostics import angular_acceleration_error
from control.sensor_filter_experiment import variant_config
from control.vehicle_params import vehicle_params


def state():
    x = np.zeros(17)
    x[9] = 1.
    x[13:] = 500.
    return x


def hybrid(**kwargs):
    return ProperHybrid(lambda t, x: np.array([10., 0., 0., 0.]),
                        vehicle_params, dt=.002, **kwargs)


@pytest.mark.parametrize('cutoff', [0., -1., float('nan'), float('inf')])
def test_invalid_filter_cutoff_is_rejected(cutoff):
    with pytest.raises(ValueError, match='f_cut'):
        hybrid(f_cut=cutoff)


@pytest.mark.parametrize('cutoff', [12.5, 25., 50.])
def test_s1_gyro_derivative_and_rotor_force_use_same_filter(cutoff):
    ctrl = hybrid(f_cut=cutoff, time_align='S1')
    x = state()
    ctrl(0., x)
    ctrl(.002, x)
    f0 = ctrl.feedback_probe['rotor_thrust_used'].copy()
    x[10] = 1.
    x[13:] = 550.
    ctrl(.004, x)
    a = 1.-np.exp(-2*np.pi*cutoff*.002)
    f1 = ctrl.feedback_probe['rotor_thrust_raw']
    np.testing.assert_allclose(ctrl.feedback_probe['rotor_thrust_used'], f0+a*(.5*(f1+f0)-f0))
    np.testing.assert_allclose(ctrl.feedback_probe['omega_dot_filtered'], [a/.002, 0., 0.])


def test_startup_guard_waits_for_arrived_telemetry_and_reset_clears_readiness():
    x = state()
    obs = RotorObserver(x, {'schema': 'sensor/1'}, .002)
    packet = Measurement('rpm', 0., .004, x[13:], 1)
    ctrl = hybrid(time_align='S1', alloc_mode='A1', rotor_startup_guard=True)
    for t in [0., .002, .004, .006]:
        x[13:] = obs.update(np.full(4, 500.), [packet] if t == 0. else [], now=t)
        ctrl.set_rotor_feedback_ready(obs.ready)
        command = ctrl(t, x)
        if t < .004:
            assert ctrl.probe['path'] == 'fallback_rotor_unavailable'
            assert np.all(command > 0.)
            assert ctrl._f_filt is None
        elif t == .004:
            assert ctrl.probe['path'] == 'fallback_init'
        else:
            assert ctrl.probe['path'] == 'A1'
            assert np.all(ctrl.feedback_probe['rotor_thrust_used'] > 0.)
    ctrl.reset()
    ctrl(0., x)
    assert ctrl.probe['path'] == 'fallback_rotor_unavailable'


def test_diagnostic_ignores_derivative_before_controller_initialization():
    times = np.arange(5)*.002
    rates = np.tile([2., 0., 0.], (5, 1))*times[:, None]
    _, filtered = angular_acceleration_error(times, np.zeros((5, 3)), rates, 'S1', 25.,
                                            active_steps=[False, False, False, True, True])
    np.testing.assert_array_equal(filtered[:2], 0.)
    np.testing.assert_allclose(filtered[2:, 0], 2*(1.-np.exp(-2*np.pi*25.*np.array([.002, .004]))))


def test_controller_variant_preserves_sensor_independent_arena_fields():
    base = json.loads(open('configs/arena.json').read())
    original = deepcopy(base)
    result = variant_config(base, 'F13', 'guarded25')
    assert result['controllers']['F13'] == dict(base['controllers']['F13'],
                                               time_align='S1', indi_cutoff_hz=25., rotor_startup_guard=True)
    result['controllers']['F13'] = deepcopy(base['controllers']['F13'])
    assert result == base == original


def test_f13_builder_keeps_indi_options_out_of_nmpc(monkeypatch):
    import control.nmpc_f13 as module
    captured = {}
    def adapter(params, **kwargs):
        captured.update(kwargs)
        return lambda t, x: np.array([10., 0., 0., 0.])
    monkeypatch.setattr(module, 'F13VirtualAdapter', adapter)
    ctrl = module.build_f13_controller(vehicle_params, f_cut=25., time_align='S1',
                                      rotor_startup_guard=True, N=5)
    assert ctrl.f_cut == 25. and ctrl.time_align == 'S1' and ctrl.rotor_startup_guard
    assert captured == dict(v_ref=None, z_ref=0., N=5)


@pytest.mark.parametrize('field,value', [('indi_cutoff_hz', float('nan')), ('rotor_startup_guard', 'false')])
def test_arena_rejects_invalid_experiment_options(field, value):
    base = json.loads(open('configs/arena.json').read())
    base['controllers']['V13'][field] = value
    with pytest.raises(ValueError, match=field):
        validate_config(base)
