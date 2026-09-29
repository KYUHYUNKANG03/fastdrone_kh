import numpy as np
import pytest
from control.sensor_feedback_diagnostics import angular_acceleration_error


def test_constant_rate_bias_does_not_create_angular_acceleration():
    t = np.arange(10)*.002
    true = np.column_stack([t**2, -t, t])
    measured = true+np.array([.02, -.03, .01])
    raw, filtered = angular_acceleration_error(t, true, measured, "S1")
    np.testing.assert_allclose(raw, 0., atol=1e-13)
    np.testing.assert_allclose(filtered, 0., atol=1e-13)


def test_linear_rate_error_has_exact_exponential_filter_response():
    t = np.arange(10)*.002
    slope = np.array([2., -1., 3.])
    raw, filtered = angular_acceleration_error(t, np.zeros((10, 3)), t[:, None]*slope, "S1")
    np.testing.assert_allclose(raw, np.tile(slope, (9, 1)))
    expected = (1.-np.exp(-2.*np.pi*50.*t[1:]))[:, None]*slope
    np.testing.assert_allclose(filtered, expected, atol=1e-13)


def test_nonincreasing_timestamps_are_rejected():
    with pytest.raises(ValueError, match="timestamps"):
        angular_acceleration_error([0., 0.], np.zeros((2, 3)), np.zeros((2, 3)), "S0")


def test_large_interval_matches_controller_time_clamp():
    raw, filtered = angular_acceleration_error(
        [0., .5], np.zeros((2, 3)), np.array([[0., 0., 0.], [1., 0., 0.]]), "S0")
    assert raw[0, 0] == pytest.approx(5.)
    assert filtered[0, 0] == pytest.approx(5.*.2/(.2+1./(2.*np.pi*50.)))
