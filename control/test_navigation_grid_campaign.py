import pytest

from control.navigation_grid_campaign import navigation_profile


def test_navigation_grid_endpoints_match_sensor_blocks():
    nominal = 'configs/sensors/nominal.json'; stress = 'configs/sensors/stress.json'
    low = navigation_profile(nominal, stress, 0., 0.)
    high = navigation_profile(nominal, stress, 1., 1.)
    assert low['imu']['rate_hz'] == 500.
    assert high['imu']['rate_hz'] == 250.
    assert low['gnss']['pos_sigma'] == .8
    assert high['gnss']['pos_sigma'] == 3.
    assert high['gnss']['outage_windows'] == []


def test_navigation_grid_midpoint_and_matched_covariance():
    profile = navigation_profile('configs/sensors/nominal.json', 'configs/sensors/stress.json', .5, .5)
    assert profile['imu']['accel_noise_density'] == pytest.approx(.075)
    assert profile['gnss']['rate_hz'] == pytest.approx(7.5)
    assert profile['estimator']['gnss_pos_sigma'] == profile['gnss']['pos_sigma']
