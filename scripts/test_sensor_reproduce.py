from copy import deepcopy
import math
from scripts.sensor_reproduce import compare


def test_comparison_keeps_failures_and_domain_verdicts_distinct_from_reproduction():
    ref = dict(tracking_pass=True, model_domain_valid=False, passed=False,
               failure_reasons=['propulsion_model_domain'], rmse_z=.1, sensor_seed=4)
    assert not compare(ref, deepcopy(ref))
    changed = dict(ref, model_domain_valid=True)
    assert compare(ref, changed) == ['model_domain_valid: False -> True']
    assert compare(ref, dict(ref, sensor_seed=5))


def test_numeric_tolerance_missing_values_and_nan_fail():
    assert not compare(dict(rmse_z=1.), dict(rmse_z=1.0005))
    assert compare(dict(rmse_z=1.), dict(rmse_z=1.002))
    assert compare(dict(rmse_z=1.), {})
    assert compare(dict(rmse_z=math.nan), dict(rmse_z=math.nan))
