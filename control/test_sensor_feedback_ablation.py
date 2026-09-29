from copy import deepcopy
import pytest

from control.navigation_grid_campaign import navigation_profile
from control.sensor_feedback_ablation import CHANGES, variant


@pytest.mark.parametrize("name", CHANGES)
def test_ablation_changes_exactly_one_declared_field(name):
    base = navigation_profile("configs/sensors/nominal.json", "configs/sensors/stress.json", 0., 0.)
    original = deepcopy(base)
    output = variant(base, name)
    block, key, value = CHANGES[name]
    assert output[block][key] == value
    output[block][key] = base[block][key]
    output["name"] = base["name"]
    assert output == base
    assert base == original
