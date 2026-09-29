from control.sensor_campaign import make_subsystem_group_variant, make_subsystem_variant, make_variant
import pytest
import json
from types import SimpleNamespace
import control.sensor_campaign as campaign


def test_sensor_variant_changes_only_requested_specification():
    base = {"schema": "sensor/1", "name": "x", "gnss": {"latency_s": .1},
            "imu": {"rate_hz": 500., "accel_noise_density": .1, "gyro_noise_density": .01},
            "rpm": {"sigma": 2.}}
    variant = make_variant(base, "gnss_latency", .4)
    assert variant["gnss"]["latency_s"] == .4
    assert variant["imu"]["rate_hz"] == 500.
    assert variant["rpm"]["sigma"] == 2.


def test_sensor_noise_scale_is_symmetric_for_imu():
    base = {"schema": "sensor/1", "name": "x", "imu": {"accel_noise_density": .1, "gyro_noise_density": .01}}
    variant = make_variant(base, "imu_noise_scale", 3.)
    assert variant["imu"]["accel_noise_density"] == pytest.approx(.3)
    assert variant["imu"]["gyro_noise_density"] == pytest.approx(.03)


def test_gnss_outage_duration_variant_uses_fixed_start():
    base = {"schema": "sensor/1", "name": "x", "gnss": {"outage_windows": []}}
    assert make_variant(base, "gnss_outage_duration", 0.)["gnss"]["outage_windows"] == []
    assert make_variant(base, "gnss_outage_duration", 2.5)["gnss"]["outage_windows"] == [[5., 7.5]]
    with pytest.raises(ValueError):
        make_variant(base, "gnss_outage_duration", -1.)


def test_subsystem_variant_replaces_only_selected_block():
    base = {"schema": "sensor/1", "name": "base", "gnss": {"latency_s": .1},
            "rpm": {"sigma": 2.}}
    stress = {"schema": "sensor/1", "name": "stress", "gnss": {"latency_s": .5},
              "rpm": {"sigma": 40.}}
    variant = make_subsystem_variant(base, stress, "gnss")
    assert variant["gnss"]["latency_s"] == .5
    assert variant["rpm"]["sigma"] == 2.


def test_subsystem_group_replaces_each_named_block():
    base = {"schema": "sensor/1", "name": "base", "gnss": {"latency_s": .1}, "rpm": {"sigma": 2.}}
    stress = {"schema": "sensor/1", "name": "stress", "gnss": {"latency_s": .5}, "rpm": {"sigma": 40.}}
    variant = make_subsystem_group_variant(base, stress, ["gnss", "rpm"])
    assert variant["gnss"]["latency_s"] == .5
    assert variant["rpm"]["sigma"] == 40.


def test_failed_rerun_cannot_reuse_a_previous_success(tmp_path, monkeypatch):
    old = tmp_path/"runs"/"truth"/"GSLQR"/"arena_previous"
    old.mkdir(parents=True)
    (old/"trials.jsonl").write_text(json.dumps({"controller": "GSLQR", "passed": True})+"\n")
    seen = []
    def failed(command, **kwargs):
        seen.extend(command)
        return SimpleNamespace(stdout="", stderr="startup failed", returncode=1)
    monkeypatch.setattr(campaign.subprocess, "run", failed)
    result = campaign.run_campaign("configs/arena.json", None, tmp_path, feedback="truth")
    assert result["records"][0]["passed"] is False
    assert result["records"][0]["run_dir"] is None
    assert result["records"][0]["failure_reasons"] == ["no_trial_output"]
    assert seen[seen.index("--feedback")+1] == "truth"
    assert "--sensor-profile" not in seen


def test_truth_campaign_rejects_sensor_sweep(tmp_path):
    with pytest.raises(ValueError, match="sensor variants"):
        campaign.run_campaign("configs/arena.json", None, tmp_path, feedback="truth",
                              sweep="imu_rate", values=[250.])
