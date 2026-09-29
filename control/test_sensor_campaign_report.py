from control.sensor_campaign_report import classify


def test_domain_exclusion_is_separate_from_tracking_failure():
    assert classify({"passed": False, "tracking_pass": True, "model_domain_valid": False}) == "model_domain_exclusion"
    assert classify({"passed": False, "tracking_pass": False, "model_domain_valid": True}) == "tracking_failure"


def test_pass_is_prioritized():
    assert classify({"passed": True, "tracking_pass": True, "model_domain_valid": True}) == "pass"


def test_campaign_timeout_is_explicit():
    assert classify({"passed": False, "campaign_timed_out": True}) == "campaign_timeout"
