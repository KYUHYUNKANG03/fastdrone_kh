import json
import pytest
from scripts import sensor_preparation as prep, sensor_preparation_parallel as parallel
from scripts import sensor_preparation_split as split
from scripts.test_sensor_preparation import fake_runner


def prepared(tmp_path):
    parent, child = tmp_path/'parent', tmp_path/'child'
    prep.save_plan(parent, prep.make_plan(seeds=[3]))
    before = prep.execute(parent, 'integration', fake_runner)
    return parent, child, before


def test_snapshot_and_merge_reuse_truth_and_preserve_all_failures(tmp_path):
    parent, child, before = prepared(tmp_path)
    # Reading an atomic checkpoint is allowed while its producer is active.
    (parent/'run.lock').write_text('active parent')
    split.snapshot(parent, child)
    assert json.loads((parent/'results.json').read_text()) == before
    calls = []
    def runner(*a, **kw):
        assert kw['feedback'] == 'sensors'
        calls.append(kw)
        return fake_runner(*a, **kw)
    parallel.execute(child, runner, workers=1)
    assert len(calls) == 16
    with pytest.raises(FileExistsError):
        split.merge(parent, child)
    assert not (child/'run.lock').exists()
    (parent/'run.lock').unlink()
    receipt = split.merge(parent, child)
    after = json.loads((parent/'results.json').read_text())
    assert after['complete'] and len(after['records']) == 43
    assert after['records'][:len(before['records'])] == before['records']
    assert len(receipt['added_trials']) == 16
    assert all(r['campaign_timed_out'] for r in after['records'])
    assert receipt['source_executor']['workers'] == 1


def test_conflicting_duplicates_cannot_replace_the_parent(tmp_path):
    parent, child, before = prepared(tmp_path)
    split.snapshot(parent, child)
    doc = json.loads((child/'results.json').read_text())
    doc['records'][0]['passed'] = True
    (child/'results.json').write_text(json.dumps(doc))
    with pytest.raises(ValueError, match='conflicting duplicate'):
        split.merge(parent, child)
    assert json.loads((parent/'results.json').read_text()) == before
    assert not (parent/'run.lock').exists() and not (child/'run.lock').exists()


def test_different_plans_and_unknown_records_are_refused(tmp_path):
    parent, child, before = prepared(tmp_path)
    prep.save_plan(child, prep.make_plan(seeds=[4]))
    with pytest.raises(ValueError, match='different preparation plans'):
        split.merge(parent, child)
    doc = dict(before, records=before['records']+[dict(before['records'][0],trial_id='unknown')])
    (parent/'results.json').write_text(json.dumps(doc))
    with pytest.raises(ValueError, match='unknown recorded'):
        split.snapshot(parent, tmp_path/'another')
    assert not (tmp_path/'another').exists()


def test_existing_snapshot_is_not_overwritten(tmp_path):
    parent, child, _ = prepared(tmp_path)
    split.snapshot(parent, child)
    with pytest.raises(ValueError, match='new directory'):
        split.snapshot(parent, child)
