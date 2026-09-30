import json
import threading
from pathlib import Path
import pytest

from scripts import sensor_preparation as prep, sensor_preparation_parallel as parallel
from scripts.test_sensor_preparation import fake_runner


def test_workers_overlap_with_unique_paths_and_stable_records(tmp_path):
    prep.save_plan(tmp_path, prep.make_plan(seeds=[3]))
    barrier = threading.Barrier(2, timeout=10)
    paths = set()
    mutex = threading.Lock()
    def runner(*args, **kwargs):
        with mutex:
            assert args[2] not in paths
            paths.add(args[2])
        barrier.wait()
        return fake_runner(*args, **kwargs)
    result = parallel.execute(tmp_path, runner)
    assert len(result['records']) == len(paths) == 18
    assert not result['complete'] and all(r['campaign_timed_out'] for r in result['records'])
    assert parallel.execute(tmp_path, lambda *a, **k: pytest.fail('completed failure retried')) == result
    plan = prep.checked_plan(tmp_path)
    ids = {r['trial_id'] for r in result['records']}
    assert [r['trial_id'] for r in result['records']] == [t['trial_id'] for t in plan['tasks'] if t['trial_id'] in ids]
    assert not (tmp_path/'run.lock').exists()


def test_broken_pair_saves_both_results_then_stops(tmp_path):
    prep.save_plan(tmp_path, prep.make_plan(seeds=[3]))
    calls = []
    def runner(*args, **kwargs):
        calls.append(kwargs)
        result = fake_runner(*args, **kwargs)
        result['records'][0]['failure_reasons'] = ['no_trial_output']
        return result
    with pytest.raises(RuntimeError, match='no trial output'):
        parallel.execute(tmp_path, runner)
    assert len(calls) == 2
    data = json.loads((tmp_path/'results.json').read_text())
    assert len(data['records']) == 2
    assert not (tmp_path/'run.lock').exists()


def test_conflicting_executor_or_writer_refused(tmp_path):
    prep.save_plan(tmp_path, prep.make_plan(seeds=[3]))
    (tmp_path/'run.lock').write_text('busy')
    with pytest.raises(FileExistsError):
        parallel.execute(tmp_path, fake_runner)
    (tmp_path/'run.lock').unlink()
    parallel.execute(tmp_path, fake_runner)
    contract = tmp_path/'parallel_executor.json'
    doc = json.loads(contract.read_text()); doc['workers'] = 4
    contract.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match='contract changed'):
        parallel.execute(tmp_path, fake_runner)


def test_finished_serial_truth_controls_are_shared_not_rerun(tmp_path):
    prep.save_plan(tmp_path, prep.make_plan(seeds=[3]))
    serial = prep.execute(tmp_path, 'integration', fake_runner)
    calls = []
    def runner(*args, **kwargs):
        calls.append(kwargs)
        assert kwargs['feedback'] == 'sensors'
        return fake_runner(*args, **kwargs)
    final = parallel.execute(tmp_path, runner)
    assert len(calls) == 16 and final['complete']
    assert len(final['records']) == len(serial['records'])+16 == 43
