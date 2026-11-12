#!/usr/bin/env python3
"""External scheduler-style caller. Only subprocess + public JSON artifacts."""
import argparse
import json
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--cli', default='./p1sim')
parser.add_argument('--config', default='configs/extensions.json')
parser.add_argument('--output', required=True)
args = parser.parse_args()
output = Path(args.output).resolve()
output.mkdir(parents=True, exist_ok=False)
results = []
for index, seed in enumerate((42, 42, 43)):
    destination = output/str(index)
    subprocess.run([str(Path(args.cli).resolve()), 'dev', 'batch', '--config', args.config,
                    '--seed', str(seed), '--output', str(destination)], check=True)
    result = json.loads((destination/'result.json').read_text())
    ready = json.loads((destination/'ready.json').read_text())
    final = json.loads((destination/'final.json').read_text())
    assert result['exit_status'] == 'COMPLETED' and result['clean_teardown']
    assert ready['foundation']['seed'] == seed and final['foundation']['epoch'] == 0
    assert final['foundation']['targets'] and final['messages']['image'] > 0
    assert final['foundation']['sensor_effect']['delay_seconds'] > 0
    assert final['foundation']['plugins']['sensor']['entry'] == 'examples.visibility_plugin:PeriodicVisibility'
    results.append(dict(result=result, identity=ready['foundation'], message_counts=final['messages']))
assert results[0]['result']['run_id'] != results[1]['result']['run_id']
assert results[0]['result']['run_key'] == results[1]['result']['run_key']
assert results[0]['result']['run_key'] != results[2]['result']['run_key']
(output/'caller-result.json').write_text(json.dumps(dict(passed=True, runs=results), indent=2)+'\n')
print(json.dumps(dict(passed=True, serial_runs=len(results))))
