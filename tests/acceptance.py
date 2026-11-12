#!/usr/bin/env python3
"""Exercise the public CLI and real ROS runtime; keep compact JSON evidence."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import urllib.request

root = Path(__file__).resolve().parents[1]
cli = str(root / 'p1sim')
results = {}

def run(name, *args):
    result = subprocess.run([cli, 'dev', *args], text=True, capture_output=True)
    results[name] = dict(exit=result.returncode, stdout=result.stdout[-8000:], stderr=result.stderr[-2000:])
    if result.returncode:
        raise RuntimeError(name + ' failed')
    return result

try:
    run('doctor', 'doctor')
    run('up', 'up')
    with urllib.request.urlopen('http://127.0.0.1:' + os.environ.get('P1SIM_STATUS_PORT', '8765') + '/', timeout=5) as reply:
        page = reply.read().decode()
        results['live_view'] = {'http_status': reply.status, 'has_live_polling': "fetch('/status')" in page}
    run('smoke', 'run', '--', 'python3', '/opt/p1sim/examples/smoke.py')
    consumer = subprocess.Popen([cli, 'dev', 'run', '--', 'ros2', 'launch', '/opt/p1sim/examples/consumer.launch.py'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        time.sleep(4)
        run('consumer_tf', 'run', '--', 'python3', '/opt/p1sim/examples/consumer_check.py')
        run('consumer_graph', 'run', '--', 'ros2', 'node', 'info', '/developer_vehicle_consumer')
    finally:
        os.killpg(consumer.pid, signal.SIGINT)
        try:
            out, _ = consumer.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(consumer.pid, signal.SIGTERM)
            out, _ = consumer.communicate(timeout=5)
        results['consumer_launch'] = dict(exit=consumer.returncode, output=out[-4000:])
    time.sleep(1)
    graph = run('consumer_after_stop', 'run', '--', 'ros2', 'node', 'list').stdout
    if '/developer_vehicle_consumer' in graph:
        raise RuntimeError('Consumer did not stop after Ctrl-C')
    name = 'acceptance-' + str(int(time.time()))
    recorder = subprocess.Popen([cli, 'dev', 'record', name], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    time.sleep(2)
    run('recorded_smoke', 'run', '--', 'python3', '/opt/p1sim/examples/smoke.py')
    time.sleep(3)
    os.killpg(recorder.pid, signal.SIGINT)
    output, _ = recorder.communicate(timeout=20)
    results['record'] = dict(exit=recorder.returncode, output=output)
    if recorder.returncode:
        raise RuntimeError('record failed')
    run('bag_info', 'run', '--', 'ros2', 'bag', 'info', '/data/bags/' + name)
    run('status', 'status')
    run('reset', 'reset')
    run('reset_smoke', 'run', '--', 'python3', '/opt/p1sim/examples/smoke.py')
    run('down', 'down')
    run('second_up', 'up')
    run('second_smoke', 'run', '--', 'python3', '/opt/p1sim/examples/smoke.py')
    run('final_down', 'down')
    results['passed'] = True
except Exception as error:
    results['passed'] = False
    results['error'] = str(error)
    subprocess.run([cli, 'dev', 'logs'])
    subprocess.run([cli, 'dev', 'down'])
finally:
    destination = Path(sys.argv[1]) if len(sys.argv) > 1 else Path('acceptance.json')
    destination.write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps({'passed': results['passed'], 'result': str(destination)}))
sys.exit(0 if results['passed'] else 1)
