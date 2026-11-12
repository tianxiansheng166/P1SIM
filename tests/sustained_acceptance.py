#!/usr/bin/env python3
"""Two clean AutoAim starts through the public CLI, with real bag audits."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
STATE = Path(os.environ['P1SIM_STATE_DIR'])
CLI = [str(ROOT/'p1sim'), 'dev']
PORT = os.environ.get('P1SIM_STATUS_PORT', '8765')
results = {}
children = []


def run(*args, timeout=160):
    p = subprocess.run(CLI+list(args), capture_output=True, text=True, timeout=timeout)
    results[' '.join(args)] = dict(exit=p.returncode, stdout=p.stdout[-6000:], stderr=p.stderr[-3000:])
    if p.returncode:
        raise RuntimeError('Failed: '+' '.join(args))
    return p.stdout


def start(name, args):
    log = open(STATE/(name+'.log'), 'w')
    p = subprocess.Popen(CLI+args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    children.append((p, log))
    return p


def stop(p):
    if p.poll() is None:
        os.killpg(p.pid, signal.SIGINT)
        p.wait(timeout=30)


def status():
    return json.load(urllib.request.urlopen('http://127.0.0.1:'+PORT+'/status', timeout=5))


try:
    STATE.mkdir(parents=True, exist_ok=True)
    for index in (1, 2):
        run('up')
        recorder = start(f'record-{index}', ['record', f'sustained-{index}'])
        time.sleep(2)
        observer = start(f'observer-{index}', ['run', '--', 'python3', '/opt/p1sim/examples/autoaim_check.py',
                         '--timeout', '45', '--output', f'/data/closure-{index}.json'])
        time.sleep(1)
        consumer = start(f'consumer-{index}', ['run', '--', 'ros2', 'launch', 'rm_vision_bringup',
                         'vision_bringup.launch.py', 'camera:=external', 'inference_device:=CPU'])
        first_tracking = None
        deadline = time.monotonic()+50
        while time.monotonic() < deadline:
            time.sleep(.25)
            current = status()
            if current['autoaim']['latest'].get('tracking') and first_tracking is None:
                first_tracking = current['simulation_time_ns']/1e9
            if first_tracking is not None and current['simulation_time_ns']/1e9 >= first_tracking+6:
                break
        else:
            raise RuntimeError(f'No sustained observation window on start {index}')
        (STATE/f'status-{index}.json').write_text(json.dumps(current, indent=2)+'\n')
        (STATE/f'live-{index}.html').write_bytes(urllib.request.urlopen('http://127.0.0.1:'+PORT+'/').read())
        stop(recorder)
        observer.wait(timeout=50)
        if observer.returncode:
            raise RuntimeError('Real loop observer failed')
        run('run', '--', 'python3', '-m', 'examples.autoaim_bag_check', f'/data/bags/sustained-{index}',
            '--output', f'/data/sustained-{index}.json')
        stop(consumer)
        time.sleep(2)
        graph = run('run', '--', 'ros2', 'node', 'list')
        if any('/'+name in graph for name in ('detector', 'rm_tracker', 'rm_planner', 'virtual_serial')):
            raise RuntimeError('Consumer remained after Ctrl-C')
        results[f'consumer_{index}_stopped'] = True
        run('down')
        containers = subprocess.check_output(['docker', 'ps', '-aq', '--filter', 'label=com.docker.compose.project='+os.environ['P1SIM_PROJECT']], text=True).strip()
        networks = subprocess.check_output(['docker', 'network', 'ls', '-q', '--filter', 'label=com.docker.compose.project='+os.environ['P1SIM_PROJECT']], text=True).strip()
        if containers or networks:
            raise RuntimeError('Task resources remained after down')
        results[f'start_{index}_clean'] = True
    results['pass'] = True
except Exception as error:
    results.update(error=str(error), **{'pass': False})
finally:
    for p, log in reversed(children):
        stop(p)
        log.close()
    run('down')
    (STATE/'acceptance.json').write_text(json.dumps(results, indent=2)+'\n')
print(json.dumps(results, indent=2))
raise SystemExit(0 if results['pass'] else 1)
