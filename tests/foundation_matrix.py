#!/usr/bin/env python3
"""Additional scheduler-facing trajectory, external-plugin and failure cases."""
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
OUTPUT=Path(os.environ['P1SIM_MATRIX_OUTPUT'])
OUTPUT.mkdir(parents=True,exist_ok=False)
CLI=[str(ROOT/'p1sim'),'dev','batch']
results={}
for mode in ('linear','accel'):
    config=dict(seed=7,duration_seconds=2,plugins={'target':{'entry':'p1sim_dev.builtins:Trajectory',
        'config':dict(mode=mode,velocity_m_s=[0,.05,0],acceleration_m_s2=[0,.02,0])}})
    path=OUTPUT/(mode+'.json');path.write_text(json.dumps(config))
    p=subprocess.run(CLI+['--config',str(path),'--output',str(OUTPUT/mode)],capture_output=True,text=True,timeout=180)
    assert p.returncode==0,(p.stdout,p.stderr)
    status=json.loads((OUTPUT/mode/'final.json').read_text())
    initial=json.loads((OUTPUT/mode/'ready.json').read_text())
    final_targets=status['foundation']['targets']
    assert any(final_targets[n]['applied']['position_m']!=v['applied']['position_m'] for n,v in initial['foundation']['targets'].items() if n in final_targets and 'applied' in v)
    world=ET.parse(ROOT/'worlds/developer.sdf').getroot().find('world')
    for name, entry in final_targets.items():
        if 'applied' not in entry:
            continue
        applied=entry['applied'];t=applied['source_time_ns']/1e9
        initial_y=float(world.find(f"model[@name='{name}']/pose").text.split()[1])
        expected=initial_y+.05*t+(.5*.02*t*t if mode=='accel' else 0)
        assert abs(applied['position_m'][1]-expected)<1e-9
        assert entry['application_time_ns']>=applied['source_time_ns']
    results[mode]=dict(passed=True,targets=final_targets,result=json.loads((OUTPUT/mode/'result.json').read_text()))
# A separately mounted module, not a switch in the runtime or preinstalled package.
plugins=OUTPUT/'plugins';plugins.mkdir()
(plugins/'user_models.py').write_text('''from p1sim_dev.builtins import Timing, World, Trajectory
class MyTiming(Timing):
    version = '1.2.3'
class MyWorld(World):
    version = '1.2.3'
class WrongAPI(Trajectory):
    api_version = 2
''')
config=dict(seed=9,duration_seconds=1,workspace=dict(path=str(plugins),plugin_paths=['/user_ws']),
 plugins={'timing':dict(entry='user_models:MyTiming',config=dict(delay_seconds=.02)),
          'world':dict(entry='user_models:MyWorld',config={})})
path=OUTPUT/'external.json';path.write_text(json.dumps(config))
p=subprocess.run(CLI+['--config',str(path),'--output',str(OUTPUT/'external')],capture_output=True,text=True,timeout=180)
assert p.returncode==0,(p.stdout,p.stderr)
status=json.loads((OUTPUT/'external'/'final.json').read_text())
assert status['foundation']['plugins']['timing']['version']=='1.2.3'
assert status['foundation']['plugins']['world']['entry']=='user_models:MyWorld'
assert status['foundation']['sensor_effect']['delay_seconds']==.02
results['external']=dict(passed=True,identity=status['foundation'])
config['plugins']['target']=dict(entry='user_models:WrongAPI',config={})
path=OUTPUT/'incompatible.json';path.write_text(json.dumps(config))
p=subprocess.run(CLI+['--config',str(path),'--output',str(OUTPUT/'incompatible')],capture_output=True,text=True,timeout=180)
result=json.loads((OUTPUT/'incompatible'/'result.json').read_text())
assert p.returncode!=0 and result['exit_status']=='FAILED' and result['clean_teardown']
assert not (OUTPUT/'incompatible'/'runtime'/'gazebo.log').exists(), 'Bad API started Gazebo'
assert 'Incompatible plugin' in (OUTPUT/'incompatible'/'batch.log').read_text()
results['incompatible']=dict(passed=True,result=result,rejected_before_gazebo=True)
path=OUTPUT/'external-stop.json';path.write_text(json.dumps(dict(duration_seconds=None)))
stopfile=OUTPUT/'stop'
p=subprocess.Popen(CLI+['--config',str(path),'--stop-file',str(stopfile),'--output',str(OUTPUT/'external-stop')],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
try:
    deadline=time.monotonic()+120
    while not (OUTPUT/'external-stop'/'ready.json').exists() and time.monotonic()<deadline:
        assert p.poll() is None, p.communicate()
        time.sleep(.2)
    assert (OUTPUT/'external-stop'/'ready.json').exists()
    stopfile.touch()
    stdout,stderr=p.communicate(timeout=40)
    assert p.returncode==0,(stdout,stderr)
finally:
    if p.poll() is None:
        p.terminate();p.wait(timeout=40)
results['external_stop']=dict(passed=True,result=json.loads((OUTPUT/'external-stop'/'result.json').read_text()))
p=subprocess.Popen(CLI+['--config',str(path),'--stop-file',str(OUTPUT/'never-stop'),'--output',str(OUTPUT/'interrupted')],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
try:
    deadline=time.monotonic()+15
    while not (OUTPUT/'interrupted'/'runtime'/'run.json').exists() and time.monotonic()<deadline:
        assert p.poll() is None
        time.sleep(.02)
    assert (OUTPUT/'interrupted'/'runtime'/'run.json').exists()
    time.sleep(.5)
    p.terminate()
    stdout,stderr=p.communicate(timeout=50)
    result=json.loads((OUTPUT/'interrupted'/'result.json').read_text())
    assert p.returncode!=0 and result['exit_status']=='INTERRUPTED' and result['clean_teardown'],(stdout,stderr,result)
finally:
    if p.poll() is None:
        p.terminate();p.wait(timeout=50)
results['interrupted_startup']=dict(passed=True,result=result)
(OUTPUT/'matrix-result.json').write_text(json.dumps(dict(passed=True,cases=results),indent=2)+'\n')
print(json.dumps(dict(passed=True,cases=list(results))))
