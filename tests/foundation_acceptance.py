#!/usr/bin/env python3
"""Public config/API test of reset identity, dynamics and recorded envelopes."""
import json
import os
from pathlib import Path
import subprocess
import time
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
STATE=Path(os.environ['P1SIM_STATE_DIR'])
STATE.mkdir(parents=True,exist_ok=True)
CLI=[str(ROOT/'p1sim'),'dev']
PORT=os.environ.get('P1SIM_STATUS_PORT','8765')
result={}
def run(*args):
    p=subprocess.run(CLI+list(args),capture_output=True,text=True,timeout=180)
    result[' '.join(args)]=dict(exit=p.returncode,stdout=p.stdout[-2000:],stderr=p.stderr[-1000:])
    if p.returncode:
        raise RuntimeError('Failed '+' '.join(args))
def status():
    return json.load(urllib.request.urlopen('http://127.0.0.1:'+PORT+'/status'))
def post(path,payload):
    request=urllib.request.Request('http://127.0.0.1:'+PORT+'/'+path,data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
    return json.load(urllib.request.urlopen(request))
try:
    run('up','--config',str(ROOT/'configs/extensions.json'))
    first=status()
    post('record/start',{'name':'foundation'})
    time.sleep(2)  # Allow the bag recorder's reliable receipt subscription to discover the publisher.
    run('run','--','python3','/opt/p1sim/examples/foundation_check.py')
    assert post('record/stop',{})['recording_exit'] in (0,-2)
    run('run','--','python3','-c', '''import json,rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
r=rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri='/data/bags/foundation',storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
counts={}
while r.has_next():
 topic,data,_=r.read_next()
 if topic in ('/p1sim/state','/p1sim/camera_timing','/p1sim/target_state','/p1sim/command_received','/p1sim/gimbal_applied','/p1sim/visibility'):
  value=json.loads(deserialize_message(data,String).data)
  assert value['schema_version']==1 and value['epoch']==0 and value['seed']==42
  assert value['run_id'] and value['config_digest'] and value['source']['runtime_checksums']
  counts[topic]=counts.get(topic,0)+1
assert len(counts)==6, counts
open('/data/bag-contract.json','w').write(json.dumps(dict(passed=True,counts=counts),indent=2))''')
    run('reset')
    second=status()
    assert second['ready']
    for key in ('run_id','run_key','seed','config_digest','plugins'):
        assert first['foundation'][key]==second['foundation'][key],key
    assert second['foundation']['epoch']==first['foundation']['epoch']+1
    result['reset']=dict(passed=True,before=first['foundation'],after=second['foundation'])
    result['passed']=True
except Exception as error:
    result.update(passed=False,error=str(error))
finally:
    run('down')
    (STATE/'acceptance.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(passed=result['passed'])))
raise SystemExit(0 if result['passed'] else 1)
