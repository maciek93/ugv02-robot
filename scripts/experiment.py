#!/usr/bin/env python3
"""Run one chat-approved experiment and collect selected images plus telemetry.

The approval text must be a real human instruction authorising this exact trial
or the bounded sequence it belongs to. This helper never grants itself consent.
"""
import argparse
import base64
import json
from pathlib import Path
import secrets
import subprocess
import time
import requests

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('action', choices=('continuous_forward','continuous_left','continuous_right','pwm_forward','pwm_left','pwm_right','pwm_spin_right','pwm_spin_left','pulse_forward','pulse_left','pulse_right','forward','left','right'))
parser.add_argument('value', type=float)
parser.add_argument('--label', required=True)
parser.add_argument('--context', choices=('supported','clear-floor'), required=True)
parser.add_argument('--approval', required=True)
parser.add_argument('--fault', choices=('camera','perception'))
parser.add_argument('--wait-ready', type=float, default=15)
parser.add_argument('--start-delay', type=float, default=0)
parser.add_argument('--pulse-gap', type=float, default=0, help='Bench-only pause between pulses for cancellation tests')
args = parser.parse_args()
key = (root/'.runtime/agent-key').read_text().strip()
headers = {'X-Agent-Key': key}
base = 'http://127.0.0.1:5001/agent'
directory = root/'data/experiments'/(time.strftime('%Y%m%d-%H%M%S')+'-'+secrets.token_hex(3))
directory.mkdir(parents=True)

def rpc(path, data=None):
    r = requests.get(base+path,headers=headers,timeout=2) if data is None else requests.post(base+path,headers=headers,json=data,timeout=2)
    result = r.json()
    if not r.ok: raise RuntimeError(result.get('error','Request failed'))
    return result

def pi_rpc(path, data=None):
    keys=json.loads((root/'.runtime/pi-keys.json').read_text())
    r=requests.get('http://127.0.0.1:5002'+path,headers={'X-Robot-Key':keys['agent']},timeout=2) if data is None else requests.post('http://127.0.0.1:5002'+path,headers={'X-Robot-Key':keys['agent']},json=data,timeout=2)
    if not r.ok: raise RuntimeError(r.json().get('error','Pi request failed'))
    return r.json()

def snapshot(name):
    observation = rpc('/observe')
    (directory/(name+'.jpg')).write_bytes(base64.b64decode(observation.pop('jpeg')))
    (directory/(name+'.json')).write_text(json.dumps(observation,indent=2))
    return observation

def inject_fault():
    if args.fault == 'perception':
        import psutil
        pid = int((root/'.runtime/gateway.pid').read_text())
        children = [p for p in psutil.Process(pid).children() if '--multiprocessing-fork' in p.cmdline()]
        if len(children) != 1: raise RuntimeError('Cannot uniquely identify vision worker; fault not injected')
        children[0].terminate()
    else:
        script = '''import pathlib,subprocess,os,signal
pid=int(subprocess.check_output(['systemctl','show','ugv-supervised','-p','MainPID','--value']))
candidates=[]
for p in pathlib.Path('/proc').iterdir():
 if p.name.isdigit():
  try:
   status=(p/'status').read_text()
   parent=int(next(x for x in status.splitlines() if x.startswith('PPid:')).split()[1])
   if parent==pid and b'--multiprocessing-fork' in (p/'cmdline').read_bytes(): candidates.append(int(p.name))
  except (OSError,StopIteration): pass
assert len(candidates)==1,'Cannot uniquely identify camera worker'
os.kill(candidates[0],signal.SIGTERM)
print('Camera fault injected')
'''
        subprocess.run(['ssh','-S',str(root/'.runtime/ssh-control'),'-o','BatchMode=yes',
                        'ugvpi@ugvpi.local','python3 -'],input=script,text=True,check=True)

trial = {'label':args.label,'physical_context':args.context,'approval':args.approval,
         'action':args.action,'value':args.value,'fault':args.fault,
         'started':time.strftime('%Y-%m-%dT%H:%M:%S%z'),'status':'running'}
configuration=root/'data/active-configuration.json'
if configuration.exists(): trial['configuration']=json.loads(configuration.read_text())
samples = []
executed = False
faulted = False
try:
    deadline = time.monotonic()+args.wait_ready
    while True:
        health = rpc('/status')
        if health['state']!='active': raise RuntimeError('Start an observation session first')
        if args.context=='supported' and health['mode']!='bench': raise RuntimeError('Supported trials require bench mode')
        if args.context=='clear-floor' and health['mode']=='bench': raise RuntimeError('Floor trials must not use bench mode')
        if args.context=='clear-floor' and not trial.get('configuration',{}).get('camera_pose_confirmed'):
            raise RuntimeError('Record and confirm the fixed floor camera configuration first')
        if args.fault and health['mode']!='bench': raise RuntimeError('Fault injection is supported-wheel only')
        if args.pulse_gap and (health['mode']!='bench' or not 0 < args.pulse_gap <= 5): raise RuntimeError('Pulse gap is bench-only and at most five seconds')
        if not health['blocking_reason']: break
        if time.monotonic()>deadline: raise RuntimeError(health['blocking_reason'])
        time.sleep(.1)
    trial['before'] = snapshot('before')
    if args.start_delay:
        if not 0 <= args.start_delay <= 10: raise ValueError('Start delay must be at most ten seconds')
        print('Test ready; waiting before motor commands.',flush=True)
        time.sleep(args.start_delay)
    proposal = rpc('/propose',{'action':args.action,'value':args.value})
    trial['proposal'] = proposal
    subprocess.run([str(root/'.venv/bin/python'),str(root/'scripts/chat_control.py'),'approve',
                    proposal['id'],'--human-instruction',args.approval],check=True,stdout=subprocess.DEVNULL)
    (pi_rpc if args.pulse_gap or args.action.startswith(('continuous_', 'pwm_')) else rpc)('/execute',{'proposal_id':proposal['id']})
    executed=True
    print(json.dumps({'experiment':str(directory),'proposal_id':proposal['id'],'started':True}),flush=True)
    expiry=time.monotonic()+125
    last = None
    next_pulse_at = time.monotonic()
    while time.monotonic()<expiry:
        health=rpc('/status')
        samples.append({'mac_monotonic':time.monotonic(), 'state':health})
        if args.action.startswith(('continuous_', 'pwm_')) and health['execution']:
            try:
                pi_rpc('/renew',{'proposal_id':proposal['id']})
            except RuntimeError as error:
                if str(error)=='No matching approved continuous trial':
                    ended = rpc('/status')
                    if not ended['moving'] and ended['last_reason']=='continuous trial completed':
                        samples.append({'mac_monotonic':time.monotonic(), 'state':ended})
                        break
                if str(error) not in ('A fresh observation is required between renewals',
                                     'Observe after Stop before moving again'):
                    raise
        if args.pulse_gap and health['execution'] and not health['moving'] and time.monotonic()>=next_pulse_at:
            result=pi_rpc('/pulse',{'proposal_id':proposal['id']})
            next_pulse_at=time.monotonic()+args.pulse_gap+.25
        if args.fault and not faulted and health['moving']:
            inject_fault()
            faulted=True
        if not health['execution'] and not health['moving']:
            last=health
            break
        time.sleep(.025)
    else: raise RuntimeError('Trial time budget exceeded')
    time.sleep(.3)
    last=rpc('/status')
    trial['final']=last
    trial['software_stopped']=not last['moving'] and not last['execution']
    trial['fault_injected']=faulted
    trial['status']='completed' if trial['software_stopped'] else 'failed'
    trial['physical_result']='awaiting human observation'
    try: trial['after']=snapshot('after')
    except Exception as error: trial['after_unavailable']=str(error)
except Exception as error:
    trial.update(status='failed',error=str(error))
finally:
    if executed:
        try: rpc('/stop',{})
        except Exception as error: trial['stop_unreachable']=str(error)
    (directory/'trial.json').write_text(json.dumps(trial,indent=2))
    (directory/'telemetry.json').write_text(json.dumps(samples,indent=2))
    print(json.dumps({'experiment':str(directory),'status':trial['status'],
                      'software_stopped':trial.get('software_stopped'),'error':trial.get('error')},indent=2))
raise SystemExit(0 if trial['status']=='completed' else 1)
