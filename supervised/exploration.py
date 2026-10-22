"""Mac mission orchestration, persistent evidence and a separate 20 Hz worker."""
import base64
from collections import deque
import json
import math
from pathlib import Path
import secrets
import threading
import time
from datetime import datetime, timezone

from camera import atomic_json
from explore_profile import digest


def timestamp():return datetime.now(timezone.utc).isoformat()


class Journal:
    def __init__(self,root,mission,profile):
        self.root=Path(root);self.directory=self.root/'data/missions'/mission['id']
        self.directory.mkdir(parents=True,exist_ok=False)
        self.manifest={'version':1,'mission':mission,'profile_hash':digest(profile),'models':profile['models'],
                       'started':timestamp(),'status':'active','room':mission['start_room'],
                       'checkpoints':[],'connections':[],'doorways':[],'incomplete':[]}
        self.ring=deque(maxlen=10);self.lock=threading.RLock()
        self.save()

    def save(self):atomic_json(self.directory/'mission.json',self.manifest)

    def event(self,event,**data):
        with self.lock:
            with (self.directory/'events.jsonl').open('a') as f:f.write(json.dumps({'time':timestamp(),'event':event,**data},allow_nan=False)+'\n')

    def checkpoint(self,nav,room,connection=None,action="forward"):
        with self.lock:
            checkpoint_id=secrets.token_hex(8)
            prefix=self.directory/checkpoint_id
            prefix.with_suffix('.jpg').write_bytes(base64.b64decode(nav['jpeg']))
            if nav.get('annotated_jpeg'):prefix.with_suffix('.annotated.jpg').write_bytes(base64.b64decode(nav['annotated_jpeg']))
            info={k:v for k,v in nav.items() if k not in ('jpeg','annotated_jpeg','received_mac')}
            atomic_json(prefix.with_suffix('.json'),info)
            record={'id':checkpoint_id,'room':room,'reference':prefix.with_suffix('.jpg').name,
                    'pose':nav['pose'],'uncertainty_m':nav['uncertainty_m'],'frame_id':nav['frame_id'],'description':''}
            previous=self.manifest['checkpoints'][-1]['id'] if self.manifest['checkpoints'] else None
            self.manifest['checkpoints'].append(record);self.manifest['room']=room
            if previous:self.manifest['connections'].append({'from':previous,'to':checkpoint_id,'direction':'rotation' if action in ('left','right') else 'observed-forward','action':action,'doorway':connection})
            self.save();self.event('checkpoint',**record);return record

    def diagnostic(self,nav):
        if not self.ring or nav.get('frame_id')!=self.ring[-1].get('frame_id'):self.ring.append(nav)

    def failure(self,reason):
        with self.lock:
            self.manifest['incomplete'].append(reason)
            directory=self.directory/'failure';directory.mkdir(exist_ok=True)
            for i,n in enumerate(self.ring):
                if n.get('jpeg'):(directory/f'{i:02}.jpg').write_bytes(base64.b64decode(n['jpeg']))
                atomic_json(directory/f'{i:02}.json',{k:v for k,v in n.items() if k not in ('jpeg','annotated_jpeg')})
            self.save()

    def finish(self,outcome):
        with self.lock:
            self.manifest.update(status=outcome,finished=timestamp());self.save()
            lines=['# Camera-only survey', '',f"Outcome: **{outcome}**",f"Ending room label: {self.manifest['room']}",
                   'Approximate local poses only; unknown regions and reverse routes remain unverified.','', '## Visited checkpoints','']
            for c in self.manifest['checkpoints']:
                lines.extend([f"- {c['id']} — {c['room']}: {c['description'] or 'No semantic annotation'}; uncertainty {c['uncertainty_m']:.3f} m.",f"  [Reference image]({c['reference']})"])
            lines+=['','## Traversed connections','']+[f"- {c['from']} → {c['to']}; doorway: {c['doorway'] or 'none recorded'}. Return direction unverified." for c in self.manifest['connections']]
            lines+=['','## Observed doorways / obstacles / uncertainty','',json.dumps(self.manifest['doorways'],indent=2),'']
            lines += [f'- {x}' for x in self.manifest['incomplete']]
            (self.directory/'report.md').write_text('\n'.join(lines)+'\n')
            # Persistent references are knowledge only. No authority or clear map is restored.
            knowledge=self.root/'data/checkpoints';knowledge.mkdir(exist_ok=True)
            atomic_json(knowledge/(self.manifest['mission']['id']+'.json'),{'mission':self.manifest['mission']['id'],
                'directory':str(self.directory),'checkpoints':self.manifest['checkpoints'],'connections':self.manifest['connections']})
            return str((self.directory/'report.md').resolve())


def revisit_candidates(rgb_jpeg,root):
    import cv2
    import numpy as np
    image=cv2.imdecode(np.frombuffer(base64.b64decode(rgb_jpeg),np.uint8),0)
    orb=cv2.ORB_create(nfeatures=800);k,d=orb.detectAndCompute(image,None)
    if d is None:return {'state':'unknown','matches':[]}
    matches=[]
    for path in sorted((Path(root)/'data/checkpoints').glob('*.json')):
        bundle=json.loads(path.read_text())
        for record in bundle['checkpoints']:
            reference=cv2.imread(str(Path(bundle['directory'])/record['reference']),0)
            if reference is None:continue
            kr,dr=orb.detectAndCompute(reference,None)
            if dr is None:continue
            pairs=cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(d,dr,k=2)
            good=[pair[0] for pair in pairs if len(pair)==2 and pair[0].distance<.7*pair[1].distance]
            if len(good)<20:continue
            src=np.float32([k[m.queryIdx].pt for m in good]);dst=np.float32([kr[m.trainIdx].pt for m in good])
            H,inliers=cv2.findHomography(src,dst,cv2.RANSAC,3)
            if H is not None and inliers.sum()>=20 and inliers.mean()>=.7:
                matches.append({'checkpoint':record['id'],'room':record['room'],'inliers':int(inliers.sum())})
    return {'state':'ambiguous' if len(matches)>1 else 'suggested' if matches else 'unknown','matches':matches}


class Explorer:
    def __init__(self,gateway,root,runtime):
        self.gateway=gateway;self.root=Path(root);self.runtime=Path(runtime)
        self.lock=threading.RLock();self.cancel=threading.Event();self.thread=None
        self.mission=None;self.profile=None;self.journal=None;self.offered={};self.last=None
        for path in (self.root/'data/missions').glob('*/mission.json'):
            saved=json.loads(path.read_text())
            if saved.get('status')=='active':
                j=Journal.__new__(Journal);j.root=self.root;j.directory=path.parent;j.manifest=saved
                j.lock=threading.RLock();j.ring=deque(maxlen=10)
                j.failure('Gateway restarted; location and movement authority not restored')
                j.finish('interrupted')

    def read_nav(self):
        try:n=json.loads((self.runtime/'navigation.json').read_text())
        except (OSError,ValueError):raise PermissionError('Geometry unavailable')
        if n.get('session')!=self.gateway.session or time.monotonic()-n.get('received_mac',0)>.35:
            raise PermissionError('Geometry stale or from another session')
        if n.get('tracking')!='good' or n.get('error'):raise PermissionError(n.get('error') or 'Visual tracking lost')
        if self.profile and (n.get('configuration_id')!=self.profile['configuration_id'] or n.get('models')!=self.profile['models']):
            raise PermissionError('Geometry/model profile changed')
        return n

    def identity(self,goal_id=None):
        result={'mission_id':self.mission['id'],'session':self.mission['session'],'configuration_id':self.mission['configuration_id']}
        if goal_id:result['goal_id']=goal_id
        return result

    def start(self,mission_id):
        with self.lock:
            if self.thread and self.thread.is_alive():raise PermissionError('Navigation worker busy')
            h=self.gateway.rpc('/health');m=h['mission']['active']
            if not m or m['id']!=mission_id:raise PermissionError('Mission must be approved through chat first')
            if self.journal and self.journal.manifest['status']=='active':raise PermissionError('A mission is already attached')
            self.mission=m;self.profile=h['explore_profile'];self.cancel.clear();self.offered={}
            n=self.read_nav()
            self.journal=Journal(self.root,m,self.profile);self.journal.checkpoint(n,m['start_room'])
            self.last={'outcome':'ready','revisits':revisit_candidates(n['jpeg'],self.root)}
            return self.status()

    def status(self):
        return {'mission':self.mission,'running':bool(self.thread and self.thread.is_alive()),'last':self.last,
                'directory':str(self.journal.directory) if self.journal else None}

    def candidates(self):
        with self.lock:
            if not self.mission or not self.journal:raise PermissionError('Start an approved mission')
            if self.thread and self.thread.is_alive():raise PermissionError('Wait for the active leg')
            h=self.gateway.rpc('/health')
            if not h['mission']['active'] or h['mission']['active']['id']!=self.mission['id']:raise PermissionError('Mission ended; fresh approval required')
            n=self.read_nav()
            if n['captured']<=h.get('last_stop_captured',-1):raise PermissionError('Wait for stationary observation')
            snapshot=secrets.token_hex(8);self.offered={}
            for i,c in enumerate(n['candidates']):
                key=f'{snapshot}-{i+1}'
                self.offered[key]={**c,'id':key,'frame_id':n['frame_id'],'created_mac':time.monotonic(), 'pose':n['pose']}
            path=self.journal.directory/'candidates.jpg';path.write_bytes(base64.b64decode(n['annotated_jpeg']))
            return {'candidates':list(self.offered.values()),'image':str(path.resolve()),'frame_id':n['frame_id'],
                    'room':self.journal.manifest['room'],'blocking_reason':None if self.offered else 'No validated swept path; unknown clearance remains blocked'}

    def check_boundary(self, goal, doorway_id):
        if goal['action'] != 'forward':
            return
        checkpoint = self.journal.manifest['checkpoints'][-1]['id']
        reach = goal['value'] + self.profile['length_m']/2 + max(self.profile['braking_m'], self.profile['fallback_stop_m']) + self.profile['margin_m']
        for observed in self.journal.manifest['doorways']:
            if observed.get('checkpoint_id') == checkpoint and reach >= observed['distance_m']:
                if observed['id'] != doorway_id:
                    raise PermissionError('Known doorway boundary requires an approved explicit crossing; choose a shorter approach')

    def choose(self,request):
        with self.lock:
            if self.thread and self.thread.is_alive():raise PermissionError('A leg is already running')
            if set(request)-{'candidate_id','doorway_id'}:raise ValueError('Only candidate and optional doorway ID accepted')
            goal=self.offered.get(request.get('candidate_id'))
            if not goal or time.monotonic()-goal['created_mac']>30:raise PermissionError('Expired or already consumed candidate')
            self.check_boundary(goal, request.get('doorway_id'))
            doorway=None
            if request.get('doorway_id'):
                doorway=next((x for x in self.mission['doorways'] if x['id']==request['doorway_id'] and x['from_room']==self.journal.manifest['room']),None)
                if not doorway:raise PermissionError('Doorway outside approved directed scope')
                observed=next((x for x in self.journal.manifest['doorways'] if x.get('id')==doorway['id']),None)
                if not observed or observed.get('surface')!=self.profile['surface'] or observed.get('uncertain',True) or observed.get('checkpoint_id')!=self.journal.manifest['checkpoints'][-1]['id']:
                    raise PermissionError('Doorway or new surface is uncertain; clarify before crossing')
                if goal['action']!='forward':raise ValueError('Doorway crossing requires a forward leg')
                if goal['value'] < observed['distance_m'] + self.profile['length_m']/2 + .03:
                    raise PermissionError('This leg only approaches the doorway; do not record a crossing')
            n=self.read_nav()
            from navigation import LocalMap, wrapped
            if math.dist(n['pose'][:2],goal['pose'][:2])>.02 or abs(wrapped(n['pose'][2]-goal['pose'][2]))>.03:raise PermissionError('Pose changed; request candidates again')
            if not LocalMap.restore(self.profile,n['map']).clear(n['pose'],goal['action'],goal['value'],n['captured'],n['uncertainty_m']):
                raise PermissionError('Candidate no longer has clearance')
            data={**self.identity(goal['id']),'frame_id':n['frame_id'],'action':goal['action'],'value':goal['value']}
            self.gateway.rpc('/navigation/begin',data,role='navigation')
            self.offered={};self.cancel.clear()
            self.thread=threading.Thread(target=self.drive,args=(goal,n,doorway),daemon=True,name='navigation-20hz')
            self.thread.start();return {'goal_id':goal['id'],'outcome':'running'}

    def drive(self,goal,start,doorway):
        from navigation import LegController, LocalMap
        controller=LegController(goal,start['pose'],time.monotonic());seq=0;outcome='cancelled';reason=None
        tick=time.monotonic();last_frame=None;current=start
        try:
            while not self.cancel.is_set():
                now=time.monotonic()
                if now-tick>.1:raise PermissionError('Navigation missed its control deadline')
                current=self.read_nav();self.journal.diagnostic(current)
                outcome,command,remaining=controller.step(current['pose'],now)
                if outcome:break
                if current['frame_id']!=last_frame:
                    if not LocalMap.restore(self.profile,current['map']).clear(current['pose'],goal['action'],remaining,current['captured'],current['uncertainty_m']):
                        outcome='blocked';reason='Swept footprint no longer clear';break
                    last_frame=current['frame_id']
                data={**self.identity(goal['id']),'frame_id':current['frame_id'],'seq':seq,'left':command[0],'right':command[1]}
                self.gateway.rpc('/navigation/renew',data,role='navigation');seq+=1
                self.journal.event('control',goal_id=goal['id'],seq=seq,frame_id=current['frame_id'],pose=current['pose'],wheels=command)
                tick+=.05
                if self.cancel.wait(max(0,tick-time.monotonic())):break
            if self.cancel.is_set():outcome='cancelled'
            self.gateway.rpc('/navigation/finish',{**self.identity(goal['id']),'outcome':outcome},role='navigation')
            if outcome=='reached':
                # Require a post-stop camera frame before claiming the checkpoint.
                stopped=self.gateway.rpc('/health')['last_stop_captured'];deadline=time.monotonic()+1
                while time.monotonic()<deadline:
                    if self.cancel.wait(.05):raise PermissionError('Cancelled during checkpoint capture')
                    current=self.read_nav()
                    if current['captured']>stopped:break
                else:raise PermissionError('No fresh stopped checkpoint')
                h=self.gateway.rpc('/health')
                if h['moving'] or not h['mission']['active']:raise PermissionError('Mission interrupted at checkpoint')
                room=doorway['to_room'] if doorway else self.journal.manifest['room']
                cp=self.journal.checkpoint(current,room,doorway['id'] if doorway else None,goal['action'])
                self.last={'outcome':'reached','checkpoint':cp,'goal_id':goal['id']}
            else:
                self.journal.failure(reason or outcome);self.journal.finish(outcome)
        except Exception as e:
            reason=str(e);outcome='budget_exhausted' if 'budget_exhausted' in reason else 'tracking_lost'
            try:self.gateway.rpc('/navigation/stop',{},role='navigation')
            except Exception:pass
            self.journal.failure(reason);self.journal.finish(outcome)
        finally:
            if outcome!='reached':self.last={'outcome':outcome,'reason':reason,'goal_id':goal['id']}
            self.journal.event('leg_finished',**self.last)

    def annotate(self,data):
        if not self.journal or self.thread and self.thread.is_alive():raise PermissionError('Annotate a stopped checkpoint')
        if set(data)-{'description','doorway'}:raise ValueError('Unexpected annotation fields')
        with self.journal.lock:
            if 'description' in data:
                if not isinstance(data['description'],str) or len(data['description'])>2000:raise ValueError('Invalid description')
                self.journal.manifest['checkpoints'][-1]['description']=data['description']
            if 'doorway' in data:
                d=data['doorway']
                if not isinstance(d,dict) or set(d)!={'id','surface','uncertain','notes','distance_m'} or type(d['uncertain']) is not bool:raise ValueError('Doorway annotation requires id, surface, uncertain, notes, distance_m')
                from explore_profile import number
                number(d['distance_m'],.01,2,'doorway distance')
                d={**d,'checkpoint_id':self.journal.manifest['checkpoints'][-1]['id']}
                if not isinstance(d['id'],str) or not d['id'] or len(d['id'])>80:raise ValueError('Invalid doorway ID')
                self.journal.manifest['doorways']=[x for x in self.journal.manifest['doorways'] if x['id']!=d['id']]+[d]
            self.journal.save();return {'ok':True}

    def stop(self,reason='cancelled'):
        self.cancel.set();self.offered={}
        if self.thread and self.thread is not threading.current_thread():self.thread.join(.5)
        if self.journal and self.journal.manifest['status']=='active':
            if reason!='completed':self.journal.failure(reason)
            self.last={'outcome':reason,'report':self.journal.finish(reason)}
        return self.status()
