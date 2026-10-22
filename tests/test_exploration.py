import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import time
import unittest
import numpy as np
import cv2

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'supervised'))
from explore_profile import validate_profile, camera_matches
from mission import Mission
from navigation import LocalMap, LegController, FloorTracker
from exploration import Journal, Explorer, revisit_candidates
from guardian import Guardian, create_app
from test_supervised import FakeChassis, FakeCamera


def profile():
    return {'version':1,'kind':'camera-explore','configuration_id':'test','surface':'wood',
        'camera':{'device':'test-camera','identity':'usb-test','mounting_id':'mount-1','focus':0.,'focus_verified':True,'width':640,'height':480,'rotation_degrees':180,
                  'matrix':[[500,0,320],[0,500,240],[0,0,1]],'distortion':[0]*5,'image_to_ground':[[0,-.002,.96],[-.002,0,.64],[0,0,1]]},
        'width_m':.2,'length_m':.3,'track_m':.15,'camera_forward_m':0.,'blind_forward_m':.1,
        'upper_speed_m_s':.05,'upper_turn_rad_s':.5,'fallback_timeout_s':.1,'braking_m':.01,'fallback_stop_m':.02,'margin_m':.02,
        'encoder_m_per_unit':.001,'encoder_signs':[1,1],'motor_polarity':[-1,-1],
        'max_translation_disagreement_m':.02,'max_yaw_disagreement_rad':.08,
        'floor_probability':.95,'near_limit':.8,'score_low':0.,'score_high':1.,'min_sharpness':50,
        'min_features':12,'clearance_max_age_s':5.,'max_reprojection_px':1.5,'max_geometry_error_m':.015,
        'models':{k:{'sha256':'a'*64} for k in ('floor','depth','detector')},
        'validation':{k:{'passed':True,'evidence':['test-only']} for k in ('motion','camera','watchdog','perception','latency')}}


def spec():
    return {'configuration_id':'test','rooms':['a','b'],'doorways':[{'id':'ab','from_room':'a','to_room':'b'}],
            'start_room':'a','starting_clearance_m':1.,'duration_s':600,'travel_m':10}


class MissionTests(unittest.TestCase):
    def setUp(self):self.time=10.;self.m=Mission(lambda:self.time);self.p=profile()
    def approve(self):
        proposal=self.m.propose(spec(),'session',self.p);self.m.approve(proposal['id'],'session',self.p)
        self.identity={'mission_id':proposal['id'],'session':'session','configuration_id':'test','goal_id':'goal'}
    def begin(self,action='forward',value=.1):
        self.approve();self.m.begin({**self.identity,'action':action,'value':value,'frame_id':'f'},'session',self.p)
    def renew(self,seq=0,left=.05,right=.05):return self.m.renew({**self.identity,'seq':seq,'left':left,'right':right},'session',self.p)
    def test_profile_evidence_and_legacy_gate(self):
        with self.assertRaises(ValueError):validate_profile({'upper_speed_m_s':.1})
        for field in self.p['validation']:
            p=copy.deepcopy(self.p);p['validation'][field]['passed']=False
            with self.subTest(field=field),self.assertRaises(PermissionError):validate_profile(p)
        p=copy.deepcopy(self.p);p['camera']['image_to_ground']=[[0]*3]*3
        with self.assertRaises(ValueError):validate_profile(p)
    def test_approval_consumed_once_scope_and_expiry(self):
        self.approve()
        with self.assertRaises(PermissionError):self.m.approve(self.identity['mission_id'],'session',self.p)
        for field,value in [('session','old'),('configuration_id','old'),('mission_id','old')]:
            with self.subTest(field=field),self.assertRaises(PermissionError):self.m.check({**self.identity,field:value},'session',self.p)
        self.time+=601
        with self.assertRaises(PermissionError):self.m.check(self.identity,'session',self.p)
    def test_reservation_counts_new_lease_time_only(self):
        self.begin();self.renew();self.assertAlmostEqual(self.m.active['reserved_m'],.0325)
        self.time+=.05;self.renew(1);self.assertAlmostEqual(self.m.active['reserved_m'],.035)
    def test_missed_renewal_replay_and_reverse(self):
        self.begin();self.renew()
        with self.assertRaises(PermissionError):self.renew()
        self.time+=.11
        with self.assertRaises(PermissionError):self.renew(1)
        self.setUp();self.begin()
        with self.assertRaises(ValueError):self.renew(left=-.01)
    def test_stop_and_goal_reuse(self):
        self.begin();self.renew();self.m.finish_leg('goal','reached')
        with self.assertRaises(PermissionError):self.m.begin({**self.identity,'action':'forward','value':.1},'session',self.p)
        self.m.cancel('Stop')
        with self.assertRaises(PermissionError):self.renew(1)
        self.assertIsNone(Mission(lambda:self.time).active)
    def test_quota_deadline_and_profile_change(self):
        self.begin();self.m.active['travel_m']=.001
        with self.assertRaises(PermissionError):self.renew()
        self.m.active['travel_m']=10;self.p['margin_m']=.1
        with self.assertRaises(PermissionError):self.renew()
    def test_fallback_must_cover_verified_timeout_and_braking(self):
        p=profile();p['fallback_timeout_s']=3.
        with self.assertRaises(ValueError):validate_profile(p)

    def test_nonfinite_and_wrong_turn(self):
        self.begin('left',15)
        with self.assertRaises(ValueError):self.renew()
        with self.assertRaises(ValueError):self.renew(left=float('nan'))
        self.renew(left=-.05,right=.05)


class GeometryTests(unittest.TestCase):
    def test_unknown_and_full_turn_footprint(self):
        p=profile();m=LocalMap(p)
        self.assertFalse(m.clear([0,0,0],'forward',.1,10))
        m.seed(1,10);self.assertTrue(m.clear([0,0,0],'forward',.1,10))
        self.assertTrue(m.clear([0,0,0],'left',15,10))
        m.cells[m.key((-.15,-.15))]=(-1,10,0)
        self.assertFalse(m.clear([0,0,0],'left',15,10))
        self.assertFalse(m.clear([0,0,0],'forward',.1,16))
    def test_depth_unknown_not_free_and_temporal_confirmation(self):
        p=profile();m=LocalMap(p);mask=np.ones((480,640),np.uint8)
        m.update(mask,[0,0,0],10)
        key=m.key((.5,.1));self.assertEqual(m.cells[key][0],0)
        m.update(mask,[0,0,0],10.1);m.update(mask,[0,0,0],10.2)
        self.assertEqual(m.cells[key][0],1)
        m.update(np.zeros_like(mask),[0,0,0],10.3);self.assertEqual(m.cells[key][0],-1)
    def test_braking_margin_and_uncertainty(self):
        p=profile();m=LocalMap(p);m.seed(.6,10)
        self.assertTrue(m.clear([0,0,0],'forward',.1,10))
        self.assertFalse(m.clear([0,0,0],'forward',.1,10,.5))
    def test_leg_arrival_stall_and_lateral_drift(self):
        c=LegController({'action':'forward','value':.25},[0,0,0],0)
        self.assertEqual(c.step([.23,0,0],.5)[0],'reached')
        c=LegController({'action':'forward','value':.25},[0,0,0],0)
        self.assertEqual(c.step([0,0,0],1.1)[0],'tracking_lost')
        self.assertEqual(c.step([.1,.04,0],.3)[0],'tracking_lost')
    def test_turn_direction_and_target(self):
        c=LegController({'action':'left','value':15},[0,0,0],0)
        _,(left,right),_=c.step([0,0,0],.1);self.assertLess(left,0);self.assertGreater(right,0)
        self.assertEqual(c.step([0,0,math.radians(14)],.5)[0],'reached')
    def test_accumulated_wheel_slip_is_not_accepted_as_progress(self):
        p=profile();t=FloorTracker(p);rng=np.random.default_rng(42)
        gray=(rng.random((480,640))*255).astype('uint8');floor=np.ones_like(gray)
        results=[]
        for i in range(9):
            results.append(t.update(gray,floor,{'captured':1+i*.1,'chassis':{'age_s':0,'data':{'odl':i*5,'odr':i*5}}}))
        self.assertTrue(any('Accumulated' in (r['error'] or '') for r in results))
        self.assertEqual(results[-1]['tracking'],'lost')

    def test_floor_tracking_static_low_texture_gap_and_encoder_reset(self):
        p=profile();tracker=FloorTracker(p);rng=np.random.default_rng(7)
        gray=(rng.random((480,640))*255).astype('uint8');floor=np.ones_like(gray)
        def obs(t,od=0):return {'captured':t,'chassis':{'age_s':0,'data':{'odl':od,'odr':od}}}
        self.assertEqual(tracker.update(gray,floor,obs(1))['tracking'],'lost')
        self.assertEqual(tracker.update(gray,floor,obs(1.1))['tracking'],'good')
        self.assertIn('Encoder',tracker.update(gray,floor,obs(1.2,1000))['error'])
        self.assertIn('relocalise',tracker.update(gray,floor,obs(2))['error'])
        tracker=FloorTracker(p);tracker.update(gray,floor,obs(1))
        self.assertIn('gap',tracker.update(gray,floor,obs(2))['error'])
        tracker=FloorTracker(p);flat=np.full_like(gray,127)
        tracker.update(flat,floor,obs(1));self.assertIn('texture',tracker.update(flat,floor,obs(1.1))['error'])


class GuardianExploreTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.time=10.;self.now=lambda:self.time
        self.camera=FakeCamera(Path(self.temp.name),self.now);self.chassis=FakeChassis(self.now);self.chassis.motor_polarity=(-1,-1)
        self.g=Guardian(self.chassis,self.camera,Path(self.temp.name),self.now)
        self.keys={'agent':'agent','operator':'operator','vision':'vision','navigation':'nav'}
        self.client=create_app(self.g,self.keys).test_client();self.g.explore_profile=profile()
        self.post('/operator/session',{'operation':'start','mode':'explore'},'operator');self.fresh()
    def tearDown(self):self.chassis.fail=False;self.g.close();self.temp.cleanup()
    def post(self,path,data,role='agent'):return self.client.post(path,json=data,headers={'X-Robot-Key':self.keys[role]})
    def fresh(self):
        self.time+=.01;self.camera.fresh();self.g.heartbeat_at=self.time
        self.camera.metadata.update(device='test-camera',identity='usb-test',focus=0.,focus_stable=True,width=640,height=480,rotation_degrees=180,sharpness=100,identical_frames=0)
        payload={'frame_id':self.camera.metadata['frame_id'],'captured':self.time,'session':self.g.policy.session,
                 'depth_ready':True,'near_score':.2,'hazard':None,'error':None,'signature':[100]*48,
                 'geometry':{'configuration_id':'test','tracking':'good','error':None}}
        self.assertEqual(self.post('/perception',payload,'vision').status_code,200)
    def begin(self):
        proposal=self.post('/mission/propose',spec()).json
        self.assertEqual(self.post('/operator/mission/approve',{'mission_id':proposal['id']},'operator').status_code,200)
        self.identity={'mission_id':proposal['id'],'session':self.g.policy.session,'configuration_id':'test','goal_id':'goal'}
        self.fresh()
        self.assertEqual(self.post('/navigation/begin',{**self.identity,'frame_id':self.g.vision['frame_id'],'action':'forward','value':.1},'navigation').status_code,200)
    def renew(self,seq=0):
        return self.post('/navigation/renew',{**self.identity,'frame_id':self.g.vision['frame_id'],'seq':seq,'left':.05,'right':.05},'navigation')
    def test_agent_and_empty_key_cannot_drive_navigation(self):
        for role in ('agent','vision'):
            self.assertEqual(self.post('/navigation/begin',{},role).status_code,403)
        self.assertEqual(self.post('/operator/mission/approve',{}).status_code,403)
        self.assertEqual(self.client.post('/navigation/renew',json={}).status_code,403)
    def test_missing_validation_cannot_start(self):
        self.post('/operator/session',{'operation':'pause','mode':'explore'},'operator')
        self.g.explore_profile['validation']['motion']['passed']=False
        self.assertEqual(self.post('/operator/session',{'operation':'start','mode':'explore'},'operator').status_code,423)
    def test_navigation_renew_stop_replay(self):
        self.begin();self.assertEqual(self.renew().status_code,200);self.assertEqual(self.chassis.writes[-1],(.05,.05))
        self.post('/stop',{});self.assertEqual(self.renew(1).status_code,423);self.assertIsNone(self.g.mission.active)
    def test_worker_loss_stops_and_cancels(self):
        self.begin();self.renew();self.time+=.11;time.sleep(.05)
        self.assertEqual(self.chassis.writes[-1],(0,0));self.assertIsNone(self.g.mission.active)
    def test_camera_config_and_geometry_fault_stop(self):
        self.begin();self.renew();self.camera.metadata['focus']=2.;time.sleep(.05)
        self.assertEqual(self.chassis.writes[-1],(0,0));self.assertIsNone(self.g.mission.active)
    def test_navigation_finish_retains_mission_only_on_reached(self):
        self.begin();self.renew()
        r=self.post('/navigation/finish',{**self.identity,'outcome':'reached'},'navigation')
        self.assertEqual(r.status_code,200);self.assertIsNotNone(self.g.mission.active);self.assertFalse(self.g.deadline)
    def test_fault_at_idle_cancels_mission(self):
        self.begin();self.renew();self.post('/navigation/finish',{**self.identity,'outcome':'reached'},'navigation')
        self.g.vision['geometry']['tracking']='lost';time.sleep(.05);self.assertIsNone(self.g.mission.active)


class PersistenceTests(unittest.TestCase):
    def test_report_directed_edges_and_no_authority_restore(self):
        import base64
        with tempfile.TemporaryDirectory() as directory:
            mission={**spec(),'id':'test-mission'};j=Journal(directory,mission,profile())
            nav={'jpeg':base64.b64encode(b'image').decode(),'pose':[0,0,0],'uncertainty_m':.01,'frame_id':'one','map':{}}
            a=j.checkpoint(nav,'a');nav['frame_id']='two';b=j.checkpoint(nav,'b','ab')
            report=Path(j.finish('completed')).read_text()
            self.assertIn(a['id']+' → '+b['id'],report);self.assertIn('Return direction unverified',report)
            knowledge=json.loads(next((Path(directory)/'data/checkpoints').glob('*.json')).read_text())
            self.assertNotIn('active',knowledge);self.assertNotIn('starting_clearance_m',knowledge)
    def test_calibration_pdf_size_and_square_geometry(self):
        from pypdf import PdfReader
        path=Path(__file__).resolve().parents[1]/'output/pdf/checkerboard-20mm.pdf'
        r=PdfReader(path);self.assertEqual(len(r.pages),1)
        self.assertAlmostEqual(float(r.pages[0].mediabox.width),297/25.4*72,places=2)
        self.assertIn('100 mm',r.pages[0].extract_text())


class NavigatorIntegrationTests(unittest.TestCase):
    def test_complete_leg_persists_stopped_checkpoint(self):
        import base64
        class FakeGateway:
            session='session'
            def __init__(self):
                self.calls=[];self.stopped=0;self.active={**spec(),'id':'mission','session':'session'}
            def rpc(self,path,data=None,role='agent'):
                self.calls.append((path,data,role))
                if path=='/health':return {'mission':{'active':self.active},'explore_profile':profile(),'moving':False,'last_stop_captured':self.stopped}
                if path=='/navigation/finish':self.stopped=time.monotonic()
                return {}
        with tempfile.TemporaryDirectory() as directory:
            gateway=FakeGateway();e=Explorer(gateway,directory,directory);count=[0];moving=[False]
            ok,jpeg=cv2.imencode('.jpg',np.full((480,640,3),127,np.uint8));encoded=base64.b64encode(jpeg).decode()
            def nav():
                count[0]+=1
                x=min(.09,max(0,count[0]-3)*.015) if moving[0] else 0.
                now=time.monotonic();m=LocalMap(profile());m.seed(2,now)
                return {'session':'session','frame_id':str(count[0]),'captured':now,'received_mac':now,'configuration_id':'test',
                        'pose':[x,0,0],'uncertainty_m':0.,'tracking':'good','error':None,'map':m.snapshot(),
                        'models':profile()['models'],'candidates':[{'action':'forward','value':.1,'endpoint':[.1,0]}],
                        'jpeg':encoded,'annotated_jpeg':encoded}
            e.read_nav=nav;e.start('mission');candidate=e.candidates()['candidates'][0]['id'];moving[0]=True
            e.choose({'candidate_id':candidate});e.thread.join(3)
            self.assertFalse(e.thread.is_alive());self.assertEqual(e.last['outcome'],'reached',e.last)
            self.assertEqual(len(e.journal.manifest['checkpoints']),2)
            self.assertTrue(any(path=='/navigation/renew' for path,_,_ in gateway.calls))
            with self.assertRaises(PermissionError):e.choose({'candidate_id':candidate})
    def test_stop_cancels_local_worker_even_if_network_fails(self):
        from mac import Gateway
        class Local:
            cancel=__import__('threading').Event()
            def stop(self,reason):self.reason=reason
        gateway=Gateway.__new__(Gateway);gateway.explorer=Local()
        def fail(*a,**kw):
            self.assertTrue(gateway.explorer.cancel.is_set())
            raise OSError('network lost')
        gateway.rpc=fail
        with self.assertRaises(OSError):gateway.stop_motion()
        self.assertEqual(gateway.explorer.reason,'cancelled')
    def test_restart_marks_previous_mission_interrupted(self):
        with tempfile.TemporaryDirectory() as directory:
            j=Journal(directory,{**spec(),'id':'old'},profile())
            Explorer(object(),directory,directory)
            self.assertEqual(json.loads((j.directory/'mission.json').read_text())['status'],'interrupted')
            self.assertTrue((j.directory/'report.md').exists())

class ValidationRegressionTests(unittest.TestCase):
    def test_machine_validation_evidence_must_pass_and_match_configuration(self):
        from explore_profile import verify_recorded_validation, configuration_fingerprint
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'records').mkdir();p=profile()
            for name in p['validation']:
                path=root/'records'/f'{name}.json'
                p['validation'][name]['evidence']=[str(path.relative_to(root))]
                kind={'perception':'held-out-perception','latency':'stationary-navigation-latency'}.get(name,'human-evidence')
                path.write_text(json.dumps({'kind':kind,'passed':True,'profile_fingerprint':configuration_fingerprint(p)}))
            verify_recorded_validation(p,root)
            p['margin_m']+=.01
            with self.assertRaises(PermissionError):verify_recorded_validation(p,root)
            p['margin_m']-=.01
            path=root/'records/perception.json';r=json.loads(path.read_text());r['passed']=False;path.write_text(json.dumps(r))
            with self.assertRaises(PermissionError):verify_recorded_validation(p,root)
    def test_camera_quality_failure_latches_tracking_loss(self):
        from explore_vision import ExploreVision
        class Segmenter:
            def predict(self,rgb,probability):return np.ones(rgb.shape[:2],np.uint8)
        for change in ({'sharpness':0},{'identical_frames':3},{'quality_reason':'Image too dark'}):
            p=profile();engine=ExploreVision.__new__(ExploreVision);engine.p=p;engine.segmenter=Segmenter()
            engine.tracker=FloorTracker(p);engine.map=LocalMap(p);engine.seeded=None
            rng=np.random.default_rng(1);rgb=rng.integers(0,256,(480,640,3),dtype=np.uint8)
            o={**p['camera'],'focus_stable':True,'captured':10.,'sharpness':100.,'chassis':{'age_s':0,'data':{'odl':0,'odr':0}}}
            engine.process(rgb,o);o['captured']+=.1
            self.assertEqual(engine.process(rgb,o)[0]['tracking'],'good')
            o['captured']+=.1;bad={**o,**change}
            self.assertEqual(engine.process(rgb,bad)[0]['tracking'],'lost')
            o['captured']+=.1;self.assertIn('relocalise',engine.process(rgb,o)[0]['error'])
    def test_navigation_worker_failure_stops_and_saves_report(self):
        class Gateway:
            session='session'
            def __init__(self):self.calls=[]
            def rpc(self,path,*args,**kwargs):self.calls.append(path);return {}
        with tempfile.TemporaryDirectory() as directory:
            g=Gateway();e=Explorer(g,directory,directory);e.profile=profile();e.mission={**spec(),'id':'mission','session':'session'}
            e.journal=Journal(directory,e.mission,e.profile)
            def failed():raise OSError('simulated perception worker crash')
            e.read_nav=failed
            e.drive({'id':'goal','action':'forward','value':.1},{'pose':[0,0,0]},None)
            self.assertEqual(e.last['outcome'],'tracking_lost');self.assertEqual(g.calls,['/navigation/stop'])
            self.assertIn('worker crash',(e.journal.directory/'report.md').read_text())

    def test_models_reject_changed_preprocessing_and_weights(self):
        from model_assets import sha, verify_models
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'segformer-b0').mkdir()
            paths=['segformer-b0/config.json','segformer-b0/preprocessor_config.json','segformer-b0/model.safetensors','depth','detector']
            for path in paths:(root/path).write_bytes(b'test model')
            manifest={'floor':{'sha256':sha(root/paths[2]),'assets':{p:sha(root/p) for p in paths[:3]}},
                      **{k:{'path':k,'sha256':sha(root/k)} for k in ('depth','detector')}}
            verify_models(root,manifest)
            for path in paths:
                (root/path).write_bytes(b'changed')
                with self.subTest(path=path),self.assertRaises(ValueError):verify_models(root,manifest)
                (root/path).write_bytes(b'test model')
    def test_supplementary_vetoes_match_live_limits(self):
        from supplementary_vision import veto_reason
        p=profile();good={'depth_ready':True,'near_score':.2,'hazard':None}
        self.assertIsNone(veto_reason(p,good))
        for bad in ({'hazard':'Visible cat'},{'depth_ready':False},{'near_score':.8},{'near_score':-1},{'near_score':float('nan')}):
            with self.subTest(bad=bad):self.assertIsNotNone(veto_reason(p,{**good,**bad}))
    def test_usb_identity_and_incompatible_camera(self):
        from camera import usb_identity
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);usb=root/'usb-port';usb.mkdir();(usb/'idVendor').write_text('1234');(usb/'idProduct').write_text('5678')
            video=root/'video0';video.mkdir();(video/'device').symlink_to(usb,target_is_directory=True)
            self.assertEqual(usb_identity(0,root),'1234:5678:port=usb-port')
            (usb/'serial').write_text('camera-1');self.assertEqual(usb_identity(0,root),'1234:5678:serial=camera-1')
            self.assertIsNone(usb_identity(1,root))
        p=profile();frame={**p['camera'],'focus_stable':True};self.assertTrue(camera_matches(p,frame))
        self.assertFalse(camera_matches(p,{**frame,'identity':'different camera'}))
    def test_controller_rejects_drift_outside_checked_envelope(self):
        c=LegController({'action':'forward','value':.5},[0,0,0],0)
        self.assertEqual(c.step([0,0,math.radians(6)],.1)[0],'tracking_lost')
        c=LegController({'action':'left','value':15},[0,0,0],0)
        self.assertEqual(c.step([.04,0,.1],.1)[0],'tracking_lost')
    def test_stale_cross_session_geometry_and_stale_goals(self):
        from camera import atomic_json
        class Gateway:session='new'
        with tempfile.TemporaryDirectory() as directory:
            e=Explorer(Gateway(),directory,directory)
            for session,received in [('old',time.monotonic()),('new',time.monotonic()-1)]:
                atomic_json(Path(directory)/'navigation.json',{'session':session,'received_mac':received})
                with self.assertRaises(PermissionError):e.read_nav()
            e.offered={'goal':{'created_mac':time.monotonic()-31}}
            with self.assertRaises(PermissionError):e.choose({'candidate_id':'goal'})
    def test_known_boundary_cannot_be_crossed_without_doorway(self):
        class Journal:
            manifest={'checkpoints':[{'id':'checkpoint'}], 'doorways':[{'id':'ab','checkpoint_id':'checkpoint','distance_m':.35}]}
        e=Explorer.__new__(Explorer);e.profile=profile();e.journal=Journal()
        e.check_boundary({'action':'forward','value':.1},None)
        with self.assertRaises(PermissionError):e.check_boundary({'action':'forward','value':.5},None)
        with self.assertRaises(PermissionError):e.check_boundary({'action':'forward','value':.5},'other')
        e.check_boundary({'action':'forward','value':.5},'ab')

if __name__=='__main__':unittest.main()

