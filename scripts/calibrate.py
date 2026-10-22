#!/usr/bin/env python3
"""Stationary camera calibration and evidence templates. Never commands motors."""
import argparse
import base64
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'supervised'))
from camera import atomic_json


def target(path):
    from reportlab.pdfgen import canvas
    from reportlab.lib.pagesizes import A4, landscape
    from reportlab.lib.units import mm
    path.parent.mkdir(parents=True,exist_ok=True)
    c=canvas.Canvas(str(path),pagesize=landscape(A4))
    c.setTitle('UGV02 camera calibration - 20 mm checkerboard')
    c.setFont('Helvetica-Bold',14);c.drawString(18*mm,195*mm,'UGV02 CAMERA CALIBRATION')
    c.setFont('Helvetica',9);c.drawString(18*mm,188*mm,'A4 landscape | Print ACTUAL SIZE / 100% | Disable Fit to page | Mount flat')
    x0,y0=48.5*mm,34*mm
    for row in range(7):
        for col in range(10):
            c.setFillColorRGB(*((0,0,0) if (row+col)%2==0 else (1,1,1)))
            c.rect(x0+col*20*mm,y0+row*20*mm,20*mm,20*mm,fill=1,stroke=0)
    c.setStrokeColorRGB(0,0,0);c.setFillColorRGB(0,0,0)
    c.rect(x0,y0,200*mm,140*mm,stroke=1,fill=0)
    c.setFont('Helvetica',9);c.drawCentredString(148.5*mm,179*mm,'FAR EDGE when lying on the floor; robot faces this sheet from below')
    c.drawString(48.5*mm,28*mm,'NEAR LEFT: first internal corner is 20 mm right and 20 mm forward from here')
    c.line(48.5*mm,19*mm,148.5*mm,19*mm)
    for i in range(6):c.line((48.5+i*20)*mm,17*mm,(48.5+i*20)*mm,21*mm)
    c.drawString(154*mm,18*mm,'Ruler check: this line must measure 100 mm')
    c.setFont('Helvetica',8);c.drawString(18*mm,8*mm,'10 x 7 squares; 9 x 6 internal corners. Each square must measure 20 mm.')
    c.save()


def corners(image):
    import cv2
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    if cv2.Laplacian(gray,cv2.CV_64F).var()<50:raise ValueError('Image blurred')
    ok,found=cv2.findChessboardCornersSB(gray,(9,6),flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:raise ValueError('All 9 x 6 internal corners must be visible')
    return found


def signature(found,w,h):
    import numpy as np
    a=found.reshape(-1,2)
    return np.array([*a.mean(0)/[w,h],np.ptp(a[:,0])/w,np.ptp(a[:,1])/h,
                     (a[8,1]-a[0,1])/max(1,np.linalg.norm(a[8]-a[0]))])


def capture(directory,count):
    import cv2
    import numpy as np
    import requests
    directory.mkdir(parents=True,exist_ok=True)
    headers={'X-Agent-Key':(ROOT/'.runtime/agent-key').read_text().strip()}
    accepted=[];seen=set();deadline=time.monotonic()+300
    print('Robot must already be in a stationary calibration session. Hold the flat checkerboard at varied angles and across the image. Capture times out after five minutes.',flush=True)
    while len(accepted)<count and time.monotonic()<deadline:
        r=requests.get('http://127.0.0.1:5001/agent/observe',headers=headers,timeout=3);r.raise_for_status();o=r.json()
        if o['status']['moving'] or o['status'].get('execution') or o['status']['mode']!='calibration':raise RuntimeError('Stationary calibration session required')
        if o['frame_id'] in seen:time.sleep(.2);continue
        seen.add(o['frame_id'])
        image=cv2.imdecode(np.frombuffer(base64.b64decode(o.pop('jpeg')),np.uint8),cv2.IMREAD_COLOR)
        try:
            found=corners(image);s=signature(found,image.shape[1],image.shape[0])
            if any(np.linalg.norm(s-old)<.12 for old in accepted):raise ValueError('Change position, distance or angle')
        except ValueError as e:print(str(e),flush=True);time.sleep(.5);continue
        prefix=directory/f'view-{len(accepted):02}'
        cv2.imwrite(str(prefix.with_suffix('.jpg')),image);atomic_json(prefix.with_suffix('.json'),o)
        accepted.append(s);print(f'Accepted {len(accepted)}/{count}',flush=True);time.sleep(.3)
    if len(accepted)<count:raise RuntimeError(f'Only {len(accepted)} diverse views captured; add views before solving')


def snapshot(path):
    import requests
    headers={'X-Agent-Key':(ROOT/'.runtime/agent-key').read_text().strip()}
    r=requests.get('http://127.0.0.1:5001/agent/observe',headers=headers,timeout=3);r.raise_for_status();o=r.json()
    if o['status']['moving'] or o['status'].get('execution') or o['status']['mode']!='calibration':raise ValueError('Stationary calibration session required')
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_bytes(base64.b64decode(o.pop('jpeg')));atomic_json(path.with_suffix('.json'),o)


def intrinsics(directory,out):
    import cv2
    import numpy as np
    points=np.zeros((54,3),np.float32);points[:,:2]=np.mgrid[0:9,0:6].T.reshape(-1,2)*.02
    samples=[];signatures=[];identities=[];size=None
    for path in sorted(directory.glob('view-*.jpg')):
        im=cv2.imread(str(path));found=corners(im)
        current=(im.shape[1],im.shape[0])
        if size and size!=current:raise ValueError('Mixed image sizes')
        size=current;samples.append(found);signatures.append(signature(found,*size))
        meta=json.loads(path.with_suffix('.json').read_text());identities.append({k:meta[k] for k in ('device','identity','width','height','rotation_degrees','focus','focus_stable')})
    if len(samples)<15 or any(x!=identities[0] for x in identities):raise ValueError('Need >=15 views with identical camera settings')
    if not identities[0].get('identity'):raise ValueError('USB camera identity unavailable')
    if not identities[0]['focus_stable']:raise ValueError('Focus could not be locked; stabilise it before calibration')
    spread=np.ptp(np.array(signatures),axis=0)
    if spread[0]<.25 or spread[1]<.2 or max(spread[2:4])<.15 or spread[4]<.15:raise ValueError('Insufficient position, scale or angle diversity')
    rms,K,D,rvec,tvec=cv2.calibrateCamera([points]*len(samples),samples,size,None,None)
    errors=[]
    for s,r,t in zip(samples,rvec,tvec):
        projected,_=cv2.projectPoints(points,r,t,K,D);errors.append(float(np.sqrt(np.mean((projected-s)**2))))
    if rms>1. or max(errors)>1.5:raise ValueError(f'Calibration error too high: RMS {rms:.3f}, max view {max(errors):.3f}px')
    atomic_json(out,{**identities[0],'focus_verified':True,'matrix':K.tolist(),'distortion':D.ravel().tolist(),
                     'rms_px':rms,'view_errors_px':errors,'evidence':str(directory.resolve())})


def floor(args):
    import cv2
    import numpy as np
    camera=json.loads(args.intrinsics.read_text());im=cv2.imread(str(args.image))
    meta=json.loads(args.image.with_suffix('.json').read_text())
    if any(meta.get(k)!=camera[k] for k in ('device','identity','width','height','rotation_degrees','focus')):raise ValueError('Floor image camera settings differ')
    rect=cv2.undistort(im,np.array(camera['matrix']),np.array(camera['distortion']))
    found=corners(rect).reshape(6,9,2)
    if args.reverse_rows:found=found[::-1]
    if args.reverse_columns:found=found[:,::-1]
    preview=rect.copy()
    for label,uv in [('0 NEAR LEFT',found[0,0]),('8 NEAR RIGHT',found[0,8]),('45 FAR LEFT',found[5,0])]:
        cv2.putText(preview,label,tuple(uv.astype(int)),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,0,255),2)
    cv2.imwrite(str(args.output.with_suffix('.preview.jpg')),preview)
    if not args.orientation_confirmed:
        raise ValueError('Inspect preview. Orient corners using --reverse-rows/--reverse-columns, then add --orientation-confirmed only after checking all three labels.')
    if args.near_edge_m<=0:raise ValueError('Measure the near edge forward from chassis centre')
    world=np.array([(args.near_edge_m+.02+r*.02,args.left_edge_m-.02-c*.02) for r in range(6) for c in range(9)],np.float32)
    H,_=cv2.findHomography(found.reshape(-1,2),world,0)
    inv=np.linalg.inv(H);projected=cv2.perspectiveTransform(world.reshape(-1,1,2),inv).reshape(-1,2)
    error=float(np.sqrt(np.mean((projected-found.reshape(-1,2))**2)))
    if error>1.5:raise ValueError('Floor plane reprojection failed')
    object_points=np.column_stack((world,np.zeros(len(world)))).astype(np.float32)
    ok,r,t=cv2.solvePnP(object_points,found.reshape(-1,2),np.array(camera['matrix']),np.zeros(5))
    if not ok:raise ValueError('Floor pose failed')
    R,_=cv2.Rodrigues(r);position=(-R.T@t).ravel()
    if not .03 <= position[2] <= 1.:raise ValueError('Implausible camera height; check board axes and measured placement')
    camera.update(image_to_ground=H.tolist(),floor_reprojection_px=error,camera_position_m=position.tolist(),
                  camera_height_m=float(position[2]),world_to_camera_rotation=R.tolist(),world_to_camera_translation_m=t.ravel().tolist(),
                  floor_evidence=str(args.image.resolve()))
    atomic_json(args.output,camera)


def draft(camera_path,out,configuration,surface):
    camera=json.loads(camera_path.read_text());camera['mounting_id']=configuration
    manifest=ROOT/'models/explore-manifest.json'
    models=json.loads(manifest.read_text()) if manifest.exists() else {'floor':'pending','depth':'pending','detector':'pending'}
    p={'version':1,'kind':'camera-explore','configuration_id':configuration,'surface':surface,'camera':camera,
       'models':models,'validation':{k:{'passed':False,'evidence':[]} for k in ('motion','camera','watchdog','perception','latency')},
       'motor_polarity':[-1,-1],'encoder_signs':None,'encoder_m_per_unit':None,
       'width_m':None,'length_m':None,'track_m':None,'camera_forward_m':0,'blind_forward_m':None,
       'upper_speed_m_s':None,'upper_turn_rad_s':None,'fallback_timeout_s':None,'braking_m':None,'fallback_stop_m':None,'margin_m':.05,
       'max_translation_disagreement_m':.02,'max_yaw_disagreement_rad':.08,'floor_probability':.95,
       'near_limit':None,'score_low':None,'score_high':None,'min_sharpness':50,'min_features':12,
       'clearance_max_age_s':5.,'max_reprojection_px':1.5,'max_geometry_error_m':.015}
    atomic_json(out,p)


def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='command',required=True)
    a=sub.add_parser('target');a.add_argument('--output',type=Path,default=ROOT/'output/pdf/checkerboard-20mm.pdf')
    a=sub.add_parser('capture');a.add_argument('directory',type=Path);a.add_argument('--count',type=int,choices=range(15,31),default=20)
    a=sub.add_parser('snapshot');a.add_argument('image',type=Path)
    a=sub.add_parser('intrinsics');a.add_argument('directory',type=Path);a.add_argument('--output',type=Path,required=True)
    a=sub.add_parser('floor');a.add_argument('intrinsics',type=Path);a.add_argument('image',type=Path);a.add_argument('--output',type=Path,required=True);a.add_argument('--near-edge-m',type=float,required=True);a.add_argument('--left-edge-m',type=float,required=True)
    for flag in ('reverse-rows','reverse-columns','orientation-confirmed'):a.add_argument('--'+flag,action='store_true')
    a=sub.add_parser('draft-profile');a.add_argument('camera',type=Path);a.add_argument('--output',type=Path,required=True);a.add_argument('--configuration',required=True);a.add_argument('--surface',required=True)
    a=sub.add_parser('check-profile');a.add_argument('profile',type=Path)
    args=parser.parse_args()
    if args.command=='target':target(args.output)
    elif args.command=='capture':capture(args.directory,args.count)
    elif args.command=='snapshot':snapshot(args.image)
    elif args.command=='intrinsics':intrinsics(args.directory,args.output)
    elif args.command=='floor':floor(args)
    elif args.command=='draft-profile':draft(args.camera,args.output,args.configuration,args.surface)
    else:
        from explore_profile import validate_profile
        validate_profile(json.loads(args.profile.read_text()));print('Profile structure and recorded validation pass. This is not independent physical attestation.')

if __name__=='__main__':main()
