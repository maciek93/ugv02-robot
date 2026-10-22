"""Camera-ground geometry, conservative local map and bounded leg control.

Coordinates: metres, x forward / y left; positive yaw is left. Pixels are raw
upright camera pixels unless explicitly undistorted. No depth value is metric.
"""
import math
from collections import deque
import time
import cv2
import numpy as np


def transform(points, pose):
    a = np.asarray(points, dtype=np.float64).reshape(-1,2)
    c,s = math.cos(pose[2]), math.sin(pose[2])
    return a @ np.array([[c,s],[-s,c]]) + np.asarray(pose[:2])


def project(points, matrix):
    return cv2.perspectiveTransform(np.asarray(points, np.float32).reshape(-1,1,2), np.asarray(matrix, np.float64)).reshape(-1,2)


def wrapped(a): return (a+math.pi) % (2*math.pi)-math.pi


class FloorTracker:
    def __init__(self, profile):
        self.p = profile
        self.previous = self.previous_mask = self.telemetry = self.captured = None
        self.pose = np.zeros(3)
        self.uncertainty = 0.
        self.reason = 'Warming up'
        self.agreement = deque()
        self.ever_good = False
        self.requires_relocalisation = False

    def update(self, gray, floor, observation):
        p = self.p
        t = observation['captured']
        chassis = observation['chassis']
        data = chassis.get('data') or {}
        try:
            if self.requires_relocalisation: raise ValueError('Restart stationary session to relocalise after tracking loss')
            wheels = np.array([float(data[k]) for k in ('odl','odr')])
            if not np.isfinite(wheels).all() or chassis.get('age_s', 99) > .75: raise ValueError('Wheel telemetry unavailable')
            if self.previous is None: raise ValueError('Warming up')
            dt = t-self.captured
            if not 0 < dt <= .5: raise ValueError('Tracking capture gap')
            mask = ((self.previous_mask > 0)*255).astype('uint8')
            features = cv2.goodFeaturesToTrack(self.previous, 250, .01, 5, mask=mask)
            if features is None or len(features) < p['min_features']: raise ValueError('Insufficient floor texture')
            dest, good, _ = cv2.calcOpticalFlowPyrLK(self.previous, gray, features, None)
            back, backgood, _ = cv2.calcOpticalFlowPyrLK(gray, self.previous, dest, None)
            keep = good.ravel().astype(bool) & backgood.ravel().astype(bool) & (np.linalg.norm(back-features, axis=2).ravel()<1.)
            uv = dest.reshape(-1,2)
            inside = (uv[:,0]>=0)&(uv[:,0]<gray.shape[1])&(uv[:,1]>=0)&(uv[:,1]<gray.shape[0])
            ix = np.clip(uv[:,0].astype(int),0,gray.shape[1]-1); iy=np.clip(uv[:,1].astype(int),0,gray.shape[0]-1)
            keep &= inside & (floor[iy,ix]>0)
            if keep.sum() < p['min_features']: raise ValueError('Floor tracking lost')
            old = project(features.reshape(-1,2)[keep], p['camera']['image_to_ground'])
            new = project(uv[keep], p['camera']['image_to_ground'])
            plausible = np.isfinite(old).all(axis=1)&np.isfinite(new).all(axis=1)&(np.linalg.norm(old,axis=1)<3)&(np.linalg.norm(new,axis=1)<3)
            old,new=old[plausible],new[plausible]
            if len(old)<p['min_features']: raise ValueError('Insufficient metric floor tracks')
            affine, inliers = cv2.estimateAffinePartial2D(new, old, method=cv2.RANSAC, ransacReprojThreshold=p['max_geometry_error_m'])
            if affine is None or inliers.sum()<p['min_features'] or inliers.mean()<.7: raise ValueError('Floor geometry inconsistent')
            scale = math.hypot(affine[0,0],affine[1,0])
            if abs(scale-1)>.03: raise ValueError('Camera pitch, scale or floor changed')
            visual = np.array([affine[0,2],affine[1,2], math.atan2(affine[1,0],affine[0,0])])
            delta = (wheels-self.telemetry)*p['encoder_m_per_unit']*np.array(p['encoder_signs'])
            if np.max(np.abs(delta)) > p['upper_speed_m_s']*dt + .01: raise ValueError('Encoder reset or implausible travel')
            odom = np.array([delta.mean(),0.,(delta[1]-delta[0])/p['track_m']])
            if np.linalg.norm(visual[:2]-odom[:2])>p['max_translation_disagreement_m'] or abs(wrapped(visual[2]-odom[2]))>p['max_yaw_disagreement_rad']:
                raise ValueError('Visual and encoder motion disagree')
            if np.linalg.norm(visual[:2]) > p['upper_speed_m_s']*dt+.01 or abs(visual[2])>p['upper_turn_rad_s']*dt+.03:
                raise ValueError('Visual motion exceeds calibration')
            self.agreement.append((t, visual-odom))
            while self.agreement and t-self.agreement[0][0]>.75:self.agreement.popleft()
            disagreement=np.sum([d for _,d in self.agreement],axis=0)
            if len(self.agreement)>=3 and (np.linalg.norm(disagreement[:2])>p['max_translation_disagreement_m'] or abs(disagreement[2])>p['max_yaw_disagreement_rad']):
                raise ValueError('Accumulated visual/encoder disagreement: possible slip or stall')
            fused = visual*.7+odom*.3
            self.pose[:2] = transform([fused[:2]],self.pose)[0]
            self.pose[2] = wrapped(self.pose[2]+fused[2])
            self.uncertainty += np.linalg.norm(visual[:2]-odom[:2])*.3
            self.reason = None
            self.ever_good = True
        except (ValueError, KeyError, TypeError, cv2.error) as e:
            self.reason = str(e)
            if self.ever_good:self.requires_relocalisation=True
        finally:
            self.previous, self.previous_mask, self.captured = gray.copy(), floor.copy(), t
            self.telemetry = locals().get('wheels')
        return {'pose':self.pose.tolist(),'uncertainty_m':float(self.uncertainty), 'tracking':'lost' if self.reason else 'good','error':self.reason}


class LocalMap:
    """5 cm world grid. Unknown/expired cells never count as free."""
    resolution = .05
    ttl = 5.
    def __init__(self, profile):
        self.p=profile; self.ttl=profile['clearance_max_age_s']; self.cells={}; self.observed=set()

    def key(self, point): return tuple(np.floor(np.asarray(point)/self.resolution).astype(int))

    def seed(self, radius, now, pose=(0,0,0)):
        # Explicit human-confirmed starting disk; not inferred from the front image.
        for x in np.arange(-radius,radius,self.resolution):
            for y in np.arange(-radius,radius,self.resolution):
                if math.hypot(x,y)+self.resolution*math.sqrt(2) <= radius:
                    key=self.key(transform([(x,y)],pose)[0])
                    if self.cells.get(key,(0,0,0))[0] != -1:self.cells[key]=(1,now,3)

    def update(self, floor, pose, now):
        p=self.p; Hinv=np.linalg.inv(np.array(p['camera']['image_to_ground']))
        # Evaluate full projected cell patches, not a single pixel per cell.
        xs=np.arange(max(p['blind_forward_m'],p['length_m']/2),2.01,self.resolution)
        ys=np.arange(-1.5,1.51,self.resolution)
        ground=np.array([(x,y) for x in xs for y in ys])
        world=transform(ground,pose)
        half=self.resolution/2
        offsets=np.array([[-half,-half],[-half,half],[half,half],[half,-half],[0,0]])
        uv=project((ground[:,None,:]+offsets).reshape(-1,2),Hinv).reshape(-1,5,2)
        h,w=floor.shape
        for cell,points in zip(world,uv):
            if not np.isfinite(points).all() or np.any(points[:,0]<0) or np.any(points[:,0]>=w) or np.any(points[:,1]<0) or np.any(points[:,1]>=h): continue
            polygon=np.round(points[:4]).astype(np.int32)
            x,y,bw,bh=cv2.boundingRect(polygon)
            if bw*bh>h*w//3: continue
            roi=floor[y:y+bh,x:x+bw]
            mask=np.zeros(roi.shape,np.uint8);cv2.fillConvexPoly(mask,polygon-[x,y],1)
            accepted=bool(mask.sum() and np.all(roi[mask>0]>0))
            key=self.key(cell);old=self.cells.get(key,(0,0,0))
            if now == old[1]:
                accepted = accepted and old[0] != -1
                hits = old[2] if accepted else 0
            else:
                hits=old[2]+1 if accepted and now-old[1]<=.5 else int(accepted)
            self.cells[key]=(1 if accepted and hits>=3 else (-1 if not accepted else 0),now,hits)
            self.observed.add(key)
        # Prune stale/distant observations. Underbody stays known because it is
        # physically occupied, but no surrounding clearance is invented.
        for key in list(self.cells):
            point=(np.array(key)+.5)*self.resolution
            if np.linalg.norm(point-np.asarray(pose[:2]))>4 or now-self.cells[key][1]>self.ttl:
                del self.cells[key]
        self.mark_underbody(pose,now)

    def mark_underbody(self,pose,now):
        p=self.p
        for x in np.arange(-p['length_m']/2+self.resolution,p['length_m']/2-self.resolution,self.resolution):
            for y in np.arange(-p['width_m']/2+self.resolution,p['width_m']/2-self.resolution,self.resolution):
                self.cells[self.key(transform([(x,y)],pose)[0])]=(1,now,3)

    def clear(self,pose,action,value,now,uncertainty=0):
        p=self.p; margin=p['margin_m']+uncertainty+max(p['braking_m'],p['fallback_stop_m'])+self.resolution*math.sqrt(2)/2
        # Bound lateral drift and heading corrections over the entire future leg.
        margin += .03 + (math.hypot(p['length_m']/2,p['width_m']/2) + (value if action=='forward' else 0))*math.sin(math.radians(5))
        stop_yaw=p['upper_turn_rad_s']*(p['fallback_timeout_s']+p['braking_m']/p['upper_speed_m_s'])
        if action=='forward':
            # Differential steering may continue during a Pi/command-loss failure.
            margin += math.hypot(p['length_m']/2,p['width_m']/2)*min(2.,stop_yaw)
        length=p['length_m']/2+margin; width=p['width_m']/2+margin
        body=np.array([(x,y) for x in np.arange(-length,length+self.resolution/2,self.resolution/2)
                      for y in np.arange(-width,width+self.resolution/2,self.resolution/2)])
        if action=='forward':
            poses=[(pose[0]+d*math.cos(pose[2]),pose[1]+d*math.sin(pose[2]),pose[2]) for d in np.linspace(0,value,max(2,int(value/.02)+1))]
        else:
            sign=1 if action=='left' else -1
            # Include rotation after command loss, not only the requested angle.
            angle=min(2*math.pi,math.radians(value)+stop_yaw)
            poses=[(*pose[:2],pose[2]+sign*a) for a in np.linspace(0,angle,max(2,int(math.degrees(angle))+1))]
        cloud=np.concatenate([transform(body,at) for at in poses])
        keys=np.unique(np.floor(cloud/self.resolution).astype(int),axis=0)
        for key in map(tuple,keys):
            state,stamp,_=self.cells.get(key,(0,0,0))
            if state!=1 or now-stamp>self.ttl: return False
        return True

    def candidates(self,pose,now,uncertainty):
        results=[]
        for distance in (.5,.25,.1):
            if self.clear(pose,'forward',distance,now,uncertainty):
                results.append({'action':'forward','value':distance,'endpoint':transform([(distance,0)],pose)[0].tolist()})
        for action in ('left','right'):
            if self.clear(pose,action,15,now,uncertainty): results.append({'action':action,'value':15,'endpoint':list(pose[:2])})
        return results

    def snapshot(self):
        return {'resolution_m':self.resolution,'ttl_s':self.ttl,'cells':[[int(k[0]),int(k[1]),int(v[0]),float(v[1]),int(v[2])] for k,v in self.cells.items()], 'sensor':'usb-camera'}

    @classmethod
    def restore(cls,p,data):
        m=cls(p);m.cells={tuple(row[:2]):tuple(row[2:]) for row in data['cells']};return m


class LegController:
    def __init__(self,goal,pose,now):
        self.goal=goal;self.origin=np.array(pose);self.started=now;self.progress_at=now;self.best=0.

    def step(self,pose,now):
        action=self.goal['action']; target=self.goal['value']
        if action=='forward':
            delta=np.array(pose[:2])-self.origin[:2]
            progress=float(delta @ np.array([math.cos(self.origin[2]),math.sin(self.origin[2])]))
            lateral=abs(float(delta @ np.array([-math.sin(self.origin[2]),math.cos(self.origin[2])])))
            if lateral>.03: return 'tracking_lost',(0.,0.),target-progress
            remaining=target-progress; heading=wrapped(pose[2]-self.origin[2])
            if abs(heading)>math.radians(5): return 'tracking_lost',(0.,0.),remaining
            if remaining<=min(.03,target*.2): return 'reached',(0.,0.),0
            speed=min(.05,max(.015,remaining*.4));correction=max(-.015,min(.015,heading*.05))
            command=(max(0,min(.05,speed+correction)),max(0,min(.05,speed-correction)))
            unit=.005
        else:
            if np.linalg.norm(np.array(pose[:2])-self.origin[:2])>.03: return 'tracking_lost',(0.,0.),target
            sign=1 if action=='left' else -1
            progress=sign*math.degrees(wrapped(pose[2]-self.origin[2]));remaining=target-progress
            if remaining<=2.: return 'reached',(0.,0.),0
            speed=min(.05,max(.015,remaining*.005));command=(-sign*speed,sign*speed);unit=.5
        if progress>self.best+unit:self.best=progress;self.progress_at=now
        if now-self.progress_at>1.:return 'tracking_lost',(0.,0.),remaining
        return None,command,max(0,remaining)
