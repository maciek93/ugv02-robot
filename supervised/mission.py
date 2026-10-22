"""Pi-owned mission authority, independent of navigation and model execution."""
import copy
import math
import secrets
from explore_profile import number, validate_profile, digest


class Mission:
    def __init__(self, now):
        self.now = now
        self.proposal = self.active = self.leg = None
        self.last = None
        self.used_goals = set()

    def cancel(self, reason):
        if self.active or self.proposal:
            self.last = {**copy.deepcopy(self.active or self.proposal), 'outcome': reason}
        self.proposal = self.active = self.leg = None
        self.used_goals.clear()

    def propose(self, spec, session, profile):
        validate_profile(profile)
        if self.active: raise PermissionError('Finish the active mission first')
        if not isinstance(spec, dict) or set(spec) != {'configuration_id','rooms','doorways','start_room','starting_clearance_m','duration_s','travel_m'}:
            raise ValueError('Unexpected mission fields')
        if spec['configuration_id'] != profile['configuration_id']:
            raise PermissionError('Configuration changed')
        if not isinstance(spec['rooms'], list) or not 1 <= len(spec['rooms']) <= 20 or any(not isinstance(x,str) or not x.strip() or len(x)>80 for x in spec['rooms']):
            raise ValueError('Choose 1-20 named rooms')
        if spec['start_room'] not in spec['rooms']: raise ValueError('Starting room outside scope')
        if not isinstance(spec['doorways'], list): raise ValueError('Doorways must be a list of named directed connections')
        for d in spec['doorways']:
            if not isinstance(d, dict) or set(d) != {'id','from_room','to_room'} or not isinstance(d['id'],str) or not d['id'] or any(d[k] not in spec['rooms'] for k in ('from_room','to_room')):
                raise ValueError('Invalid doorway scope')
        if len({d['id'] for d in spec['doorways']}) != len(spec['doorways']): raise ValueError('Duplicate doorways')
        for name, bounds in {'starting_clearance_m':(.0,2.),'duration_s':(1,600),'travel_m':(.01,10)}.items():
            number(spec[name], *bounds, name)
        self.proposal = {**copy.deepcopy(spec), 'id':secrets.token_hex(12), 'session':session,
                         'profile_hash':digest(profile), 'created':self.now()}
        return copy.deepcopy(self.proposal)

    def approve(self, mission_id, session, profile):
        p = self.proposal
        if not p or p['id'] != mission_id or p['session'] != session or self.now()-p['created'] > 120:
            raise PermissionError('Fresh matching mission proposal required')
        validate_profile(profile)
        if digest(profile) != p['profile_hash']: raise PermissionError('Profile changed')
        self.active = {**p, 'expires':self.now()+p['duration_s'], 'reserved_m':0., 'started':self.now()}
        self.proposal = None
        self.used_goals.clear()
        return copy.deepcopy(self.active)

    def check(self, identity, session, profile):
        m = self.active
        if not m or identity.get('mission_id') != m['id'] or identity.get('session') != session or identity.get('configuration_id') != m['configuration_id']:
            raise PermissionError('No matching active mission')
        if self.now() >= m['expires'] or m['reserved_m'] >= m['travel_m']:
            raise PermissionError('budget_exhausted')
        if digest(profile) != m['profile_hash']: raise PermissionError('Profile changed')

    def begin(self, data, session, profile):
        self.check(data, session, profile)
        if self.leg: raise PermissionError('A navigation leg is already active')
        if not isinstance(data.get('goal_id'),str) or not data['goal_id'] or data['goal_id'] in self.used_goals:
            raise PermissionError('Duplicate or invalid goal')
        action = data.get('action')
        if action == 'forward':
            value = number(data.get('value'), .01, .5, 'forward metres')
            cap = min(.5, value+max(.03, value*.2)) / profile['upper_speed_m_s']
        elif action in ('left','right'):
            value = number(data.get('value'), 1, 15, 'turn degrees')
            cap = math.radians(value) / profile['upper_turn_rad_s']
        else: raise ValueError('Only forward and bounded turns supported')
        # Count possible post-lease firmware fallback travel for every leg.
        # Never refund it based on encoders or an optimistic successful Stop.
        if self.active['reserved_m'] + profile['fallback_stop_m'] >= self.active['travel_m']:
            raise PermissionError('budget_exhausted')
        self.active['reserved_m'] += profile['fallback_stop_m']
        self.used_goals.add(data['goal_id'])
        self.leg = {**copy.deepcopy(data), 'started':self.now(), 'end':min(self.now()+cap, self.active['expires']),
                    'seq':-1, 'last_renew':None, 'reserved_until':self.now()}
        return copy.deepcopy(self.leg)

    def renew(self, data, session, profile):
        self.check(data, session, profile)
        leg = self.leg
        if not leg or data.get('goal_id') != leg['goal_id']: raise PermissionError('No matching leg')
        now = self.now()
        if now >= leg['end']: raise PermissionError('budget_exhausted')
        if leg['last_renew'] is not None and now-leg['last_renew'] > .1:
            raise PermissionError('Navigation renewal missed 100 ms deadline')
        if type(data.get('seq')) is not int or data['seq'] != leg['seq']+1:
            raise PermissionError('Replayed or out-of-order renewal')
        left, right = (number(data.get(k), -.05, .05, k) for k in ('left','right'))
        if leg['action'] == 'forward' and (left < 0 or right < 0): raise ValueError('No reverse motion')
        if leg['action'] == 'left' and not left <= 0 <= right: raise ValueError('Wrong turn direction')
        if leg['action'] == 'right' and not right <= 0 <= left: raise ValueError('Wrong turn direction')
        end = min(now+.25, leg['end'])
        # Reserve worst-case wheel travel for every newly authorised time interval,
        # including braking/failure exposure. Unused reservation is never refunded.
        extra = max(0., end-leg['reserved_until']) * profile['upper_speed_m_s']
        if self.active['reserved_m']+extra > self.active['travel_m']:
            raise PermissionError('budget_exhausted')
        self.active['reserved_m'] += extra
        leg.update(seq=data['seq'], last_renew=now, reserved_until=end)
        return left, right, end

    def finish_leg(self, goal_id, outcome):
        if not self.leg or self.leg['goal_id'] != goal_id: raise PermissionError('No matching leg')
        if outcome not in ('reached','blocked','tracking_lost','cancelled','budget_exhausted'):
            raise ValueError('Invalid outcome')
        self.leg = None
        if outcome != 'reached': self.cancel(outcome)

    def status(self):
        return copy.deepcopy({'proposal':self.proposal,'active':self.active,'leg':self.leg,'last':self.last})
