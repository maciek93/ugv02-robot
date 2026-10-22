"""Generate Nav2's measured chassis footprint without editing source config."""
import json
import math
import re
from pathlib import Path
import yaml

from metric_mission import validate_metric_profile

source = Path('/etc/ugv-supervised/metric-profile.json')
observation_source = Path('/var/lib/ugv-maps/observation-profile.json')
runtime = Path('/run/ugv-nav')
runtime.mkdir(parents=True, exist_ok=True)
slam = yaml.safe_load(Path('/opt/robot/ros2/config/slam.yaml').read_text())
if source.exists():
    profile = validate_metric_profile(json.loads(source.read_text()))
    (runtime / 'metric-profile.json').write_text(json.dumps(profile))
    params = yaml.safe_load(Path('/opt/robot/ros2/config/nav2_params.yaml').read_text())
    half_l, half_w = profile['length_m'] / 2, profile['width_m'] / 2
    footprint = f'[[{half_l}, {half_w}], [{half_l}, {-half_w}], [{-half_l}, {-half_w}], [{-half_l}, {half_w}]]'
    for name in ('local_costmap', 'global_costmap'):
        params[name][name]['ros__parameters']['footprint'] = footprint
    (runtime / 'nav2_params.yaml').write_text(yaml.safe_dump(params, sort_keys=False))
    reuse = Path('/etc/ugv-supervised/reuse-map.json')
    if reuse.exists():
        selection = json.loads(reuse.read_text())
        mission_id = selection.get('mission_id')
        pose = selection.get('initial_pose')
        if not isinstance(mission_id, str) or not re.fullmatch(r'[0-9a-f]{24}', mission_id) or not isinstance(pose, list) or len(pose) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) for v in pose):
            raise ValueError('Invalid saved-map selection')
        base = Path('/var/lib/ugv-maps') / mission_id / 'survey'
        if not base.with_suffix('.posegraph').is_file() or not base.with_suffix('.data').is_file():
            raise ValueError('Selected pose graph is incomplete')
        slam['slam_toolbox']['ros__parameters'].update(mode='localization', map_file_name=str(base), map_start_pose=pose)
elif observation_source.exists():
    observation = json.loads(observation_source.read_text())
    required = {'status', 'configuration_id', 'length_m', 'width_m', 'track_m',
                'lidar_x_m', 'lidar_y_m', 'lidar_yaw_rad', 'encoder_m_per_unit',
                'encoder_signs', 'upper_speed_m_s', 'evidence'}
    if set(observation) != required or observation['status'] != 'observation-only':
        raise ValueError('Invalid observation-only profile')
    for key, low, high in (('length_m', .1, 1.), ('width_m', .1, 1.),
                           ('track_m', .1, 1.), ('lidar_x_m', -.5, .5),
                           ('lidar_y_m', -.5, .5), ('lidar_yaw_rad', -math.pi, math.pi),
                           ('encoder_m_per_unit', .00001, 1.),
                           ('upper_speed_m_s', .02, .5)):
        value = observation[key]
        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f'Invalid observation {key}')
    if not isinstance(observation['configuration_id'], str) or not observation['configuration_id']:
        raise ValueError('Missing observation configuration')
    if not isinstance(observation['evidence'], str) or len(observation['evidence']) < 30:
        raise ValueError('Missing observation evidence')
    if observation['encoder_signs'] not in ([1, 1], [1, -1], [-1, 1], [-1, -1]):
        raise ValueError('Invalid observation encoder signs')
    (runtime / 'observation-profile.json').write_text(json.dumps(observation))
if source.exists() or observation_source.exists():
    (runtime / 'slam.yaml').write_text(yaml.safe_dump(slam, sort_keys=False))
