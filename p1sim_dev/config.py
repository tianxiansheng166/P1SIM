"""Foundation V1 canonical configuration. No scheduler/business semantics."""
import copy
import hashlib
import json
import math
from pathlib import Path

FOUNDATION_VERSION = '1.0.0'
INTERFACE_VERSION = PLUGIN_API_VERSION = CONFIG_VERSION = 1
DEFAULT = dict(config_version=1, foundation_version=FOUNDATION_VERSION, interface_version=1,
    plugin_api_version=1, scene='developer', seed=0, duration_seconds=None, headless=True,
    ready_timeout_seconds=120, namespace='', topics={}, recording=False,
    workspace=dict(path='', setup='', plugin_paths=[]),
    plugins={k: dict(entry='p1sim_dev.builtins:'+v, config={}) for k, v in
             dict(target='Trajectory', gimbal='KinematicGimbal', timing='Timing', sensor='Sensor', world='World').items()})


def number(value, name, minimum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(name + ' must be finite numeric')
    if minimum is not None and value < minimum:
        raise ValueError(name + ' below minimum')
    return value


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def normalize(value):
    if not isinstance(value, dict) or set(value) - set(DEFAULT):
        raise ValueError('Unknown config fields or non-object config')
    result = copy.deepcopy(DEFAULT)
    for key, val in value.items():
        if key in ('workspace', 'plugins'):
            if not isinstance(val, dict) or set(val)-set(DEFAULT[key]):
                raise ValueError('Unknown '+key+' fields')
            result[key].update(copy.deepcopy(val))
        else:
            result[key] = copy.deepcopy(val)
    for name, version in dict(config_version=1, foundation_version=FOUNDATION_VERSION,
                              interface_version=1, plugin_api_version=1).items():
        if type(result[name]) is not type(version) or result[name] != version:
            raise ValueError('Unsupported '+name)
    if type(result['seed']) is not int or not 0 <= result['seed'] < 2**63:
        raise ValueError('seed must be an integer in [0, 2^63)')
    if not isinstance(result['scene'], str) or not result['scene']:
        raise ValueError('scene must be a nonempty string')
    if result['scene'] not in ('developer', 'autoaim'):
        # External world plugins can resolve other scene names.
        if not isinstance(result['plugins']['world'], dict) or result['plugins']['world'].get('entry') == DEFAULT['plugins']['world']['entry']:
            raise ValueError('Unknown built-in scene')
    if type(result['headless']) is not bool or type(result['recording']) is not bool:
        raise ValueError('headless/recording must be boolean')
    if result['duration_seconds'] is not None:
        number(result['duration_seconds'], 'duration_seconds', .001)
    number(result['ready_timeout_seconds'], 'ready_timeout_seconds', .1)
    if not isinstance(result['namespace'], str) or not all(c.isalnum() or c in '/_' for c in result['namespace']):
        raise ValueError('Invalid namespace')
    allowed_topics = {'image', 'camera_info', 'joint_states', 'imu', 'gimbal_command', 'planner', 'tracker'}
    result['namespace'] = result['namespace'].strip('/')
    if not isinstance(result['topics'], dict) or set(result['topics'])-allowed_topics:
        raise ValueError('Unknown topic mapping')
    for topic in result['topics'].values():
        if not isinstance(topic, str) or not topic.startswith('/') or not all(c.isalnum() or c in '/_' for c in topic):
            raise ValueError('Topic mappings must be absolute ROS names')
    workspace = result['workspace']
    if not all(isinstance(workspace[k], str) for k in ('path', 'setup')) or not isinstance(workspace['plugin_paths'], list):
        raise ValueError('Invalid workspace references')
    if not all(isinstance(p, str) and (p == '/user_ws' or p.startswith('/user_ws/')) and '..' not in Path(p).parts for p in workspace['plugin_paths']):
        raise ValueError('Plugin paths must be within the mounted /user_ws')
    for kind, spec in result['plugins'].items():
        if not isinstance(spec, dict) or set(spec)-{'entry', 'config'} or not isinstance(spec.get('entry'), str) or ':' not in spec['entry'] or not isinstance(spec.get('config'), dict):
            raise ValueError('Invalid '+kind+' plugin specification')
    canonical(result)
    return result


def load(path=None, seed=None):
    value = json.loads(Path(path).read_text()) if path else {}
    if seed is not None:
        value['seed'] = seed
    return normalize(value)


def digest(config):
    return hashlib.sha256(canonical(config).encode()).hexdigest()


def atomic_json(path, value):
    path = Path(path)
    temp = path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
    temp.replace(path)
