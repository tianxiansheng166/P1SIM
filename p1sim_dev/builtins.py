"""Reference platform models. All times are simulation seconds, angles radians."""
from collections import deque
import math
from pathlib import Path
from p1sim_dev.plugins import Plugin
from p1sim_dev.config import number


def fields(config, allowed):
    if set(config)-set(allowed):
        raise ValueError('Unknown plugin fields: '+str(set(config)-set(allowed)))


def vector(value, name, count=3):
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(name+' vector size')
    return [number(v, name) for v in value]


class World(Plugin):
    kind = 'world'
    observables = ('world_path',)

    def resolve(self, scene, root):
        return str(Path(root)/'worlds'/(scene+'.sdf'))


class Trajectory(Plugin):
    kind = 'target'
    observables = ('position_m', 'velocity_m_s', 'developer_truth')

    def configure(self, config):
        fields(config, ('mode', 'velocity_m_s', 'acceleration_m_s2', 'reverse_period_seconds', 'offset_m', 'seed_offset_m'))
        self.mode = config.get('mode', 'static')
        if self.mode not in ('static', 'linear', 'accel', 'reverse'):
            raise ValueError('Unknown trajectory mode')
        self.velocity = vector(config.get('velocity_m_s', [0, 0, 0]), 'velocity')
        self.acceleration = vector(config.get('acceleration_m_s2', [0, 0, 0]), 'acceleration')
        self.offset = vector(config.get('offset_m', [0, 0, 0]), 'offset')
        self.seed_offset = number(config.get('seed_offset_m', 0), 'seed_offset_m', 0)
        self.period = number(config.get('reverse_period_seconds', 1), 'reverse_period', .001)

    def reset(self, context):
        super().reset(context)
        self.origin = [v+self.random.uniform(-self.seed_offset, self.seed_offset) for v in self.offset]

    def state(self, seconds, initial_pose):
        t = number(seconds, 'simulation seconds', 0)
        if self.mode == 'static':
            delta, velocity = [0]*3, [0]*3
        elif self.mode == 'reverse':
            phase = t % (2*self.period)
            progress = phase if phase <= self.period else 2*self.period-phase
            direction = 1 if phase < self.period else -1
            delta = [v*progress for v in self.velocity]
            velocity = [v*direction for v in self.velocity]
        else:
            accel = self.acceleration if self.mode == 'accel' else [0]*3
            delta = [v*t+.5*a*t*t for v, a in zip(self.velocity, accel)]
            velocity = [v+a*t for v, a in zip(self.velocity, accel)]
        return dict(position_m=[p+o+d for p, o, d in zip(initial_pose[:3], self.origin, delta)],
                    velocity_m_s=velocity, rpy_rad=list(initial_pose[3:]), developer_truth=True)


class KinematicGimbal(Plugin):
    kind = 'gimbal'
    observables = ('q_rad', 'v_rad_s', 'receipt_time_ns', 'command_sequence')

    def configure(self, config):
        fields(config, ('rate_limit_rad_s', 'acceleration_limit_rad_s2', 'saturation_rad', 'application_delay_seconds'))
        self.rate = number(config['rate_limit_rad_s'], 'rate_limit', .000001) if 'rate_limit_rad_s' in config else None
        self.accel = number(config['acceleration_limit_rad_s2'], 'acceleration_limit', .000001) if 'acceleration_limit_rad_s2' in config else None
        self.saturation = vector(config['saturation_rad'], 'saturation', 2) if 'saturation_rad' in config else [None, math.pi/2]
        if any(s is not None and s <= 0 for s in self.saturation):
            raise ValueError('Saturation must be positive')
        self.delay_seconds = number(config.get('application_delay_seconds', 0), 'application_delay', 0)

    def reset(self, context):
        super().reset(context)
        self.q, self.v, self.goal = [0., 0.], [0., 0.], [0., 0.]
        self.pending = deque()
        self.last_time = 0.
        self.sequence = self.receipt_time_ns = 0

    def submit(self, q, simulation_ns, sequence):
        q = vector(list(q), 'command', 2)
        self.pending.append((simulation_ns/1e9+self.delay_seconds, q, simulation_ns, sequence))

    def step(self, seconds):
        dt = max(0, seconds-self.last_time)
        self.last_time = seconds
        while self.pending and self.pending[0][0] <= seconds:
            _, goal, self.receipt_time_ns, self.sequence = self.pending.popleft()
            self.goal = [max(-s, min(s, g)) if s is not None else g for g, s in zip(goal, self.saturation)]
        if self.rate is None and self.accel is None:
            self.v = [(g-q)/dt if dt > 0 else 0. for g,q in zip(self.goal,self.q)]
            self.q = list(self.goal)  # Exact absolute-angle compatibility, including same-clock receipts.
        elif dt > 0:
            for i in range(2):
                difference = self.goal[i]-self.q[i]
                desired = max(-self.rate, min(self.rate, difference/dt)) if self.rate is not None else difference/dt
                velocity = max(self.v[i]-self.accel*dt, min(self.v[i]+self.accel*dt, desired)) if self.accel is not None else desired
                self.q[i] += velocity*dt
                if self.saturation[i] is not None:
                    self.q[i] = max(-self.saturation[i], min(self.saturation[i], self.q[i]))
                self.v[i] = velocity
        return dict(q_rad=list(self.q), v_rad_s=list(self.v), receipt_time_ns=self.receipt_time_ns, command_sequence=self.sequence)


class Timing(Plugin):
    kind = 'timing'
    observables = ('delay_seconds',)

    def configure(self, config):
        fields(config, ('delay_seconds', 'jitter_seconds'))
        self.base = number(config.get('delay_seconds', 0), 'delay_seconds', 0)
        self.jitter = number(config.get('jitter_seconds', 0), 'jitter_seconds', 0)
        if self.base+self.jitter > 5:
            raise ValueError('Reference timing delay is bounded at 5 simulator seconds')

    def delay(self, source_seconds):
        return max(0, self.base+self.random.uniform(-self.jitter, self.jitter))


class Sensor(Plugin):
    kind = 'sensor'
    observables = ('occluded', 'dropped', 'calibration_scale')

    def process(self, source_seconds, image):
        return dict(image=image, dropped=False, occluded=False, calibration_scale=1.)
