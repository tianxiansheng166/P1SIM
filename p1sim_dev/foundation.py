"""Run identity, plugin ownership and persisted lifecycle envelope."""
import hashlib
import json
from pathlib import Path
import sys
import time
from p1sim_dev.config import normalize, digest, atomic_json, FOUNDATION_VERSION
from p1sim_dev.plugins import Registry


class Foundation:
    def __init__(self, state='/data'):
        self.state = Path(state)
        self.envelope = json.loads((self.state/'run.json').read_text())
        root = Path(__file__).resolve().parents[1]
        for name, expected in self.envelope['source'].get('runtime_checksums', {}).items():
            if hashlib.sha256((root/name).read_bytes()).hexdigest() != expected:
                raise ValueError('Loaded runtime/source mismatch: '+name)
        self.config = normalize(self.envelope['config'])
        if digest(self.config) != self.envelope['config_digest']:
            raise ValueError('Run configuration digest mismatch')
        for path in self.config['workspace']['plugin_paths']:
            sys.path.insert(0, path)
        self.registry = Registry(self.config, self.envelope['epoch'])
        self.envelope['plugins'] = self.registry.identities
        self.simulation_ns = 0
        self.phase = 'STARTING'
        self.event('STARTING')

    def identity(self):
        return {key: self.envelope[key] for key in ('run_id', 'run_key', 'foundation_version', 'interface_version',
                'plugin_api_version', 'config_digest', 'seed', 'epoch', 'source')}

    def observable(self, kind, **values):
        return dict(schema='p1sim.'+kind, schema_version=1, **self.identity(), **values)

    def event(self, phase):
        self.phase = phase
        value = self.observable('lifecycle', state=phase, simulation_time_ns=self.simulation_ns,
                                clock_behavior='restart_at_zero_per_epoch', plugins=self.registry.identities,
                                wall_time_unix=time.time())
        atomic_json(self.state/'manifest.json', value)
        with (self.state/'lifecycle.jsonl').open('a') as output:
            output.write(json.dumps(value)+'\n')
        return value

    def close(self):
        self.registry.close()
        self.event('ENDED')
