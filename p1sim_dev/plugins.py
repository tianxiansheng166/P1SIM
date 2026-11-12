"""Versioned Python plugin boundary, deliberately independent of ROS."""
from dataclasses import dataclass
import hashlib
import importlib
import inspect
import random
from pathlib import Path
from p1sim_dev.config import PLUGIN_API_VERSION


@dataclass(frozen=True)
class Context:
    seed: int
    epoch: int
    kind: str

    def rng(self):
        # Independent streams: adding a plugin never changes another's seed stream.
        return random.Random(f'{self.seed}:{self.kind}')


class Plugin:
    api_version = 1
    version = '1.0.0'
    kind = None
    observables = ()

    def configure(self, config):
        if config:
            raise ValueError('Unsupported plugin configuration')

    def start(self, context):
        self.reset(context)

    def reset(self, context):
        self.context = context
        self.random = context.rng()

    def close(self):
        pass


class Registry:
    def __init__(self, config, epoch):
        self.plugins = {}
        self.identities = {}
        try:
            for kind, spec in config['plugins'].items():
                module, name = spec['entry'].split(':', 1)
                cls = getattr(importlib.import_module(module), name)
                if not issubclass(cls, Plugin) or type(cls.api_version) is not int or cls.api_version != PLUGIN_API_VERSION or cls.kind != kind:
                    raise ValueError('Incompatible plugin '+spec['entry'])
                if not isinstance(cls.version, str) or not isinstance(cls.observables, tuple):
                    raise ValueError('Plugin must declare version and observables')
                required = dict(target=('state',), gimbal=('submit', 'step'), sensor=('process',), timing=('delay',), world=('resolve',))[kind]
                if not all(callable(getattr(cls, method, None)) for method in required):
                    raise ValueError('Missing plugin callback: '+kind)
                plugin = cls()
                self.plugins[kind] = plugin
                plugin.configure(spec['config'])
                plugin.start(Context(config['seed'], epoch, kind))
                source = Path(inspect.getfile(cls))
                self.identities[kind] = dict(entry=spec['entry'], version=cls.version, api_version=cls.api_version,
                    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(), observables=cls.observables,
                    config=spec['config'])
        except Exception:
            self.close()
            raise

    def __getitem__(self, key):
        return self.plugins[key]

    def close(self):
        for plugin in reversed(list(self.plugins.values())):
            plugin.close()
