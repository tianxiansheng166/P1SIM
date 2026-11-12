"""Contract, numerical and invalid-input checks independent of ROS."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from p1sim_dev.config import normalize, digest
from p1sim_dev.plugins import Registry, Context
from p1sim_dev.builtins import Trajectory, KinematicGimbal
from p1sim_dev.host import prepare
from p1sim_dev.host import source_identity
from p1sim_dev.foundation import Foundation

ROOT = Path(__file__).resolve().parents[1]


class FoundationTest(unittest.TestCase):
    def test_release_archive_identity_without_git_and_tamper_rejection(self):
        # Mirrors a Release tarball extraction, which has no .git directory.
        from shutil import copyfile
        import hashlib
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            paths = ('release.json', 'p1sim', 'Dockerfile', 'compose.yaml',
                     'p1sim_dev/host.py')
            for name in paths:
                (root/name).parent.mkdir(parents=True, exist_ok=True)
                copyfile(ROOT/name,root/name)
            (root/'SHA256SUMS').write_text(''.join(
                hashlib.sha256((root/name).read_bytes()).hexdigest()+'  '+name+'\n'
                for name in paths))
            result = source_identity(root)
            self.assertIsNone(result['git_sha'])
            self.assertFalse(result['dirty'])
            self.assertEqual(result['identity_source'],'VERIFIED_RELEASE_ARCHIVE')
            self.assertTrue(result['content_digest'])
            (root/'release.json').write_text('{}')
            with self.assertRaisesRegex(ValueError,'SOURCE_ARCHIVE_CHECKSUM_MISMATCH'):
                source_identity(root)

    def test_config_canonical_and_fail_fast(self):
        a = normalize({'seed': 12})
        self.assertEqual(digest(a), digest(normalize(json.loads(json.dumps(a)))))
        for config in ({'seed': True}, {'seed': -1}, {'seed': 1.5}, {'config_version': 2},
                       {'interface_version': True}, {'duration_seconds': float('nan')},
                       {'topics': {'oops':'/a'}}, {'workspace': {'plugin_paths':['/tmp/other']}},
                       {'testspec': {}}, {'plugins': {'sensor': {'entry':'x:y','config':[]}}}):
            with self.subTest(config=config), self.assertRaises(ValueError):
                normalize(config)

    def trajectory(self, mode, seed=3, epoch=0):
        p = Trajectory()
        p.configure(dict(mode=mode, velocity_m_s=[1,2,0], acceleration_m_s2=[2,0,0], seed_offset_m=.1))
        p.start(Context(seed, epoch, 'target'))
        return p

    def test_trajectory_units_reverse_reset_seed(self):
        initial = [2,0,1,0,0,0]
        for mode, delta in [('static',[0,0,0]),('linear',[2,4,0]),('accel',[6,4,0]),('reverse',[0,0,0])]:
            plugin = self.trajectory(mode)
            origin = plugin.state(0,initial)['position_m']
            value = plugin.state(2,initial)
            for o,v,d in zip(origin,value['position_m'],delta):
                self.assertAlmostEqual(v-o,d)
            plugin.reset(Context(3,1,'target'))
            self.assertEqual(value, plugin.state(2,initial))
            self.assertNotEqual(value, self.trajectory(mode,seed=4).state(2,initial))
        reverse = self.trajectory('reverse')
        self.assertEqual(reverse.state(.5,initial)['position_m'],reverse.state(1.5,initial)['position_m'])
        self.assertEqual(reverse.state(1.5,initial)['velocity_m_s'],[-1,-2,0])

    def test_exact_unlimited_compatibility_without_clock_advance(self):
        p = KinematicGimbal()
        p.configure({})
        p.start(Context(0,0,'gimbal'))
        p.submit([1e15,.1],0,1)
        self.assertEqual(p.step(0)['q_rad'],[1e15,.1])
        self.assertEqual(p.step(.1)['v_rad_s'],[0.,0.])

    def test_gimbal_delay_rate_acceleration_saturation(self):
        p = KinematicGimbal()
        p.configure(dict(application_delay_seconds=.2,rate_limit_rad_s=.5,
                         acceleration_limit_rad_s2=1.,saturation_rad=[.4,.4]))
        p.start(Context(0,0,'gimbal'))
        p.submit([2,-2],0,1)
        previous = p.step(0)
        for tick in range(1,101):
            state = p.step(tick*.01)
            if tick < 20:
                self.assertEqual(state['q_rad'],[0,0])
            for q,v,oldv in zip(state['q_rad'], state['v_rad_s'], previous['v_rad_s']):
                self.assertLessEqual(abs(q),.4)
                self.assertLessEqual(abs(v),.5)
                self.assertLessEqual(abs(v-oldv),.01000001)
            previous = state
        self.assertEqual(state['command_sequence'],1)
        self.assertEqual(state['receipt_time_ns'],0)

    def test_registry_external_and_incompatible(self):
        config = normalize({'plugins': {'sensor': {'entry':'examples.visibility_plugin:PeriodicVisibility','config':{}}}})
        r = Registry(config,0)
        self.assertEqual(r.identities['sensor']['api_version'],1)
        from types import SimpleNamespace
        image = SimpleNamespace(data=b'123456')
        self.assertTrue(r['sensor'].process(.01,image)['occluded'])
        self.assertEqual(image.data,bytes(6))
        r.close()
        config['plugins']['sensor']['entry'] = 'p1sim_dev.builtins:Trajectory'
        with self.assertRaises(ValueError):
            Registry(config,0)
        from unittest.mock import patch
        with patch.object(Trajectory,'api_version',2), self.assertRaises(ValueError):
            Registry(normalize({}),0)

    def test_persisted_epoch_identity(self):
        with tempfile.TemporaryDirectory() as d:
            state = Path(d)
            envelope = prepare(ROOT,state,normalize({'seed':42}))
            f = Foundation(state)
            first = f.registry['target'].state(1,[0]*6)
            f.event('READY')
            f.event('RESETTING')
            f.close()
            envelope['epoch'] += 1
            (state/'run.json').write_text(json.dumps(envelope))
            f = Foundation(state)
            self.assertEqual(f.identity()['epoch'],1)
            self.assertEqual(f.identity()['run_id'],envelope['run_id'])
            self.assertEqual(first,f.registry['target'].state(1,[0]*6))
            f.close()


if __name__ == '__main__':
    unittest.main()
