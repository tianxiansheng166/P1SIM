"""Workspace setup scripts must not erase or reinterpret the user's command."""
from pathlib import Path
import runpy
import subprocess
import tempfile
import unittest
from unittest.mock import patch, Mock


class LauncherTest(unittest.TestCase):
    def test_doctor_missing_image_is_fast_and_does_not_pull_or_build(self):
        cli = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'p1sim'))
        compose = Mock(side_effect=lambda *args, **kwargs: (
            Mock(stdout='p1sim-foundation:missing-test-image\n')
            if args[:2] == ('config', '--images') else Mock(returncode=0)))
        with tempfile.TemporaryDirectory() as directory:
            with patch.dict(cli['main'].__globals__, {
                'compose': compose,
                'STATE': Path(directory)/'state'
            }), patch('sys.argv', ['p1sim', 'dev', 'doctor']), \
                 patch('subprocess.run', side_effect=[
                     Mock(returncode=0), Mock(returncode=1)
                 ]) as docker_calls:
                self.assertEqual(cli['main'](), 2)
        self.assertEqual(docker_calls.call_count, 2)
        self.assertEqual(docker_calls.call_args_list[1].args[0][:3],
                         ['docker', 'image', 'inspect'])
        self.assertEqual(compose.call_count, 2)
        self.assertEqual(compose.call_args_list[1].args[:2],
                         ('config', '--images'))

    def test_doctor_installed_image_disables_registry_pulls(self):
        cli = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'p1sim'))
        compose = Mock(side_effect=lambda *args, **kwargs: (
            Mock(stdout='p1sim-foundation:my-local-image\n')
            if args[:2] == ('config', '--images') else Mock(returncode=0)))
        with tempfile.TemporaryDirectory() as directory:
            with patch.dict(cli['main'].__globals__, {
                'compose': compose,
                'STATE': Path(directory)/'state'
            }), patch('sys.argv', ['p1sim', 'dev', 'doctor']), \
                 patch('subprocess.run', side_effect=[
                     Mock(returncode=0), Mock(returncode=0)
                 ]):
                self.assertEqual(cli['main'](), 0)
        args = compose.call_args_list[-1].args
        self.assertIn('--pull', args)
        self.assertEqual(args[args.index('--pull') + 1], 'never')

    def test_setup_that_clears_positional_arguments(self):
        namespace = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'p1sim'))
        fake = Mock()
        fake.wait.return_value = 0
        with patch('subprocess.Popen', return_value=fake) as popen:
            namespace['execute'](['printf', '%s', 'spaces; $(echo no)'], interactive=False)
        invocation = popen.call_args.args[0]
        index = invocation.index('-lc')
        script = invocation[index+1]
        with tempfile.TemporaryDirectory() as directory:
            setup = Path(directory) / 'setup.bash'
            setup.write_text('set --\nexport SETUP_WORKED=yes\n')
            # Replace only the ROS underlay source for this dependency-free test.
            script = script.replace('source /opt/ros/humble/setup.bash;', ':;')
            result = subprocess.run(['bash', '-c', script, 'p1sim', 'printf', '%s', 'spaces; $(echo no)'],
                                    env={'PATH': '/usr/bin:/bin', 'P1SIM_WORKSPACE_SETUP': str(setup)},
                                    text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout, 'spaces; $(echo no)')


if __name__ == '__main__':
    unittest.main()
