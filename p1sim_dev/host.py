"""Host-side public run preparation and scheduler-callable batch primitive."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import uuid
from p1sim_dev.config import load, digest, atomic_json, FOUNDATION_VERSION


def source_identity(root):
    root = Path(root).resolve()
    # A GitHub Release source tarball deliberately contains no .git directory.
    # For that deployment, bind the same identity to a fully verified,
    # immutable SHA256SUMS manifest. Never invent a Git commit SHA.
    git_checkout = (root/'.git').exists()
    if git_checkout:
        tracked = subprocess.check_output(
            ['git', '-C', str(root), 'ls-files', '--cached', '--others',
             '--exclude-standard', '-z']).decode().split('\0')
        checksums = {name: hashlib.sha256((root/name).read_bytes()).hexdigest()
                     for name in tracked if name and name != 'SHA256SUMS' and (root/name).is_file()}
        git_sha = subprocess.check_output(
            ['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
        dirty = bool(subprocess.check_output(
            ['git', '-C', str(root), 'status', '--porcelain'], text=True).strip())
        identity_source = 'GIT_CHECKOUT'
    else:
        manifest = root/'SHA256SUMS'
        if not manifest.is_file():
            raise ValueError('SOURCE_ARCHIVE_SHA256SUMS_REQUIRED')
        checksums = {}
        for line in manifest.read_text().splitlines():
            if len(line) < 67 or line[64:66] != '  ':
                raise ValueError('SOURCE_ARCHIVE_MANIFEST_INVALID')
            expected, name = line[:64], line[66:]
            rel = Path(name)
            if (not all(c in '0123456789abcdef' for c in expected) or
                not name or rel.is_absolute() or '..' in rel.parts or
                name in checksums or name == 'SHA256SUMS'):
                raise ValueError('SOURCE_ARCHIVE_MANIFEST_INVALID')
            path = root/rel
            if not path.is_file() or path.is_symlink():
                raise ValueError('SOURCE_ARCHIVE_MISSING_FILE:'+name)
            actual = hashlib.sha256(path.read_bytes()).hexdigest()
            if actual != expected:
                raise ValueError('SOURCE_ARCHIVE_CHECKSUM_MISMATCH:'+name)
            checksums[name] = actual
        required = ('release.json','p1sim','Dockerfile','compose.yaml',
                    'p1sim_dev/host.py')
        if any(name not in checksums for name in required):
            raise ValueError('SOURCE_ARCHIVE_MANIFEST_INCOMPLETE')
        for folder in ('p1sim_dev','worlds'):
            for path in (root/folder).rglob('*'):
                if (path.is_file() and '__pycache__' not in path.parts and
                    path.suffix != '.pyc' and
                    path.relative_to(root).as_posix() not in checksums):
                    raise ValueError('SOURCE_ARCHIVE_UNLISTED_RUNTIME_FILE')
        git_sha = None
        dirty = False
        identity_source = 'VERIFIED_RELEASE_ARCHIVE'
    runtime_checksums = {name: sha for name, sha in checksums.items() if name.startswith(('p1sim_dev/', 'worlds/')) or name == 'release.json'}
    return dict(runtime_checksums=runtime_checksums, git_sha=git_sha,
        dirty=dirty, identity_source=identity_source,
        content_digest=digest(checksums),
        release=json.loads((root/'release.json').read_text()))


def prepare(root, state, config):
    state.mkdir(parents=True, exist_ok=True)
    run_key = digest(config)
    envelope = dict(run_id=uuid.uuid4().hex, run_key=run_key, config_digest=run_key,
                    config=config, seed=config['seed'], epoch=0, foundation_version=FOUNDATION_VERSION,
                    interface_version=1, plugin_api_version=1, source=source_identity(root))
    atomic_json(state/'run.json', envelope)
    return envelope


def configure_env(env, config):
    env['P1SIM_SCENE'] = config['scene']
    for name, key in [('planner', 'P1SIM_GIMBAL_TOPIC'), ('tracker', 'P1SIM_TRACKER_TOPIC')]:
        if name in config['topics']:
            env[key] = config['topics'][name]
    if config['workspace']['path']:
        env['P1SIM_USER_WORKSPACE'] = str(Path(config['workspace']['path']).expanduser().resolve())
    env['P1SIM_WORKSPACE_SETUP'] = config['workspace']['setup']


def batch(root, args):
    parser = argparse.ArgumentParser(description='One isolated headless Foundation run (duration in simulator seconds)')
    parser.add_argument('--config', required=True)
    parser.add_argument('--seed', type=int)
    parser.add_argument('--output', required=True)
    parser.add_argument('--ready-timeout', type=float)
    parser.add_argument('--stop-file', help='External stop mode: end when this path exists')
    opts = parser.parse_args(args)
    config = load(opts.config, opts.seed)
    if opts.ready_timeout is not None:
        config['ready_timeout_seconds'] = opts.ready_timeout
        config = load_value(config)
    if not config['headless']:
        raise ValueError('batch requires headless=true')
    if config['duration_seconds'] is None and not opts.stop_file:
        raise ValueError('External-stop batch requires --stop-file')
    output = Path(opts.output).expanduser().resolve()
    # Never reuse a scheduler output, accidentally reset a running project or erase evidence.
    output.mkdir(parents=True, exist_ok=False)
    state = output/'runtime'
    run = prepare(root, state, config)
    env = dict(os.environ, P1SIM_STATE_DIR=str(state), P1SIM_PROJECT='p1sim-'+run['run_id'][:12], P1SIM_PREPARED='1')
    configure_env(env, config)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        env['P1SIM_STATUS_PORT'] = str(sock.getsockname()[1])
    cli = [str(root/'p1sim'), 'dev']
    result = dict(schema='p1sim.batch_result', schema_version=1, run_id=run['run_id'],
                  run_key=run['run_key'], exit_status='FAILED', clean_teardown=False)
    log = (output/'batch.log').open('w')
    def call(*command):
        process = subprocess.Popen(cli+list(command), env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            code = process.wait()
        except BaseException:
            for signum, timeout in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
                if process.poll() is not None:
                    break
                os.killpg(process.pid, signum)
                try:
                    process.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    continue
            raise
        if code:
            raise subprocess.CalledProcessError(code, cli+list(command))
    previous_handler = signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        call('up')
        import urllib.request
        def status():
            with urllib.request.urlopen('http://127.0.0.1:'+env['P1SIM_STATUS_PORT']+'/status', timeout=5) as reply:
                return json.load(reply)
        first = status()
        atomic_json(output/'ready.json', first)
        if config['recording']:
            request = urllib.request.Request('http://127.0.0.1:'+env['P1SIM_STATUS_PORT']+'/record/start',
                data=b'{"name":"batch"}', headers={'Content-Type':'application/json'})
            urllib.request.urlopen(request, timeout=15).close()
        deadline = time.monotonic()+max(120, (config['duration_seconds'] or 0)*30)
        with (output/'observables.jsonl').open('w') as observations:
            while True:
                current = status()
                observations.write(json.dumps(current)+'\n')
                observations.flush()
                if not current['ready']:
                    raise RuntimeError('Runtime lost READY')
                elapsed = (current['simulation_time_ns']-first['simulation_time_ns'])/1e9
                if config['duration_seconds'] is not None and elapsed >= config['duration_seconds']:
                    break
                if opts.stop_file and Path(opts.stop_file).exists():
                    break
                if config['duration_seconds'] is not None and time.monotonic() > deadline:
                    raise TimeoutError('Simulation duration watchdog expired')
                time.sleep(.1)
        atomic_json(output/'final.json', current)
        result.update(exit_status='COMPLETED', simulation_elapsed_seconds=elapsed)
    except KeyboardInterrupt:
        result.update(exit_status='INTERRUPTED')
    except Exception as error:
        result['error'] = str(error)
    finally:
        try:
            call('down')
            project = env['P1SIM_PROJECT']
            containers = subprocess.check_output(['docker', 'ps', '-aq', '--filter', 'label=com.docker.compose.project='+project], text=True).strip()
            networks = subprocess.check_output(['docker', 'network', 'ls', '-q', '--filter', 'label=com.docker.compose.project='+project], text=True).strip()
            result['clean_teardown'] = not containers and not networks
        except Exception as error:
            result['teardown_error'] = str(error)
        log.close()
        signal.signal(signal.SIGTERM, previous_handler)
        atomic_json(output/'result.json', result)
    print(json.dumps(result))
    return 0 if result['exit_status'] == 'COMPLETED' and result['clean_teardown'] else 1


def load_value(value):
    from p1sim_dev.config import normalize
    return normalize(value)
