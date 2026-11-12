"""Run normal user commands, with explicit process-group cleanup from the host."""
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time


def main():
    action, token, *command = sys.argv[1:]
    if not re.fullmatch('[0-9a-f]{32}', token):
        raise ValueError('Invalid command identity')
    path = Path('/tmp') / ('p1sim-command-' + token)
    if action == 'stop':
        if path.exists():
            pid = int(path.read_text())
            try:
                if os.getpgid(pid) != pid:
                    raise ValueError('Command group identity changed')
                for signum, wait in ((signal.SIGINT, 5), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
                    os.killpg(pid, signum)
                    deadline = time.monotonic() + wait
                    while path.exists() and time.monotonic() < deadline:
                        time.sleep(0.1)
                    if not path.exists():
                        break
            except ProcessLookupError:
                pass
        return 0
    process = subprocess.Popen(command, start_new_session=True)
    path.write_text(str(process.pid))
    try:
        return process.wait()
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        path.unlink(missing_ok=True)


if __name__ == '__main__':
    sys.exit(main())
