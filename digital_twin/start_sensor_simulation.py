"""Start or reuse the current hand server at the single canonical address."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from urllib.error import URLError
from urllib.request import ProxyHandler, build_opener
import webbrowser

import bridge_server
from server_config import HOST, PORT, URL, WORKSPACE, source_signature


def read_state(path):
    with build_opener(ProxyHandler({})).open(URL+path, timeout=3) as response:
        return json.load(response)


def stop_stale_server():
    """Stop only the Python listener verified to serve this workspace."""
    if os.name != 'nt':
        raise RuntimeError('Stop the existing hand server, then launch again.')
    # Workspace was checked through HTTP; independently check the port owner
    # and executable command before terminating it. Never stop robot control.
    script = f"""
$listener = @(Get-NetTCPConnection -LocalAddress '{HOST}' -LocalPort {PORT} -State Listen)
if ($listener.Count -ne 1) {{ throw 'Listener changed; restart cancelled.' }}
$server = Get-CimInstance Win32_Process -Filter ('ProcessId = ' + $listener[0].OwningProcess)
if ($server.Name -notin @('python.exe','pythonw.exe') -or $server.CommandLine -notmatch '(bridge_server|start_sensor_simulation)\\.py') {{ throw 'Unrecognized server; restart cancelled.' }}
& taskkill.exe /PID $server.ProcessId /F
if ($LASTEXITCODE -ne 0) {{ throw 'Could not stop the outdated server.' }}
"""
    subprocess.run(['powershell.exe', '-NoProfile', '-NonInteractive', '-Command', script],
                   check=True, creationflags=subprocess.CREATE_NO_WINDOW)
    deadline = time.monotonic()+5
    while time.monotonic() < deadline:
        try:
            read_state('/health')
        except (URLError, OSError):
            return
        time.sleep(.1)
    raise RuntimeError('Old server did not stop. Close its window and launch again.')


def main():
    try:
        health = read_state('/health')
    except (URLError, OSError):
        health = None
    if health is not None:
        state = health if health.get('workspace') else read_state('/api/identification')
        if Path(state.get('workspace', '')).resolve() != WORKSPACE:
            raise RuntimeError(f'{URL} is occupied by another application; nothing was stopped.')
        if health.get('source_signature') == source_signature():
            print('Opening the current hand server at '+URL)
            webbrowser.open(URL)
            return
        print('Restarting the outdated hand viewer at '+URL)
        stop_stale_server()
    sys.argv = [sys.argv[0], '--port', str(PORT), '--open-browser']
    try:
        bridge_server.main()
    except OSError as error:
        raise RuntimeError(f'Could not start {URL}: {error}') from error


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, subprocess.CalledProcessError, ValueError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1) from error
