"""Double-click launcher entry point; reuse an already running local viewer."""
import json
from pathlib import Path
import sys
from urllib.error import URLError
from urllib.request import ProxyHandler, build_opener
import webbrowser

import bridge_server


def main():
    url = 'http://127.0.0.1:18768'
    try:
        with build_opener(ProxyHandler({})).open(url + '/api/identification', timeout=2) as response:
            state = json.load(response)
        if (isinstance(state, dict) and state.get('status') in ('idle', 'running', 'complete', 'error')
                and Path(state.get('workspace', '')).resolve() == Path(__file__).resolve().parents[1]):
            print('Opening the sensor simulation already running at ' + url)
            webbrowser.open(url)
            return
    except (URLError, OSError, ValueError):
        pass
    sys.argv = [sys.argv[0], '--port', '18768', '--open-browser']
    try:
        bridge_server.main()
    except OSError as error:
        print('Could not start the simulation. Port 18768 may be in use. ' + str(error), file=sys.stderr)
        raise SystemExit(1) from error


if __name__ == '__main__':
    main()
