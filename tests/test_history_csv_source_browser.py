"""Run the browser integration with an in-process server on port 0."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading

ROOT = Path(__file__).resolve().parents[1]
if not (ROOT / 'digital_twin').exists():
    ROOT = Path('C:/Users/karl/Desktop/personal_branch_hand/blackforest')
sys.path.insert(0, str(ROOT / 'digital_twin'))
import bridge_server as bridge

if __name__ == '__main__':
    with tempfile.TemporaryDirectory() as folder:
        server = bridge.ThreadingHTTPServer(('127.0.0.1', 0), bridge.Handler)
        server.capture_directory = folder
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            env = {**os.environ, 'TEST_RECEIVER_URL': f'http://127.0.0.1:{server.server_port}'}
            result = subprocess.run(['node', str(Path(__file__).with_suffix('.js'))], env=env, timeout=180)
        finally:
            server.shutdown()
            server.server_close()
            worker.join()
            if hasattr(server, 'received_runs'):
                server.received_runs.close()
        raise SystemExit(result.returncode)
