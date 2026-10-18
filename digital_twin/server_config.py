"""One local address for telemetry, taxels, the viewer and estimation."""
import hashlib
from pathlib import Path

HOST = '127.0.0.1'
PORT = 18765
URL = f'http://{HOST}:{PORT}'
WORKSPACE = Path(__file__).resolve().parents[1]


def source_signature():
    """Compare running code with files on disk without reusing stale modules."""
    folder = Path(__file__).resolve().parent
    digest = hashlib.sha256()
    files = [folder/'bridge_server.py', folder/'received_runs.py', folder/'server_config.py',
             folder/'start_sensor_simulation.py', folder/'hand_twin_6dof.html']
    files += sorted((folder/'sensor_identification').glob('*.py'))
    files += sorted((folder/'sensor_identification').glob('*.js'))
    for path in files:
        if path.is_file():
            digest.update(str(path.relative_to(folder)).encode('utf-8'))
            digest.update(path.read_bytes())
    return digest.hexdigest()
