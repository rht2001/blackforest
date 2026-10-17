"""Read positions already calculated by the open digital twin.

Each item is {id, x, y, z, pressure}, in left_base_link metres. Pressure is
calibrated sensor counts, or None when unavailable/stale. An empty list means
the bridge has not received positions from a viewer within four seconds.
"""
import json
from urllib.request import ProxyHandler, build_opener


def get_taxel_positions(base_url='http://127.0.0.1:18765', *, timeout=1):
    """Read all 1062 displayed taxel positions and pressures. No FK or polling."""
    with build_opener(ProxyHandler({})).open(base_url.rstrip('/')+'/api/taxels', timeout=timeout) as response:
        return json.load(response)
