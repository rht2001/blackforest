"""Local telemetry bridge for the 3D RH56DFTP hand viewer.

Start: python bridge_server.py
Viewer: http://127.0.0.1:18765/
Python program: POST JSON to http://127.0.0.1:18765/api/state

The server NEVER opens a Modbus connection or sends commands to hardware.
No third-party dependencies. Listens ONLY on 127.0.0.1.
"""
from __future__ import annotations
import argparse
from copy import deepcopy
import json
import math
import threading
import time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlsplit, parse_qs

from tactile_layout import REGIONS

FIELDS = ('angles', 'angles_set', 'angles_act', 'force_act', 'position_act', 'tactile', 'tactile_regions', 'tactile_means', 'source', 'simulated')
STATE = {'angles_set': None, 'angles_act': None, 'force_act': None, 'tactile': {}, 'tactile_regions': {}, 'field_received_at': {}, 'received_at': 0, 'seq': 0}
LOCK = threading.Lock()
TAXELS = []
TAXELS_RECEIVED_AT = 0
TAXEL_IDS = {f'{key}:{i}' for key, (rows, cols) in REGIONS.items() for i in range(rows*cols)}
HTML = Path(__file__).with_name('hand_twin_6dof.html')
TACTILE_HTML = Path(__file__).with_name('tactile_2d.html')

IDENTIFICATION_SETUP_LOCK = threading.Lock()


def identification_service(server):
    # Import optional geometry dependencies only when this feature is used.
    with IDENTIFICATION_SETUP_LOCK:
        if not hasattr(server, 'sensor_identification'):
            from sensor_identification.web_service import IdentificationService
            server.sensor_identification = IdentificationService()
        return server.sensor_identification

def vector(value, name, lo, hi):
    if value is None: return None
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError(f'{name} must be a list of 6 numbers')
    vals=[]
    for x in value:
        if not isinstance(x, (float, int)) or isinstance(x,bool) or not math.isfinite(x) or not lo <= x <= hi:
            raise ValueError(f'{name} contains invalid values ({lo}..{hi})')
        vals.append(round(float(x),3))
    return vals

def normalize(payload):
    if not isinstance(payload,dict):raise ValueError('Expected a JSON object')
    if 'tactile_matrix' in payload:
        from blackforest_adapter import datadict_payload
        payload=datadict_payload(payload,valid_regions=payload.get('tactile_valid'),simulated=payload.get('simulated',False))
    out={}
    for name in ('angles','angles_set','angles_act'):
        if name in payload:out[name]=vector(payload[name],name,0,1000)
    if 'force_act' in payload:out['force_act']=vector(payload['force_act'],'force_act',-4000,4000)
    if 'position_act' in payload:out['position_act']=vector(payload['position_act'],'position_act',0,65535)
    if 'tactile_means' in payload:
        means=payload['tactile_means']
        if not isinstance(means,dict) or any(k not in REGIONS or isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not 0<=v<=4095 for k,v in means.items()):raise ValueError('Invalid region means')
        out['tactile_means']=dict(means)
    if 'source' in payload:
        if not isinstance(payload['source'],str) or len(payload['source'])>160:raise ValueError('Invalid data source')
        out['source']=payload['source']
    if 'simulated' in payload:
        if type(payload['simulated']) is not bool:raise ValueError('simulated must be boolean')
        out['simulated']=payload['simulated']
    if 'tactile' in payload:
        t=payload['tactile']
        if not isinstance(t,dict):raise ValueError('tactile must be an object')
        clean={}
        for name, value in t.items():
            if name not in ('little','ring','middle','index','thumb','palm'):continue
            if isinstance(value,dict):
                value={k:v for k,v in value.items() if k in ('max','mean')}
                if not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) and 0<=v<=4095 for v in value.values()):
                    raise ValueError('Tactile values outside 0..4095')
            elif not (isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value) and 0<=value<=4095):
                raise ValueError('Tactile values outside 0..4095')
            clean[name]=value
        out['tactile']=clean
    if 'tactile_regions' in payload:
        regions = payload['tactile_regions']
        if not isinstance(regions, dict): raise ValueError('tactile_regions must be an object')
        clean = {}
        for key, matrix in regions.items():
            if key not in REGIONS: raise ValueError('Unbekannte Sensor region: '+key)
            rows, cols = REGIONS[key]
            if not isinstance(matrix,list) or len(matrix)!=rows or any(not isinstance(row,list) or len(row)!=cols for row in matrix):
                raise ValueError('Wrong matrix dimensions: '+key)
            if any(not isinstance(v,(int,float)) or isinstance(v,bool) or not math.isfinite(v) or not 0<=v<=4095 for row in matrix for v in row):
                raise ValueError('Sensor values outside 0..4095')
            clean[key]=matrix
        out['tactile_regions']=clean
    if not out:raise ValueError('No valid fields. Allowed: '+', '.join(FIELDS))
    return out

class Handler(BaseHTTPRequestHandler):
    server_version='HandTwinBridge/1.0'
    def log_message(self,*args):pass
    def send_data(self,body,code=200,ctype='application/json; charset=utf-8'):
        self.send_response(code)
        self.send_header('Content-Type',ctype)
        self.send_header('Content-Length',str(len(body)))
        self.send_header('Cache-Control','no-cache, no-store')
        self.send_header('Access-Control-Allow-Origin','*')
        self.send_header('Access-Control-Allow-Methods','GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers','Content-Type')
        self.end_headers();self.wfile.write(body)
    def respond_json(self,data,code=200):
        self.send_data(json.dumps(data,ensure_ascii=False,allow_nan=False).encode('utf-8'),code)
    def do_OPTIONS(self):self.send_data(b'',204)
    def do_GET(self):
        url = urlsplit(self.path)
        if self.path in ('/sensor_identification/simulation.js', '/sensor_identification/measurements.js', '/sensor_identification/history.js'):
            self.send_data(Path(__file__).with_name('sensor_identification').joinpath(self.path.rsplit('/', 1)[1]).read_bytes(),
                           ctype='text/javascript; charset=utf-8')
            return
        if url.path in ('/api/identification', '/identification/report'):
            try:
                service = identification_service(self.server)
                if url.path == '/api/identification':
                    self.respond_json(service.snapshot())
                else:
                    report = service.report()
                    if report is None:
                        self.respond_json({'error': 'Run identification first'}, 404)
                    else:
                        self.send_data(report, ctype='text/html; charset=utf-8')
            except ImportError:
                self.respond_json({'error': 'Install geometry tools: uv sync --extra sensor-identification'}, 503)
            return
        if url.path in ('/api/taxels', '/api/taxels/'):
            try:
                after = float(parse_qs(url.query).get('after', ['0'])[0])
                if not math.isfinite(after):
                    raise ValueError('Invalid after timestamp')
            except ValueError:
                self.respond_json({'error': 'Invalid after timestamp'}, 400)
                return
            with LOCK:
                fresh = 0 <= time.time()-TAXELS_RECEIVED_AT <= 4 and TAXELS_RECEIVED_AT > after
                points = deepcopy(TAXELS) if fresh else []
            self.respond_json(points)
        elif self.path in ('/tactile','/tactile/','/tactile_2d.html'):
            try:self.send_data(TACTILE_HTML.read_bytes(),200,'text/html; charset=utf-8')
            except FileNotFoundError:self.respond_json({'error':'2D map missing: restore tactile_2d.html from the package'},404)
        elif self.path in ('/3d','/hand_twin_6dof.html'):
            try:self.send_data(Path(__file__).with_name('hand_twin_6dof.html').read_bytes(),200,'text/html; charset=utf-8')
            except FileNotFoundError:self.respond_json({'error':'3D viewer missing'},404)
        elif self.path in ('/','/index.html'):
            try:self.send_data(HTML.read_bytes(),200,'text/html; charset=utf-8')
            except FileNotFoundError:self.respond_json({'error':'HTML file not found'},404)
        elif self.path in ('/api/state','/api/state/'):
            with LOCK: self.respond_json(dict(STATE))
        elif self.path == '/health':self.respond_json({'status':'ok','mode':'read-only-from-hand'})
        else:self.respond_json({'error':'Not found'},404)
    def do_POST(self):
        if self.path == '/api/identification':
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 1 <= length <= 524288:
                    raise ValueError('Identification JSON must be 1..524288 bytes')
                origin = self.headers.get('Origin')
                if origin is not None and origin not in (f'http://127.0.0.1:{self.server.server_port}',
                                                          f'http://localhost:{self.server.server_port}'):
                    # Consume the bounded body so Windows can deliver the 403
                    # before closing, rather than resetting an unread socket.
                    self.rfile.read(length)
                    self.respond_json({'error': 'Open identification in the local simulation'}, 403)
                    return
                request = json.loads(self.rfile.read(length))
                service = identification_service(self.server)
                state = service.start(request)
                self.respond_json(state if state else service.snapshot(), 202 if state else 409)
            except ImportError:
                self.respond_json({'error': 'Install geometry tools: uv sync --extra sensor-identification'}, 503)
            except (TypeError, ValueError, UnicodeDecodeError) as error:
                self.respond_json({'error': str(error)}, 400)
            return
        if self.path in ('/api/taxels', '/api/taxels/'):
            self.receive_taxels()
            return
        if self.path not in ('/api/state','/api/state/'):
            self.respond_json({'error':'Not found'},404);return
        try:
            length=int(self.headers.get('Content-Length','0'))
            if not 1<=length<=65536:raise ValueError('JSON must contain 1 to 65536 bytes')
            msg=json.loads(self.rfile.read(length))
            changes=normalize(msg)
            with LOCK:
                if 'angles' in changes and 'angles_set' not in changes:
                    changes['angles_set']=changes.pop('angles')
                changes.pop('angles',None)
                now=time.time()
                STATE['field_received_at']={**STATE['field_received_at'], **{key:now for key in changes}}
                # A new position snapshot explicitly invalidates absent actual values.
                if 'angles_set' in changes and 'angles_act' not in changes:
                    changes['angles_act']=None
                    STATE['field_received_at']['angles_act']=0
                STATE.update(changes)
                STATE['received_at']=time.time();STATE['seq']+=1
                seq=STATE['seq']
            self.respond_json({'ok':True,'seq':seq})
        except (ValueError,UnicodeDecodeError,json.JSONDecodeError) as exc:
            self.respond_json({'ok':False,'error':str(exc)},400)

    def receive_taxels(self):
        """Cache viewer positions/pressure; never recompute FK or alter telemetry."""
        global TAXELS, TAXELS_RECEIVED_AT
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 1 <= length <= 524288:
                raise ValueError('Taxel JSON must contain 1 to 524288 bytes')
            points = json.loads(self.rfile.read(length))
            if not isinstance(points, list) or len(points) != len(TAXEL_IDS):
                raise ValueError('Expected all 1062 taxel positions')
            for p in points:
                if not isinstance(p, dict) or set(p) != {'id', 'x', 'y', 'z', 'pressure'} or not isinstance(p['id'], str):
                    raise ValueError('Expected {id,x,y,z,pressure} for each taxel')
                if any(isinstance(p[k], bool) or not isinstance(p[k], (int, float)) or not math.isfinite(p[k]) for k in ('x', 'y', 'z')):
                    raise ValueError('Taxel coordinates must be finite numbers')
                value = p['pressure']
                if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= 4095):
                    raise ValueError('Pressure must be null or a finite number in 0..4095')
            if {p['id'] for p in points} != TAXEL_IDS:
                raise ValueError('Taxel IDs are missing, duplicated, or unknown')
            with LOCK:
                TAXELS = points
                TAXELS_RECEIVED_AT = time.time()
            self.respond_json({'ok': True})
        except (ValueError, UnicodeDecodeError) as exc:
            self.respond_json({'error': str(exc)}, 400)

def main():
    global HTML
    ap=argparse.ArgumentParser(description='Read-only telemetry bridge for RH56DFTP viewer')
    ap.add_argument('--port',type=int,default=18765)
    ap.add_argument('--open-browser', action='store_true', help='Open the local viewer')
    ap.add_argument('--view',choices=['3d','tactile'],default='3d')
    args=ap.parse_args()
    if args.view=='tactile':HTML=TACTILE_HTML
    if not 1<=args.port<=65535: ap.error('Invalid TCP port')
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    print(f'Hand viewer ({args.view}) available at: http://127.0.0.1:{args.port}/',flush=True)
    print(f'POST endpoint: http://127.0.0.1:{args.port}/api/state (localhost only)',flush=True)
    print('This bridge does NOT send motor commands.',flush=True)
    if args.open_browser:
        import webbrowser
        webbrowser.open(f'http://127.0.0.1:{args.port}/')
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close()

if __name__=='__main__':main()
