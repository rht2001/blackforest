"""Local telemetry bridge for the 3D RH56DFTP hand viewer.

Start: python bridge_server.py
Viewer: http://127.0.0.1:18765/
Python program: POST JSON to http://127.0.0.1:18765/api/state

The server NEVER opens a Modbus connection or sends commands to hardware.
No third-party dependencies. Listens ONLY on 127.0.0.1.
"""
from __future__ import annotations
import argparse
import json
import math
import threading
import time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from pathlib import Path

from tactile_layout import REGIONS

FIELDS = ('simulated', 'angles', 'angles_set', 'angles_act', 'force_act', 'tactile', 'tactile_regions')
STATE = {'simulated': None, 'angles_set': None, 'angles_act': None, 'force_act': None, 'tactile': {}, 'tactile_regions': {}, 'field_received_at': {}, 'received_at': 0, 'seq': 0}
LOCK = threading.Lock()
HTML = Path(__file__).with_name('hand_twin_6dof.html')
TACTILE_HTML = Path(__file__).with_name('tactile_2d.html')

def vector(value, name, lo, hi):
    if value is None: return None
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError(f'{name} muss eine Liste von 6 Zahlen sein')
    vals=[]
    for x in value:
        if not isinstance(x, (float, int)) or isinstance(x,bool) or not math.isfinite(x) or not lo <= x <= hi:
            raise ValueError(f'{name} enthält ungültige Werte ({lo}..{hi})')
        vals.append(round(float(x),3))
    return vals

def normalize(payload):
    if not isinstance(payload,dict):raise ValueError('Erwartet JSON-Objekt')
    out={}
    if 'simulated' in payload:
        if not isinstance(payload['simulated'], bool): raise ValueError('simulated must be boolean')
        out['simulated'] = payload['simulated']
    for name in ('angles','angles_set','angles_act'):
        if name in payload:out[name]=vector(payload[name],name,0,1000)
    if 'force_act' in payload:out['force_act']=vector(payload['force_act'],'force_act',-4000,4000)
    if 'tactile' in payload:
        t=payload['tactile']
        if not isinstance(t,dict):raise ValueError('tactile muss ein Objekt sein')
        clean={}
        for name, value in t.items():
            if name not in ('little','ring','middle','index','thumb','palm'):continue
            if isinstance(value,dict):
                value={k:v for k,v in value.items() if k in ('max','mean')}
                if not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) and 0<=v<=4095 for v in value.values()):
                    raise ValueError('Taktile Werte außerhalb 0..4095')
            elif not (isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value) and 0<=value<=4095):
                raise ValueError('Taktile Werte außerhalb 0..4095')
            clean[name]=value
        out['tactile']=clean
    if 'tactile_regions' in payload:
        regions = payload['tactile_regions']
        if not isinstance(regions, dict): raise ValueError('tactile_regions muss ein Objekt sein')
        clean = {}
        for key, matrix in regions.items():
            if key not in REGIONS: raise ValueError('Unbekannte Sensorregion: '+key)
            rows, cols = REGIONS[key]
            if not isinstance(matrix,list) or len(matrix)!=rows or any(not isinstance(row,list) or len(row)!=cols for row in matrix):
                raise ValueError('Falsche Matrixdimension: '+key)
            if any(not isinstance(v,(int,float)) or isinstance(v,bool) or not math.isfinite(v) or not 0<=v<=4095 for row in matrix for v in row):
                raise ValueError('Sensorzellen außerhalb 0..4095')
            clean[key]=matrix
        out['tactile_regions']=clean
    if not out:raise ValueError('Keine gültigen Felder. Erlaubt: '+', '.join(FIELDS))
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
        if self.path in ('/tactile','/tactile/','/tactile_2d.html'):
            try:self.send_data(TACTILE_HTML.read_bytes(),200,'text/html; charset=utf-8')
            except FileNotFoundError:self.respond_json({'error':'2D-Karte fehlt: python build_tactile_2d.py ausführen'},404)
        elif self.path in ('/3d','/hand_twin_6dof.html'):
            try:self.send_data(Path(__file__).with_name('hand_twin_6dof.html').read_bytes(),200,'text/html; charset=utf-8')
            except FileNotFoundError:self.respond_json({'error':'3D-Viewer fehlt'},404)
        elif self.path in ('/','/index.html'):
            try:self.send_data(HTML.read_bytes(),200,'text/html; charset=utf-8')
            except FileNotFoundError:self.respond_json({'error':'HTML-Datei nicht gefunden'},404)
        elif self.path in ('/api/state','/api/state/'):
            with LOCK: self.respond_json(dict(STATE))
        elif self.path == '/health':self.respond_json({'status':'ok','mode':'read-only-from-hand'})
        else:self.respond_json({'error':'Not found'},404)
    def do_POST(self):
        if self.path not in ('/api/state','/api/state/'):
            self.respond_json({'error':'Not found'},404);return
        try:
            length=int(self.headers.get('Content-Length','0'))
            if not 1<=length<=65536:raise ValueError('JSON muss 1 bis 65536 Bytes umfassen')
            msg=json.loads(self.rfile.read(length))
            changes=normalize(msg)
            changes.setdefault('simulated', None)
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

def main():
    global HTML
    ap=argparse.ArgumentParser(description='Read-only telemetry bridge for RH56DFTP viewer')
    ap.add_argument('--port',type=int,default=18765)
    ap.add_argument('--view',choices=['3d','tactile'],default='3d')
    args=ap.parse_args()
    if args.view=='tactile':HTML=TACTILE_HTML
    if not 1<=args.port<=65535: ap.error('Ungültiger TCP-Port')
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    print(f'Hand-Viewer ({args.view}) geöffnet unter: http://127.0.0.1:{args.port}/',flush=True)
    print(f'POST-Ziel: http://127.0.0.1:{args.port}/api/state (nur localhost)',flush=True)
    print('Diese Schnittstelle sendet KEINE Motorbefehle.',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close()

if __name__=='__main__':main()
