"""Passive run capture. Only packets accepted by the bridge are recorded."""
import csv
import json
import time
import uuid
from collections import deque
from pathlib import Path

from tactile_layout import REGIONS


class ReceivedRuns:
    timeout = 15
    max_bytes = 128 * 1024 * 1024
    columns = ['index', 'run_id', 'state', 'angles', 'tactile_matrix',
               'slip_detected', 'received_at', 'source_index', 'reported_state']

    def __init__(self, folder):
        self.folder = Path(folder)
        self.active = None
        self.stream = None
        self.error = None
        self.auto_on_close = True
        self.armed = False
        self.motion_source = None
        self.motion_time = 0
        self.pre_roll = deque(maxlen=10)
        self.open_frames = 0
        self.replaying = False
        if self.folder.exists():
            for path in self.folder.glob('*.json'):
                try:
                    meta = json.loads(path.read_text(encoding='utf-8'))
                    if meta.get('status') == 'recording':
                        with path.with_suffix('.csv').open(newline='', encoding='utf-8') as stream:
                            meta['samples'] = sum(1 for _ in csv.DictReader(stream, delimiter=';'))
                        meta.update(status='interrupted', reason='receiver_restarted')
                        self._save(meta)
                except (OSError, ValueError):
                    continue

    def _save(self, meta):
        path = self.folder / (meta['id'] + '.json')
        temp = path.with_suffix('.json.tmp')
        temp.write_text(json.dumps(meta, indent=2), encoding='utf-8')
        temp.replace(path)

    def _finish(self, status, reason, now):
        if not self.active:
            return
        self.stream.close()
        self.active.update(status=status, reason=reason, finished_at=now)
        self._save(self.active)
        self.active = self.stream = None
        self.armed = False
        self.pre_roll.clear()

    def expire(self, now=None):
        now = time.time() if now is None else now
        if self.active and (self.active['samples'] or self.active['capture_mode'] == 'state') and now - self.active['last_received_at'] > self.timeout:
            self._finish('interrupted', 'telemetry_timeout', now)

    def _start(self, owner, now, mode):
        self.folder.mkdir(parents=True, exist_ok=True)
        key = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime(now)) + '_' + uuid.uuid4().hex[:12]
        self.active = dict(id=key, status='recording', source_id=owner, capture_mode=mode,
                           started_at=now, last_received_at=now, samples=0,
                           skipped_samples=0, missing_source_samples=0,
                           last_source_index=None, source_sequence_available=False,
                           capture_start={'manual': 'manual_window', 'closure': 'finger_closure', 'state': 'first_received_probing_sample'}[mode],
                           timing='Receiver arrival times; sensor acquisition skew unknown')
        self.stream = (self.folder / (key + '.csv')).open('x', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.stream, fieldnames=self.columns, delimiter=';')
        self.writer.writeheader()
        self.stream.flush()
        self._save(self.active)
        self.error = None
        self.open_frames = 0
        self.armed = False
        if mode != 'closure':
            self.pre_roll.clear()
        if mode == 'closure':
            self.active['motion_trigger'] = dict(open_min_raw=850, closed_below_raw=800,
                closing_fingers=2, reopening_consecutive_samples=3, pre_roll_samples=len(self.pre_roll),
                interpretation='Finger-motion capture window; remote probing state is unknown')

    def start_manual(self):
        self.expire()
        if self.active:
            raise ValueError('A capture is already recording')
        self._start(None, time.time(), 'manual')

    def finish_manual(self):
        if not self.active or self.active['capture_mode'] not in ('manual', 'closure'):
            raise ValueError('No user-controlled or finger-motion capture is recording')
        self._finish('complete', 'manual_finish', time.time())

    def set_auto(self, enabled):
        self.auto_on_close = enabled
        self.armed = False
        self.pre_roll.clear()

    def add(self, packet, now=None):
        now = time.time() if now is None else now
        try:
            self._add(packet, now)
        except OSError as error:
            # A recording failure must not break telemetry delivery.
            self.error = str(error)
            if self.stream:
                self.stream.close()
            self.active = self.stream = None

    def _add(self, packet, now):
        self.expire(now)
        state = packet.get('state')
        reported_state = state or ''
        owner = packet.get('source_id') or packet.get('source') or 'default'
        angles = packet.get('angles_act')
        coherent = angles is not None and bool(packet.get('tactile_regions'))
        if not self.active and coherent:
            if owner != self.motion_source or now - self.motion_time > self.timeout:
                self.armed = False
                self.pre_roll.clear()
            self.motion_source, self.motion_time = owner, now
            if self.auto_on_close and state != 'thumb_probing':
                if all(v >= 850 for v in angles[:4]):
                    if not self.armed:
                        self.pre_roll.clear()
                    self.armed = True
                if self.armed and sum(v < 800 for v in angles[:4]) >= 2:
                    self._start(owner, now, 'closure')
                    self.armed = False
                    self.replaying = True
                    try:
                        for previous, stamp in list(self.pre_roll):
                            self._add(previous, stamp)
                    finally:
                        self.replaying = False
                        self.pre_roll.clear()
                elif self.armed:
                    self.pre_roll.append((packet, now))
        if self.active and self.active['capture_mode'] in ('manual', 'closure'):
            if self.active['source_id'] is None:
                if not packet.get('angles_act') or not packet.get('tactile_regions'):
                    return
                self.active['source_id'] = owner
            state = 'thumb_probing'  # Explicit user-selected capture window, not an inferred robot state.
        if self.active and owner != self.active['source_id']:
            return
        if state == 'thumb_probing' and not self.active:
            self._start(owner, now, 'state')
        if not self.active:
            return
        if state and (self.active['capture_mode'] == 'state' or coherent):
            self.active['last_received_at'] = now
        duplicate = False
        if state:
            index = packet.get('sample_index')
            old = self.active['last_source_index']
            if index is not None:
                if old is not None and index <= old:
                    duplicate = True
                else:
                    if old is not None:
                        self.active['missing_source_samples'] += max(0, index - old - 1)
                    self.active['last_source_index'] = index
                self.active['source_sequence_available'] = True
        if state in ('thumb_probing', 'probing_complete') and not duplicate:
            # Never combine the bridge's cached angles with new tactile data.
            angles = packet.get('angles_act')
            regions = packet.get('tactile_regions')
            if angles is None or not regions:
                self.active['skipped_samples'] += 1
            else:
                matrix = [[None] * 17 for _ in range(112)]
                for column, (key, (height, width)) in enumerate(REGIONS.items()):
                    if key in regions:
                        values = [v for row in regions[key] for v in row]
                        for row, value in enumerate(values):
                            matrix[row][column] = value
                self.writer.writerow(dict(index=self.active['samples'], run_id=self.active['id'],
                    state=state, angles=json.dumps(angles), tactile_matrix=json.dumps(matrix),
                    slip_detected=packet.get('slip_detected', False), received_at=now,
                    source_index=index, reported_state=reported_state))
                self.stream.flush()
                self.active['samples'] += 1
                if self.stream.tell() >= self.max_bytes:
                    self._finish('interrupted', 'size_limit', now)
                    return
        if self.active['capture_mode'] == 'closure' and coherent and not self.replaying:
            self.open_frames = self.open_frames + 1 if all(v >= 850 for v in angles[:4]) else 0
            if self.open_frames >= 3:
                self._finish('complete', 'hand_reopened', now)
                self.armed = True
                return
        if state in ('probing_complete', 'cylinder_confirmed'):
            self._finish('complete', 'probing_finished', now)
        elif state and state != 'thumb_probing':
            self._finish('interrupted', state, now)

    def snapshot(self):
        self.expire()
        runs = []
        if self.folder.exists():
            for path in sorted(self.folder.glob('*.json'), reverse=True)[:50]:
                try:
                    meta = json.loads(path.read_text(encoding='utf-8'))
                    if not self.active or meta['id'] != self.active['id']:
                        runs.append(meta)
                except (OSError, ValueError, KeyError):
                    continue
        runs.sort(key=lambda meta: meta.get('started_at', 0), reverse=True)
        return dict(active=dict(self.active) if self.active else None, runs=runs, error=self.error,
                    auto_on_close=self.auto_on_close, closure_armed=self.armed)

    def read(self, key):
        # Only server-generated IDs may select a file; no arbitrary paths.
        if len(key) != 29 or any(c not in '0123456789TZ_abcdef' for c in key):
            raise ValueError('Invalid capture ID')
        meta = json.loads((self.folder / (key + '.json')).read_text(encoding='utf-8'))
        if meta['status'] == 'recording':
            raise ValueError('Run is still recording')
        return (self.folder / (key + '.csv')).read_bytes()

    def close(self):
        self._finish('interrupted', 'receiver_stopped', time.time())
