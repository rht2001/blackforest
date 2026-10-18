"""Embed the two repository STL parts, without changing their shapes/topology."""
import hashlib
from pathlib import Path
import struct

SOURCES = [('vegabar_29', 'VEGABAR 29', 'VEGABAR_29_STL.stl', '#45cabb'),
           ('vegabar_38', 'VEGABAR 38', 'VEGABAR_38.stl', '#edb358')]


def build(directory=None):
    directory = Path(directory) if directory else Path(__file__).resolve().parents[1]/'documents'
    models = []
    for identity, label, filename, color in SOURCES:
        blob = (directory/filename).read_bytes()
        if len(blob) < 84:
            raise ValueError('Invalid binary STL: '+filename)
        count = struct.unpack_from('<I', blob, 80)[0]
        if not count or len(blob) != 84+50*count:
            raise ValueError('Invalid STL triangle count: '+filename)
        vertices, indices, lookup = [], [], {}
        for face in range(count):
            for point in struct.iter_unpack('<3f', blob[84+50*face+12:84+50*face+48]):
                if point not in lookup:
                    lookup[point] = len(vertices)
                    vertices.append(point)
                indices.append(lookup[point])
        low = [min(p[k] for p in vertices) for k in range(3)]
        high = [max(p[k] for p in vertices) for k in range(3)]
        center = [(a+b)/2 for a, b in zip(low, high)]
        models.append({'id': identity, 'name': label, 'color': color,
                       'source': 'documents/'+filename, 'source_sha256': hashlib.sha256(blob).hexdigest(),
                       'units': 'metres', 'source_unit_scale_to_metres': .001,
                       'source_center_mm': center, 'source_bounds_mm': [low, high],
                       'vertices': [round((p[k]-center[k])*.001, 10) for p in vertices for k in range(3)],
                       'indices': indices})
    return models
