#!/usr/bin/env python3
"""Package procedural car artwork as standard glTF 2.0 GLBs (stdlib only).

Run after either car authoring script, or directly to migrate legacy meshes.
Material extras preserve the game's paint and finish controls; other viewers
see regular vertex colors and metallic/roughness materials.
"""
import json
from pathlib import Path
import struct

ROOT = Path(__file__).resolve().parents[1] / 'data/cars'
PARTS = ('wheel_rl', 'wheel_rr', 'wheel_fl', 'wheel_fr')


def export(folder):
    folder = Path(folder)
    spec = json.loads((folder / 'car.json').read_text())
    binary = bytearray()
    doc = {'asset': {'version': '2.0', 'generator': 'carslop procedural car exporter'},
           'scene': 0, 'scenes': [{'nodes': list(range(5))}],
           'nodes': [{'name': 'body', 'mesh': 0}],
           'meshes': [], 'materials': [], 'accessors': [], 'bufferViews': []}

    def view(data, target):
        binary.extend(b'\0' * (-len(binary) % 4))
        index = len(doc['bufferViews'])
        doc['bufferViews'].append({'buffer': 0, 'byteOffset': len(binary),
                                   'byteLength': len(data), 'target': target})
        binary.extend(data)
        return index

    def accessor(buffer_view, count, kind, component=5126, **kwargs):
        index = len(doc['accessors'])
        doc['accessors'].append({'bufferView': buffer_view, 'componentType': component,
                                 'count': count, 'type': kind, **kwargs})
        return index

    for filename in (spec.get('mesh', 'body.mesh'), spec.get('wheel_mesh', 'wheel.mesh')):
        raw = (folder / filename).read_bytes()
        if raw[:8] != b'NFSCAR01':
            raise ValueError(f'{filename}: expected NFSCAR01 source artwork')
        count, = struct.unpack_from('<I', raw, 8)
        vertices = list(struct.iter_unpack('<11f', raw[12:]))
        if len(vertices) != count or count == 0 or count % 3:
            raise ValueError(f'{filename}: invalid vertex count')
        v = view(b''.join(struct.pack('<9f', *p[:9]) for p in vertices), 34962)
        doc['bufferViews'][v]['byteStride'] = 36
        attributes = {
            'POSITION': accessor(v, count, 'VEC3', byteOffset=0,
                                 min=[min(p[i] for p in vertices) for i in range(3)],
                                 max=[max(p[i] for p in vertices) for i in range(3)]),
            'COLOR_0': accessor(v, count, 'VEC3', byteOffset=12),
            'NORMAL': accessor(v, count, 'VEC3', byteOffset=24),
        }
        groups = {}
        for i in range(0, count, 3):
            finish = vertices[i][9:11]
            if any(p[9:11] != finish for p in vertices[i:i+3]):
                raise ValueError('a triangle must use a single finish')
            groups.setdefault(finish, []).extend((i, i+1, i+2))
        primitives = []
        for finish, indices in groups.items():
            material = len(doc['materials'])
            doc['materials'].append({
                'name': f'finish_{finish[0]:g}_{finish[1]:g}',
                'pbrMetallicRoughness': {'baseColorFactor': [1, 1, 1, 1],
                                        'metallicFactor': 1 if finish[0] == 3 else 0,
                                        'roughnessFactor': 1 - finish[1]},
                'extras': {'carslop_material': finish},
            })
            iv = view(struct.pack(f'<{len(indices)}I', *indices), 34963)
            ia = accessor(iv, len(indices), 'SCALAR', 5125)
            primitives.append({'attributes': attributes, 'indices': ia, 'material': material})
        doc['meshes'].append({'primitives': primitives})
    for name, (x, z) in zip(PARTS, ((-1, -1), (1, -1), (-1, 1), (1, 1))):
        doc['nodes'].append({'name': name, 'mesh': 1,
                             'translation': [x * spec['half_track'], spec['wheel_height'],
                                             z * spec['wheelbase'] / 2]})
    doc['buffers'] = [{'byteLength': len(binary)}]
    encoded = json.dumps(doc, separators=(',', ':')).encode()
    encoded += b' ' * (-len(encoded) % 4)
    binary.extend(b'\0' * (-len(binary) % 4))
    result = (struct.pack('<III', 0x46546c67, 2, 28 + len(encoded) + len(binary)) +
              struct.pack('<I4s', len(encoded), b'JSON') + encoded +
              struct.pack('<I4s', len(binary), b'BIN\0') + binary)
    (folder / 'model.glb').write_bytes(result)
    print(f'{folder.name}: model.glb ({len(result):,} bytes)')


if __name__ == '__main__':
    for folder in sorted(ROOT.iterdir()):
        if (folder / 'car.json').is_file():
            export(folder)
