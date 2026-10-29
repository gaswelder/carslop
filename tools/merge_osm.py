#!/usr/bin/env python3
"""Merge an existing local OSM snapshot with a regional extension.

Existing objects win when the extension has no version metadata. When both
versions are known, keep the newer version. Shared node IDs remain shared.
"""
import argparse
import gzip
from pathlib import Path
import xml.etree.ElementTree as ET

def read(path):
    root=ET.fromstring(gzip.decompress(path.read_bytes()) if path.suffix=='.gz' else path.read_bytes())
    if root.find('remark') is not None:raise ValueError('incomplete Overpass snapshot')
    return root

def merge(original,extension,output):
    items={}
    for root in [read(extension),read(original)]:
        for e in root:
            if e.tag not in ('node','way','relation'):continue
            key=(e.tag,int(e.attrib['id']))
            previous=items.get(key)
            if previous is None or 'version' not in previous.attrib or ('version' in e.attrib and int(e.attrib['version'])>=int(previous.attrib['version'])):items[key]=e
    root=ET.Element('osm',version='0.6',generator='CarsLOP offline snapshot merge')
    ET.SubElement(root,'note').text='OpenStreetMap contributors, ODbL. Merged snapshots; see sources/manifest.json. Existing Engomi objects preserved when extension version metadata is absent.'
    for kind in ['node','way','relation']:
        for key in sorted(k for k in items if k[0]==kind):root.append(items[key])
    ET.ElementTree(root).write(output,encoding='utf-8',xml_declaration=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('original',type=Path);p.add_argument('extension',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();merge(a.original,a.extension,a.output)
