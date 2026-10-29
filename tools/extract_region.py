#!/usr/bin/env python3
"""Extract a reference-complete urban OSM snapshot from a regional PBF, offline.

Three bounded streaming passes: locate ways/areas, complete multipolygon way
members, then write only the needed nodes. The output retains original IDs.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import osmium

URBAN_BBOX=(35.10,33.25,35.23,33.47)

def element(obj,kind):
    e=ET.Element(kind,id=str(obj.id),version=str(obj.version))
    for tag in obj.tags:ET.SubElement(e,'tag',k=tag.k,v=tag.v)
    if kind=='node':e.set('lat',f'{obj.location.lat:.7f}');e.set('lon',f'{obj.location.lon:.7f}')
    elif kind=='way':
        for n in obj.nodes:ET.SubElement(e,'nd',ref=str(n.ref))
    else:
        for m in obj.members:ET.SubElement(e,'member',type={'n':'node','w':'way','r':'relation'}[m.type],ref=str(m.ref),role=m.role)
    return e

def extract(source,output,bbox=URBAN_BBOX):
    inside=set();ways={};relations={};members=set()
    class Select(osmium.SimpleHandler):
        def node(self,n):
            if bbox[0]<=n.location.lat<=bbox[2] and bbox[1]<=n.location.lon<=bbox[3]:inside.add(n.id)
        def way(self,w):
            if any(n.ref in inside for n in w.nodes):ways[w.id]=element(w,'way')
        def relation(self,r):
            if r.tags.get('type')=='multipolygon' and all(m.type!='r' for m in r.members) and any(m.type=='w' and m.ref in ways for m in r.members):
                relations[r.id]=element(r,'relation');members.update(m.ref for m in r.members if m.type=='w')
    print('Selecting region and multipolygons...',flush=True);Select().apply_file(str(source))
    class Complete(osmium.SimpleHandler):
        def way(self,w):
            if w.id in members and w.id not in ways:ways[w.id]=element(w,'way')
    print('Completing area members...',flush=True);Complete().apply_file(str(source))
    needed=set(inside)
    for way in ways.values():needed.update(int(n.attrib['ref']) for n in way.findall('nd'))
    timestamp=None
    with osmium.io.Reader(str(source)) as reader:
        timestamp=reader.header().get('osmosis_replication_timestamp')
    output.parent.mkdir(parents=True,exist_ok=True)
    count=0
    with output.open('wb') as file:
        file.write(b'<?xml version="1.0" encoding="UTF-8"?>\n<osm version="0.6" generator="CarsLOP regional PBF extraction">\n')
        if timestamp:file.write(ET.tostring(ET.Element('meta',osm_base=timestamp))+b'\n')
        class Nodes(osmium.SimpleHandler):
            def node(self,n):
                nonlocal count
                if n.id in needed:file.write(ET.tostring(element(n,'node'))+b'\n');count+=1
        print('Writing complete node geometry...',flush=True);Nodes().apply_file(str(source))
        for ident in sorted(ways):file.write(ET.tostring(ways[ident])+b'\n')
        for ident in sorted(relations):file.write(ET.tostring(relations[ident])+b'\n')
        file.write(b'</osm>\n')
    result=dict(bbox=list(bbox),source=source.name,source_timestamp=timestamp,nodes=count,ways=len(ways),multipolygons=len(relations))
    output.with_suffix('.json').write_text(json.dumps(result,indent=2)+'\n')
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('output',type=Path);p.add_argument('--bbox',nargs=4,type=float,default=URBAN_BBOX)
    a=p.parse_args();print(json.dumps(extract(a.source,a.output,tuple(a.bbox)),indent=2))
