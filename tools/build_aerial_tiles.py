#!/usr/bin/env python3
"""Tile a saved aerial map export for local runtime projection. No network calls.
Requires Pillow; source metadata and redistribution terms remain with the pack.
"""
import argparse
import json
import math
from pathlib import Path
import shutil
import zlib
from PIL import Image


def local(mx, my):
    lat=math.degrees(2*math.atan(math.exp(my/6378137))-math.pi/2)
    lon=math.degrees(mx/6378137)
    return ((lon-33.320)*math.pi/180*6371000*math.cos(math.radians(35.162)),
            -(lat-35.162)*math.pi/180*6371000)


def build(image_path, export_path, output, size=512, gutter=8):
    image=Image.open(image_path).convert('RGB')
    if image.width%size or image.height%size:raise ValueError('image dimensions must be multiples of tile size')
    extent=json.loads(export_path.read_text())['extent']
    nw=local(extent['xmin'],extent['ymax']);se=local(extent['xmax'],extent['ymin'])
    output.mkdir(parents=True,exist_ok=True)
    nx,nz=image.width//size,image.height//size
    lines=['AERIAL 1',f'size {size} {gutter}']
    # Replicate the image boundary, while internal gutters contain real neighbours.
    padded=Image.new('RGB',(image.width+2*gutter,image.height+2*gutter))
    padded.paste(image,(gutter,gutter))
    for x in range(gutter):
        padded.paste(image.crop((0,0,1,image.height)),(x,gutter))
        padded.paste(image.crop((image.width-1,0,image.width,image.height)),(gutter+image.width+x,gutter))
    padded.paste(padded.crop((0,gutter,padded.width,gutter+1)).resize((padded.width,gutter)),(0,0))
    padded.paste(padded.crop((0,gutter+image.height-1,padded.width,gutter+image.height)).resize((padded.width,gutter)),(0,gutter+image.height))
    for z in range(nz):
        for x in range(nx):
            raw=padded.crop((x*size,z*size,(x+1)*size+2*gutter,(z+1)*size+2*gutter)).tobytes()
            name=f'{x}_{z}.rgb';(output/name).write_bytes(raw)
            bounds=[nw[0]+(se[0]-nw[0])*x/nx,nw[1]+(se[1]-nw[1])*z/nz,
                    nw[0]+(se[0]-nw[0])*(x+1)/nx,nw[1]+(se[1]-nw[1])*(z+1)/nz]
            lines.append('tile '+' '.join(f'{v:.6f}' for v in bounds)+f' {name} {zlib.crc32(raw):08x}')
    (output/'tiles.txt').write_text('\n'.join(lines)+'\n')
    shutil.copyfile(export_path,output/'source-export.json')
    print(f'{nx*nz} aerial tiles, {size}px plus {gutter}px gutters; {nw} to {se}')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--image',type=Path,default=Path('artifacts/aerial-preview/engomi-2019.jpg'))
    p.add_argument('--export',type=Path,default=Path('artifacts/aerial-preview/export.json'))
    p.add_argument('--output',type=Path,default=Path('data/engomi/aerial'))
    args=p.parse_args();build(args.image,args.export,args.output)
