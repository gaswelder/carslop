#!/usr/bin/env python3
"""Build original fictional car GLBs from parametric surfaces; no external assets.

Usage: python3 tools/build_fictional_cars.py agolf [--output FILE.glb]
"""
import argparse
import json
import math
from pathlib import Path
import struct

ROOT = Path(__file__).resolve().parents[1]
PI = math.pi
PAINT, RUBBER, GLASS, METAL, LENS, BRAKE, REVERSE = range(7)
FINISH = [(1, .82), (0, .12), (2, .94), (3, .84), (4, .75), (5, .7), (6, .7)]
WHITE = (.87, .89, .84)
DARK = (.025, .03, .035)
CHROME = (.67, .71, .72)


def sub(a, b):
    return tuple(x-y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def unit(v):
    length = math.sqrt(sum(x*x for x in v))
    return tuple(x / max(length, 1e-12) for x in v)


class Mesh:
    def __init__(self):
        self.vertices = []
        self.groups = {}

    def triangle(self, points, material, color, normals=None):
        face = cross(sub(points[1], points[0]), sub(points[2], points[0]))
        if sum(x*x for x in face) < 1e-14:
            return
        normals = normals or [unit(face)] * 3
        ids = self.groups.setdefault(material, [])
        for p, n in zip(points, normals):
            ids.append(len(self.vertices))
            self.vertices.append((*p, *color, *n))

    def quad(self, points, material, color):
        for ids in [(0, 1, 2), (0, 2, 3)]:
            self.triangle([points[i] for i in ids], material, color)

    def patch(self, nu, nv, point, material, color):
        def normal(u, v):
            eps = .0001
            return unit(cross(sub(point(u+eps, v), point(u-eps, v)),
                              sub(point(u, v+eps), point(u, v-eps))))
        for j in range(nv):
            for i in range(nu):
                uv = [(i/nu, j/nv), ((i+1)/nu, j/nv),
                      ((i+1)/nu, (j+1)/nv), (i/nu, (j+1)/nv)]
                points = [point(u, v) for u, v in uv]
                normals = [normal(u, v) for u, v in uv]
                m = material((i+.5)/nu, (j+.5)/nv) if callable(material) else material
                c = color((i+.5)/nu, (j+.5)/nv) if callable(color) else color
                for ids in [(0, 1, 2), (0, 2, 3)]:
                    self.triangle([points[k] for k in ids], m, c, [normals[k] for k in ids])

    def box(self, lo, hi, material, color):
        x, y, z = lo
        X, Y, Z = hi
        p = [(x,y,z),(X,y,z),(X,Y,z),(x,Y,z),(x,y,Z),(X,y,Z),(X,Y,Z),(x,Y,Z)]
        for ids in [(0,3,2,1),(4,5,6,7),(0,4,7,3),(1,2,6,5),(0,1,5,4),(3,7,6,2)]:
            self.quad([p[i] for i in ids], material, color)

    def disc(self, center, radius, material, color, axis='z', ring=0.0):
        for i in range(48):
            a, b = i*2*PI/48, (i+1)*2*PI/48
            def p(t, r):
                if axis == 'z':
                    return (center[0]+r*math.cos(t), center[1]+r*math.sin(t), center[2])
                return (center[0], center[1]+r*math.cos(t), center[2]+r*math.sin(t))
            if ring:
                self.quad([p(a,ring), p(a,radius), p(b,radius), p(b,ring)], material, color)
            else:
                self.triangle([center,p(a,radius),p(b,radius)], material, color)


def profile(stations, z):
    index = next((i for i in range(len(stations)-1) if z <= stations[i+1][0]), len(stations)-2)
    a, b = stations[index:index+2]
    prev = stations[max(index-1, 0)]
    nxt = stations[min(index+2, len(stations)-1)]
    span = b[0]-a[0]
    t = min(max((z-a[0])/span, 0), 1)
    out = []
    for k in range(1, len(a)):
        ma = (b[k]-prev[k])/(b[0]-prev[0])
        mb = (nxt[k]-a[k])/(nxt[0]-a[0])
        out.append((2*t**3-3*t*t+1)*a[k] + (t**3-2*t*t+t)*span*ma
                   + (-2*t**3+3*t*t)*b[k] + (t**3-t*t)*span*mb)
    return out


class Coachwork:
    def __init__(self, mesh, stations, axles, radius, cabin):
        self.mesh, self.stations, self.axles = mesh, stations, axles
        self.radius, self.cabin = radius, cabin
        self.cy = radius-.48

    def deck(self, q, z):
        w, center, shoulder = profile(self.stations, z)
        return (w*q, center+(shoulder-center)*math.sin(q*PI/2)**2+.012*(1-q*q), z)

    def skin(self, hatch=False, arches=PAINT):
        b = self.mesh
        lo, hi = self.stations[0][0], self.stations[-1][0]
        b.patch(32, 112, lambda u,v:self.deck(u*2-1, lo+(hi-lo)*v), PAINT, WHITE)
        for side in [-1,1]:
            def flank(u,v):
                z = lo+(hi-lo)*u
                w, _, top = profile(self.stations, z)
                bottom = -.31
                for axle in self.axles:
                    d = z-axle
                    r = self.radius+.04
                    if abs(d) < r:
                        bottom = max(bottom, self.cy+math.sqrt(r*r-d*d))
                bottom = min(bottom, top-.015)
                y = bottom+(top-bottom)*v
                x = side*(w-.045*(1-v)**2)
                return (x,y,z)
            b.patch(140, 12, flank, PAINT, WHITE)
            for axle in self.axles:
                def arch(u,v):
                    a = u*PI
                    r = self.radius+.038+v*.025
                    z = axle+r*math.cos(a)
                    w = profile(self.stations,z)[0]
                    return (side*(w+.004), self.cy+r*math.sin(a), z)
                b.patch(48, 3, arch, arches, DARK if arches == RUBBER else WHITE)
                w = profile(self.stations, axle)[0]
                b.disc((side*(w-.22),self.cy,axle),self.radius+.04,RUBBER,(.018,.021,.025),'x')
        for z in [lo,hi]:
            b.patch(32, 12, lambda u,v:(self.deck(u*2-1,z)[0], -.31+(self.deck(u*2-1,z)[1]+.31)*v,z), PAINT, WHITE)
        clo, chi = self.cabin[0][0], self.cabin[-1][0]
        def greenhouse(u,v):
            z = clo+(chi-clo)*v
            w, top = profile(self.cabin,z)
            q = u*2-1
            base = self.deck(0,z)[1]+.01
            shape = max(0.0, min(1.0, (1-abs(q))/.16)) if hatch else (1-.08*q*q-.92*q**10)
            return (q*w, base+(top-base)*shape-.008*min(abs(q)/.84,1)**2,z)
        def glazing(u,v):
            z = clo+(chi-clo)*v
            q = abs(u*2-1)
            rear = clo+.10 < z < self.cabin[1][0]-.05 and q < .84
            front = self.cabin[-2][0]+.08 < z < chi-.08 and q < .85
            side = (.885 if hatch else .70) < q < .985 and clo+.20 < z < chi-.22 and abs(z+.22)>.055
            return GLASS if rear or front or side else PAINT
        b.patch(56, 96, greenhouse, glazing, lambda u,v:(.08,.13,.15) if glazing(u,v)==GLASS else WHITE)
        # Sills, rub strips, door seams, handles and mirror housings.
        for side in [-1,1]:
            w = profile(self.stations,0)[0]
            b.box((side*w-.012,-.21,-.95),(side*w+.012,-.15,.88),RUBBER,DARK)
            b.box((side*(w+.008)-.012,.11,-1.15),(side*(w+.008)+.012,.14,1.12),RUBBER,DARK)
            for z in [-.88,.75]:
                y = profile(self.stations,z)[2]
                b.box((side*w-.003,-.15,z-.003),(side*w+.003,y,z+.003),RUBBER,(.07,.065,.06))
            b.box((side*(w+.015)-.012,.24,-.65),(side*(w+.015)+.012,.265,-.49),RUBBER,DARK)
            b.box((side*(w+.035)-.06,.32,.60),(side*(w+.035)+.06,.39,.75),PAINT,WHITE)
            b.box((side*(w+.035)-.045,.333,.594),(side*(w+.035)+.045,.378,.603),METAL,(.4,.48,.5))

    def stripe(self, center, width, zlo, zhi, color):
        self.mesh.patch(2,32,lambda u,v: tuple(p+(0.004 if k==1 else 0) for k,p in enumerate(self.deck((center+(u-.5)*width)/profile(self.stations,zlo+(zhi-zlo)*v)[0],zlo+(zhi-zlo)*v))),RUBBER,color)


def wheel(radius, width, side, style):
    b = Mesh()
    b.patch(64,16,lambda u,v:(width*.5*math.sin(v*2*PI),
        (radius-.035+.035*math.cos(v*2*PI))*math.cos(u*2*PI),
        (radius-.035+.035*math.cos(v*2*PI))*math.sin(u*2*PI)),RUBBER,(.025,.029,.032))
    face = side*(width*.5+.002)
    b.disc((face,0,0),radius*.83,METAL,CHROME,'x',radius*.76)
    b.disc((face-side*.012,0,0),radius*.76,RUBBER,(.065,.075,.08),'x')
    count = 8 if style=='mesh' else 5
    for i in range(count):
        a = i*2*PI/count
        for offset in ([-.28,.28] if style=='mesh' else [0]):
            inner = (math.cos(a)*radius*.19,math.sin(a)*radius*.19)
            outer = (math.cos(a+offset)*radius*.77,math.sin(a+offset)*radius*.77)
            d = unit((0,outer[0]-inner[0],outer[1]-inner[1]))
            w = .009 if style=='mesh' else .027
            normal = (-d[2]*w,d[1]*w)
            b.quad([(face,inner[0]+normal[0],inner[1]+normal[1]),(face,outer[0]+normal[0],outer[1]+normal[1]),
                    (face,outer[0]-normal[0],outer[1]-normal[1]),(face,inner[0]-normal[0],inner[1]-normal[1])],METAL,CHROME)
    b.disc((face+side*.004,0,0),radius*.20,METAL,(.5,.54,.56),'x')
    for i in range(5):
        a=i*2*PI/5
        b.disc((face+side*.006,math.cos(a)*radius*.12,math.sin(a)*radius*.12),.012,METAL,(.8,.82,.83),'x')
    return b


FONT = {
 'A':[14,17,17,31,17,17,17], 'G':[14,17,16,23,17,17,15],
 'O':[14,17,17,17,17,17,14], 'L':[16,16,16,16,16,16,31],
 'F':[31,16,16,30,16,16,16], 'T':[31,4,4,4,4,4,4],
 'I':[14,4,4,4,4,4,14], 'S':[15,16,16,14,1,1,30],
 'M':[17,27,21,21,17,17,17], 'Q':[14,17,17,17,21,18,13],
 'U':[17,17,17,17,17,17,14], 'P':[30,17,17,30,16,16,16],
 'N':[17,25,21,19,17,17,17], 'D':[30,17,17,17,17,17,30],
 '2':[14,17,1,2,4,8,31], '6':[14,16,16,30,17,17,14],
 '9':[14,17,17,15,1,1,14], '1':[4,12,4,4,4,4,14],
 ' ':[0]*7,
}


def badge(b, text, center, size, color=CHROME):
    z = center[2]
    for i,ch in enumerate(text):
        for row,bits in enumerate(FONT[ch]):
            for col in range(5):
                if bits & (1 << (4-col)):
                    x=center[0]+(i*6+col-len(text)*3)*size
                    y=center[1]+(3-row)*size
                    b.quad([(x,y,z),(x+size*.85,y,z),(x+size*.85,y+size*.85,z),(x,y+size*.85,z)],METAL,color)


def tail_pair(b, z, outer, inner, ylo, yhi):
    for side in [-1,1]:
        xlo, xhi = sorted([side*inner,side*outer])
        b.box((xlo-.015,ylo-.015,z-.003),(xhi+.015,yhi+.015,z+.035),RUBBER,DARK)
        b.box((xlo,ylo,z-.006),(xhi,yhi,z+.025),BRAKE,(.75,.035,.02))
        mid = side*(inner+.055)
        b.box((mid-.045,ylo+.015,z-.012),(mid+.045,yhi-.015,z-.005),REVERSE,(.45,.48,.48))
        # Fine fluting remains visible without obscuring the lamp face.
        for k in range(1,4):
            y=ylo+(yhi-ylo)*k/4
            b.box((xlo,y,z-.009),(xhi,y+.002,z-.006),LENS,(.45,.035,.02))


def agolf():
    b = Mesh()
    stations=[(-2.0,.76,.29,.30),(-1.65,.84,.31,.33),(-1.2,.85,.30,.34),
              (-.5,.83,.29,.31),(.4,.83,.29,.31),(1.25,.85,.29,.32),(1.8,.82,.28,.30),(2.0,.78,.27,.29)]
    axles=[-1.225,1.25]
    cabin=[(-1.87,.68,.38),(-1.27,.72,.91),(-.65,.72,.94),(.32,.70,.93),(.95,.70,.31)]
    shell=Coachwork(b,stations,axles,.30,cabin)
    shell.skin(hatch=True,arches=RUBBER)
    # Broad black bumpers and chin, red-edged four-lamp grille.
    b.box((-.82,-.20,1.94),(.82,-.04,2.09),RUBBER,DARK)
    b.box((-.80,-.29,1.93),(.80,-.21,2.03),RUBBER,DARK)
    b.box((-.78,-.19,-2.08),(.78,-.04,-1.95),RUBBER,DARK)
    b.box((-.76,.025,2.005),(.76,.255,2.035),RUBBER,DARK)
    for y in [.025,.249]: b.box((-.76,y,2.04),(.76,y+.007,2.047),LENS,(.82,.025,.018))
    for x in [-.76,.753]: b.box((x,.025,2.04),(x+.007,.256,2.047),LENS,(.82,.025,.018))
    for i in range(6):
        y=.045+i*.035
        b.box((-.73,y,2.036),(.73,y+.006,2.041),RUBBER,(.07,.08,.085))
    for side in [-1,1]:
        for x,r in [(side*.585,.103),(side*.325,.086)]:
            b.disc((x,.14,2.052),r+.009,METAL,(.4,.42,.43))
            b.disc((x,.14,2.056),r,LENS,(.71,.77,.72))
        b.box((side*.68-.08,-.165,2.095),(side*.68+.08,-.11,2.10),LENS,(.95,.34,.035))
        b.box((side*.40-.08,-.165,2.095),(side*.40+.08,-.11,2.10),LENS,(.7,.76,.7))
    b.disc((0,.145,2.057),.064,METAL,CHROME,ring=.049)
    badge(b,'A',(0,.14,2.06),.009)
    badge(b,'GTI',(.18,.21,2.06),.0055,(.9,.03,.02))
    tail_pair(b,-2.035,.77,.43,-.015,.20)
    b.box((-.68,.89,-1.39),(.68,.93,-1.21),RUBBER,DARK)
    b.box((-.20,-.13,-2.10),(.20,-.065,-2.083),METAL,(.65,.65,.57))
    badge(b,'AGOLF',(-.28,.245,-2.041),.008)
    badge(b,'GTI',(.55,.245,-2.041),.007)
    # Bonnet shutlines and twin exhaust tips.
    for side in [-1,1]:
        shell.stripe(side*.56,.005,1.0,1.94,(.04,.04,.04))
    for x in [-.55,-.48]: b.box((x-.025,-.27,-2.15),(x+.025,-.22,-1.99),METAL,(.4,.42,.43))
    return b,axles,.30,.715,'mesh'


def export(path, body, axles, radius, track, style):
    blob=bytearray()
    doc={'asset':{'version':'2.0','generator':'Cars LOP fictional coachwork'},'scene':0,
         'scenes':[{'nodes':list(range(5))}], 'nodes':[], 'meshes':[], 'materials':[],
         'buffers':[{'byteLength':0}], 'bufferViews':[], 'accessors':[]}
    for code, gloss in FINISH:
        doc['materials'].append({'pbrMetallicRoughness':{'baseColorFactor':[1,1,1,1],
            'metallicFactor':int(code==3),'roughnessFactor':1-gloss},'extras':{'carslop_material':[code,gloss]}})
    def add(data,target):
        index=len(doc['bufferViews'])
        doc['bufferViews'].append({'buffer':0,'byteOffset':len(blob),'byteLength':len(data),'target':target})
        blob.extend(data)
        return index
    parts=[('body',body,None)]
    for i,(axle,side) in enumerate([(axles[0],-1),(axles[0],1),(axles[1],-1),(axles[1],1)]):
        parts.append((['wheel_rl','wheel_rr','wheel_fl','wheel_fr'][i],wheel(radius,.18,side,style),(side*track,radius-.48,axle)))
    for name,mesh,translation in parts:
        data=b''.join(struct.pack('<9f',*v) for v in mesh.vertices)
        view=add(data,34962);doc['bufferViews'][view]['byteStride']=36
        start=len(doc['accessors'])
        for offset in [0,12,24]:
            a={'bufferView':view,'byteOffset':offset,'componentType':5126,'count':len(mesh.vertices),'type':'VEC3'}
            if offset==0:
                a['min']=[min(v[k] for v in mesh.vertices) for k in range(3)]
                a['max']=[max(v[k] for v in mesh.vertices) for k in range(3)]
            doc['accessors'].append(a)
        primitives=[]
        for mat,indices in sorted(mesh.groups.items()):
            vi=add(struct.pack('<'+'I'*len(indices),*indices),34963)
            ai=len(doc['accessors']);doc['accessors'].append({'bufferView':vi,'componentType':5125,'count':len(indices),'type':'SCALAR'})
            primitives.append({'attributes':{'POSITION':start,'COLOR_0':start+1,'NORMAL':start+2},'indices':ai,'material':mat})
        node={'name':name,'mesh':len(doc['meshes'])}
        if translation:node['translation']=translation
        doc['nodes'].append(node);doc['meshes'].append({'primitives':primitives})
    doc['buffers'][0]['byteLength']=len(blob)
    text=json.dumps(doc,separators=(',',':')).encode();text+=b' '*((-len(text))%4)
    blob+=b'\0'*((-len(blob))%4)
    raw=struct.pack('<III',0x46546c67,2,28+len(text)+len(blob))+struct.pack('<II',len(text),0x4e4f534a)+text+struct.pack('<II',len(blob),0x004e4942)+blob
    path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(raw)
    print(f'Built {path}: {len(raw):,} bytes')


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('car',choices=['agolf'])
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    export(args.output or ROOT/'data/cars/club/model.glb',*agolf())
