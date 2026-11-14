#!/usr/bin/env python3
"""Build original fictional car GLBs from parametric surfaces; no external assets.

Usage: python3 tools/build_fictional_cars.py {agolf,mosquito,pitsunda} [--output FILE.glb]
"""
import argparse
from dataclasses import dataclass
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
        # A parametric patch can collapse to a point along one edge (for
        # example a triangular window tip). Its derivative normal is zero
        # there, although the emitted triangle still has a valid face.
        normals = [n if sum(x*x for x in n) > 1e-12 else unit(face) for n in normals]
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

    def disc(self, center, radius, material, color, axis='z', ring=0.0, floor_y=None, facing=1):
        for i in range(48):
            a, b = i*2*PI/48, (i+1)*2*PI/48
            def p(t, r):
                if axis == 'z':
                    return (center[0]+r*math.cos(t), center[1]+facing*r*math.sin(t), center[2])
                y = center[1]+r*math.cos(t)
                return (center[0], max(y, floor_y) if floor_y is not None else y, center[2]+facing*r*math.sin(t))
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

    def skin(self, hatch=False, arches=PAINT, rub_strip=True, doors=3):
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
                def lining(u,v):
                    a = u*PI
                    r = self.radius+.038
                    z = axle+r*math.cos(a)
                    w = profile(self.stations,z)[0]
                    return (side*(w-.23*v),self.cy+r*math.sin(a),z)
                b.patch(48,4,lining,RUBBER,(.018,.021,.025))
                w = profile(self.stations, axle)[0]
                b.disc((side*(w-.22),self.cy,axle),self.radius+.04,RUBBER,(.018,.021,.025),'x',floor_y=-.31)
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
            side = (.885 if hatch else .82) < q < .985 and clo+.20 < z < chi-.22 and abs(z+.22)>.055
            return GLASS if rear or front or side else PAINT
        b.patch(56, 96, greenhouse, glazing, lambda u,v:(.08,.13,.15) if glazing(u,v)==GLASS else WHITE)
        # Sills, rub strips, door seams, handles and mirror housings.
        for side in [-1,1]:
            w = profile(self.stations,0)[0]
            b.box((side*w-.012,-.21,-.95),(side*w+.012,-.15,.88),RUBBER,DARK)
            if rub_strip:
                b.box((side*(w+.008)-.012,.11,-1.15),(side*(w+.008)+.012,.14,1.12),RUBBER,DARK)
            for z in ([-.88,.75] if doors==3 else [-1.2,-.25,.75]):
                y = profile(self.stations,z)[2]
                b.box((side*w-.003,-.15,z-.003),(side*w+.003,y,z+.003),RUBBER,(.07,.065,.06))
            for z in ([-.65] if doors==3 else [-.90,-.05]):
                b.box((side*(w+.015)-.012,.24,z),(side*(w+.015)+.012,.265,z+.16),RUBBER,DARK)
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
    if style == 'pitsunda':
        return pitsunda_wheel(b, radius, side, face)
    if style == 'wire':
        return wire_wheel(b, radius, width, side, face)
    b.disc((face,0,0),radius*.83,METAL,CHROME,'x',radius*.76)
    b.disc((face-side*.012,0,0),radius*.76,RUBBER,(.065,.075,.08),'x')
    count = 8 if style=='mesh' else (7 if style=='modern' else 5)
    for i in range(count):
        a = i*2*PI/count
        for offset in ([-.28,.28] if style=='mesh' else ([-.075,.075] if style=='modern' else [0])):
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


def pitsunda_wheel(b, radius, side, face):
    # Reference: eight slim paired gunmetal spokes and substantial tire sidewalls.
    b.disc((face,0,0),radius*.91,RUBBER,(.026,.028,.029),'x',radius*.66,facing=side)
    b.disc((face+side*.002,0,0),radius*.70,METAL,(.32,.34,.34),'x',radius*.66,facing=side)
    b.disc((face,0,0),radius*.66,RUBBER,(.045,.048,.048),'x',facing=side)
    for i in range(8):
        for offset in [-.07,.07]:
            angle=i*2*PI/8
            def point(r,a):
                return (face+side*.006,r*math.cos(a),r*math.sin(a))
            points=[point(radius*.18,angle+offset-.055),point(radius*.665,angle+offset*.65-.030),
                    point(radius*.665,angle+offset*.65+.030),point(radius*.18,angle+offset+.055)]
            b.quad(points if side>0 else points[::-1],METAL,(.36,.38,.38))
    b.disc((face+side*.009,0,0),radius*.19,METAL,(.25,.27,.27),'x',facing=side)
    b.disc((face+side*.011,0,0),radius*.075,METAL,CHROME,'x',radius*.055,facing=side)
    for i in range(4):
        a=i*2*PI/4
        b.disc((face+side*.012,radius*.135*math.cos(a),radius*.135*math.sin(a)),
               .010,METAL,CHROME,'x',facing=side)
    return b


def wire_wheel(b, radius, width, side, face):
    # Thick sidewalls and fine radial spokes traced from the reference wheels.
    b.disc((face,0,0),radius*.90,RUBBER,(.025,.029,.032),'x',radius*.64,facing=side)
    b.disc((face+side*.002,0,0),radius*.66,METAL,CHROME,'x',radius*.61,facing=side)
    b.disc((face,0,0),radius*.61,RUBBER,(.035,.045,.05),'x',facing=side)
    for i in range(36):
        angle=i*2*PI/36
        inner=radius*.21
        outer=radius*.61
        spread=.011
        spokes=[(face+side*.005,inner*math.cos(angle-spread),inner*math.sin(angle-spread)),
                (face+side*.005,outer*math.cos(angle+.04-spread),outer*math.sin(angle+.04-spread)),
                (face+side*.005,outer*math.cos(angle+.04+spread),outer*math.sin(angle+.04+spread)),
                (face+side*.005,inner*math.cos(angle+spread),inner*math.sin(angle+spread))]
        b.quad(spokes if side > 0 else spokes[::-1],METAL,CHROME)
    b.disc((face+side*.008,0,0),radius*.23,METAL,CHROME,'x',facing=side)
    b.disc((face+side*.010,0,0),radius*.14,METAL,(.40,.45,.48),'x',radius*.115,facing=side)
    for i in range(5):
        angle=i*2*PI/5
        b.disc((face+side*.012,radius*.17*math.cos(angle),radius*.17*math.sin(angle)),
               .012,METAL,(.82,.85,.85),'x',facing=side)
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


def badge(b, text, center, size, color=CHROME, facing=1):
    z = center[2]
    for i,ch in enumerate(text):
        for row,bits in enumerate(FONT[ch]):
            for col in range(5):
                if bits & (1 << (4-col)):
                    x=center[0]+(i*6+col-len(text)*3)*size
                    y=center[1]+(3-row)*size
                    points=[(x,y,z),(x+size*.85,y,z),(x+size*.85,y+size*.85,z),(x,y+size*.85,z)]
                    b.quad(points if facing > 0 else points[::-1],METAL,color)


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


@dataclass(frozen=True)
class TracePoint:
    x: float
    y: float


@dataclass(frozen=True)
class WheelFit:
    rear_radius: float
    front_radius: float
    rear_width: float = .18
    front_width: float = .18


@dataclass(frozen=True)
class WindowSpan:
    top: float
    bottom: float


class MosquitoTrace:
    """Side elevation traced in pixels from the supplied blue coupe reference.

    Bumper bounds 269..1525 map to 4.724 m; roof/ground 179..548 map
    to the annotated 1.298 m height. +Z points toward the image's left.
    Keeping the source landmarks here makes the silhouette editable directly.
    """
    def __init__(self, mesh):
        self.mesh = mesh
        self.top = [TracePoint(269,355), TracePoint(288,340), TracePoint(330,319),
                    TracePoint(405,304), TracePoint(515,292), TracePoint(621,282),
                    TracePoint(680,286), TracePoint(716,299), TracePoint(795,301),
                    TracePoint(1035,295), TracePoint(1150,280), TracePoint(1306,267),
                    TracePoint(1406,283), TracePoint(1490,300), TracePoint(1506,311),
                    TracePoint(1512,359)]
        self.lower = [TracePoint(269,415), TracePoint(292,438), TracePoint(322,463),
                      TracePoint(372,476), TracePoint(614,480), TracePoint(1128,471),
                      TracePoint(1353,449), TracePoint(1448,437), TracePoint(1480,424),
                      TracePoint(1512,381)]
        self.roof = [TracePoint(685,290), TracePoint(720,255), TracePoint(765,213),
                     TracePoint(800,190), TracePoint(844,182), TracePoint(946,179),
                     TracePoint(1046,181), TracePoint(1110,186), TracePoint(1165,198),
                     TracePoint(1190,214), TracePoint(1260,251), TracePoint(1306,268)]
        self.arches = [
            [TracePoint(380,476), TracePoint(384,438), TracePoint(395,410),
             TracePoint(415,389), TracePoint(445,380), TracePoint(483,377),
             TracePoint(521,380), TracePoint(551,391), TracePoint(574,412),
             TracePoint(589,445), TracePoint(603,479)],
            [TracePoint(1130,471), TracePoint(1140,429), TracePoint(1153,397),
             TracePoint(1175,375), TracePoint(1205,365), TracePoint(1247,361),
             TracePoint(1287,364), TracePoint(1318,376), TracePoint(1339,400),
             TracePoint(1350,431), TracePoint(1355,449)]]

    @staticmethod
    def height(pixel_y):
        return (548-pixel_y)*1.298/369-.48

    @staticmethod
    def length(pixel_x):
        return (897-pixel_x)*4.724/1256

    @staticmethod
    def at(points, x):
        for a, b in zip(points, points[1:]):
            if x <= b.x:
                t = max(0, min(1, (x-a.x)/(b.x-a.x)))
                return a.y+(b.y-a.y)*t
        return points[-1].y

    def width(self, x):
        return self.at([TracePoint(269,.79), TracePoint(330,.86),
                        TracePoint(490,.91), TracePoint(720,.86),
                        TracePoint(1035,.87), TracePoint(1250,.93),
                        TracePoint(1450,.87), TracePoint(1512,.80)], x)

    def bottom(self, x):
        y = self.at(self.lower, x)
        for arch in self.arches:
            if arch[0].x <= x <= arch[-1].x:
                y = min(y, self.at(arch, x))
        return y

    def side(self, side, point, offset=0):
        top = self.at(self.top, point.x)
        low = self.bottom(point.x)
        t = max(0, min(1, (point.y-top)/max(1, low-top)))
        # Crown the fenders, tuck the rocker, and retain the long belt crease.
        inset = .036*t*t + .012*math.sin(t*2*PI)
        return (side*(self.width(point.x)-inset+offset),
                self.height(point.y), self.length(point.x))

    def cabin_side(self, side, point, offset=0):
        top = self.at(self.roof, point.x)
        base = self.at(self.top, point.x)
        t = max(0, min(1, (point.y-top)/max(1,base-top)))
        return (side*(.715+(self.width(point.x)-.715)*t+offset),
                self.height(point.y), self.length(point.x))

    def line(self, side, points, material, color, width=1.3, cabin=False, offset=.004):
        surface = self.cabin_side if cabin else self.side
        samples = []
        for a, b in zip(points, points[1:]):
            steps = max(1, math.ceil(math.hypot(b.x-a.x,b.y-a.y)/2))
            samples.extend(TracePoint(a.x+(b.x-a.x)*i/steps,
                                      a.y+(b.y-a.y)*i/steps) for i in range(steps))
        samples.append(points[-1])
        for a, b in zip(samples, samples[1:]):
            d = math.hypot(b.x-a.x,b.y-a.y)
            dx, dy = -(b.y-a.y)*width/(2*d), (b.x-a.x)*width/(2*d)
            corners = [TracePoint(a.x+dx,a.y+dy), TracePoint(b.x+dx,b.y+dy),
                       TracePoint(b.x-dx,b.y-dy), TracePoint(a.x-dx,a.y-dy)]
            vertices = [surface(side, p, offset) for p in corners]
            self.mesh.quad(vertices if side > 0 else vertices[::-1], material, color)

    def roof_half_width(self, x):
        return .715

    def shell(self, close_front=True):
        b = self.mesh
        # Split at every contour landmark so the cutouts follow the trace exactly.
        cuts = sorted({p.x for points in [self.top,self.lower,*self.arches]
                       for p in points})
        for side in [-1,1]:
            for left,right in zip(cuts,cuts[1:]):
                def flank(u,v):
                    x = left+(right-left)*(u if side > 0 else 1-u)
                    y = self.bottom(x)+(self.at(self.top,x)-self.bottom(x))*v
                    return self.side(side,TracePoint(x,y))
                b.patch(max(1,int((right-left)/3)),16,flank,PAINT,WHITE)
            for arch in self.arches:
                self.line(side,arch,METAL,CHROME,width=1.7,offset=.007)
                for a,c in zip(arch,arch[1:]):
                    outer_a,outer_c = self.side(side,a),self.side(side,c)
                    inner_a = (outer_a[0]-side*.18,outer_a[1],outer_a[2])
                    inner_c = (outer_c[0]-side*.18,outer_c[1],outer_c[2])
                    b.quad([outer_a,outer_c,inner_c,inner_a],RUBBER,DARK)
            for left,right in zip(self.roof,self.roof[1:]):
                def pillar(u,v):
                    x=left.x+(right.x-left.x)*(u if side > 0 else 1-u)
                    y=self.at(self.top,x)+(self.at(self.roof,x)-self.at(self.top,x))*v
                    return self.cabin_side(side,TracePoint(x,y))
                b.patch(max(1,int((right.x-left.x)/4)),8,pillar,PAINT,WHITE)
        # Hood/deck and a softly crowned roof connect both traced side panels.
        for left,right in zip(self.top,self.top[1:]):
            def deck(u,v):
                x=left.x+(right.x-left.x)*u
                q=1-v*2
                return (q*self.width(x), self.height(self.at(self.top,x))+.022*(1-q*q), self.length(x))
            b.patch(max(1,int((right.x-left.x)/4)),24,deck,PAINT,WHITE)
        for left,right in zip(self.roof,self.roof[1:]):
            def roof(u,v):
                x=left.x+(right.x-left.x)*u
                q=1-v*2
                # The centre of the roof reaches the traced highest point.
                return (q*self.roof_half_width(x),self.height(self.at(self.roof,x))-.018*q*q,self.length(x))
            b.patch(max(1,int((right.x-left.x)/3)),24,roof,PAINT,WHITE)
        for x in ([self.top[0].x] if close_front else []):
            b.quad([self.side(-1,TracePoint(x,self.at(self.lower,x))),
                    self.side(1,TracePoint(x,self.at(self.lower,x))),
                    self.side(1,TracePoint(x,self.at(self.top,x))),
                    self.side(-1,TracePoint(x,self.at(self.top,x)))],PAINT,WHITE)

    def windows(self):
        # Hardtop opening and the slim divider in front of the triangular quarter glass.
        windows = [[TracePoint(747,291),TracePoint(835,211),TracePoint(997,211),
                    TracePoint(1027,217),TracePoint(1026,291)],
                   [TracePoint(1040,219),TracePoint(1074,241),TracePoint(1135,273),
                    TracePoint(1040,289)]]
        for side in [-1,1]:
            for outline in windows:
                # Vertical strips follow the curved pillar surface without
                # large decal triangles cutting through the painted shell.
                cuts=sorted({p.x for p in outline} |
                            {p.x for p in self.roof+self.top
                             if min(p.x for p in outline) < p.x < max(p.x for p in outline)})
                for left,right in zip(cuts,cuts[1:]):
                    def limits(x):
                        x=max(min(p.x for p in outline),min(max(p.x for p in outline),x))
                        crossings=[]
                        for a,c in zip(outline,outline[1:]+outline[:1]):
                            if a.x != c.x and min(a.x,c.x) <= x <= max(a.x,c.x):
                                crossings.append(a.y+(c.y-a.y)*(x-a.x)/(c.x-a.x))
                        return WindowSpan(top=min(crossings),bottom=max(crossings))
                    def pane(u,v):
                        x=left+(right-left)*(u if side > 0 else 1-u)
                        span=limits(x)
                        return self.cabin_side(side,TracePoint(x,span.top+(span.bottom-span.top)*v),.009)
                    self.mesh.patch(max(1,math.ceil((right-left)/2)),6,pane,GLASS,(.12,.17,.20))
                self.line(side,outline+outline[:1],RUBBER,DARK,width=5,cabin=True,offset=.008)
            surround=[TracePoint(738,295),TracePoint(830,203),TracePoint(999,203),
                      TracePoint(1038,210),TracePoint(1083,241),TracePoint(1142,275)]
            self.line(side,surround,METAL,CHROME,width=1.8,cabin=True,offset=.011)
            self.line(side,[TracePoint(747,295),TracePoint(1028,296)],METAL,CHROME,
                      width=2,cabin=True,offset=.011)
        # Windshield and backlight lie on the transverse sloping roof surface.
        for start,end in [(706,803)]:
            def glass(u,v):
                x=start+(end-start)*u
                q=(1-v*2)*.88
                return (q*.715,self.height(self.at(self.roof,x))-.018*q*q+.009,self.length(x))
            self.mesh.patch(end-start,32,glass,GLASS,(.12,.17,.20))


class MosquitoRear:
    """Rear reference landmarks: bumper x=291..857, roof/ground y=134..501.

    The reference's 1.88 m width and 1.298 m height set the elevation scale.
    Rearward layers follow the curved tail; lens faces retain the brake and
    reverse material codes used by the game's working lights.
    """
    def __init__(self, mesh, side_trace):
        self.mesh = mesh
        self.side_trace = side_trace
        self.rear = side_trace.length(1512)

    @staticmethod
    def height(pixel_y):
        return (501-pixel_y)*1.298/367-.48

    def point(self, point, offset=0):
        x=(point.x-574)*1.88/566
        y=self.height(point.y)
        # Rounded quarters tuck forward; the bumper is the rearmost surface.
        z=self.rear-.020+.020*(abs(x)/.94)**4+.005*(y-.1)**2-offset
        return (x,y,z)

    def panel(self, outline, material, color, offset=0):
        # Small strips conform decals to the tail's transverse curvature.
        cuts=sorted({p.x for p in outline})
        for left,right in zip(cuts,cuts[1:]):
            def bounds(x):
                x=max(min(p.x for p in outline),min(max(p.x for p in outline),x))
                crossings=[]
                for a,c in zip(outline,outline[1:]+outline[:1]):
                    if a.x != c.x and min(a.x,c.x) <= x <= max(a.x,c.x):
                        crossings.append(a.y+(c.y-a.y)*(x-a.x)/(c.x-a.x))
                return WindowSpan(top=min(crossings),bottom=max(crossings))
            def surface(u,v):
                x=left+(right-left)*u
                span=bounds(x)
                return self.point(TracePoint(x,span.top+(span.bottom-span.top)*v),offset)
            self.mesh.patch(max(1,math.ceil((right-left)/3)),6,surface,material,color)

    def line(self, points, material, color, width=1.5, offset=.006):
        for a,c in zip(points,points[1:]):
            steps=max(1,math.ceil(math.hypot(c.x-a.x,c.y-a.y)/3))
            for i in range(steps):
                start=TracePoint(a.x+(c.x-a.x)*i/steps,a.y+(c.y-a.y)*i/steps)
                end=TracePoint(a.x+(c.x-a.x)*(i+1)/steps,a.y+(c.y-a.y)*(i+1)/steps)
                d=math.hypot(end.x-start.x,end.y-start.y)
                dx=-(end.y-start.y)*width/(2*d)
                dy=(end.x-start.x)*width/(2*d)
                self.mesh.quad([self.point(p,offset) for p in
                    [TracePoint(start.x+dx,start.y+dy),TracePoint(end.x+dx,end.y+dy),
                     TracePoint(end.x-dx,end.y-dy),TracePoint(start.x-dx,start.y-dy)]][::-1],
                    material,color)

    def rectangle(self, left, top, right, bottom, material, color, offset=0):
        self.panel([TracePoint(left,top),TracePoint(right,top),
                    TracePoint(right,bottom),TracePoint(left,bottom)],material,color,offset)

    def shell(self):
        # The upper tail panel and rounded lower valance share a continuous skin.
        contour=[TracePoint(280,246),TracePoint(287,253),TracePoint(312,262),
                 TracePoint(332,270),TracePoint(345,274),TracePoint(362,262),
                 TracePoint(390,250),TracePoint(409,232),TracePoint(420,210),
                 TracePoint(425,180)]
        for top,bottom in zip(contour,contour[1:]):
            def skin(u,v):
                y=top.x+(bottom.x-top.x)*v
                half=self.side_trace.at(contour,y)
                return self.point(TracePoint(574+(u*2-1)*half,y))
            self.mesh.patch(80,6,skin,PAINT,WHITE)
        panel=[TracePoint(333,286),TracePoint(815,286),TracePoint(828,295),
               TracePoint(842,333),TracePoint(306,333),TracePoint(320,297)]
        self.panel(panel,PAINT,(.31,.40,.40),.003)
        self.line(panel+[panel[0]],RUBBER,DARK,width=2.5,offset=.005)
        # Deck shut line sits above the inset lamp panel.
        self.line([TracePoint(330,280),TracePoint(450,278),TracePoint(700,278),
                   TracePoint(818,280)],RUBBER,DARK,width=1.5)

    def lamps(self):
        for side in [-1,1]:
            def reflect(point):
                return TracePoint(574+side*(point.x-574),point.y)
            outline=[TracePoint(338,298),TracePoint(478,298),TracePoint(484,302),
                     TracePoint(484,322),TracePoint(480,326),TracePoint(328,326),
                     TracePoint(330,309)]
            outline=[reflect(p) for p in outline]
            self.panel(outline,RUBBER,(.018,.022,.025),.009)
            self.line(outline+outline[:1],METAL,CHROME,width=1.8,offset=.012)
            # Three smoked red cells, each divided into upper and lower lenses.
            for left,right in [(336,382),(385,431),(434,480)]:
                for top,bottom in [(301,310),(314,322)]:
                    a=reflect(TracePoint(left,top));c=reflect(TracePoint(right,bottom))
                    self.rectangle(min(a.x,c.x),a.y,max(a.x,c.x),c.y,BRAKE,
                                   (.56,.025,.02) if left==385 else (.25,.016,.015),.014)
            self.line([reflect(TracePoint(331,312)),reflect(TracePoint(483,312))],
                      METAL,(.35,.38,.39),width=1,offset=.016)
            # Low rectangular reversing lamps have their own chrome bezels.
            reverse=[TracePoint(384,380),TracePoint(429,380),TracePoint(433,383),
                     TracePoint(433,394),TracePoint(429,398),TracePoint(384,398),
                     TracePoint(380,394),TracePoint(380,384)]
            reverse=[reflect(p) for p in reverse]
            self.panel(reverse,RUBBER,DARK,.006)
            self.line(reverse+reverse[:1],METAL,CHROME,width=1.8,offset=.009)
            a=reflect(TracePoint(385,383));c=reflect(TracePoint(429,395))
            self.rectangle(min(a.x,c.x),a.y,max(a.x,c.x),c.y,REVERSE,(.60,.63,.60),.012)
            for x in range(388,429,5):
                self.line([reflect(TracePoint(x,384)),reflect(TracePoint(x,394))],
                          METAL,(.32,.35,.36),width=.8,offset=.014)

    def bumper(self):
        # Rounded rail, including upper/lower returns, follows the full 1.88 m width.
        def rail(u,v):
            q=u*2-1
            angle=v*2*PI
            x=q*.94
            y=self.height(345)+.040*math.cos(angle)+.008*q*q
            z=self.rear-.021-.027*math.sin(angle)+.11*abs(q)**6
            return (x,y,z)
        self.mesh.patch(112,24,rail,METAL,CHROME)
        self.line([TracePoint(321,364),TracePoint(400,367),TracePoint(748,367),
                   TracePoint(827,364)],METAL,(.46,.50,.51),width=3,offset=.012)
        for center in [493,655]:
            # Slim vertical over-riders extend below the main rail.
            def guard(u,v):
                angle=-u*2*PI
                y=self.height(353)+(self.height(407)-self.height(353))*v
                return ((center-574)*1.88/566+.015*math.cos(angle),y,
                        self.rear-.019-.024*math.sin(angle))
            self.mesh.patch(20,16,guard,METAL,CHROME)
        for x in [389,761]:
            center=self.point(TracePoint(x,345),.018)
            self.mesh.disc((center[0],center[1],self.rear-.046),.009,METAL,(.44,.48,.49),facing=-1)

    def badges_and_plate(self):
        self.rectangle(514,356,636,424,RUBBER,DARK,.010)
        self.rectangle(522,370,626,424,METAL,(.19,.22,.23),.014)
        self.rectangle(528,377,621,421,RUBBER,(.91,.92,.88),.017)
        # Blank plate matches the supplied image; its rim remains visible.
        plate=[TracePoint(528,377),TracePoint(621,377),TracePoint(621,421),
               TracePoint(528,421),TracePoint(528,377)]
        self.line(plate,METAL,(.56,.59,.57),width=1,offset=.019)
        for left in [554,576]:
            letter=[TracePoint(left+16,305),TracePoint(left+5,304),TracePoint(left+1,307),
                    TracePoint(left+3,311),TracePoint(left+14,312),TracePoint(left+16,315),
                    TracePoint(left+12,318),TracePoint(left+1,317)]
            self.line(letter,RUBBER,DARK,width=4.5,offset=.011)
            self.line(letter,METAL,CHROME,width=2.1,offset=.014)
        lock=self.point(TracePoint(574,291),.014)
        self.mesh.disc(lock,.018,METAL,CHROME,ring=.012,facing=-1)
        self.line([TracePoint(573,289),TracePoint(575,293)],RUBBER,DARK,width=1,offset=.016)
        # Keep the fictional model's own name on the deck badge.
        lettering=Mesh()
        badge(lettering,'MOSQUITO',(.53,0,0),.0028,facing=-1)
        for start in range(0,len(lettering.vertices),3):
            points=[]
            for vertex in lettering.vertices[start:start+3]:
                pixel=1488-vertex[1]*2000
                points.append((vertex[0],
                    self.side_trace.height(self.side_trace.at(self.side_trace.top,pixel))+.004,
                    self.side_trace.length(pixel)))
            self.mesh.triangle(points,METAL,CHROME)

    def rear_window(self):
        def surface(u,v,lift):
            x=1178+104*u
            q=1-v*2
            return (q*(.60+.065*u),
                    self.side_trace.height(self.side_trace.at(self.side_trace.roof,x))-.018*q*q+lift,
                    self.side_trace.length(x))
        self.mesh.patch(104,36,lambda u,v:surface(u,v,.012),RUBBER,DARK)
        self.mesh.patch(100,32,lambda u,v:surface(.035+.93*u,.025+.95*v,.016),
                        GLASS,(.12,.17,.20))
        for edge in [0,1]:
            self.mesh.patch(104,2,lambda u,v:surface(u,edge+(v-.5)*.008,.019),METAL,CHROME)
            self.mesh.patch(2,36,lambda u,v:surface(edge+(u-.5)*.012,v,.019),METAL,CHROME)

    def build(self):
        self.shell()
        self.lamps()
        self.bumper()
        self.badges_and_plate()
        self.rear_window()
        axle=self.side_trace.length(1248)
        self.mesh.box((-.60,-.25,axle-.05),(.60,-.20,axle+.05),RUBBER,DARK)
        def differential(u,v):
            azimuth=u*2*PI
            elevation=v*PI
            return (.10*math.sin(elevation)*math.cos(azimuth),
                    -.22+.09*math.cos(elevation),
                    axle+.10*math.sin(elevation)*math.sin(azimuth))
        self.mesh.patch(32,16,differential,RUBBER,(.035,.04,.04))


def mosquito():
    b = Mesh()
    trace = MosquitoTrace(b)
    trace.shell()
    trace.windows()
    for side in [-1,1]:
        # Full length coupe door, chrome rocker, pull handle and three quarter vents.
        trace.line(side,[TracePoint(720,305),TracePoint(693,367),TracePoint(712,465),
                         TracePoint(1045,454),TracePoint(1048,352),TracePoint(1035,297)],
                   RUBBER,(.055,.06,.065),width=1.5)
        trace.line(side,[TracePoint(612,470),TracePoint(1128,457)],METAL,CHROME,width=3)
        trace.line(side,[TracePoint(991,322),TracePoint(1033,320)],METAL,CHROME,width=4)
        trace.line(side,[TracePoint(1000,326),TracePoint(1026,325)],RUBBER,DARK,width=2)
        for x in [1100,1117,1134]:
            trace.line(side,[TracePoint(x,431),TracePoint(x+12,395),TracePoint(x+23,372)],
                       METAL,CHROME,width=7,offset=.007)
            trace.line(side,[TracePoint(x+1,430),TracePoint(x+13,395),TracePoint(x+23,373)],
                       RUBBER,DARK,width=4,offset=.009)
        trace.line(side,[TracePoint(282,391),TracePoint(330,390)],METAL,CHROME,width=5)
        trace.line(side,[TracePoint(1462,368),TracePoint(1480,367)],METAL,CHROME,width=6)
        trace.line(side,[TracePoint(1464,368),TracePoint(1478,367)],RUBBER,DARK,width=3,offset=.007)
        # Small fender SS emblem and the chrome mirror at the windshield foot.
        for x in [640,658]:
            trace.line(side,[TracePoint(x+12,397),TracePoint(x+3,397),TracePoint(x,400),
                             TracePoint(x+11,401),TracePoint(x+9,404),TracePoint(x,404)],
                       METAL,CHROME,width=1.5)
        mount=trace.side(side,TracePoint(781,302),.02)
        b.box((mount[0]-.022,mount[1],mount[2]-.015),
              (mount[0]+.022,mount[1]+.075,mount[2]+.015),METAL,CHROME)
        b.box((side*.99-.045,mount[1]+.06,mount[2]-.055),
              (side*.99+.045,mount[1]+.135,mount[2]+.025),METAL,CHROME)
    front=trace.length(269)-.014
    b.box((-.78,trace.height(410),front),(.78,trace.height(355),front+.004),RUBBER,DARK)
    for side in [-1,1]:
        b.disc((side*.64,trace.height(382),front+.009),.094,METAL,CHROME)
        b.disc((side*.64,trace.height(382),front+.014),.082,LENS,(.72,.77,.74))
    b.box((-.80,trace.height(429),front-.015),(.80,trace.height(411),front+.008),METAL,CHROME)
    MosquitoRear(b,trace).build()
    # Match the slightly taller rear tire in the supplied side elevation.
    fit=WheelFit(rear_radius=.334,front_radius=.306,rear_width=.255,front_width=.225)
    axles=[trace.length(1248),trace.length(491)]
    return b,axles,fit,.70,'wire'


class PitsundaTrace(MosquitoTrace):
    """Landmarks from the supplied side, front and rear photographs.

    Side bumper tips 364..1400 = 4.065 m, body width = 1.695 m.
    Ground 524 / roof 120 = 1.455 m; shark fin adds .040 m.
    Coordinates stay in reference pixels so future edits can follow the photos.
    """
    def __init__(self, mesh):
        self.mesh = mesh
        def points(values):
            return [TracePoint(x, y) for x, y in values]
        self.top = points([(364,338),(388,322),(430,301),(500,278),(590,257),
                           (659,244),(740,252),(890,249),(1050,239),(1195,226),
                           (1280,239),(1349,273),(1395,320),(1400,348)])
        self.lower = points([(364,440),(372,453),(419,466),(650,466),
                             (950,463),(1145,460),(1345,442),(1384,433),(1400,400)])
        self.roof = points([(650,246),(710,207),(777,166),(830,141),(896,126),
                            (967,120),(1080,121),(1179,129),(1226,146),
                            (1280,188),(1328,221),(1368,258),(1380,289)])
        self.arches = [points([(422,466),(430,421),(445,382),(465,355),(490,339),
                              (521,329),(549,328),(580,336),(610,352),(633,381),
                              (646,424),(650,466)]),
                       points([(1145,460),(1155,415),(1170,373),(1194,344),
                               (1226,326),(1258,321),(1288,326),(1317,342),
                               (1337,369),(1347,406),(1345,442)])]

    @staticmethod
    def height(pixel_y):
        return (524-pixel_y)*1.455/404-.48

    @staticmethod
    def length(pixel_x):
        return (882-pixel_x)*4.065/1036

    def width(self, x):
        return self.at([TracePoint(364,.62),TracePoint(400,.77),TracePoint(490,.837),
                        TracePoint(650,.815),TracePoint(950,.817),TracePoint(1140,.83),
                        TracePoint(1255,.8475),TracePoint(1340,.80),TracePoint(1400,.70)],x)

    def roof_half_width(self, x):
        return self.at([TracePoint(650,.69),TracePoint(800,.675),
                        TracePoint(1100,.67),TracePoint(1225,.64),TracePoint(1350,.68)],x)

    def cabin_side(self, side, point, offset=0):
        top = self.at(self.roof,point.x)
        base = self.at(self.top,point.x)
        t = max(0,min(1,(point.y-top)/max(1,base-top)))
        roof_width = self.roof_half_width(point.x)
        return (side*(roof_width+(self.width(point.x)-roof_width)*t+offset),
                self.height(point.y),self.length(point.x))

    def shell(self):
        # The shared tracing machinery builds curved flanks with actual arch holes.
        super().shell(close_front=False)
        # Close the hatch end below the rear glazing.
        for side in [-1,1]:
            for arch in self.arches:
                self.line(side,arch,PAINT,WHITE,width=3.4,offset=.010)
        for side in [-1,1]:
            for pixel in [539,1255]:
                center=(side*(self.width(pixel)-.16),-.162,self.length(pixel))
                self.mesh.disc(center,.46,RUBBER,(.018,.02,.021),'x',floor_y=-.27,facing=side)
            # Side projection of the swept headlamp, independent of the front face.
            lamp=[TracePoint(366,338),TracePoint(413,318),TracePoint(478,305),
                  TracePoint(459,325),TracePoint(410,351),TracePoint(373,358)]
            center=TracePoint(417,332)
            for a,c in zip(lamp,lamp[1:]+lamp[:1]):
                self.mesh.triangle([self.side(side,p,.009) for p in
                                    ([center,a,c] if side>0 else [center,c,a])],LENS,(.62,.68,.69))
            self.line(side,lamp+[lamp[0]],RUBBER,DARK,width=1.7,offset=.012)
            # Red side faces wrap forward from the hatch onto the rear quarters.
            outline=[TracePoint(1280,228),TracePoint(1317,225),TracePoint(1354,232),
                     TracePoint(1368,255),TracePoint(1352,271),TracePoint(1327,267)]
            center=TracePoint(1333,246)
            for a,c in zip(outline,outline[1:]+outline[:1]):
                self.mesh.triangle([self.cabin_side(side,p,.014) for p in
                                    ([center,a,c] if side>0 else [center,c,a])],BRAKE,(.64,.028,.04))

    def windows(self):
        outlines = [
            [(702,253),(735,224),(788,187),(839,163),(899,148),(964,146),
             (971,153),(951,236),(819,248)],
            [(991,145),(1075,147),(1146,157),(1198,176),(1213,190),
             (1193,211),(1155,224),(981,237)]]
        for side in [-1,1]:
            for outline in outlines:
                points=[TracePoint(x,y) for x,y in outline]
                center=TracePoint(sum(p.x for p in points)/len(points),sum(p.y for p in points)/len(points))
                # Tessellate each fan on the curved surface, avoiding a flat pane
                # intersecting the crowned greenhouse.
                for a,c in zip(points,points[1:]+points[:1]):
                    def pane(u,v):
                        edge=TracePoint(a.x+(c.x-a.x)*u,a.y+(c.y-a.y)*u)
                        point=TracePoint(center.x+(edge.x-center.x)*v,center.y+(edge.y-center.y)*v)
                        return self.cabin_side(side,point,.007)
                    self.mesh.patch(10,10,lambda u,v:pane(u if side>0 else 1-u,v),GLASS,(.105,.135,.14))
                self.line(side,points+[points[0]],RUBBER,(.018,.021,.023),width=4,cabin=True,offset=.009)
            self.line(side,[TracePoint(979,140),TracePoint(961,244)],RUBBER,(.035,.038,.04),width=18,cabin=True,offset=.006)
        # Curved windscreen and hatch glass fit the roof and belt contours.
        for rear in [False,True]:
            start,end=(1207,1320) if rear else (673,833)
            def glass(u,v):
                x=start+(end-start)*v+12*(abs(2*u-1)**4)*(1-2*v)
                q=(2*u-1)*(.89 if rear else .92)
                roof_width=self.roof_half_width(x)*(1 if rear else 1-.18*v)
                y=self.height(self.at(self.roof,x))-.018*q*q+.008
                return (q*roof_width,y,self.length(x))
            self.mesh.patch(36,32,glass,GLASS,(.085,.115,.12))
            for edge in [0,1]:
                self.mesh.patch(2,32,lambda u,v:glass(edge+(u-.5)*.018,v),RUBBER,DARK)

    def details(self):
        b=self.mesh
        for side in [-1,1]:
            self.line(side,[TracePoint(688,254),TracePoint(674,294),TracePoint(672,369),
                           TracePoint(680,431),TracePoint(688,446),TracePoint(954,445),
                           TracePoint(978,246),TracePoint(992,139)],RUBBER,(.045,.045,.043),width=1.5,cabin=False)
            self.line(side,[TracePoint(1220,164),TracePoint(1241,202),TracePoint(1246,240),
                           TracePoint(1234,285),TracePoint(1215,324),TracePoint(1179,365),
                           TracePoint(1132,430),TracePoint(1110,444),TracePoint(954,445)],RUBBER,(.045,.045,.043),width=1.5)
            self.line(side,[TracePoint(693,427),TracePoint(825,414),TracePoint(955,402),
                           TracePoint(1107,386)],PAINT,(.72,.73,.72),width=5,offset=.003)
            self.line(side,[TracePoint(648,455),TracePoint(956,451),TracePoint(1144,450)],RUBBER,DARK,width=5)
            for x,y in [(920,292),(1185,267)]:
                handle=[TracePoint(x-27,y+1),TracePoint(x-20,y-6),TracePoint(x+11,y-8),TracePoint(x+27,y-3),TracePoint(x+23,y+4),TracePoint(x-15,y+8)]
                self.line(side,handle+[handle[0]],RUBBER,(.025,.027,.027),width=7,offset=.006)
                self.line(side,[TracePoint(x-21,y+2),TracePoint(x+23,y-1)],PAINT,(.72,.73,.72),width=5,offset=.012)
            # Mirror with a tapered shell and separate stalk, matched to x=737..812.
            mount=self.cabin_side(side,TracePoint(747,251),.01)
            b.box((side*.86-.035,mount[1]-.02,mount[2]-.02),(side*.86+.035,mount[1]+.045,mount[2]+.045),RUBBER,DARK)
            def mirror(u,v):
                az=u*2*PI; el=v*PI
                return (side*(.898+.074*math.sin(el)*math.cos(az)),
                        self.height(242)+.073*math.cos(el),
                        self.length(773)+.145*math.sin(el)*math.sin(az))
            b.patch(28,16,mirror,PAINT,(.53,.54,.53))
            b.box((side*.898-.055,self.height(256),self.length(807)),
                  (side*.898+.055,self.height(230),self.length(805)),GLASS,(.24,.28,.28))
        # Small rear roof spoiler, high stop lamp, shark fin, and rear wiper.
        z=self.length(1222)
        b.box((-.61,self.height(151),z-.05),(.61,self.height(140),z+.08),PAINT,WHITE)
        b.box((-.17,self.height(153),z-.052),(.17,self.height(148),z-.05),BRAKE,(.45,.012,.018))
        b.triangle([(-.012,self.height(124),self.length(1110)),(-.012,self.height(108.894),self.length(1163)),(-.012,self.height(124),self.length(1161))],PAINT,WHITE)
        b.triangle([(.012,self.height(124),self.length(1161)),(.012,self.height(108.894),self.length(1163)),(.012,self.height(124),self.length(1110))],PAINT,WHITE)

    @staticmethod
    def elevation(point, rear=False, offset=0):
        # Front reference: body x=360..852, ground 558, roof 151.
        # Rear reference: body x=350..850, ground 569, roof 149.
        if rear:
            x=(point.x-600)*1.695/500
            y=(569-point.y)*1.455/420-.48
            z=MosquitoTrace.at([TracePoint(150,-1.20),TracePoint(260,-1.80),
                                TracePoint(300,-1.94),TracePoint(380,-2.015),
                                TracePoint(450,-2.0325),TracePoint(505,-1.96)],point.y)+.11*(abs(x)/.8475)**3
            return (x,y,max(z,-2.0075)-offset)
        x=(point.x-606)*1.695/492
        y=(558-point.y)*1.455/407-.48
        z=2.0005-.07*(abs(x)/.8475)**3-.20*max(0,(365-point.y)/60)-.055*max(0,(point.y-440)/100)
        return (x,y,z+offset)

    def panel(self, outline, material, color, rear=False, offset=.005):
        points=[TracePoint(x,y) for x,y in outline]
        center=TracePoint(sum(p.x for p in points)/len(points),sum(p.y for p in points)/len(points))
        area=sum(a.x*c.y-c.x*a.y for a,c in zip(points,points[1:]+points[:1]))
        reverse=(area<0) != rear
        for a,c in zip(points,points[1:]+points[:1]):
            def surface(u,v):
                edge=TracePoint(a.x+(c.x-a.x)*u,a.y+(c.y-a.y)*u)
                return self.elevation(TracePoint(center.x+(edge.x-center.x)*v,center.y+(edge.y-center.y)*v),rear,offset)
            self.mesh.patch(4,4,lambda u,v:surface(1-u if reverse else u,v),material,color)

    def face_line(self, outline, material, color, rear=False, width=2, offset=.01):
        points=[TracePoint(x,y) for x,y in outline]
        for a,c in zip(points,points[1:]):
            d=math.hypot(c.x-a.x,c.y-a.y)
            dx,dy=-(c.y-a.y)*width/(2*d),(c.x-a.x)*width/(2*d)
            def ribbon(u,v):
                t=u if rear else 1-u
                point=TracePoint(a.x+(c.x-a.x)*t+dx*(2*v-1),
                                 a.y+(c.y-a.y)*t+dy*(2*v-1))
                return self.elevation(point,rear,offset)
            self.mesh.patch(max(2,int(d/12)),1,ribbon,material,color)

    def faces(self):
        b=self.mesh
        front=[(383,334),(422,360),(471,365),(739,365),(789,360),(831,334),
               (848,391),(849,491),(836,528),(786,535),(429,535),(373,521),(363,490),(362,391)]
        self.panel(front,PAINT,WHITE,offset=0)
        grille=[(471,395),(499,388),(712,388),(739,397),(730,435),(711,465),
                (677,482),(608,495),(541,482),(506,467),(485,440),(471,395)]
        self.panel(grille[:-1],RUBBER,(.012,.016,.018),offset=.008)
        self.face_line(grille,METAL,(.66,.68,.67),width=4,offset=.014)
        self.face_line([(477,395),(504,413),(608,425),(711,413),(736,395)],METAL,(.42,.45,.45),width=2,offset=.018)
        for y in [407,417,430,442,454,465]:
            half=119-abs(y-415)*.49
            self.face_line([(606-half,y),(606+half,y)],RUBBER,(.065,.07,.073),width=1.4,offset=.013)
        self.panel([(496,478),(539,489),(607,501),(677,489),(715,478),(772,519),(441,519)],RUBBER,(.014,.017,.018),offset=.012)
        self.panel([(560,435),(655,435),(655,459),(560,459)],RUBBER,DARK,offset=.03)
        self.panel([(563,437),(652,437),(652,456),(563,456)],RUBBER,(.92,.93,.91),offset=.033)
        for side in [-1,1]:
            def reflect(outline):
                return [(606+side*(x-606),y) for x,y in outline]
            lamp=[(383,334),(398,340),(439,368),(453,382),(443,399),(420,404),(393,398),(377,382),(377,357)]
            self.panel(reflect(lamp),RUBBER,DARK,offset=.012)
            self.panel(reflect([(385,340),(400,346),(437,372),(447,383),(439,395),(418,399),(395,392),(382,378)]),LENS,(.61,.68,.70),offset=.017)
            center=self.elevation(TracePoint(606+side*(410-606),378),offset=.024)
            b.disc(center,.052,METAL,(.62,.65,.65),facing=1)
            b.disc((center[0],center[1],center[2]+.002),.038,LENS,(.35,.43,.45))
            self.face_line(reflect([(385,342),(401,350),(436,374)]),LENS,(.82,.86,.85),width=3,offset=.022)
            self.panel(reflect([(383,371),(390,388),(397,391),(394,377)]),LENS,(.65,.29,.075),offset=.024)
            fog=[(415,450),(400,457),(386,474),(382,494),(409,503),(417,490)]
            self.panel(reflect(fog),RUBBER,DARK,offset=.018)
            self.face_line(reflect([(414,451),(395,463),(384,483),(383,495),(409,501)]),METAL,CHROME,width=3,offset=.023)
            center=self.elevation(TracePoint(606+side*(402-606),486),offset=.03)
            b.disc(center,.028,METAL,CHROME)
            b.disc((center[0],center[1],center[2]+.002),.020,LENS,(.48,.54,.53))
        center=self.elevation(TracePoint(608,411),offset=.029)
        b.disc(center,.061,METAL,CHROME,ring=.049)
        badge(b,'P',(center[0],center[1],center[2]+.002),.009)
        rear=[(380,262),(422,254),(600,267),(778,254),(820,262),(836,335),
              (848,407),(838,486),(819,503),(381,503),(361,483),(351,408),(363,335)]
        self.panel(rear,PAINT,WHITE,rear=True,offset=0)
        self.face_line([(428,302),(433,340),(450,366),(477,377),(721,377),(749,365),(766,338),(772,302)],RUBBER,(.03,.032,.032),rear=True,width=2,offset=.007)
        self.panel([(402,443),(484,430),(504,408),(695,408),(719,430),(798,443),(827,476),(814,500),(386,500),(371,476)],RUBBER,(.018,.021,.023),rear=True,offset=.009)
        self.panel([(520,413),(680,413),(680,450),(520,450)],RUBBER,DARK,rear=True,offset=.017)
        self.panel([(523,416),(677,416),(677,446),(523,446)],RUBBER,(.93,.94,.92),rear=True,offset=.019)
        for side in [-1,1]:
            def reflect(outline):
                return [(600+side*(x-600),y) for x,y in outline]
            lamp=[(384,262),(427,268),(489,280),(482,296),(456,308),(414,314),(384,306),(378,289)]
            self.panel(reflect(lamp),RUBBER,DARK,True,.012)
            self.panel(reflect([(386,266),(426,272),(482,283),(476,293),(453,303),(413,308),(386,302),(382,289)]),BRAKE,(.64,.028,.04),True,.017)
            self.panel(reflect([(439,279),(476,285),(468,294),(441,300)]),REVERSE,(.67,.71,.71),True,.022)
            self.face_line(reflect([(384,293),(393,303),(423,305),(428,275)]),LENS,(.65,.68,.67),True,width=3,offset=.024)
            self.panel(reflect([(392,477),(410,475),(426,491),(384,491)]),BRAKE,(.56,.018,.024),True,.023)
        center=self.elevation(TracePoint(600,295),True,.016)
        b.disc(center,.053,METAL,CHROME,ring=.041,facing=-1)
        badge(b,'P',(center[0],center[1],center[2]-.002),.008,facing=-1)
        label=self.elevation(TracePoint(472,322),True,.014)
        badge(b,'PITSUNDA',label,.0045,facing=-1)
        label=self.elevation(TracePoint(742,321),True,.014)
        badge(b,'2',label,.006,facing=-1)
        b.disc((.42,-.345,-2.025),.034,METAL,CHROME,ring=.027,facing=-1)
        b.disc((.42,-.345,-2.022),.027,RUBBER,DARK,facing=-1)
        self.face_line([(495,245),(566,250),(653,267)],RUBBER,DARK,True,width=6,offset=.04)


def pitsunda():
    b=Mesh()
    trace=PitsundaTrace(b)
    trace.shell()
    trace.windows()
    trace.details()
    trace.faces()
    fit=WheelFit(rear_radius=.318,front_radius=.318,rear_width=.185,front_width=.185)
    return b,[trace.length(1255),trace.length(539)],fit,.735,'pitsunda'

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
    fit=radius if isinstance(radius,WheelFit) else WheelFit(rear_radius=radius,front_radius=radius)
    for i,(axle,side) in enumerate([(axles[0],-1),(axles[0],1),(axles[1],-1),(axles[1],1)]):
        wheel_radius=fit.rear_radius if i < 2 else fit.front_radius
        wheel_width=fit.rear_width if i < 2 else fit.front_width
        parts.append((['wheel_rl','wheel_rr','wheel_fl','wheel_fr'][i],wheel(wheel_radius,wheel_width,side,style),(side*track,wheel_radius-.48,axle)))
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
    parser.add_argument('car',choices=['agolf','mosquito','pitsunda'])
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    builder, folder = {'agolf': (agolf,'1991-agolf-gti'), 'mosquito': (mosquito,'1969-mosquito-ss'), 'pitsunda': (pitsunda,'2016-pitsunda-2')}[args.car]
    export(args.output or ROOT/f'data/cars/{folder}/model.glb',*builder())
