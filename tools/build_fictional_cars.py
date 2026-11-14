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
                    return (center[0]+r*math.cos(t), center[1]+r*math.sin(t), center[2])
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


@dataclass(frozen=True)
class TracePoint:
    x: float
    y: float


@dataclass(frozen=True)
class WheelFit:
    rear_radius: float
    front_radius: float


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

    def shell(self):
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
                q=v*2-1
                return (q*self.width(x), self.height(self.at(self.top,x))+.022*(1-q*q), self.length(x))
            b.patch(max(1,int((right.x-left.x)/4)),24,deck,PAINT,WHITE)
        for left,right in zip(self.roof,self.roof[1:]):
            def roof(u,v):
                x=left.x+(right.x-left.x)*u
                q=v*2-1
                # The centre of the roof reaches the traced highest point.
                return (q*.715,self.height(self.at(self.roof,x))-.018*q*q,self.length(x))
            b.patch(max(1,int((right.x-left.x)/3)),24,roof,PAINT,WHITE)
        for x in [269,1512]:
            b.quad([self.side(-1,TracePoint(x,self.at(self.lower,x))),
                    self.side(1,TracePoint(x,self.at(self.lower,x))),
                    self.side(1,TracePoint(x,self.at(self.top,x))),
                    self.side(-1,TracePoint(x,self.at(self.top,x)))][::1 if x==269 else -1],PAINT,WHITE)

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
        for start,end in [(706,803),(1182,1278)]:
            def glass(u,v):
                x=start+(end-start)*u
                q=(v*2-1)*.88
                return (q*.715,self.height(self.at(self.roof,x))-.018*q*q+.009,self.length(x))
            self.mesh.patch(end-start,32,glass,GLASS,(.12,.17,.20))


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
    rear=trace.length(1512)
    b.box((-.78,trace.height(410),front),(.78,trace.height(355),front+.004),RUBBER,DARK)
    for side in [-1,1]:
        b.disc((side*.64,trace.height(382),front+.009),.094,METAL,CHROME)
        b.disc((side*.64,trace.height(382),front+.014),.082,LENS,(.72,.77,.74))
    b.box((-.80,trace.height(429),front-.015),(.80,trace.height(411),front+.008),METAL,CHROME)
    b.box((-.84,trace.height(382),rear-.048),(.84,trace.height(359),rear+.012),METAL,CHROME)
    tail_pair(b,rear-.014,.73,.36,trace.height(346),trace.height(321))
    badge(b,'SS',(0,trace.height(340),rear-.026),.012)
    # Rear valance trim follows the short upswept tail visible in the reference.
    b.box((-.79,trace.height(443),trace.length(1482)),
          (.79,trace.height(435),trace.length(1452)),METAL,CHROME)
    # Match the slightly taller rear tire in the supplied side elevation.
    fit=WheelFit(rear_radius=.334,front_radius=.306)
    axles=[trace.length(1248),trace.length(491)]
    return b,axles,fit,.79,'wire'


def pitsunda():
    b = Mesh()
    stations=[(-2.03,.68,.30,.33),(-1.70,.80,.38,.44),(-1.27,.85,.36,.49),
              (-.65,.81,.32,.40),(.45,.80,.27,.37),(1.30,.85,.23,.43),
              (1.78,.78,.17,.30),(2.03,.57,.15,.23)]
    axles=[-1.27,1.30]
    cabin=[(-1.82,.67,.42),(-1.00,.68,1.02),(-.15,.65,1.03),(.28,.65,.94),(.90,.70,.30)]
    shell=Coachwork(b,stations,axles,.295,cabin)
    shell.skin(rub_strip=False,doors=5)
    # Swept lamps, chrome wings and a deep shield grille distinguish the nose.
    grille=[(-.43,.20,2.034),(.43,.20,2.034),(.39,-.02,2.055),(.20,-.22,2.065),(-.20,-.22,2.065),(-.39,-.02,2.055)]
    center=(0,-.01,2.07)
    for i in range(len(grille)):
        b.triangle([center,grille[i],grille[(i+1)%len(grille)]],METAL,CHROME)
    inner=[(x*.94,y*.92,z+.005) for x,y,z in grille]
    for i in range(len(inner)):
        b.triangle([(0,-.01,2.077),inner[i],inner[(i+1)%len(inner)]],RUBBER,DARK)
    for i in range(9):
        y=-.17+i*.039
        w=.23+(y+.17)*.48
        b.box((-w,y,2.078),(w,y+.004,2.082),RUBBER,(.065,.075,.08))
    b.disc((0,.07,2.086),.056,METAL,CHROME,ring=.045)
    badge(b,'P',(0,.07,2.089),.009)
    for side in [-1,1]:
        outline=[]
        for x,z in [(side*.43,2.055),(side*.76,1.90),(side*.78,1.74),(side*.49,1.96)]:
            y=shell.deck(x/profile(stations,z)[0],z)[1]+.012
            outline.append((x,y,z))
        b.quad(outline,METAL,CHROME)
        c=tuple(sum(p[k] for p in outline)/4 for k in range(3))
        light=[tuple(c[k]+(p[k]-c[k])*.91+(0.005 if k==2 else 0) for k in range(3)) for p in outline]
        b.quad(light,LENS,(.65,.73,.77))
        b.box((side*.52-.075,-.22,2.041),(side*.52+.075,-.10,2.057),RUBBER,DARK)
        b.disc((side*.52,-.158,2.065),.034,LENS,(.74,.78,.74))
        # A narrow painted crease follows the side surface rather than a rub strip.
        def crease(u,v):
            z=-1.15+2.35*u
            w,_,top=profile(stations,z)
            y=.18+.06*u+v*.006
            fraction=(y+.31)/(top+.31)
            x=side*(w-.045*(1-fraction)**2+.003)
            return (x,y,z)
        b.patch(64,1,crease,PAINT,(.97,.97,.97))
        # High wraparound rear lamps and an inset clear reversing section.
        rear=[(side*.45,.33,-2.05),(side*.78,.40,-1.83),(side*.80,.28,-1.90),(side*.50,.25,-2.055)]
        b.quad(rear,BRAKE,(.76,.025,.025))
        x=side*.56
        b.box((x-.045,.31,-2.062),(x+.045,.345,-2.055),REVERSE,(.45,.48,.48))
    b.box((-.60,-.24,-2.055),(.60,-.16,-2.02),RUBBER,DARK)
    b.box((-.20,-.12,-2.078),(.20,-.025,-2.055),METAL,(.67,.68,.62))
    badge(b,'PITSUNDA',(0,-.075,-2.081),.0067)
    badge(b,'2',(.40,.20,-2.056),.010)
    # Rear hatch spoiler and a separate high-level stop strip.
    b.box((-.59,.90,-1.30),(.59,.95,-1.14),PAINT,WHITE)
    b.box((-.26,.901,-1.307),(.26,.914,-1.302),BRAKE,(.65,.025,.025))
    return b,axles,.295,.735,'modern'


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
        parts.append((['wheel_rl','wheel_rr','wheel_fl','wheel_fr'][i],wheel(wheel_radius,.18,side,style),(side*track,wheel_radius-.48,axle)))
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
