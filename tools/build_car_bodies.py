#!/usr/bin/env python3
"""Rebuild the Muscle and Club NFSCAR01 artwork (standard library only).

Run from any directory. Tuning, wheel assets and Kestrel artwork are untouched.
Coordinates are metres, Y up, Z forward; paint is selected at runtime.
"""
import math
from pathlib import Path
import struct

ROOT = Path(__file__).resolve().parents[1] / 'data/cars'
PAINT = ((.92, .92, .92), (1., .64))
BLACK = ((.025, .031, .037), (0., .18))
GLASS = ((.075, .14, .19), (0., .92))
CHROME = ((.57, .62, .65), (0., .95))
RED = ((.78, .02, .015), (0., .55))
LAMP = ((1., .94, .72), (0., .7))
AMBER = ((1., .32, .035), (0., .55))


def sub(a, b):
    return tuple(x-y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def unit(a):
    length = math.sqrt(sum(x*x for x in a))
    return tuple(x / length for x in a) if length > 1e-12 else (0., 1., 0.)


class Mesh:
    def __init__(self):
        self.vertices = []

    def tri(self, pts, material, outward, normals=None):
        n = cross(sub(pts[1], pts[0]), sub(pts[2], pts[0]))
        if sum(x*x for x in n) < 1e-18:
            return
        if sum(x*y for x, y in zip(n, outward)) < 0:
            pts = [pts[0], pts[2], pts[1]]
            normals = [normals[0], normals[2], normals[1]] if normals else None
            n = tuple(-x for x in n)
        n = unit(n)
        for i, p in enumerate(pts):
            self.vertices.append((*p, *material[0], *(normals[i] if normals else n), *material[1]))

    def quad(self, pts, mat, outward):
        for ids in ((0, 1, 2), (0, 2, 3)):
            self.tri([pts[i] for i in ids], mat, outward)

    def surface(self, fn, nu, nv, mat, outward):
        def normal(u, v):
            e = .0001
            n = unit(cross(sub(fn(min(1, u+e), v), fn(max(0, u-e), v)),
                           sub(fn(u, min(1, v+e)), fn(u, max(0, v-e)))))
            return n if sum(a*b for a, b in zip(n, outward)) >= 0 else tuple(-x for x in n)
        grid = [[(fn(i/nu, j/nv), normal(i/nu, j/nv)) for j in range(nv+1)] for i in range(nu+1)]
        for i in range(nu):
            for j in range(nv):
                p = [grid[i][j], grid[i+1][j], grid[i+1][j+1], grid[i][j+1]]
                for ids in ((0, 1, 2), (0, 2, 3)):
                    self.tri([p[k][0] for k in ids], mat, outward, [p[k][1] for k in ids])

    def box(self, lo, hi, mat):
        for axis in range(3):
            a, b = (axis+1) % 3, (axis+2) % 3
            for side in (0, 1):
                pts = []
                for u, v in ((0, 0), (1, 0), (1, 1), (0, 1)):
                    p = list(lo)
                    p[axis] = (lo, hi)[side][axis]
                    p[a], p[b] = (lo, hi)[u][a], (lo, hi)[v][b]
                    pts.append(p)
                n = [0, 0, 0]
                n[axis] = side*2-1
                self.quad(pts, mat, n)

    def line(self, points, radius, mat, segments=8):
        for a, b in zip(points, points[1:]):
            direction = unit(sub(b, a))
            u = unit(cross(direction, (0, 1, 0) if abs(direction[1]) < .9 else (1, 0, 0)))
            v = cross(direction, u)
            for k in range(segments):
                angles = [k*math.tau/segments, (k+1)*math.tau/segments]
                offsets = [tuple(radius*(math.cos(t)*u[i]+math.sin(t)*v[i]) for i in range(3)) for t in angles]
                pts = [tuple(p[i]+o[i] for i in range(3)) for p, o in ((a, offsets[0]), (b, offsets[0]), (b, offsets[1]), (a, offsets[1]))]
                self.quad(pts, mat, unit(offsets[0]))

    def save(self, path):
        path.write_bytes(b'NFSCAR01'+struct.pack('<I', len(self.vertices))+
                         b''.join(struct.pack('<11f', *v) for v in self.vertices))


def build(kind):
    muscle = kind == 'muscle'
    length, width, axle = (2.16, 1.13, 1.45) if muscle else (1.77, 1.0, 1.15)
    roof = 1.04 if muscle else 1.24
    rear, rear_roof, front_roof, front = (-1.16, -.59, .28, .96) if muscle else (-1.48, -1.13, .28, .83)
    m = Mesh()

    def w(z):
        # Tapered ends, a subtle waist, and full shoulders over each axle.
        end = max(0, (abs(z)/length-.76)/.24)
        return width-.10*end*end-.035*math.exp(-(z/.7)**2)

    def shoulder(z):
        return .41 + (.055 if muscle else .025)*math.exp(-((z+axle)/.55)**2) - .055*(abs(z)/length)**6

    def bottom(z):
        d = min(abs(z-axle), abs(z+axle))
        return max(-.23, -.02+math.sqrt(max(0, .455**2-d*d))) if d < .455 else -.23

    def side(s, z, v):
        low, high = bottom(z), shoulder(z)+.055
        y = low+(high-low)*v
        x = w(z)-.045*(1-v)**2+.022*math.sin(math.pi*v)
        return (s*x, y, z)

    for s in (-1, 1):
        m.surface(lambda u, v: side(s, -length+2*length*u, v), 192, 8, PAINT, (s, 0, 0))
        # Rolled metal arch lips and dark inner wheel wells, not painted-on circles.
        for center in (-axle, axle):
            points = []
            inner = []
            for i in range(81):
                t = -.48+(math.pi+.96)*i/80
                z = center+.455*math.cos(t)
                y = -.02+.455*math.sin(t)
                points.append((s*(w(z)+.009), y, z))
                inner.append((s*(w(z)-.09), y-.008, z))
            m.line(points, .018, PAINT)
            for i in range(80):
                m.quad([points[i], points[i+1], inner[i+1], inner[i]], BLACK, (0, -1, 0))
        # Sill, belt trim and door shutlines follow the actual curved body.
        m.line([(s*(w(z)+.008), shoulder(z)+.035, z) for z in [-length+2*length*i/120 for i in range(121)]], .012, CHROME if muscle else BLACK)
        m.line([(s*(w(z)-.018), -.18, z) for z in [-axle+.48+(2*axle-.96)*i/40 for i in range(41)]], .026, BLACK)
        for z in ((-.73, .74) if muscle else (-.67, .67)):
            m.line([side(s, z, v) for v in (.05, .25, .5, .75, .98)], .006, BLACK)
        z = -.48
        m.box((s*w(z)-.045, .29, z-.105), (s*w(z)+.045, .32, z+.105), CHROME)

    # Crown the bonnet/deck; the side tangent rolls gently into the fenders.
    def top(x, z):
        return shoulder(z)+.055+.045*(1-(x/w(z))**2)

    m.surface(lambda u, v: ((2*v-1)*w(-length+2*length*u), top((2*v-1)*w(-length+2*length*u), -length+2*length*u), -length+2*length*u), 96, 24, PAINT, (0, 1, 0))
    m.box((-.75, -.225, -length+.06), (.75, -.20, length-.06), BLACK)
    for z, facing in ((length, 1), (-length, -1)):
        m.surface(lambda u, v, z=z: ((2*u-1)*w(z), -.23+v*(shoulder(z)+.285), z), 32, 12, PAINT, (0, 0, facing))

    # Cabin is a separate, crowned loft; steep hatch versus long fastback.
    def cabin_width(z):
        return w(z)-.15

    def cabin_y(z):
        if z < rear_roof:
            return .50+(roof-.50)*(z-rear)/(rear_roof-rear)
        if z > front_roof:
            return roof-(roof-.50)*(z-front_roof)/(front-front_roof)
        return roof

    def cabin_point(z, t):
        y = cabin_y(z)
        return (t*(cabin_width(z)-.18*(y-.50)/(roof-.50)), y-.035*t*t, z)

    for a, b in ((rear, rear_roof), (rear_roof, front_roof), (front_roof, front)):
        m.surface(lambda u, v, a=a, b=b: cabin_point(a+(b-a)*u, 2*v-1), 16, 24, PAINT, (0, 1, 0))
    for s in (-1, 1):
        m.surface(lambda u, v: (s*(cabin_width(rear+(front-rear)*u)-.18*v*(cabin_y(rear+(front-rear)*u)-.5)/(roof-.5)), .49+(cabin_y(rear+(front-rear)*u)-.035-.49)*v, rear+(front-rear)*u), 64, 8, PAINT, (s, 0, 0))
        # Two glass panes with a substantial B pillar and rubber/chrome seals.
        panes = [[(rear+.17, .55), (rear_roof+.08, roof-.12), (-.32, roof-.12), (-.32, .55)],
                 [(-.24, .55), (-.24, roof-.12), (front_roof-.035, roof-.12), (front-.12, .55)]]
        for pane in panes:
            def glass_point(z, y):
                height = cabin_y(z)
                v = (y-.49)/(height-.035-.49)
                return (s*(cabin_width(z)-.18*v*(height-.5)/(roof-.5)+.012), y, z)
            def window(u, v):
                left = tuple(pane[0][i]*(1-v)+pane[1][i]*v for i in range(2))
                right = tuple(pane[3][i]*(1-v)+pane[2][i]*v for i in range(2))
                return glass_point(*(left[i]*(1-u)+right[i]*u for i in range(2)))
            pts = [glass_point(z, y) for z, y in pane]
            m.surface(window, 16, 8, GLASS, (s, 0, 0))
            m.line(pts+[pts[0]], .017, BLACK)
            if muscle:
                m.line(pts+[pts[0]], .006, CHROME)
        # Mirror stalk, housing, and reflective rear face.
        z = .64
        x = s*(w(z)-(.03 if muscle else .15))
        m.line([(x, .53, z), (x+s*.12, .60, z)], .018, BLACK)
        xx = x+s*.14
        m.box((xx-.085, .57, z-.08), (xx+.085, .67, z+.08), PAINT)
        m.box((xx-.071, .582, z-.083), (xx+.071, .656, z-.081), CHROME)

    # Inset front and rear screens, curved to match the cabin crown.
    for a, b, facing in ((front_roof, front, 1), (rear, rear_roof, -1)):
        def screen(u, v, a=a, b=b):
            z = a+(b-a)*(.10+.80*u)
            p = cabin_point(z, (2*v-1)*.88)
            return (p[0], p[1]+.009, p[2])
        m.surface(screen, 16, 20, GLASS, (0, 1, facing))
        for fixed in (0, 1):
            m.line([screen(fixed, i/32) for i in range(33)], .017, BLACK)
            m.line([screen(i/32, fixed) for i in range(33)], .017, BLACK)
    for s in (-1, 1):
        z = front-.10
        m.line([(s*.11, cabin_y(z)+.016, z), (s*.55, cabin_y(z)+.007, z-.045)], .009, BLACK)

    # Bonnet seams and cowl vents.
    for s in (-1, 1):
        m.line([(s*.67, top(s*.67, z)+.006, z) for z in [front+.04+(length-front-.16)*i/60 for i in range(61)]], .005, BLACK)
    for i in range(15):
        x = -.52+i*.074
        m.line([(x, top(x, front+.03)+.008, front+.03), (x, top(x, front+.12)+.008, front+.12)], .008, BLACK)

    # Recessed fascia and individually modelled lenses, grille bars and bumpers.
    for facing in (-1, 1):
        z = facing*(length-.015)
        m.box((-.84 if muscle else -.76, .015, z-.014), (.84 if muscle else .76, .285, z+.014), BLACK)
        m.box((-w(z)+.04, -.13, z-.035), (w(z)-.04, -.045, z+.035), CHROME if muscle else BLACK)
        m.box((-.19, -.12, z+facing*.037-.006), (.19, -.005, z+facing*.037+.006), ((.76, .78, .72), (0, .2)))
        if facing == 1:
            for i in range(9):
                x = -.42+i*.105
                m.box((x-.009, .035, z-.025), (x+.009, .255, z+.026), CHROME if muscle else BLACK)
            for y in (.075, .14, .205):
                m.box((-.44, y-.005, z+.025), (.44, y+.005, z+.035), CHROME)
        for s in (-1, 1):
            if muscle and facing == 1:
                for x in (s*.60, s*.85):
                    for radius, mat, depth in ((.115, CHROME, .033), (.093, LAMP, .038)):
                        for i in range(48):
                            a, b = i*math.tau/48, (i+1)*math.tau/48
                            m.tri([(x, .165, z+depth), (x+radius*math.cos(a), .165+radius*math.sin(a), z+depth), (x+radius*math.cos(b), .165+radius*math.sin(b), z+depth)], mat, (0, 0, 1))
            else:
                x = s*(.69 if muscle else .61)
                m.box((x-.19, .055, z-.026), (x+.19, .255, z+.026), CHROME if muscle else BLACK)
                m.box((x-.17, .073, z-.031), (x+.17, .237, z+.031), LAMP if facing == 1 else RED)
                for i in range(1, 7):
                    xx = x-.17+i*.048
                    m.line([(xx, .078, z+facing*.033), (xx, .23, z+facing*.033)], .0025, CHROME if facing == 1 else RED)
                m.box((x+.105, .077, z-.034), (x+.164, .233, z+.034), AMBER)
    # Exhaust outlets and rear deck details distinguish the two designs.
    for x in ((-.73, .73) if muscle else (-.58,)):
        m.box((x-.065, -.215, -length-.025), (x+.065, -.15, -length+.11), CHROME)
        m.box((x-.047, -.203, -length-.028), (x+.047, -.163, -length-.026), BLACK)
    if muscle:
        # Twin raised bonnet scoops with open dark mouths.
        for x in (-.30, .30):
            m.box((x-.11, .493, 1.10), (x+.11, .54, 1.65), PAINT)
            m.box((x-.095, .499, 1.651), (x+.095, .532, 1.656), BLACK)
        m.line([(-.87, .48, -1.92), (.87, .48, -1.92)], .028, PAINT)
    else:
        m.box((-.77, 1.13, -1.25), (.77, 1.18, -1.09), BLACK)
        m.line([(.02, .68, -1.44), (.36, .78, -1.39)], .011, BLACK)
    m.save(ROOT / kind / 'body.mesh')
    print(f'{kind}: {len(m.vertices)//3:,} triangles')


if __name__ == '__main__':
    for car in ('muscle', 'club'):
        build(car)
