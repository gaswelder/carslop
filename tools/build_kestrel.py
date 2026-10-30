#!/usr/bin/env python3
"""Rebuild Kestrel's heritage coupe body. Metres; Y up, +Z forward.

Uses the shared standard-library mesh writer. Does not alter tuning or wheels.
"""
import math
from build_car_bodies import Mesh, ROOT, PAINT, BLACK, GLASS, CHROME, RED, LAMP, AMBER


class BodyMesh(Mesh):
    def line(self, points, radius, mat, segments=5):
        super().line(points, radius, mat, segments)


def build():
    m = BodyMesh()
    length, axle = 2.14, 1.30
    rubber = ((.045, .055, .06), (0., .12))
    seam = ((.055, .035, .027), (0., .16))
    alloy = ((.78, .80, .76), (0., .82))

    def width(z):
        return .985 - .18*(abs(z)/length)**6 + .018*math.exp(-((z+1.22)/.48)**2)

    def crown(x, z):
        t = abs(x)/width(z)
        # Raised wings meet a low bonnet, with no separate headlight pods.
        wing = .14*math.exp(-((z-1.98)/.40)**2) + .19*math.exp(-((z-1.30)/.75)**4) + .10*math.exp(-((z+1.30)/.66)**4)
        return .43-.10*(abs(z)/length)**5 + .045*(1-t*t) + wing*math.exp(-((t-.82)/.23)**2)

    def bottom(z):
        d = min(abs(z-axle), abs(z+axle))
        return max(-.245, -.02+math.sqrt(max(0, .445**2-d*d))) if d < .445 else -.245

    def side(s, z, v):
        y = bottom(z)+(crown(width(z), z)-bottom(z))*v
        return (s*(width(z)-.045*(1-v)**2+.012*math.sin(math.pi*v)), y, z)

    for s in (-1, 1):
        m.surface(lambda u, v: side(s, -length+2*length*u, v), 192, 6, PAINT, (s, 0, 0))
        for center in (-axle, axle):
            lip, inner = [], []
            for i in range(97):
                a = -.54+(math.pi+1.08)*i/96
                z, y = center+.445*math.cos(a), -.02+.445*math.sin(a)
                lip.append((s*(width(z)+.003), y, z))
                inner.append((s*(width(z)-.15), y-.006, z))
            m.line(lip, .012, PAINT)
            for i in range(96):
                m.quad([lip[i], lip[i+1], inner[i+1], inner[i]], rubber, (0, -1, 0))
        m.line([(s*(width(z)+.008), -.155, z) for z in [-.83+1.66*i/50 for i in range(51)]], .021, rubber)
        belt = [(s*(width(z)+.009), .27, z) for z in [-1.94+3.88*i/160 for i in range(161)]]
        for a, b in zip(belt, belt[1:]):
            if bottom(a[2]) < .25 and bottom(b[2]) < .25:
                m.line([a, b], .014, rubber)
        # A single long coupe door, its shutline shaped around the rear arch.
        outline = [(-.74, .43), (-.75, .22), (-.77, -.12), (.73, -.12), (.79, .22), (.79, .43)]
        m.line([(s*(width(z)+.012), y, z) for z, y in outline], .0045, seam)
        m.line([(s*(width(z)+.025), .335, z) for z in (-.49, -.32)], .016, CHROME)
        m.line([(s*(width(z)+.014), .314, z) for z in (-.49, -.32)], .010, rubber)

    m.surface(lambda u, v: ((2*v-1)*width(z := -length+2*length*u), crown((2*v-1)*width(z), z), z), 64, 24, PAINT, (0, 1, 0))
    m.box((-.78, -.245, -2.02), (.78, -.22, 2.02), rubber)
    for z, facing in ((length, 1), (-length, -1)):
        m.surface(lambda u, v: (x := (2*u-1)*width(z), -.245+(crown(x, z)+.245)*v, z), 24, 6, PAINT, (0, 0, facing))

    # Continuous curved roof and fastback; glass conforms to the same shell.
    profile = [(-1.70, .46), (-1.45, .59), (-1.18, .80), (-.88, .92), (-.58, 1.00), (-.22, 1.02), (.13, .995), (.37, .90), (.59, .72), (.84, .49)]
    def height(z):
        for (a, ya), (b, yb) in zip(profile, profile[1:]):
            if z <= b:
                i = profile.index((a, ya))
                ma = (yb-profile[max(0, i-1)][1])/(b-profile[max(0, i-1)][0])
                mb = (profile[min(len(profile)-1, i+2)][1]-ya)/(profile[min(len(profile)-1, i+2)][0]-a)
                t = max(0., (z-a)/(b-a))
                return (2*t**3-3*t*t+1)*ya+(t**3-2*t*t+t)*(b-a)*ma+(-2*t**3+3*t*t)*yb+(t**3-t*t)*(b-a)*mb
        return profile[-1][1]

    def cabwidth(z, y):
        return .825-.20*max(0, (y-.47)/.55)-.065*(abs(z)/1.7)**3

    def roofpoint(z, t):
        h = height(z)
        return (t*cabwidth(z, h), h-.045*t*t, z)

    m.surface(lambda u, v: roofpoint(-1.70+2.54*u, 2*v-1), 64, 24, PAINT, (0, 1, 0))
    for s in (-1, 1):
        def cabin(u, v):
            z = -1.70+2.54*u
            h = height(z)
            y = .435+(h-.045-.435)*v
            return (s*cabwidth(z, y+.045*v), y, z)
        m.surface(cabin, 64, 6, PAINT, (s, 0, 0))
        # Window upper edge follows the curved roof; quarter light and door glass.
        for a, b in ((-1.34, -.53), (-.47, .67)):
            def window(u, v):
                z = a+(b-a)*u
                low = .505
                high = max(low+.012, height(z)-.115)
                y = low+(high-low)*v
                return (s*(cabwidth(z, y+.035)+.009), y, z)
            m.surface(window, 20, 6, GLASS, (s, 0, 0))
            for fixed in (0, 1):
                m.line([window(i/48, fixed) for i in range(49)], .013, rubber)
                m.line([window(fixed, i/16) for i in range(17)], .013, rubber)
                m.line([tuple(p[j]+(s*.005 if j == 0 else 0) for j in range(3)) for p in [window(i/48, fixed) for i in range(49)]], .004, CHROME)
        # Slim vent-window divider at the A pillar.
        z = .43
        m.line([(s*(cabwidth(z, y+.035)+.020), y, z) for y in (.515, .64, height(z)-.12)], .009, CHROME)
        # Small rounded mirror, attached to the door rather than floating.
        m.line([(s*.82, .53, .57), (s*1.025, .57, .58)], .018, CHROME)
        def mirror(u, v):
            a, t = math.tau*u, math.pi*v
            return (s*(1.035+.115*math.sin(t)*math.cos(a)), .60+.065*math.cos(t), .57+.07*math.sin(t)*math.sin(a))
        m.surface(mirror, 16, 8, CHROME, (s, 0, 0))

    for a, b, facing in ((.24, .79, 1), (-1.54, -.88, -1)):
        def screen(u, v):
            z = a+(b-a)*u
            p = roofpoint(z, (2*v-1)*.89)
            return (p[0], p[1]+.009, p[2])
        m.surface(screen, 16, 16, GLASS, (0, 1, facing))
        for fixed in (0, 1):
            m.line([screen(fixed, i/48) for i in range(49)], .016, rubber)
            m.line([screen(i/48, fixed) for i in range(49)], .016, rubber)
    for s in (-1, 1):
        m.line([(s*.06, height(.74)+.025, .74), (s*.47, height(.67)+.023, .67)], .009, rubber)
        # Bonnet outline runs between the raised wings.
        m.line([(s*(.50+.045*math.sin(math.pi*i/60)), crown(.50+.045*math.sin(math.pi*i/60), z := .88+1.12*i/60)+.006, z) for i in range(61)], .004, seam)
    m.line([(x := -.50+i/40, crown(x, 2.0)+.006, 2.0) for i in range(41)], .004, seam)

    # Recessed round headlamps lean with the wings. Domed lenses and thin bezels.
    for s in (-1, 1):
        center = (s*.715, .445, 2.185)
        def lamp_point(r, a, depth):
            return (center[0]+r*math.cos(a), center[1]+r*math.sin(a)*.98+depth*.20, center[2]-r*math.sin(a)*.20+depth*.98)
        m.surface(lambda u, v: lamp_point(.168+.008*(1-v), math.tau*u, -.16*(1-v)), 32, 4, PAINT, (0, .20, .98))
        for radius, mat, depth in ((.168, PAINT, 0.), (.151, CHROME, .009), (.139, LAMP, .014)):
            def lens(u, v):
                r = radius*v
                return lamp_point(r, math.tau*u, depth+.022*(1-v*v))
            m.surface(lens, 40, 5, mat, (0, .20, .98))
        for offset in (-.08, -.04, 0, .04, .08):
            span = math.sqrt(.137**2-offset**2)
            m.line([lamp_point(math.hypot(offset, t), math.atan2(t, offset), .014+.022*(1-(math.hypot(offset, t)/.139)**2)+.002) for t in [-span+2*span*i/16 for i in range(17)]], .0015, alloy)

    # Full width bumpers, inset rubber strips, signal lamps and plate recesses.
    for facing in (-1, 1):
        def bumper(u, v):
            x = -.78+1.56*u
            return (x, -.095+.18*v, facing*(2.17-.16*(abs(x)/.78)**3))
        m.surface(bumper, 32, 4, CHROME, (0, 0, facing))
        m.line([(x := -.77+1.54*i/80, -.015, facing*(2.184-.16*(abs(x)/.78)**3)) for i in range(81)], .033, rubber)
        z = facing*2.19
        m.box((-.23, -.165, z-.012), (.23, -.035, z+.012), rubber)
        m.box((-.19, -.15, z-.014), (.19, -.055, z+.014), ((.83, .84, .77), (0., .18)))
        # Plate characters are simple stamped dark strokes, not a blank rectangle.
        for i in range(7):
            x = -.145+i*.045
            m.line([(x, -.13, z+facing*.017), (x, -.077, z+facing*.017), (x+.022, -.077, z+facing*.017)], .004, rubber)
        for s in (-1, 1):
            zz = facing*2.145
            x = s*.64
            m.box((x-.17, .105, zz-.012), (x+.17, .225, zz+.012), rubber)
            m.box((x-.15, .12, zz-.016), (x+.15, .21, zz+.016), AMBER if facing == 1 else RED)
            if facing == -1:
                m.box((x-.045, .12, zz-.019), (x+.025, .21, zz+.019), ((.78, .77, .64), (0., .55)))
            for j in range(1, 10):
                xx = x-.15+j*.03
                m.line([(xx, .125, zz+facing*.019), (xx, .205, zz+facing*.019)], .0018, alloy)
    # Rear engine cover: panel perimeter, recessed cooling slats and twin outlets.
    for s in (-1, 1):
        m.line([(s*.51, crown(.51, z := -1.98+.40*i/40)+.007, z) for i in range(41)], .004, seam)
    for i in range(12):
        z = -1.94+i*.029
        m.line([(x := -.45+.9*j/40, crown(x, z)+.012, z) for j in range(41)], .009, rubber)
    for x in (-.55, -.39):
        m.line([(x, -.19, -2.08), (x, -.19, -2.24)], .041, alloy, 20)
        m.box((x-.029, -.215, -2.244), (x+.029, -.165, -2.24), rubber)
    # Keep the authored body inside the existing car footprint.
    vertices = []
    for vertex in m.vertices:
        p = list(vertex)
        p[2] *= .97
        p[8] /= .97
        norm = math.sqrt(sum(n*n for n in p[6:9]))
        p[6:9] = [n/norm for n in p[6:9]]
        vertices.append(tuple(p))
    m.vertices = vertices
    m.save(ROOT / 'kestrel' / 'body.mesh')
    print(f'kestrel: {len(m.vertices)//3:,} triangles')


if __name__ == '__main__':
    build()
