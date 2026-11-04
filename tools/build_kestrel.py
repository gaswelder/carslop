#!/usr/bin/env python3
"""Author both Kestrel coupes and their wheels. Standard library only.

Metres, Y up, +Z forward. Wheel mounts/radius and all driving tunes are preserved.
Run from any directory; all generated artwork is original procedural geometry.
"""
import math

from build_car_bodies import Mesh, ROOT, sub, unit, cross


PAINT = ((.94, .94, .94), (1., .82))
RUBBER = ((.045, .055, .06), (0., .08))
BLACK = ((.018, .023, .027), (0., .12))
SEAM = ((.035, .027, .023), (0., .04))
GLASS = ((.055, .095, .115), (2., .96))
CHROME = ((.78, .80, .76), (3., .94))
ALLOY = ((.56, .59, .61), (3., .72))
LAMP = ((.84, .88, .90), (4., .80))
RED = ((.78, .02, .015), (4., .68))
AMBER = ((1., .27, .018), (4., .65))


def lerp(a, b, t):
    return a + (b-a)*t


def curve(knots, x):
    """Shape-preserving cubic Hermite interpolation, including flat extrema."""
    if x <= knots[0][0]:
        return knots[0][1]
    if x >= knots[-1][0]:
        return knots[-1][1]
    for i, ((a, ya), (b, yb)) in enumerate(zip(knots, knots[1:])):
        if x <= b:
            d = (yb-ya)/(b-a)

            def tangent(j):
                if j == 0:
                    return (knots[1][1]-knots[0][1])/(knots[1][0]-knots[0][0])
                if j == len(knots)-1:
                    return (knots[-1][1]-knots[-2][1])/(knots[-1][0]-knots[-2][0])
                h0, h1 = knots[j][0]-knots[j-1][0], knots[j+1][0]-knots[j][0]
                d0 = (knots[j][1]-knots[j-1][1])/h0
                d1 = (knots[j+1][1]-knots[j][1])/h1
                if d0*d1 <= 0:
                    return 0.
                return 3*(h0+h1)/((2*h1+h0)/d0+(h1+2*h0)/d1)

            t = (x-a)/(b-a)
            return ((2*t**3-3*t*t+1)*ya + (t**3-2*t*t+t)*(b-a)*tangent(i)
                    + (-2*t**3+3*t*t)*yb + (t**3-t*t)*(b-a)*tangent(i+1))
    raise AssertionError(x)


class BodyMesh(Mesh):
    def surface(self, fn, nu, nv, mat, outward):
        # Orient the parametric patch once. Reorienting each vertex against a
        # fixed vector flips normals on concave fenders and produces bright
        # streaks where the bonnet rolls up into a wing.
        def normal(u, v):
            e = .0001
            return unit(cross(sub(fn(min(1., u+e), v), fn(max(0., u-e), v)),
                              sub(fn(u, min(1., v+e)), fn(u, max(0., v-e)))))
        flip = -1 if sum(a*b for a, b in zip(normal(.5, .5), outward)) < 0 else 1
        grid = [[(fn(i/nu, j/nv), tuple(flip*n for n in normal(i/nu, j/nv)))
                 for j in range(nv+1)] for i in range(nu+1)]
        for i in range(nu):
            for j in range(nv):
                p = [grid[i][j], grid[i+1][j], grid[i+1][j+1], grid[i][j+1]]
                for ids in ((0, 1, 2), (0, 2, 3)):
                    normals = [p[k][1] for k in ids]
                    self.tri([p[k][0] for k in ids], mat,
                             tuple(sum(n[a] for n in normals) for a in range(3)), normals)

    def line(self, points, radius, mat, segments=4):
        # Sub-millimetre chord tolerance removes redundant samples on trim,
        # while preserving corners and the silhouette of small circular details.
        def simplify(path):
            if len(path) < 3:
                return path
            axis = sub(path[-1], path[0])
            length2 = sum(v*v for v in axis)
            best, split = 0., 0
            for i, p in enumerate(path[1:-1], 1):
                t = max(0., min(1., sum(a*b for a, b in zip(sub(p, path[0]), axis))/length2)) if length2 > 1e-15 else 0.
                error = sum((p[j]-path[0][j]-axis[j]*t)**2 for j in range(3))
                if error > best:
                    best, split = error, i
            if best < .00035**2:
                return [path[0], path[-1]]
            return simplify(path[:split+1])[:-1] + simplify(path[split:])
        super().line(simplify(points), radius, mat, segments)

    def ellipsoid(self, center, scale, mat, nu=24, nv=12):
        # Split hemispheres to keep the mesh writer's outward direction valid.
        for s in (-1, 1):
            self.surface(lambda u, v: (
                center[0]+scale[0]*math.sin(math.pi*v)*math.cos(math.pi*u),
                center[1]+scale[1]*math.cos(math.pi*v),
                center[2]+s*scale[2]*math.sin(math.pi*v)*math.sin(math.pi*u)),
                nu//2, nv, mat, (0, 0, s))


class Coachwork:
    length = 2.085
    axle = 1.30
    arch_width = .475
    sill = -.245

    def __init__(self, turbo):
        self.turbo = turbo

    def width(self, z):
        w = curve([(-2.085, .79), (-1.73, .955), (-1.30, .995), (-.65, .934),
                   (.25, .924), (1.30, .978), (1.78, .937), (2.085, .83)], z)
        if self.turbo:
            w += .105*math.exp(-((z+1.30)/.72)**4) + .075*math.exp(-((z-1.30)/.68)**4)
        return w

    def deck(self, z):
        return curve([(-2.085, .345), (-1.65, .48), (-.6, .475), (.60, .475),
                      (.92, .44), (1.55, .355), (1.94, .285), (2.085, .25)], z)

    def wing(self, z):
        return (.085*math.exp(-((z+1.30)/.62)**4)
                + .245*math.exp(-((z-1.63)/.80)**4))

    def section(self, z, t):
        """One continuous crown, rolled shoulder, flank and tucked-under sill."""
        w, h, f = self.width(z), self.deck(z), self.wing(z)
        patches = [
            (0., .38, [(0., h+.025), (.16*w, h+.025), (.35*w, h+.005), (.48*w, h+.012)]),
            (.38, .60, [(.48*w, h+.012), (.60*w, h+.019), (.67*w, h+f), (.79*w, h+f)]),
            (.60, .85, [(.79*w, h+f), (.95*w, h+f), (w, .33), (w, .10)]),
            (.85, 1., [(w, .10), (w, -.02), (w-.04, self.sill), (w-.082, self.sill)]),
        ]
        for a, b, points in patches:
            if t <= b:
                u = (t-a)/(b-a)
                weights = [(1-u)**3, 3*u*(1-u)**2, 3*u*u*(1-u), u**3]
                return (*[sum(p[i]*k for p, k in zip(points, weights)) for i in range(2)], z)
        raise AssertionError(t)

    def at_height(self, z, y):
        lo, hi = .60, 1.
        for _ in range(24):
            t = (lo+hi)*.5
            if self.section(z, t)[1] > y:
                lo = t
            else:
                hi = t
        return (lo+hi)*.5

    def edge(self, z):
        d = min(abs(z-self.axle), abs(z+self.axle))
        if d >= self.arch_width:
            return 1.
        y = self.sill + .685*math.sqrt(max(0., 1-(d/self.arch_width)**2))
        return self.at_height(z, y)

    def side(self, s, z, y, offset=0.):
        p = self.section(z, self.at_height(z, y))
        return s*(p[0]+offset), y, self.end_z(p[0], z)

    def end_z(self, x, z):
        t = max(0., (abs(z)-1.70)/(self.length-1.70))
        return math.copysign(abs(z)-.11*(abs(x)/self.width(z))**3*t*t, z)

    def top(self, x, z):
        lo, hi = 0., .68
        for _ in range(24):
            t = (lo+hi)*.5
            if self.section(z, t)[0] < abs(x):
                lo = t
            else:
                hi = t
        return x, self.section(z, (lo+hi)*.5)[1], self.end_z(x, z)


def build_body(turbo=False):
    m, body = BodyMesh(), Coachwork(turbo)
    length = body.length
    for s in (-1, 1):
        def skin(u, v):
            z = -length+2*length*u
            x, y, _ = body.section(z, v*body.edge(z))
            return s*x, y, body.end_z(x, z)
        m.surface(skin, 160, 24, PAINT, (s, 1, 0))

        # Folded arch lips follow the exact skin cut; wells have depth and a back.
        for axle in (-body.axle, body.axle):
            lip, inside = [], []
            for i in range(81):
                a = math.pi*i/80
                z = axle+body.arch_width*math.cos(a)
                y = body.sill+.685*math.sin(a)
                p = body.side(s, z, y, .002)
                lip.append(p)
                inside.append((s*(abs(p[0])-.17), y-.012, z))
            m.line(lip, .008 if not turbo else .012, PAINT)
            for i in range(80):
                m.quad([lip[i], inside[i], inside[i+1], lip[i+1]], RUBBER, (0, -1, 0))
            for i in range(64):
                a, b = i*math.pi/64, (i+1)*math.pi/64
                m.tri([(s*.66, -.18, axle),
                       (s*.66, -.18+.64*math.sin(a), axle+.465*math.cos(a)),
                       (s*.66, -.18+.64*math.sin(b), axle+.465*math.cos(b))], BLACK, (s, 0, 0))

        m.line([body.side(s, z, -.18, .007) for z in [-.81+1.62*i/60 for i in range(61)]],
               .018, RUBBER)
        m.line([body.side(s, z, -.217, .008) for z in [-.81+1.62*i/60 for i in range(61)]],
               .005, CHROME if not turbo else BLACK)

        # Belt trim stops cleanly at each wheel opening.
        belt = [body.side(s, z, .205, .006) for z in [-1.97+3.94*i/220 for i in range(221)]]
        for a, b in zip(belt, belt[1:]):
            if all(body.section(p[2], body.edge(p[2]))[1] < .19 for p in (a, b)):
                m.line([a, b], .013 if turbo else .008, RUBBER)
        door = [(-.72, .463), (-.725, .32), (-.74, .08), (-.70, -.13),
                (-.59, -.16), (.62, -.16), (.73, -.10), (.75, .20), (.70, .46)]
        path = []
        for (za, ya), (zb, yb) in zip(door, door[1:]):
            path += [body.side(s, lerp(za, zb, i/10), lerp(ya, yb, i/10), .003) for i in range(10)]
        path.append(body.side(s, *door[-1], .003))
        m.line(path, .0028, SEAM)
        m.line([body.side(s, z, .368, .009) for z in (-.55, -.38)], .016, BLACK)
        m.line([body.side(s, z, .378, .026) for z in (-.55, -.38)], .010, CHROME)
        m.ellipsoid(body.side(s, -.59, .366, .012), (.007, .010, .010), CHROME, 16, 8)

    m.box((-.69, -.248, -1.91), (.69, -.229, 1.91), BLACK)

    def fascia(x, y, facing, offset=0.):
        w = body.width(facing*length)
        z = facing*(length+.008-.11*(abs(x)/w)**3 + offset)
        return x, y, z

    for facing in (-1, 1):
        # Rounded nose/tail closures, tucked beneath the rolled-over upper panels.
        for s in (-1, 1):
            def cap(u, v):
                x, high, _ = body.section(facing*length, u)
                x *= s*v
                y = lerp(body.sill, high, v)
                return x, y, body.end_z(x, facing*length)
            m.surface(cap, 32, 6, PAINT, (0, 0, facing))

        def bumper(u, v):
            a = math.pi*v
            x = lerp(-.875, .875, u)
            y = .033+.076*math.cos(a)
            return fascia(x, y, facing, .017+.028*math.sin(a))
        m.surface(bumper, 48, 8, PAINT if turbo else CHROME, (0, 0, facing))
        m.line([fascia(x, .034, facing, .052) for x in [-.875+1.75*i/100 for i in range(101)]],
               .031 if turbo else .023, RUBBER, 10)
        if turbo:
            for s in (-1, 1):
                for j in range(6):
                    x = s*(.765+j*.022)
                    m.line([fascia(x, y, facing, .053) for y in (-.024, .09)], .006, BLACK)
        else:
            for s in (-1, 1):
                x = s*.39
                m.ellipsoid(fascia(x, .025, facing, .045), (.032, .10, .035), CHROME, 24, 12)
                m.ellipsoid(fascia(x, .025, facing, .071), (.022, .083, .019), RUBBER, 20, 10)

        # Recess, ivory plate, raised rim and a legible fictional registration.
        py = -.139 if facing == 1 else .172
        pz = fascia(0, py, facing, .015)[2]
        plate(m, (0, py, pz), facing)

        for s in (-1, 1):
            x0, x1 = (.515, .82) if facing == 1 else (.36, .84)
            y0, y1 = (.145, .223) if facing == 1 else (.245, .337)
            # Surface-mounted lenses wrap around the gently curved end panel.
            def lamp(u, v, inset=0., depth=.007):
                x = s*lerp(x0+inset, x1-inset, u)
                y = lerp(y0+inset, y1-inset, v)
                return fascia(x, y, facing, depth)
            m.surface(lambda u, v: lamp(u, v, -.008, .004), 24, 4, BLACK, (0, 0, facing))
            m.surface(lamp, 24, 6, AMBER if facing == 1 else RED, (0, 0, facing))
            if facing == -1:
                m.surface(lambda u, v: lamp(.80+.20*u, v, .006, .010), 8, 4, AMBER, (0, 0, facing))
                m.surface(lambda u, v: lamp(.05+.16*u, v, .009, .010), 8, 4, LAMP, (0, 0, facing))
            for j in range(1, 22):
                m.line([lamp(j/22, v, .005, .011) for v in (0, 1)], .0011,
                       ((.48, .13, .025) if facing == 1 else (.36, .025, .016), (4., .5)), 4)

        if facing == 1:
            # The air opening sits under the plate, with real slat geometry.
            for s in (-1, 1):
                for j in range(5):
                    x = s*(.39+j*.075)
                    m.line([fascia(x, y, 1, .006) for y in (-.19, -.12)], .014, BLACK)
            m.line([fascia(x, -.234, 1, .012) for x in [-.82+1.64*i/80 for i in range(81)]],
                   .025 if turbo else .008, RUBBER)

    # A flowing roof, with narrower upper body and curved glass on the same loft.
    roof_profile = [(-1.66, .48), (-1.40, .64), (-1.10, .85), (-.78, .978),
                    (-.37, 1.013), (.02, 1.005), (.26, .95), (.50, .79), (.85, .478)]

    def height(z):
        return curve(roof_profile, z)

    def cabin(z, t):
        h = height(z)
        w = .824-.036*(abs(z)/1.66)**3
        x = w*t
        y = curve([(0, h), (.54, h-.010), (.74, h-.035),
                   (.84, h-.10), (1., .457)], t)
        return x, y, z

    def side_glass(s, z, y):
        lo, hi = .74, 1.
        for _ in range(24):
            t = (lo+hi)*.5
            if cabin(z, t)[1] > y:
                lo = t
            else:
                hi = t
        x, _, _ = cabin(z, (lo+hi)*.5)
        return s*(x+.004), y+.001, z

    for s in (-1, 1):
        m.surface(lambda u, v: (s*cabin(z := -1.66+2.51*u, v)[0], cabin(z, v)[1], z),
                  64, 20, PAINT, (s, 1, 0))
        for a, b in ((-1.38, -.49), (-.43, .719)):
            def window(u, v):
                z = lerp(a, b, u)
                low, high = .508, max(.515, height(z)-.126)
                # Tiny eased corners, avoiding pointed glass at the pillars.
                corner = .008*(abs(2*u-1)**12)
                return side_glass(s, z, lerp(low+corner, high-corner, v))
            m.surface(window, 28, 6, GLASS, (s, 0, 0))
            for fixed in (0, 1):
                for path in ([window(i/64, fixed) for i in range(65)],
                             [window(fixed, i/20) for i in range(21)]):
                    m.line(path, .010, BLACK)
                    m.line([(x+s*.0035, y+.001, z) for x, y, z in path], .0035,
                           BLACK if turbo else CHROME)
        m.line([side_glass(s, .44, y) for y in [.518+(height(.44)-.653)*i/24 for i in range(25)]],
               .006, BLACK if turbo else CHROME)
        # Rain gutter sits on the roof edge, rather than across the glass.
        m.line([(s*(cabin(z, .80)[0]+.002), cabin(z, .80)[1]+.003, z)
                for z in [-1.40+2.13*i/90 for i in range(91)]], .005, BLACK if turbo else CHROME)
        # Door-mounted mirror with separate reflective rear face and joint.
        m.line([body.side(s, .52, .46), (s*.99, .56, .52)], .014, BLACK if turbo else CHROME)
        m.ellipsoid((s*1.037, .575, .52), (.108, .055, .066), PAINT if turbo else CHROME)
        m.surface(lambda u, v: (s*1.037+.084*v*math.cos(math.tau*u),
                               .575+.040*v*math.sin(math.tau*u), .461-.004*(1-v*v)),
                  40, 8, ALLOY, (0, 0, -1))

    for a, b, facing in ((.27, .79, 1), (-1.55, -.89, -1)):
        def screen(u, v):
            z = lerp(a, b, u)
            t = (2*v-1)*(.79-.035*math.sin(math.pi*u))
            x, y, _ = cabin(z, abs(t))
            return math.copysign(x, t), y+.005, z
        m.surface(screen, 20, 24, GLASS, (0, 1, facing))
        for fixed in (0, 1):
            for path in ([screen(fixed, i/64) for i in range(65)],
                         [screen(i/64, fixed) for i in range(65)]):
                m.line(path, .012, BLACK)
                if not turbo:
                    m.line([(x, y+.003, z) for x, y, z in path], .004, CHROME)
        if facing == -1:
            for j in range(1, 10):
                m.line([(x, y+.0015, z) for x, y, z in [screen(j/10, .06+.88*i/40) for i in range(41)]],
                       .0008, ((.19, .15, .09), (0., .1)), 4)

    for s in (-1, 1):
        # Wiper spindle, arm and blade follow the lower windscreen.
        m.line([(s*.10, height(.78)+.024, .78), (s*.27, height(.72)+.017, .72)], .005, BLACK)
        m.line([(s*.25, height(.72)+.02, .72), (s*.55, height(.68)+.004, .68)], .008, BLACK)
        # Bonnet shutlines follow the rolled bonnet, with a rounded leading edge.
        m.line([(x, y+.003, z) for x, y, z in
                [body.top(s*(.48+.025*math.sin(math.pi*i/80)), .86+1.12*i/80) for i in range(81)]], .0027, SEAM)
    m.line([(x, y+.003, z) for x, y, z in
            [body.top(-.48+.96*i/64, 1.98+.025*math.sin(math.pi*i/64)) for i in range(65)]], .0027, SEAM)
    for i in range(17):
        x = -.43+i*.054
        m.line([(px, py+.003, pz) for px, py, pz in [body.top(x, .835), body.top(x, .88)]], .004, BLACK)
    # Small original Kestrel badge, with no borrowed marque artwork.
    x, y, z = body.top(0, 1.88)
    m.ellipsoid((x, y+.006, z), (.022, .004, .033), CHROME, 20, 10)
    m.line([(-.009, y+.011, z-.017), (-.009, y+.011, z+.017)], .0025, BLACK, 4)
    m.line([(.009, y+.011, z-.017), (-.007, y+.011, z), (.009, y+.011, z+.017)], .0025, BLACK, 4)
    # Circular fuel flap lies flush in the left front wing.
    points = []
    for i in range(65):
        a = math.tau*i/64
        points.append(body.side(-1, .91+.064*math.cos(a), .38+.052*math.sin(a), .002))
    m.line(points, .0022, SEAM)

    for s in (-1, 1):
        headlamp(m, s)

    # The rear lid and its recessed black grille are on the body surface.
    for s in (-1, 1):
        m.line([(x, y+.003, z) for x, y, z in
                [body.top(s*.515, -2.00+.42*i/48) for i in range(49)]], .0028, SEAM)
    m.line([(x, y+.003, z) for x, y, z in [body.top(-.515+1.03*i/64, -2.00) for i in range(65)]], .0028, SEAM)
    m.surface(lambda u, v: (x := -.46+.92*v, body.top(x, z := -1.97+.31*u)[1]+.003, z),
              12, 24, BLACK, (0, 1, 0))
    for i in range(13):
        z = -1.956+i*.0225
        m.line([(x, y+.011, zz) for x, y, zz in [body.top(-.445+.89*j/40, z) for j in range(41)]],
               .0045, ALLOY if not turbo else RUBBER)
    for x in (-.53, -.39) if turbo else (-.52,):
        exhaust(m, x)
    if turbo:
        spoiler(m, body)
    name = 'kestrel-turbo' if turbo else 'kestrel'
    m.save(ROOT / name / 'body.mesh')
    print(f'{name}: {len(m.vertices)//3:,} body triangles')


def headlamp(m, s):
    # Tilted sealed-beam lens; the short rolled bowl merges into the wing crown.
    center = (s*.733, .450, 2.012)
    n = (0, .43, .90283)

    def point(r, a, depth):
        return (center[0]+r*math.cos(a),
                center[1]+r*math.sin(a)*n[2]+depth*n[1],
                center[2]-r*math.sin(a)*n[1]+depth*n[2])

    m.surface(lambda u, v: point(lerp(.193, .168, v), math.tau*u, lerp(-.095, 0, v)),
              40, 6, PAINT, n)
    for r0, r1, depth, mat in ((.164, .171, .001, BLACK), (.151, .165, .006, CHROME),
                               (.146, .151, .008, BLACK)):
        m.surface(lambda u, v: point(lerp(r0, r1, v), math.tau*u, depth+.003*math.sin(math.pi*v)),
                  48, 2, mat, n)
    m.surface(lambda u, v: point(.146*v, math.tau*u, .010+.018*(1-v*v)), 48, 8, LAMP, n)
    # Fine optical fluting, restrained enough to remain clear at driving distance.
    for j in range(-10, 11):
        x = j*.0125
        span = math.sqrt(.142**2-x*x)
        path = []
        for i in range(25):
            y = -span+2*span*i/24
            r = math.hypot(x, y)
            path.append(point(r, math.atan2(y, x), .011+.018*(1-(r/.146)**2)))
        m.line(path, .0009, ((.70, .73, .67), (4., .6)), 4)
    for y in (-.065, 0, .065):
        span = math.sqrt(.142**2-y*y)
        m.line([point(r := math.hypot(x := -span+2*span*i/32, y), math.atan2(y, x),
                      .011+.018*(1-(r/.146)**2)) for i in range(33)], .0007, CHROME, 4)


def plate(m, center, facing):
    x, y, z = center
    m.box((x-.235, y-.058, z-.009), (x+.235, y+.058, z+.009), BLACK)
    m.box((x-.217, y-.046, z-.011), (x+.217, y+.046, z+.011), ((.83, .82, .72), (0., .12)))
    # Seven small stroke glyphs: KST 1973.
    glyphs = {
        'K': [[(0, 0), (0, 1)], [(1, 0), (0, .5), (1, 1)]],
        'S': [[(1, 1), (0, 1), (0, .5), (1, .5), (1, 0), (0, 0)]],
        'T': [[(0, 1), (1, 1)], [(.5, 1), (.5, 0)]],
        '1': [[(0, .8), (.5, 1), (.5, 0)], [(0, 0), (1, 0)]],
        '9': [[(1, .5), (0, .5), (0, 1), (1, 1), (1, 0)]],
        '7': [[(0, 1), (1, 1), (.3, 0)]],
        '3': [[(0, 1), (1, 1), (1, 0), (0, 0)], [(0, .5), (1, .5)]],
    }
    for i, char in enumerate('KST1973'):
        for stroke in glyphs[char]:
            m.line([(x+facing*(-.174+i*.051+u*.027), y-.026+v*.052, z+facing*.013)
                    for u, v in stroke], .0027, BLACK, 4)
    for s in (-1, 1):
        m.ellipsoid((x+s*.204, y, z+facing*.012), (.003, .003, .002), CHROME, 10, 4)


def exhaust(m, x):
    for r0, r1, mat in ((.034, .044, ALLOY), (0, .033, BLACK)):
        m.surface(lambda u, v: (x+lerp(r0, r1, v)*math.cos(math.tau*u),
                               -.185+lerp(r0, r1, v)*math.sin(math.tau*u), -2.163),
                  40, 2, mat, (0, 0, -1))
    m.surface(lambda u, v: (x+.044*math.cos(math.tau*u), -.185+.044*math.sin(math.tau*u), -2.03-.13*v),
              40, 4, ALLOY, (0, -1, 0))


def spoiler(m, body):
    # Swept, radiused whale-tail: a rising deck pedestal with a rolled rubber lip.
    for s in (-1, 1):
        m.surface(lambda u, v: (s*lerp(.53, .76, v),
                               lerp(body.top(.53, z := -1.95+.43*u)[1], .685-.10*u, v), z),
                  32, 12, PAINT, (s, 0, 0))

    def tail(u, v, lower=False):
        x = (2*v-1)*.945
        z = lerp(-2.055+.06*(abs(x)/.945)**6, -1.50-.04*(abs(x)/.945)**4, u)
        y = .67+.070*(1-u)**3-.09*u + .016*(abs(x)/.945)**4
        return x, y-(.035 if lower else 0), z

    m.surface(tail, 24, 32, PAINT, (0, 1, 0))
    m.surface(lambda u, v: tail(u, v, True), 24, 32, BLACK, (0, -1, 0))
    for fixed in (0, 1):
        m.line([tail(fixed, i/100) for i in range(101)], .025, RUBBER, 10)
        m.line([tail(i/64, fixed) for i in range(65)], .023, RUBBER, 10)
    m.surface(lambda u, v: (x := -.61+1.22*v, .681-.065*u, -1.97+.38*u),
              16, 24, BLACK, (0, 1, 0))
    for i in range(18):
        z = -1.96+.020*i
        m.line([(-.60, .686-.065*i/18, z), (.60, .686-.065*i/18, z)], .005, RUBBER)
    for x in (-.40, -.20, 0, .20, .40):
        m.line([(x, .69, -1.97), (x, .619, -1.59)], .004, RUBBER)


def build_wheel():
    m = BodyMesh()
    tire = ((.036, .040, .044), (0., .055))
    # Cross-section from bead to rounded shoulder, grooved tread and opposite bead.
    profile = [(-.147, .266), (-.151, .302), (-.147, .349), (-.130, .382),
               (-.106, .397), (-.085, .400), (-.078, .395), (-.071, .400),
               (-.039, .400), (-.033, .395), (-.027, .400),
               (.027, .400), (.033, .395), (.039, .400),
               (.071, .400), (.078, .395), (.085, .400), (.106, .397),
               (.130, .382), (.147, .349), (.151, .302), (.147, .266)]
    # Each profile strip has radial normals; separate strips preserve tread grooves.
    for (xa, ra), (xb, rb) in zip(profile, profile[1:]):
        def point(a, v):
            return lerp(xa, xb, v), lerp(ra, rb, v)*math.cos(a), lerp(ra, rb, v)*math.sin(a)
        for i in range(96):
            a, b = math.tau*i/96, math.tau*(i+1)/96
            pts = [point(a, 0), point(b, 0), point(b, 1), point(a, 1)]
            normals = [unit((ra-rb, (xb-xa)*math.cos(t), (xb-xa)*math.sin(t))) for t in (a, b, b, a)]
            for ids in ((0, 1, 2), (0, 2, 3)):
                m.tri([pts[j] for j in ids], tire, normals[ids[0]], [normals[j] for j in ids])
    for s in (-1, 1):
        def ring(inner, outer, depth, mat, dome=0.):
            m.surface(lambda u, v: (s*(depth+dome*math.sin(math.pi*v)),
                                   lerp(inner, outer, v)*math.cos(math.tau*u),
                                   lerp(inner, outer, v)*math.sin(math.tau*u)),
                      64, 2, mat, (s, 0, 0))

        ring(.277, .286, .149, tire, .002)
        ring(.345, .350, .147, tire, .001)
        ring(.251, .274, .148, CHROME, .008)
        ring(.225, .252, .135, ALLOY, .012)
        ring(.087, .228, .087, ((.11, .125, .13), (3., .42)))
        ring(.096, .210, .090, ((.28, .29, .28), (3., .36)))
        ring(.212, .228, .108, BLACK)
        # Five forged petals broaden toward the rim, with bevelled polished edges.
        for k in range(5):
            a = math.tau*k/5

            def petal(u, v, raised=True):
                r = lerp(.068, .236, u)
                half = curve([(0., .027), (.32, .040), (.66, .049), (1., .023)], u)
                tang = (2*v-1)*half
                depth = .139+.013*math.sin(math.pi*u)-.012*(abs(2*v-1)**6)
                if not raised:
                    depth -= .016
                return s*depth, r*math.cos(a)-tang*math.sin(a), r*math.sin(a)+tang*math.cos(a)
            m.surface(petal, 12, 4, CHROME, (s, 0, 0))
            for edge in (0, 1):
                m.surface(lambda u, v: tuple(lerp(petal(u, edge)[j], petal(u, edge, False)[j], v)
                                             for j in range(3)), 12, 1, BLACK, (s, 0, 0))
        ring(0, .084, .146, BLACK, .004)
        ring(.046, .053, .154, CHROME)
        ring(0, .046, .155, ALLOY, .003)
        for k in range(5):
            a = math.tau*(k+.5)/5
            m.ellipsoid((s*.153, .067*math.cos(a), .067*math.sin(a)), (.006, .009, .009), CHROME, 10, 4)
        # Valve stem and small sidewall moulding ribs stay within the rolling radius.
        m.line([(s*.145, .202, .126), (s*.161, .211, .131)], .004, BLACK)
        for k in range(48):
            a = math.tau*k/48
            m.line([(s*.149, r*math.cos(a), r*math.sin(a)) for r in (.319, .333)], .0008, tire, 4)
    for name in ('kestrel', 'kestrel-turbo'):
        m.save(ROOT / name / 'wheel.mesh')
    print(f'kestrel wheels: {len(m.vertices)//3:,} triangles each')


if __name__ == '__main__':
    build_body()
    build_body(turbo=True)
    build_wheel()

    from export_car_glb import export
    for name in ('kestrel', 'kestrel-turbo'):
        export(ROOT / name)
