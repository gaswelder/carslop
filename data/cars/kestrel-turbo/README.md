# Kestrel Turbo

A fictional Kestrel variant inspired by the 1975 Porsche 911 Turbo 3.0 (930).
Select **KESTREL TURBO** in the car menu. This folder is a runtime car package;
adding it to an existing build's `data/cars` and restarting discovers it.

The body is derived from the project's original Kestrel mesh, with widened
haunches, a whale-tail rear spoiler, black spoiler edging and grille, deeper
front chin, and impact-bumper strips. No Porsche artwork or badges are used.
Wheel design and engine recordings are inherited from the original Kestrel;
the local wheel mesh keeps the package's visual assets together.

Reference: [Porsche's 1975 Turbo 3.0 museum specification](https://newsroom.porsche.com/dam/jcr%3A3c07547c-36bc-4e00-825c-5ac2bbe10805/Datenblatt%20911%20Turbo%203.0%20Coup%C3%A9.pdf.PDF)
lists 191 kW / 260 PS, 1140 kg, 250 km/h, and 5.5 seconds to 100 km/h.
[Porsche's historical feature](https://newsroom.porsche.com/en_AU/2020/Lifestyle/porsche-klassik-magazine-guus-reinerink-taycan-turbo-23297.html)
lists 350 Nm and a four-speed gearbox (and a different 1195 kg weight).
We use the museum's 1140 kg specification.

The runtime tune uses 171.9 kW at the wheels (90% of 191 kW), rear drive,
four forward gears, firmer suspension and more rear grip. Gear ratios,
shift points, drag, launch force, brakes, inertia, and tire forces are game
calibrations, not a certified reproduction. The existing Kestrel axle positions
and wheel radius are retained to match the adapted mesh. The turbo builds boost progressively with throttle and RPM, loses pressure on
lift-off and shifts, and adds an airflow whoosh that grows brighter and stronger with spool speed.
Rated torque and wheel power represent full boost. All tuning is in `car.json`;
all geometry is in the mesh files.
