# Car assets

Each car has one folder containing its definition and assets:

```text
data/cars/
  README.md
  kestrel/
    car.json
    body.mesh
    wheel.mesh
    audio/
  kestrel-turbo/
    car.json
    body.mesh
    wheel.mesh
  muscle/
    car.json
    body.mesh
    wheel.mesh
  club/
    car.json
    body.mesh
    wheel.mesh
```

Car designs are runtime assets. Every `car.json` selects its body with `mesh`
and its wheel with `wheel_mesh`. Replace the assets or edit those paths, then
restart; rebuilding is unnecessary. Paths are relative to that car's folder.
Missing or invalid designs stop scene loading with the asset filename and error.
There is no compiled-in design fallback.

Audio loads the three mono 48 kHz signed 16-bit little-endian `.pcm` files from
`kestrel/audio/` once at startup, relative to the working directory. Restart the
game after replacing them; rebuilding is unnecessary. If any loop is missing
or invalid, the game warns and uses its procedural engine voice. A valid loop
has an even byte count, more than twice its overlap in samples (2242 idle,
2068 load, 2064 coast), and at most 9,600,000 bytes. Raw PCM has no header, so
the sample rate and encoding must match. Keep all of `data/cars` with packaged
builds. Audio fallback operates independently of mesh loading.

See [Kestrel sound credits and conversion commands](kestrel/audio/README.md).

Format version 1: eight ASCII bytes `NFSCAR01`, then a little-endian u32 vertex
count, followed by that many records of eleven little-endian f32 values:
position XYZ, color RGB, normal XYZ, material XY. Each three records form a
triangle. Coordinates are meters, Y up, +Z forward. No indices or textures.

Bodies use car-local coordinates. Wheel meshes are centered on the axle at the
origin and rotate about X; the game positions and animates four copies using the
car's wheel mounts. Colors, normals, and material values are stored in each mesh.
The original wheel has a 0.4 m radius; keep replacement wheels at that radius to
match the shared wheel animation/feedback calibration. Paint material X = 1
receives the selected car tint; X = 0 remains untinted. Material Y controls specular
strength. Triangle winding and normals must face outward.

## Tuning existing cars

`kestrel/car.json`, `muscle/car.json`, and `club/car.json` are the required runtime parameter
files for the original cars. Edit them and restart the executable to change
names, handling, power/drag, gearbox, or wheel placement without rebuilding.
These files contain complete definitions; custom cars inherit their current
values. Missing or invalid required files stop loading with the filename and
error, rather than silently using compiled-in tuning. Ship these files with
all builds. `half_track` sets lateral wheel spacing; `wheel_height` sets the
wheel-center Y coordinate in meters. Body and wheel artwork are external mesh assets selected by the same JSON file.

## Adding cars without recompiling

Create `data/cars/my-coupe/car.json` and put its assets in the same folder, then
restart the existing executable. Car folders are discovered at startup in folder
name order and appended to the three original cars. Folders without `car.json`
and loose JSON files are ignored. Invalid custom definitions are skipped with a
warning. Use the menu or left/right car selection to select new entries;
F1–F3 remain shortcuts for the original cars.

For example, save this as `data/cars/my-coupe/car.json`:

```json
{
  "base": "kestrel",
  "name": "MY COUPE",
  "description": "CUSTOM ROAD COUPE",
  "mass": 1100,
  "engine_force": 9500,
  "top_speed": 60,
  "half_track": 0.88,
  "mesh": "my-coupe.mesh",
  "gearbox": {
    "ratios": [2.833, 1.778, 1.217, 0.962, 0.821],
    "final_drive": 4.428571
  }
}
```

`base` is required: `kestrel`, `muscle`, or `club`. Other fields inherit that
base's settings. Omit `mesh` to reuse its body; mesh paths are relative to the
JSON file and use the format documented above. Custom meshes contain the body
only; the game adds wheels using `half_track`, `wheel_height`, and `wheelbase`.
Omit `wheel_mesh` to inherit the base wheel design, or supply a separate wheel mesh.
Engine sound inherits the base voice.

All handling values use SI units (`top_speed` is meters per second). Supported
scalar overrides are `mass`, `engine_force`, `top_speed`, `brake_force`,
`lateral_force`, `wheelbase`, and `steering_response`. The optional `dynamics`
object accepts `front_weight`, `cg_height`, `track_width`, `yaw_inertia`,
`front_drive`, `brake_bias`, `corner_stiffness` (front/rear array), `peak_torque`,
`suspension_hz`, and `steering_speed_squared`. The `gearbox` object also accepts
`rolling_radius`, `reverse_ratio`, `upshift_rpm`, and `downshift_rpm`; ratios
must be positive and descending. `road_engine` accepts `wheel_power_watts` and
`drag_accel`, or `null` to disable the power/drag model. Keep JSON definitions
and referenced assets with the executable's working-directory `data/cars`.

The [Kestrel Turbo](kestrel-turbo/README.md) is a 930-inspired wide-body variant,
loaded through the same runtime catalog as other custom cars.
