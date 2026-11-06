# Cars LOP

--- human info begins ---

Experimental project: have an AI create a decent game and not even read the code.
Every commit message is an "ask" to the coding agent.
Everything else, including README.md outside of this section, is generated.

--- human info ends ---

A driving game inspired by NFS5.

<img src="screenshot.20261104.jpg">

[Screenshot location: Engomi, Nicosia](https://www.google.com/maps/search/?api=1&query=35.1670537,33.3169556)

## Build and play

Requires Rust, SDL3 and OpenGL. Run commands from the repository root.

```sh
cargo build --release
./target/release/nfs5like
```

Choose a car and location in the menu. Bundled locations are Nicosia,
Nürburgring and Coastal Highway; play works offline.
Use `--help` for the command reference and `--locations-dir PATH` for another
location store. `play --location NAME` preselects a location.

### Controls

| Key | Action |
| --- | --- |
| Up / Down | Accelerate / brake |
| Left / Right | Steer |
| A / Z | Shift up / down (includes neutral between first and reverse) |
| Space | Handbrake |
| C / Tab | Cycle Close → Wide → Bonnet cameras |
| Esc | Pause / resume, or close the full map and return to driving |
| R | Restart |
| H | Recover the car |
| M | Open / close the full map (pauses driving) |
| Mouse wheel / + / - | Zoom the full map in / out |
| L | Return to location and car selection |
| V | Toggle audio mute |

For a burnout start, shift into neutral with **Z**, hold **Up** to rev, then
press **A** to engage first while keeping the accelerator held.

Press **C** in the full map to copy the car's Google Maps location. Zoom is
remembered per location across launches in `data/map-zoom.json`.

## Add and maintain locations

Put imported `NAME.sqlite` packages in `data/locations/`, or the chosen location
store, and restart. To import OSM XML with matching HGT elevation tiles:

```sh
./target/release/nfs5like extract-region cyprus.osm.pbf /tmp/engomi.osm \
  --bbox 35.156 33.312 35.168 33.328
./target/release/nfs5like add-location engomi-demo --source /tmp/engomi.osm \
  --bbox 35.156 33.312 35.168 33.328 --elevation N35E033.hgt.gz
```

Bounds use south, west, north, east order. Names contain letters, digits, `-`
and `_`, start with a letter or digit, and have at most 64 characters. Imports
require a new name. Keep neighboring HGT tiles together when crossing tile
boundaries. Nicosia uses `N35E033`; Nürburgring uses `N50E006` and `N50E007`.

For an authored circuit, add `NAME.track.json` to the same store:

```json
{
  "version": 1,
  "name": "Hillside Circuit",
  "width": 8,
  "points": [[0, 0, 0], [120, 2, 0], [120, 5, 120], [0, 1, 120]]
}
```

Use 4–4096 XYZ points in metres, a width of 2–30 metres and a control perimeter
up to 50 km. Omit the closing point. No recompilation is needed.

After editing imported source assets:

```sh
./target/release/nfs5like bake-city --location nicosia
# Or rebuild only visuals / collision:
./target/release/nfs5like bake-scene --location nicosia
./target/release/nfs5like bake-collision --location nicosia
```

`pack-location`, `unpack-location` and `verify-location` maintain packages.
`circuit-only --location NAME` replaces a map with its racing roads.
`./rebuild.sh` rebuilds the release executable and bundled collision data.

To prepare imagery and trees:

```sh
./target/release/nfs5like prepare-city-imagery \
  --city-data data/locations/nicosia.sqlite --work-dir /tmp/nicosia-imagery
./target/release/nfs5like bake-scene --location nicosia
```

Reuse the work directory to resume. `--limit-blocks 2` makes a sample without
changing the playable package. For manual tree review, use `preview-trees`,
review its `review.html`, then pass the accepted GeoJSON to
`import-imagery-trees` and rebuild visuals.

## Car assets

Edit `data/cars/NAME/car.json` and `model.glb`, then restart. Additional car folders
can inherit from `kestrel`, `muscle` or `club`:

```json
{
  "base": "kestrel",
  "name": "MY COUPE",
  "mass": 1100,
  "model": "model.glb"
}
```

Omit `model` to inherit artwork; paths are relative to the car folder. Use the
base JSON definitions as tuning references. Measurements use SI units;
`top_speed` is metres per second.

### GLB export

Export static triangulated glTF 2.0 GLB: metres, Y up, +Z forward, one embedded
buffer and opaque materials. Name parts `body`, `wheel_rl`, `wheel_rr`,
`wheel_fl`, `wheel_fr`. Left/right wheels use negative/positive X.
Wheel origins are axle mounts; wheels spin about X and steer about Y.
Match JSON wheelbase and rolling radius to the artwork.

Use vertex colours or solid PBR materials; textures, transparency, animation,
skinning, morphs, sparse accessors and external buffers are unsupported.
Material extras `{"carslop_material": [CODE, GLOSS]}` use gloss 0–1 and codes:
0 fixed colour, 1 paint, 2 glass, 3 metal, 4 optical lens, 5 brake lens,
6 reverse lens. Normals and winding must face outward.

## Development

```sh
cargo test
# Graphics tests require SDL3/OpenGL; car views go to /tmp/car-preview.
cargo test kestrel_art_preview -- --ignored --test-threads=1
```

Preview tests write to temporary paths shown in their source. `RUST_LOG` controls
logging. Test-only inputs are `VALIDATED_CITY` (full-import fixture),
`TEST_RUNTIME_CAR` and `TEST_LOCATION_CACHE` (subprocess fixtures).

Asset helpers:

```sh
python3 tools/build_fictional_cars.py --help
python3 tools/build_engine_audio.py SOURCE.wav
```

The audio helper requires NumPy, SciPy and ffmpeg. Source recordings belong in
ignored `artifacts/audio/source/`.

## Credits and distribution

- **Maps:** © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright),
  ODbL 1.0; extracts from Geofabrik ([Cyprus](https://download.geofabrik.de/europe/cyprus.html),
  [Rheinland-Pfalz](https://download.geofabrik.de/europe/germany/rheinland-pfalz.html)).
  Preserve attribution and source provenance when sharing derived map databases.
- **Elevation:** [Mapzen/Skadi](https://registry.opendata.aws/terrain-tiles/),
  USGS and EU-funded Copernicus EU-DEM; [source attribution and licenses](https://github.com/tilezen/joerd/blob/master/docs/attribution.md).
- **Imagery:** [Esri World Imagery](https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer)
  — Esri, Vantor, Earthstar Geographics and the GIS User Community;
  pilot imagery from [Cyprus DLS Orthoimagery 2019](https://eservices.dls.moi.gov.cy/arcgis/rest/services/BASEMAPS/Orthoimagery_2019_WebMercator/MapServer).
  Redistribution permission is unconfirmed; do not redistribute imagery or
  derived assets without authorization.
- **Engine audio:** Pole Position Production's [1981 Porsche 911 SC recording](https://pole.se/the-iconic-sound-of-a-porsche-911-sc-from-1981/),
  `Porsche_911_SC_-81_t5_onboard_driving_constant_rpms_exhausts.wav`,
  from [Sonniss GDC 2016](https://sonniss.com/gameaudiogdc/), under the
  [Sonniss GDC bundle license](https://sonniss.com/gdc-bundle-license/).
  Modified load/coast excerpts are licensed game content, separate from the
  code license; distribute within the game, never as standalone recordings,
  sample packs, sound libraries or project templates.
- **Car design references:** [Volkswagen Golf II GTI](https://www.volkswagen-newsroom.com/en/golf-2-gti-19841991-19502),
  [Chevrolet Camaro SS](https://www.gm.com/heritage/collection/chevrolet/1969-chevrolet-camaro-ss),
  [Mazda Demio/Mazda2](https://newsroom.mazda.com/en/publicity/release/2014/201407/140717a.html).
  Models are original generated artwork.
