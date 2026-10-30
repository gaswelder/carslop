# Cars LOP

A driving game inspired by NFS5: beautiful cars, pleasant roads, readable handling
and mechanical personality.

## Contents

- [Build and play](#build-and-play)
- [Add a city](#add-a-city)
- [Nicosia map](#nicosia-map)
- [Optional source tools](#optional-source-tools)
- [Development](#development)
- [Credits and distribution](#credits-and-distribution)
- [Backlog](#backlog)

## Build and play

Requires Rust and the platform's SDL3/OpenGL libraries. The game and map tools
are one executable.

```sh
cargo build --release
./target/release/nfs5like
```

Choose a location and car, then start free driving.

Press **Tab** while driving to cycle **Close → Wide → Bonnet** camera views.

Running without arguments lists the built-in routes and all imported locations
without loading city assets. Choose a location, then Start Driving to load it.
Use Left/Right to browse or click the page arrows for more locations.
`--location` preselects a city in the menu. To drive in Nicosia:

```sh
./target/release/nfs5like play --location nicosia
```

Nürburgring is also available as an offline location, covering the Nordschleife,
Grand Prix circuit, connecting lanes and surrounding roads:

```sh
./target/release/nfs5like play --location nurburgring
```

Its package is `data/locations/nurburgring.sqlite`, built from the
[Geofabrik Rheinland-Pfalz extract](https://download.geofabrik.de/europe/germany/rheinland-pfalz.html)
with Skadi N50E006 and N50E007 elevation. Bounds: `50.315 6.910 50.390 7.015`.
This is a free-drive map with generated scenery. Circuit bridge sections follow
the terrain surface; banking, bridge decks and circuit-specific barriers are
not reconstructed.

Built-in tracks need no city files. For city play, keep its `data/locations/NAME.sqlite`
package beside the executable. Play works offline.
Use `./target/release/nfs5like --help` for command syntax.

## Add a city

The workflow is **map file → city extract → import with elevation → play**.
Build the game first (`cargo build --release`) and run the commands below from
the project root.

### 1. Get the map and elevation files

Download these two files before importing:

**Map:** Open [Geofabrik Downloads](https://download.geofabrik.de/), select your
continent, then the country or smallest available region containing your city.
Download the **`.osm.pbf`** file (not the Shapefile or GeoPackage).
For Engomi, use [Cyprus](https://download.geofabrik.de/europe/cyprus.html) →
[cyprus-latest.osm.pbf](https://download.geofabrik.de/europe/cyprus-latest.osm.pbf).

**Elevation:** Use the **Skadi `.hgt.gz`** tiles from
[Mapzen Terrain Tiles](https://registry.opendata.aws/terrain-tiles/).
For Engomi and Nicosia, download
[N35E033.hgt.gz](https://elevation-tiles-prod.s3.amazonaws.com/skadi/N35/N35E033.hgt.gz).
It covers latitude 35–36° N and longitude 33–34° E. Keep it compressed.

To download both example files from a terminal:

```sh
curl -fL --create-dirs -o /tmp/city-import/cyprus.osm.pbf \
  https://download.geofabrik.de/europe/cyprus-latest.osm.pbf
curl -fL --create-dirs -o /tmp/city-import/N35E033.hgt.gz \
  https://elevation-tiles-prod.s3.amazonaws.com/skadi/N35/N35E033.hgt.gz
```

For another city, choose the elevation tile by its **southwest corner**, rounding
latitude and longitude down to whole degrees. For example, 35.162° N, 33.320° E
falls in `N35E033`; −33.9°, −70.7° falls in `S34W071`.
The download URL follows this pattern (replace both coordinate names):

```text
https://elevation-tiles-prod.s3.amazonaws.com/skadi/N35/N35E033.hgt.gz
```

The directory is the latitude part (`N35`); the filename includes latitude and
longitude (`N35E033.hgt.gz`). See the [Skadi format documentation](https://github.com/tilezen/joerd/blob/master/docs/formats.md#skadi).

Choose a small neighbourhood for your first import. Write down its bounds in
**south, west, north, east** order: minimum latitude, minimum longitude, maximum
latitude, maximum longitude, all in decimal degrees.

Elevation tiles must cover the whole area plus padding, with no missing samples.
For areas spanning tile boundaries, put the adjacent tiles in the same directory
and supply any tile covering the area to `--elevation`. Tiles must have matching
resolution. Keep coordinate filenames such as `N35E033.hgt.gz`; the importer
finds neighbouring `.hgt` or `.hgt.gz` files automatically and preserves them in
the package.

The commands below import Engomi using the two downloads above.
For your own city, replace the map path, elevation path, bounds and location name.

### 2. Extract the city from the regional map

```sh
./target/release/nfs5like extract-region \
  /tmp/city-import/cyprus.osm.pbf \
  /tmp/city-import/engomi.osm \
  --bbox 35.156 33.312 35.168 33.328
```

This writes `/tmp/city-import/engomi.osm`, the XML map used in the next step.
Always provide your city's bounds; the extractor otherwise defaults to Nicosia.

Already have a complete `.osm` XML export? Skip this step and use that file as
`--source` below. It must include all nodes referenced by its roads and buildings.
`add-location` cannot read PBF or Overpass JSON directly.

### 3. Import the map with elevation

```sh
./target/release/nfs5like add-location engomi-demo \
  --source /tmp/city-import/engomi.osm \
  --bbox 35.156 33.312 35.168 33.328 \
  --elevation /tmp/city-import/N35E033.hgt.gz
```

- `engomi-demo` is the name you will select when playing. Choose an unused name.
  Use English letters, digits, hyphens or underscores,
  start with a letter or digit, and keep it under 65 characters.
- `--source` is the XML file from step 2.
- `--bbox` is the same crop as in step 2.
- `--elevation` is the tile covering that crop. Elevation is required.

Wait for **`Added engomi-demo`**. The import also builds the city's visuals;
no separate bake command is needed. Large cities take more time, memory and disk
space. Existing locations are never overwritten.

### 4. Launch the city

```sh
./target/release/nfs5like play --location engomi-demo
```

Keep `data/locations/engomi-demo.sqlite`: the game reads its assets when you play.
Use the same location name and storage directory as in step 3. To see all imported
cities in that directory:

```sh
./target/release/nfs5like list-locations
```

### Updating a city

To change the map, bounds or elevation, repeat the import under a new name.
After renderer changes, rebuild an existing city's visuals with:

```sh
./target/release/nfs5like bake-city --location engomi-demo
```

If a city is not listed, check `--locations-dir` and your working directory.
An absolute storage path avoids this ambiguity. Locations default to
`data/locations`; override this with `--locations-dir` or `NFS_LOCATIONS_DIR`.

New cities use generated materials by default. Use
[city-wide imagery preparation](#prepare-imagery-and-trees-for-the-whole-city)
to add imagery and inferred trees.

## Nicosia map

The bundled map covers about 289 km² of Nicosia, including Engomi, Strovolos,
Lakatamia, Aglandjia and Gonyeli. Assets live in `data/locations/nicosia.sqlite`.

OpenStreetMap supplies roads, building footprints, green spaces and street
furniture. Heights and architectural details are partly estimated, with extra
detail around the Engomi starting area. Low curbs, pavements and parking areas
are drivable; buildings remain solid.

Bridges, tunnels and some complex buildings are omitted. Some roads remain
disconnected. One-way rules are not enforced, and city traffic is not simulated.

The location package contains the aerial imagery; set
`NFS_NO_AERIAL=1` to disable it. The city-wide preparation command below extends
coverage beyond the original 445 × 445 m Engomi pilot.
See [redistribution limits](#credits-and-distribution) before sharing it.

## Optional source tools

All map tools are part of the game executable. Keep downloads, caches and previews
in a temporary directory; `data/locations/` holds the playable city packages and
their rebuild inputs. Imported cities preserve their source XML and elevation tile,
so temporary input files can be removed after a successful import.

```sh
# Merge saved OSM snapshots; gzip inputs are supported.
./target/release/nfs5like merge-osm original.osm.gz extension.osm /tmp/merged.osm

# Prepare your own saved image and Web Mercator export metadata.
./target/release/nfs5like build-aerial-tiles --image /path/to/image.jpg --export /path/to/export.json --output /tmp/aerial-tiles
```

### Preview trees from imagery

```sh
./target/release/nfs5like preview-trees \
  --image /path/to/image.jpg \
  --export /path/to/export.json \
  --city-data data/locations/nicosia.sqlite \
  --output /tmp/tree-preview
```

Open `/tmp/tree-preview/review.html` to inspect candidates. Road and building
conflicts are crossed out automatically. You can override decisions and download
the reviewed GeoJSON. `--osm FILE.osm` optionally highlights existing mapped trees.

To replace a city's imagery-tree supplement with accepted proposals:

```sh
./target/release/nfs5like import-imagery-trees \
  --candidates /tmp/tree-preview/kept-candidates.geojson \
  --city-data data/locations/nicosia.sqlite
./target/release/nfs5like bake-city --location nicosia
```

Heights and crown sizes are estimates. Detection can mistake shrubs, grass and
shadows for trees. The original OSM trees remain unchanged.

### Prepare imagery and trees for the whole city

```sh
./target/release/nfs5like prepare-city-imagery \
  --city-data data/locations/nicosia.sqlite \
  --work-dir /tmp/nicosia-imagery
./target/release/nfs5like bake-city --location nicosia
```

This explicitly downloads World Imagery for the city's bounds, prepares terrain
textures, detects trees, and excludes roads, buildings and duplicates. Rerun with
the same work directory to resume. `--limit-blocks 2` downloads a sample without
changing city assets. Allow several gigabytes of disk space for preparation.

After a successful preparation and bake, the work directory can be deleted.
Keep the `.sqlite` file: it includes imagery, trees, roads, buildings, terrain,
source records and the baked scene.
Provider metadata, preparation settings and coverage counts remain inside the
package as `aerial/service.json`, `aerial/plan.json` and `aerial/source.json`.
Gameplay stays offline; the imagery cache uses at most 64 MiB of GPU memory.

World Imagery covers northern neighbourhoods where the DLS source returns blank
images. Image dates and sharpness vary. These are local development assets;
redistribution permission has not been established.

### Single-file location packages

Each location is one self-contained SQLite file, such as
`data/locations/nicosia.sqlite`. Geometry chunks and imagery tiles are read on
demand; the game does not unpack the location or load the whole database.
Assets are split into independently compressed 256 KiB blocks with checksums.
Road and collision data currently retain the existing in-memory representation.
SQLite is bundled into the executable; no database server is needed.

```sh
# Convert an existing location folder; the original folder is preserved.
./target/release/nfs5like pack-location /path/to/location data/locations/my-city.sqlite

# Check database integrity, every compressed block, and all asset hashes.
./target/release/nfs5like verify-location data/locations/my-city.sqlite

# Extract source records or edit a location offline.
./target/release/nfs5like unpack-location data/locations/my-city.sqlite /tmp/my-city
```

New imports publish `.sqlite` packages automatically. `bake-city --location NAME`
updates the baked scene in the package after intentional edits to city inputs or
when you want to adopt new geometry-generation behavior. Published locations use
their embedded geometry and do not need rebaking after game rebuilds or source
code changes. Existing packages with build fingerprints remain playable; scene
format versions and integrity checks still apply. Imagery preparation and tree-import
commands also accept a `.sqlite` path for `--city-data`; they use a temporary
working folder and replace the package after a successful update. Allow extra
disk space during offline edits. Finalized packages need no sidecar files.

## Development

Run tests:

```sh
cargo test
```

Rebuild the bundled city's visuals after renderer changes:

```sh
./target/release/nfs5like bake-city --location nicosia
```

## Credits and distribution

Map data © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright).
Saved OSM sources and OSM-derived map databases are distributed under ODbL 1.0;
game code and original generated materials are separate. Preserve attribution
and source provenance when sharing city assets. The bundled map uses the
[Geofabrik Cyprus extract](https://download.geofabrik.de/europe/cyprus.html);
source records are stored inside the package as `manifest.json` and
`sources/manifest.json`.

Elevation: [Mapzen Terrain Tiles](https://registry.opendata.aws/terrain-tiles/),
using the [Skadi N35E033 tile](https://elevation-tiles-prod.s3.amazonaws.com/skadi/N35/N35E033.hgt.gz).
Terrain data courtesy of USGS; European terrain produced using Copernicus data
and information funded by the European Union (EU-DEM). See the provider's
[source attribution and licensing](https://github.com/tilezen/joerd/blob/master/docs/attribution.md).
Local source records are stored as `elevation/heightfield.json` inside the package.

City-wide imagery: Esri World Imagery — Source: Esri, Vantor, Earthstar
Geographics, and the GIS User Community. See the
[provider service](https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer).

Original pilot imagery: Cyprus Department of Lands and Surveys,
[Orthoimagery 2019 WebMercator service](https://eservices.dls.moi.gov.cy/arcgis/rest/services/BASEMAPS/Orthoimagery_2019_WebMercator/MapServer).
Redistribution permission for the local imagery pack has not been established;
do not redistribute packages containing this imagery or its derived assets
unless authorised.

Architectural references:

- [Fereos Architects, Residence in Engomi](https://www.fereos.net/project/residence-in-engomi-nicosia-2/)
- [Antonis Zenios, E P I A S](https://www.antoniszenios.com/en/properties-for-sale-nicosia/e-p-i-a-s)
- [Fereos Developers, Hilltop specifications](https://www.fereosdevelopers.com/wp-content/uploads/d004-specifications-iw.pdf)

Architecture is an artistic interpretation, not a reconstruction of individual
addresses. Downloaded reference photographs are local study material and are not
distributable game textures.

## Backlog

### World

- [ ] Let's make Nicosia the default.
- [ ] arbitrary-city downloader
- [ ] Is it possible to track real-world coordinates in game so we can refer to locations?
- [ ] What if we use the satellite images to infer trees?

### Cars

- [x] Let's make gear changes manual: A shifts up, Z shifts down, stop using WASD for movement, keep the arrows. Add a neutral between 1 and R.
- [x] Progressive foot braking with an ABS-like grip cap, retained steering authority, and skid feedback from sliding rather than brake input.
- [x] Replace prescribed turning with axle tire forces, yaw inertia, weight transfer, gear-dependent torque, and sprung road contact.
- [ ] Is handbrake side-sliding realistic?
- [ ] Let's add Mazda Demio.

### HUD

- [ ] Need a full map view (M to open, esc to return back to the game).

### Sound

- [ ] What would it take to have realistic sounds?

New cars can be added as runtime JSON definitions with optional body meshes; no
recompilation is needed. See [the car asset guide](data/cars/README.md#adding-cars-without-recompiling). Restart the game to discover additions.

## Vehicle handling

Vehicles use a nonlinear front/rear axle tire model, integrated at steps no
larger than 1/240 second. Steering creates tire forces and a yaw moment about
the center of mass. Rear-engine Kestrel, rear-drive Muscle, and front-drive Club
have separate weight distributions, yaw inertias, brake biases, and axle
stiffnesses in `data/cars/{kestrel,muscle,club}/car.json`, loaded at startup. These values are gameplay calibrations rather than
measured specifications or a recreation of NFS5's internal physics.

Acceleration and braking transfer load between axles. Lateral transfer reduces
available grip through tire load sensitivity. Drive, braking, and cornering
share each axle's friction budget; hard acceleration can saturate driven tires,
and the handbrake locks the rear axle. Foot braking retains ABS-like regulation.
Tire forces peak and soften into sliding grip, so front and rear breakaway and
countersteering arise from forces rather than a target rotation rate. Keyboard
steering still ramps progressively and reduces lock with speed. Kestrel has
additional speed-dependent lock reduction and a firmer rear-axle response so
holding a steering key at city speeds does not provoke abrupt oversteer.

Engine torque varies with RPM, passes through the selected gear and final drive,
and is interrupted during automatic shifts. Closed-throttle engine braking
also depends on gearing. Kestrel retains its wheel-power/drag limit; the other
cars use their torque curves against route-specific road load. Wheelspin feedback reflects
driven-axle saturation, but there is no independent wheel angular-inertia solver.

A damped spring supports chassis height and permits unloading, flight, and
landing over changes in ground height. Pitch and roll respond to acceleration
through damped springs; curb impacts retain their small feedback bump. This is
an axle model with one vertical chassis degree of freedom, not four independent
suspension corners or a full 3D rigid body. Collision shapes and damage modeling
remain simplified.
