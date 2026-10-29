# Cars LOP

## Goal

A driving game, draw inspiration from NFS5: beautiful cars, pleasant roads, readable handling, mechanical personality.

## Contents

- [Run and import locations](#run-and-import-locations)
- [Offline source preparation](#offline-source-preparation)
- [Nicosia map, rendering and distribution](#nicosia-urban-area-offline-driving-map)
- [Elevation and attribution](#nicosia-urban-area-elevation)
- [Engomi visual study](#engomi-reference-area-visual-study)
- [Aerial surfaces and previews](#aerial-surfaces-runtime-integration-and-previews)
- [Backlog](#backlog)

## Run and import locations

One Rust executable handles play, local OSM imports, location listing and render
archive baking. No Python installation is needed for these commands.

```sh
cargo build --release
./target/release/nfs5like play
./target/release/nfs5like add-location my-city --source /path/to/map.osm --flat-terrain
./target/release/nfs5like list-locations
./target/release/nfs5like play --location my-city
./target/release/nfs5like bake-city --location my-city
```

For real terrain, replace `--flat-terrain` with `--elevation /path/to/N35E033.hgt.gz`
(an uncompressed `.hgt` also works). The tile must cover the crop and its 80 m
padding. Void samples are rejected. `--flat-terrain` explicitly chooses zero
elevation; it does not download or estimate terrain.

Use `--bbox SOUTH WEST NORTH EAST` to crop an extract. Otherwise the importer
uses the XML bounds, or the extent of its nodes. Coordinates are metres from the
crop centre. Public ground-level road components remain separate, and unsupported
buildings crossing the crop or containing courtyard relations are skipped.

`add-location` compiles streets, buildings, mapped ground cover, trees, controls,
elevation and `scene.pack` before making a location available. It preserves the
OSM source, supplied HGT tile and available provenance. Existing names are never
replaced; failed imports remove their temporary directory.

Locations live in the user's data directory (`$XDG_DATA_HOME/nfs5like/locations`
or `~/.local/share/nfs5like/locations` on Linux). Override it with
`--locations-dir PATH` on each command, or `NFS_LOCATIONS_DIR`. Map assets remain
external to the executable. `play --location nicosia` selects the bundled city;
ship `data/engomi` beside the executable to include it. No arguments opens the
existing menu. A selected imported location occupies the menu's city slot.
The executable still uses the platform's SDL3/OpenGL libraries.

Local imports support OSM **XML**, not PBF or Overpass JSON. Downloading locations
and multi-tile terrain are not implemented by the native CLI. Source preparation
also runs in the same executable: `extract-region` reads
PBF, `merge-osm` merges XML snapshots (including gzip), and `build-aerial-tiles`
prepares saved JPEG/PNG imagery. See [source tools](#offline-source-preparation).
Use `--help` for command syntax. `--bake-city` remains a compatibility alias.

## Offline source preparation

All source preparation is built into `nfs5like`; no Python, Osmium, Pillow, or
external helper executables are required. These commands run before SDL starts.

```sh
cargo build --release
./target/release/nfs5like extract-region data/engomi/sources/cyprus.osm.pbf region.osm --bbox 35.10 33.25 35.23 33.47
./target/release/nfs5like merge-osm original.osm.gz region.osm merged.osm
./target/release/nfs5like build-aerial-tiles --image artifacts/aerial-preview/engomi-2019.jpg --export artifacts/aerial-preview/export.json --output data/engomi/aerial
cargo test
```

`extract-region` streams multiple passes over a local PBF, retaining nodes in the
crop, complete touching ways, and non-nested multipolygons touching those ways.
It completes member ways and referenced nodes, preserves IDs, versions and tags,
and writes a JSON count/provenance report beside the XML. The default bounds are
the historical Nicosia crop above. Ways with no node inside the crop are omitted.

`merge-osm` accepts XML or gzip XML. Newer known versions win; the original wins
ties and wins when the extension lacks version metadata. Overpass error remarks
and incomplete XML are rejected before writing output. Output is deterministic.

`build-aerial-tiles` accepts JPEG/PNG and a saved Web Mercator export JSON. It
writes RGB tiles, replicated outer borders, neighbouring internal gutters, CRCs,
and source-export metadata. Defaults are `--size 512 --gutter 8`; dimensions must
be exact multiples of size. Projection retains the historical Engomi origin.
Use `--preview-output DIR` to also generate legacy `image.rgb` and `georef.txt`
for the graphical preview test. Imagery permission and provenance requirements
still apply. All commands are offline; map and image assets remain external.

`cargo test` covers the native map compiler, including road filtering, disconnected
networks, junctions, pavement separation, enclosed blocks, smoothing, signs,
landscape holes/priorities, tree conflicts, and elevation filtering/void rejection.
Native scenery retains geometry in memory; the old WKB checkpoints and road
coverage caches no longer exist, so their cache-resume tests were retired.
Ground cover uses the native layered mode beneath opaque roads and buildings.

After importing the full Nicosia crop described in [the map section](#nicosia-urban-area-offline-driving-map), run the
explicit comparison against the historical bundled assets:

```sh
NFS_VALIDATED_CITY=/path/to/imported/nicosia cargo test full_nicosia_import_preserves_network_buildings_and_surface_area -- --ignored --test-threads=1 --nocapture
```

This checks geographic node positions, road metadata, building identities and
heights, emitted asphalt area/winding, elevation loading, scenery CRCs, mapped
control/tree identities and land-cover areas (within 0.25% per style and 0.01%
overall; meshes from the two geometry libraries need not be identical). It writes
`validation-report.json` beside the imported location. The regular suite keeps
small fixtures for fast regression checks, including mixed ring winding and
triangles that collapse at the on-disk coordinate precision.

The October 29, 2026 full-city check preserved 97,732 nodes, 105,078 road segments,
72,937 buildings, all 2,627 mapped control approaches, and all 2,321 tree IDs and
dimensions. Asphalt area differed by 0.0033%; total mapped land-cover area by
0.0006%, with a maximum class difference of 0.206% (paving). This is validation
within stated tolerances, not exact mesh equivalence.

The native archive was 3.87 GiB versus the bundled compiler's 2.40 GiB. Native
meshes contain more curb/surface vertices; mesh size remains an optimization
task. The historical bundled text assets were preserved, and their render
archive was rebaked for the updated runtime source fingerprint.

Validation outputs are in `artifacts/native-validation/validation-report.json`.
The retiring scripts were saved to `artifacts/retired-python-map-tools.tar.gz`
before removal before the first Git commit.

The bundled `data/engomi` text assets retain their historical coordinates and
provenance. New imports use their crop centre and can produce a different
triangulation. Retiring the Python compiler does not regenerate bundled text data.

## Nicosia urban area: offline driving map

Map data © OpenStreetMap contributors, [ODbL 1.0](https://www.openstreetmap.org/copyright).
The saved source and derived map databases are distributed under ODbL 1.0;
original procedural materials and game code are separate. Preserve attribution
when redistributing. The opening menu and city Escape menu display attribution.
The folder retains its historical `engomi` name.

### Coverage

The crop is **35.10–35.23 N, 33.25–33.47 E**, approximately **20.0 × 14.5 km
(289 km²)**. It covers central Nicosia, Engomi, Strovolos, Lakatamia, Aglandjia,
Gonyeli and adjoining northern and southern neighbourhoods. This is a rectangular
urban-area extract, not an administrative boundary or the whole Nicosia district.
The origin remains 35.162 N, 33.320 E; +X is east and +Z south. The vicinity
minimap remains north-up and approximately 400 m across.

The database contains **97,732 road nodes, 105,078 road segments from 21,352 ways,
and 72,937 buildings**. All **165** ground-level road components are retained.
No artificial connections join separate networks. Bridges, tunnels and nonzero
layers are still excluded (209 ways); therefore road connectivity is incomplete
where crossing structures are needed. Footpaths, construction roads and private
or prohibited driving ways are excluded. Motorway/trunk roads use the generic
avenue treatment; grade-separated junctions remain future work.

The saved [Geofabrik Cyprus extract](https://download.geofabrik.de/europe/cyprus.html)
reports snapshot **2026-10-28T21:21:02Z**, downloaded October 29. The previous
prototype snapshot is merged where needed; known newer OSM object versions win.
`data/engomi/manifest.json` and `data/engomi/sources/manifest.json` record URLs, versions, counts and
SHA-256 hashes. `data/engomi/sources/cyprus.osm.pbf` and the previous compressed extract
preserve inputs. No data is downloaded at game launch.

### Appearance and driving

Road centreline shapes and building footprints come from OSM. Width uses tagged
width first, then lanes/class, clamped to 5.5–18 m. Junction fillets and short
centreline curves are reconstructed. One-way metadata is retained but is not
enforced on the player. There is no city traffic AI yet.

Buildings use explicit height where available, then floors or type defaults;
72,126 heights are estimates. Heights are clamped to 2.5–70 m. Façades, windows,
roof colours and road materials are procedural approximations, not photographs.
Boundary-crossing buildings and unsupported courtyard buildings are skipped.

Mapped scenery includes **3,627 areas and 2,321 trees**: parks, grass, scrub,
sports areas, water, parking and paving. Priority partitions overlapping uses.
Land cover is draped below opaque roads and pavement rather than cutting millions
of road edges into its geometry. Trees keep mapped positions; 68 trees conflicting
with asphalt/buildings are suppressed. Unknown tree sizes use deterministic
estimates. Unclassified ground remains dry soil. The base map uses procedural surfaces;
the optional aerial pack is described below.

Mapped controls yield **1,193 signal heads, 1,281 stop signs, 42 yield signs and
111 speed signs**. Curb positions/facing are reconstructed when not explicitly
mapped; ambiguous approaches and controls without a clear placement are skipped.
Signals use a fictional alternating 50-second cycle, not real-world timing.
Mapped speed limits on ways alone do not invent sign posts.

Elevation comes from the saved Mapzen/Skadi N35E033 HGT tile: native one-arcsecond
posts, Gaussian filtering, then a **20 m, 1010 × 733 grid**, without exaggeration.
The cropped terrain spans approximately **85–301 m EGM96**, relative to a
181.763 m local datum. Roads, curbs, car support and foundations share the same
triangulated terrain. This is regional terrain, not surveyed pavement.
See [elevation attribution](#nicosia-urban-area-elevation).

Drive across low curbs onto pavements, parking and open ground. Curbs apply a
small directional velocity loss and cosmetic body bump; buildings and the crop
boundary remain solid. Vehicle height follows the visible supporting surface.

### Rebuild and distribution

The native executable can now compile a local OSM XML extract into a separate
playable location, including the render archive:

```sh
cargo run --release -- add-location engomi-native --source data/engomi/source.osm --bbox 35.156 33.312 35.168 33.328 --elevation data/engomi/elevation/N35E033.hgt.gz
cargo run --release -- play --location engomi-native
```

This creates an independent location in the user data directory. It uses the
crop centre as its origin; the bundled Nicosia package below retains its historical
origin. See [Run and import locations](#run-and-import-locations) for storage and command options.

Ship the **entire `data/engomi` folder** beside the executable. Map/collision data
is read from external text assets; render geometry comes from `scene.pack`.
Normal builds and play require neither Python nor network access.
`NFS_CITY_DATA` selects an alternate complete map directory; `NFS_CITY_SCENE`
can override only the archive path, which must match the data/code fingerprint.
Search order is executable-relative data, working-directory data, then the
development project directory.

```sh
cargo run --release -- add-location nicosia-rebuilt --source data/engomi/source.osm --bbox 35.10 33.25 35.23 33.47 --elevation data/engomi/elevation/N35E033.hgt.gz
cargo run --release -- play --location nicosia-rebuilt
cargo test
```

Rebuilding a whole city is an offline processing job. The native compiler logs
its stages and retains road coverage in memory for furniture and scenery.
All public ground-level road components are retained for this urban crop.
The retired Python compiler's WKB checkpoints and caches are not used.
`add-location` creates an independent package and bakes its archive before
publishing it; it does not overwrite the historical bundled package.

To rebake only the bundled package's render geometry, without SDL/OpenGL:

```sh
cargo run --release -- bake-city
```

For an imported location, use `cargo run --release -- bake-city --location NAME`.
For another complete asset folder, use
`NFS_CITY_DATA=/absolute/output cargo run --release -- bake-city`.
For a fresh PBF extract, `nfs5like extract-region` streams the selected region
with complete ways and multipolygon members; `nfs5like merge-osm` merges snapshots.
Their commands are documented in [Offline source preparation](#offline-source-preparation).

### Rendering and loading

Render geometry is partitioned into 256 m cells by triangle centroid, preserving
complete positions at cell boundaries. The archive has separate shared ground,
detailed building and coarse building layers. Coarse buildings keep identical
walls, roofs and foundations and omit windows. Detail switches at 160/200 m with
hysteresis. Only frustum intersections draw; buffers beyond 696 m unload.

The renderer reads only the archive index on selection. A background worker
prefetches nearby layers within 568 m into a **32 MiB bounded CPU cache**.
Stale queued work is cancelled and priority favours visible missing layers.
OpenGL uploads remain on the render thread. A synchronous fallback prevents
holes during first entry, fast turns or teleports; slower storage can still hitch.
Decoded upload buffers are released. The road graph, collision triangles,
spatial indices, heightfield and scenery metadata still stay in CPU memory;
this is geometry paging, not complete world streaming.

The full-city pack contains **4,640 chunks**: 80,440,020 ground vertices,
20,747,568 detailed building vertices and 6,245,124 coarse building vertices,
using **2,459.3 MiB** on disk. Vertices use 24 bytes with float32 positions,
16-bit normals, 8-bit colours and an unsigned 16-bit material parameter.
Unsigned encoding preserves all 50 traffic-signal phase offsets.

Before the reference-area art pass, the October 29 development-build review
measured initial city selection at about
**18 seconds**, including loading collision metadata and validating assets.
The first render plus screenshot took about **69 ms**. One neighbourhood view
held 19 chunks (10 visible), about **31.5 MiB of GPU buffers**; temporary layer
decode peaked near **7.6 MiB**, and background cache stayed below 32 MiB.
The review process peaked around **1.65 GiB RSS**, including the initial text
loads. These are local loading/residency measurements, not a frame-rate
benchmark or a universal memory bound. World metadata paging and faster initial
loading remain the next scaling work before attempting an island-sized map.

The archive validates source/code fingerprints, ranges, size and per-layer CRCs.
Changing map or mesh source requires rebaking. The baker atomically replaces the
archive after completion; there is no silent monolithic fallback.

Graphical checks and exported neighbourhood views:

```sh
NFS_CITY_PREVIEW_DIR=artifacts/nicosia cargo test engomi_renderer_smoke -- --ignored --test-threads=1 --nocapture
```

This requires an SDL/OpenGL display. Tests exercise separated road networks,
neighbourhood coverage, nearby minimap queries, roof areas, terrain/curb support,
archive corruption, bounded prefetch and travel between distant map regions.
Human driving remains necessary to assess recognizability and road comfort.

### Engomi visual pilot

A 280 × 280 m area near the original start now uses reference-inspired facade
and rooftop details, plus local asphalt grain and paving joints. See the
[art study](#engomi-reference-area-visual-study) for scope, sources and accuracy limits.
The city-wide geometry totals above include this pilot.
The visual-pilot review peaked at about 31 MiB of temporary layer decode and
20.1 MiB of ready prefetch cache, within the existing 32 MiB cache budget.

### Optional local aerial surfaces

`data/engomi/aerial/` contains the local DLS Engomi development pack, covering about
445 × 445 m. It is projected onto the actual road, pavement and terrain meshes,
with the approved darker roads. The rest of the urban map uses procedural
materials. See [runtime imagery notes](#aerial-surfaces-runtime-integration-and-previews) for loading,
rebuilding, coverage and provenance. Imagery redistribution rights have not been
established; omit this optional local pack from distribution unless authorised.
The game works without it. There is no network access during play.

## Nicosia urban-area elevation

Source: [Mapzen Terrain Tiles](https://registry.opendata.aws/terrain-tiles/),
[Skadi N35E033.hgt.gz](https://elevation-tiles-prod.s3.amazonaws.com/skadi/N35/N35E033.hgt.gz).
The saved tile is a 3601 × 3601 signed big-endian HGT raster, north to south,
in metres above EGM96 sea level. Native spacing is one arc-second.

Attribution: Mapzen; terrain data courtesy of USGS; European terrain produced
using Copernicus data and information funded by the European Union (EU-DEM).
This is a composite dataset; individual source coverage is not inferred here.
See the provider's [source attribution and licensing](https://github.com/tilezen/joerd/blob/master/docs/attribution.md)
and [format documentation](https://github.com/tilezen/joerd/blob/master/docs/formats.md).

`data/engomi/elevation/heightfield.json` records the downloaded tile SHA-256, source URL, local datum,
processing, and generated data hashes. `data/engomi/elevation/heightfield.txt` is the offline game
asset. Rebuild a location with the native `add-location` command and
`--elevation data/engomi/elevation/N35E033.hgt.gz`; see the [map instructions](#nicosia-urban-area-offline-driving-map).
Nothing is downloaded when importing or running the game.

Heights receive a 3 × 3 Gaussian filter at native resolution, then resampling
onto a 20 m local grid. Heights are not exaggerated. Void samples cause import
failure. The heightfield is bound to the city database by CRC to reject stale
packages. Roads are clipped to the terrain triangles, so junctions and curbs
share precisely the same ground surface as vehicle contact.

## Engomi reference-area visual study

The first visual pass covers building centres within ±140 m of the original
Engomi origin (35.162 N, 33.320 E), for buildings below 22 m. This is a small
280 × 280 m neighbourhood sample, approximately 79 buildings, near the original
city start. The rest of the urban map retains the previous building treatment.
Choose Nicosia Urban Area and start driving to review the neighbourhood.

OSM supplies the footprints and tagged/estimated heights. Details in this pass
are **procedural architectural interpretations**, not an as-built reconstruction
of these addresses. They deliberately avoid invented business names or claims
that a particular rooftop installation exists on a mapped building.

### Visual references

- [Fereos Architects, Residence in Engomi (1973)](https://www.fereos.net/project/residence-in-engomi-nicosia-2/):
  historical exterior photography and the architect's description of deep
  balconies, verandas, overhangs, pale plaster and stone-clad vertical elements.
  The downloaded photograph in `artifacts/engomi-reference` is a local study
  reference, not a licensed game texture or part of the distributable map.
- [Antonis Zenios, E P I A S](https://www.antoniszenios.com/en/properties-for-sale-nicosia/e-p-i-a-s):
  contemporary Engomi apartment design, generous verandas and sliding glazing.
- [Fereos Developers, Hilltop specifications](https://www.fereosdevelopers.com/wp-content/uploads/d004-specifications-iw.pdf):
  a local example describing rooftop water storage and mechanical equipment.
  Rooftop tank/collector positions and dimensions here remain artistic estimates.

No reference imagery is sampled into the renderer. All new geometry and surface
patterns are generated by the game's own code.

### Implementation

`src/engomi_art.rs` supplies framed glazing and mullions, dark reveals, shallow
balcony fronts with rails, slab edges, parapets, shutter housings/slats, entrance
canopies, stone courses, air conditioners, water tanks and solar collectors.
Equipment is placed at a point inside a large roof triangle, with boundary
clearance; narrow roofs are left clear. Balcony fronts are a shallow suggestion
of a shaded veranda, not a structurally accurate deep balcony reconstruction.
Projections stay within 40 cm of the mapped footprint, inside the existing car
collision margin. Facade piers and windows do not alter the building collision
footprints. Slab edges, parapets and rooftop equipment remain in both building
LODs; window/rail details disappear with the near-detail layer.

The city fragment shader adds subtle world-space asphalt grain and 60 cm paving
joints only within the pilot area. These are artistic surface patterns; actual
paving layout has not been surveyed. Other locations use their previous surfaces.
Dark reveals provide local depth cues; this pass does not add sun-cast building
shadows or a general shadow-map renderer.

The new module participates in the scene archive fingerprint. Rebuild after
changing architecture rules:

```sh
cargo run -- --bake-city
```

`engomi_renderer_smoke` exports `reference-block.ppm` and `reference-facade.ppm`
in addition to the existing views. Previews live in `artifacts/engomi-reference`.
Tests check locality, geometry finiteness, collision-margin projections and roof
features surviving coarse detail. Human review is still needed for recognizability.

The next accuracy step is a set of current, address-specific photographs of this
sample, followed by explicit per-building overrides for openings, roof shape,
materials and setbacks. The current references establish a regional vocabulary;
they do not establish the appearance of every mapped building.

## Aerial surfaces: runtime integration and previews

Aerial projection is part of the normal playable city renderer. The optional
local Engomi tile pack is loaded automatically. Roads use a 0.65 colour multiplier.

### Tile loading

`data/engomi/aerial/tiles.txt` indexes 16 RGB tiles. Each tile has a 512 × 512 core
and an 8-pixel gutter containing neighbouring source pixels. The renderer reads
only the index at city selection, then reads/validates/uploads nearby tiles on
first use. CPU pixel buffers are released after each upload. Mipmaps and up to
16× anisotropic filtering preserve detail at grazing angles.

A texture-array cache reserves capacity for at most 32 tiles, additionally capped
at **32 MiB including RGBA8 mipmaps**. The current 16-tile pack reserves about
23 MiB. Nearby tiles within 568 m are prioritised by camera distance. Leaving
that area invalidates their cache slots, which can be reused; the bounded array
allocation remains reserved until the city mesh is destroyed. Returning reloads
the images; stationary frames do not reread them. Initial reads and uploads
are synchronous and can still cause a first-entry hitch.

Missing, truncated or corrupt tile payloads log a warning and use procedural
surfaces. Their filenames and CRCs are validated. A missing or malformed optional
catalog disables aerial rendering without disabling the city.

- `NFS_AERIAL_DATA=/absolute/folder` selects another compatible tile pack.
- `NFS_NO_AERIAL=1` disables aerial imagery for comparisons.
- The optional pack is resolved under the selected `NFS_CITY_DATA` directory.

Rebuild the saved local export into tiles (native Rust CLI, offline):

The command runs in the same executable as the map compiler and game.
See [source-tool options](#offline-source-preparation).

```sh
cargo run --release -- build-aerial-tiles --image artifacts/aerial-preview/engomi-2019.jpg --export artifacts/aerial-preview/export.json --output data/engomi/aerial
```

No scene-pack rebake is needed when changing only image tiles. The shader reads
world positions already stored in the mesh. The source export and provider notes
are retained beside the tile catalog. The existing image is a local development
input; redistribution permission has not been established. Do not treat it as an
openly licensed part of a distributable game. The engine also works without it.

### Source and alignment

Source: Cyprus Department of Lands and Surveys,
[Orthoimagery 2019 WebMercator service](https://eservices.dls.moi.gov.cy/arcgis/rest/services/BASEMAPS/Orthoimagery_2019_WebMercator/MapServer).
Service description: `Cached OI 2019`. Retrieved October 29, 2026.
The service name identifies the imagery collection; individual capture date
and native resolution were not independently established.

A small public map export was requested around latitude 35.162, longitude
33.320. The response supplied an adjusted EPSG:3857 extent and a 2048 × 2048
image. This is approximately 445 m across, with exported ground sampling near
22 cm/pixel; this is export sampling, not a claim about native aerial resolution.
The service metadata, export response and source JPEG are saved under
`artifacts/aerial-preview`. The aerial image is a local experiment input;
permission to redistribute it as part of the game has not been established.
It is installed locally under `data/engomi/aerial` for play; the imagery pack is optional.

The returned Web Mercator bounds are converted into the game's local coordinates.
The runtime shader derives texture UVs from each fragment's world XZ position,
using those local bounds over this small crop. It samples the image on upward
terrain, road and pavement fragments. Road-class surfaces receive a 0.65 colour multiplier to darken the aerial
asphalt while retaining its variation; pavement and other ground colours are
unchanged. Vertical curb faces retain their concrete
material; white procedural road markings are retained. Buildings and trees keep
their own materials. There is no extra covering plane in this mode and no
changes to surface geometry or vehicle support. The shader displays the aerial
colour without a second sun-lighting pass, preserving its captured shadows.

Texture filtering uses mipmaps, bilinear filtering and up to 16× anisotropic
filtering where supported. `alignment.png` overlays mapped road centrelines on
the source to check placement. The raw comparison still uses the regional terrain
grid alone, with geographically calculated UVs.

### Views

The current projected views have a `projected-` prefix:

- `projected-overview.png`: elevated view of the combined city.
- `projected-oblique.png`: lower overview with 3D buildings and textured road/pavement.
- `projected-chase.png`: chase camera, with the texture following the street surfaces.
- `projected-curb.png`: close, low-angle view of the physical pavement step.
- `projected-street.png`: near eye-height, without the player car.

`hybrid-` images record the earlier separate-sheet experiment; unprefixed images
record the ground-only comparison. They are not the current technique.

Existing building walls and roofs retain their procedural materials. Photograph
roof edges can remain visible around mapped footprints. Image capture dates,
roof displacement, road widths and inferred pavement geometry differ from the
OSM map. Photographic trees/shadows coexist with 3D vegetation. There is no
masking or cleanup. The preview deliberately retains raw image content; fine
asphalt/paving detail textures and accurate surface-specific alignment are not
included. Geometry projection preserves shape but cannot fix those image issues.

### Reproduce

The existing `image.rgb` is RGB8 data from `engomi-2019.jpg`; `georef.txt`
contains width, height and the returned Web Mercator bounds. To recreate them
from the saved JPEG/export response:

```sh
cargo run --release -- build-aerial-tiles --image artifacts/aerial-preview/engomi-2019.jpg --export artifacts/aerial-preview/export.json --output data/engomi/aerial --preview-output artifacts/aerial-preview
NFS_AERIAL_PREVIEW_DIR="$PWD/artifacts/aerial-preview" cargo test aerial_drape_preview -- --ignored --test-threads=1 --nocapture
```

The explicit test requires SDL/OpenGL and exports `projected-*.ppm`.
Set `NFS_AERIAL_RAW=1` on the same command to reproduce the ground-only comparison. The PPM files can be viewed directly in an image viewer. The capture harness is compiled only for tests; the projection and tile cache
are used by the normal game.

## Backlog

### World

[x] Use mapped green spaces, trees, parking and paving in Engomi; vary building/roof colours.

[x] First Engomi reference-area art pass: framed windows, balconies, stone piers, rooftop equipment and surface detail. See [art notes](#engomi-reference-area-visual-study).

[x] Crossable city curbs with a small bump/speed loss; drive on pavements and parking while buildings remain solid.

[x] Expand to the Nicosia urban area: 289 km², northern/southern neighbourhoods, all ground-level road components. See [map notes](#nicosia-urban-area-offline-driving-map).
[x] First expansion: chunked GPU rendering, nearby-buffer loading/unloading, and Engomi’s north/east neighbours (4.85 km²).
[x] Disk-backed city geometry paging and simpler distant buildings; render meshes no longer stay resident in CPU memory.
[x] Background geometry prefetch with a bounded 32 MiB cache; local road/minimap queries for city-scale data.
[x] Playable aerial-textured surfaces with darker roads and a bounded tile cache; initial 445 m Engomi patch. See [imagery notes](#aerial-surfaces-runtime-integration-and-previews).

[ ] Let's make Nicosia the default.

[ ] arbitrary-city downloader

[ ] Is it possible to track real-world coordinates in game so we can refer to locations?

[ ] What if we use the satellite images to infer trees?

### Cars

[x] Ratio-based RPM/shifts: original 911 901/0 for Kestrel, TREMEC Magnum for Muscle, Golf Mk1 020 for Club.

[x] Smooth progressive steering, faster centering, speed-sensitive lock and grip-limited turning.

[x] Retune weak steering: stronger short corrections and faster turn-in, with quick centering and grip limits.

[x] Remove Kestrel’s old 108 km/h power/drag limit; calibrate car-specific power and resistance for ~210 km/h on every map.

[ ] Let's make gear changes manual: A shifts up, Z shifts down, stop using WASD for movement, keep the arrows. Add a neutral between 1 and R.

[ ] Braking feels like a full wheel lock. How does NFS5 solve this?

[ ] The physics feels off, not entirely realistic. Can you describe the difference from NFS5?

[ ] Is handbrake side-sliding realistic?

[ ] Let's add Mazda Demio.

### HUD

[x] Player-centred vicinity minimap: ~400 m across, with clipped nearby roads and markers.

[x] Analog speedometer and tachometer, with redline markings and a separate gear indicator.

[ ] Need a full map view (M to open, esc to return back to the game).

### Sound

[ ] What would it take to have realistic sounds?

### Project hygiene

[x] Native Rust CLI: play, add-location from local OSM XML, list-locations and bake-city; no Python runtime required for importing and playing locations.

[x] Retire the Python map compiler after transferring regression coverage and validating a full Nicosia import. All remaining source-preparation utilities now run in the same Rust binary.
