# Nicosia urban area: offline driving map

Map data © OpenStreetMap contributors, [ODbL 1.0](https://www.openstreetmap.org/copyright).
The saved source and derived map databases are distributed under ODbL 1.0;
original procedural materials and game code are separate. Preserve attribution
when redistributing. The opening menu and city Escape menu display attribution.
The folder retains its historical `engomi` name.

## Coverage

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
`manifest.json` and `sources/manifest.json` record URLs, versions, counts and
SHA-256 hashes. `sources/cyprus.osm.pbf` and the previous compressed extract
preserve inputs. No data is downloaded at game launch.

## Appearance and driving

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
estimates. Unclassified ground remains dry soil. There is no satellite imagery.

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
See [elevation attribution](elevation/README.md).

Drive across low curbs onto pavements, parking and open ground. Curbs apply a
small directional velocity loss and cosmetic body bump; buildings and the crop
boundary remain solid. Vehicle height follows the visible supporting surface.

## Rebuild and distribution

The native executable can now compile a local OSM XML extract into a separate
playable location, including the render archive:

```sh
cargo run --release -- add-location engomi-native --source data/engomi/source.osm --bbox 35.156 33.312 35.168 33.328 --elevation data/engomi/elevation/N35E033.hgt.gz
cargo run --release -- play --location engomi-native
```

This creates an independent location in the user data directory. It uses the
crop centre as its origin; the bundled Nicosia package below retains its historical
origin. See the root README for location storage and command options.

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
Their commands are documented in [tools/README.md](../../tools/README.md).

## Rendering and loading

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

## Engomi visual pilot

A 280 × 280 m area near the original start now uses reference-inspired facade
and rooftop details, plus local asphalt grain and paving joints. See the
[art study](../../ENGOMI_ART.md) for scope, sources and accuracy limits.
The city-wide geometry totals above include this pilot.
The visual-pilot review peaked at about 31 MiB of temporary layer decode and
20.1 MiB of ready prefetch cache, within the existing 32 MiB cache budget.

## Optional local aerial surfaces

`aerial/` contains the local DLS Engomi development pack, covering about
445 × 445 m. It is projected onto the actual road, pavement and terrain meshes,
with the approved darker roads. The rest of the urban map uses procedural
materials. See [runtime imagery notes](../../AERIAL_PREVIEW.md) for loading,
rebuilding, coverage and provenance. Imagery redistribution rights have not been
established; omit this optional local pack from distribution unless authorised.
The game works without it. There is no network access during play.
