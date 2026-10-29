# Cars LOP

## Goal

A driving game, draw inspiration from NFS5: beautiful cars, pleasant roads, readable handling, mechanical personality.

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

Local imports support OSM **XML**, not PBF or Overpass JSON. Downloading locations,
multi-tile terrain and aerial-image preparation are not implemented by the native
CLI. The remaining [optional Python utilities](tools/README.md) prepare PBF/XML
sources and the saved aerial export; the replaced Python map compiler has been
retired. Rust geometry uses `geo` buffering/boolean operations and earcut
triangulation, so derived meshes need not match Shapely's triangle ordering.
Use `--help` for command syntax. `--bake-city` remains a compatibility alias.

## Backlog

### World

[x] Use mapped green spaces, trees, parking and paving in Engomi; vary building/roof colours.

[x] First Engomi reference-area art pass: framed windows, balconies, stone piers, rooftop equipment and surface detail. See [art notes](ENGOMI_ART.md).

[x] Crossable city curbs with a small bump/speed loss; drive on pavements and parking while buildings remain solid.

[x] Expand to the Nicosia urban area: 289 km², northern/southern neighbourhoods, all ground-level road components. See [map notes](data/engomi/README.md).
[x] First expansion: chunked GPU rendering, nearby-buffer loading/unloading, and Engomi’s north/east neighbours (4.85 km²).
[x] Disk-backed city geometry paging and simpler distant buildings; render meshes no longer stay resident in CPU memory.
[x] Background geometry prefetch with a bounded 32 MiB cache; local road/minimap queries for city-scale data.
[x] Playable aerial-textured surfaces with darker roads and a bounded tile cache; initial 445 m Engomi patch. See [imagery notes](AERIAL_PREVIEW.md).

[ ] Let's make Nicosia the default.

[ ] arbitrary-city downloader

[ ] Is it possible to track real-world coordinates in game so we can refer to locations?

[ ] What if we use the satellite images to infer trees?


### Cars

[x] Ratio-based RPM/shifts: original 911 901/0 for Kestrel, TREMEC Magnum for Muscle, Golf Mk1 020 for Club. See [gearbox notes](GEARBOXES.md).

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

[x] Retire the Python map compiler after transferring regression coverage and validating a full Nicosia import. Optional source-preparation utilities remain in `tools/`.
