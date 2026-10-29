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

Running without arguments opens the menu. To drive in Nicosia:

```sh
./target/release/nfs5like play --location nicosia
```

Keep the complete `data/engomi` folder beside the executable when moving the
game. Play works offline.
Use `./target/release/nfs5like --help` for command syntax.

## Add a city

Build the game first, then run these commands from the project root.

### 1. Prepare the map

Obtain a regional `.osm.pbf` file and extract your city. This example uses the
saved Cyprus map to extract Engomi:

```sh
./target/release/nfs5like extract-region data/engomi/sources/cyprus.osm.pbf data/city-import/engomi.osm --bbox 35.156 33.312 35.168 33.328
```

Replace the input path and bounds for another city. Bounds are decimal degrees
in **south, west, north, east** order. Start with a small neighbourhood.
If you already have a complete `.osm` XML export, skip extraction.
The importer requires XML with complete node references; PBF and Overpass JSON
cannot be imported directly.

### 2. Import the city

Use a unique name (`nicosia` is reserved) and the same bounds as above:

```sh
./target/release/nfs5like add-location engomi-demo --source data/city-import/engomi.osm --bbox 35.156 33.312 35.168 33.328 --elevation data/engomi/elevation/N35E033.hgt.gz --locations-dir data/locations
```

Names may contain English letters, digits, hyphens and underscores, must start
with a letter or digit, and can be up to 64 characters.

An elevation tile is required. Replace the example tile with one covering your
city; `.hgt` and `.hgt.gz` are supported. The tile must cover the crop plus 80 m
of padding and contain no missing elevation samples. Multi-tile terrain is not
yet supported.

Importing prepares everything needed to play. Wait for `Added engomi-demo`;
large cities take more time, memory and disk space. Existing names are never
overwritten. Without `--bbox`, the importer uses the XML bounds or full node extent.

### 3. Drive

```sh
./target/release/nfs5like list-locations --locations-dir data/locations
./target/release/nfs5like play --location engomi-demo --locations-dir data/locations
```

Keep `data/locations/engomi-demo` and use the same `--locations-dir` for later
commands. An absolute path avoids dependence on your working directory.
You can also set `NFS_LOCATIONS_DIR`. Omitting both uses the user data directory:
`$XDG_DATA_HOME/nfs5like/locations` or `~/.local/share/nfs5like/locations` on Linux.

To change the map, bounds or terrain, import again under a new name. After
renderer changes, rebuild an existing city's visuals with:

```sh
./target/release/nfs5like bake-city --location engomi-demo --locations-dir data/locations
```

New cities use generated materials. The optional imagery tool currently supports
the original Engomi area only.

## Nicosia map

The bundled map covers about 289 km² of Nicosia, including Engomi, Strovolos,
Lakatamia, Aglandjia and Gonyeli. Assets live in `data/engomi`.

OpenStreetMap supplies roads, building footprints, green spaces and street
furniture. Heights and architectural details are partly estimated, with extra
detail around the Engomi starting area. Low curbs, pavements and parking areas
are drivable; buildings remain solid.

Bridges, tunnels and some complex buildings are omitted. Some roads remain
disconnected. One-way rules are not enforced, and city traffic AI is not yet available.

An optional aerial pack covers a 445 × 445 m area of Engomi. It loads from
`data/engomi/aerial` when present; set `NFS_NO_AERIAL=1` to disable it.
See [redistribution limits](#credits-and-distribution) before sharing it.

## Optional source tools

The city guide covers PBF extraction. Two other tools are included:

```sh
# Combine saved OSM XML snapshots; gzip inputs are also accepted.
./target/release/nfs5like merge-osm original.osm.gz extension.osm merged.osm

# Prepare the saved Engomi image for use in the game.
./target/release/nfs5like build-aerial-tiles --image data/aerial-preview/engomi-2019.jpg --export data/aerial-preview/export.json --output data/engomi/aerial
```

Merging keeps newer versions and favours the original on ties or when the
extension has no version metadata. Image preparation requires a saved JPEG/PNG
and its Web Mercator export metadata.

## Development

Run tests:

```sh
cargo test
```

Rebuild the bundled city's visuals after renderer changes:

```sh
./target/release/nfs5like bake-city
```

## Credits and distribution

Map data © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright).
The saved source and derived map databases are distributed under ODbL 1.0;
game code and original generated materials are separate. Preserve attribution
and source provenance when sharing city assets. The bundled map uses the
[Geofabrik Cyprus extract](https://download.geofabrik.de/europe/cyprus.html);
source records are in `data/engomi/manifest.json` and `data/engomi/sources/manifest.json`.

Elevation: [Mapzen Terrain Tiles](https://registry.opendata.aws/terrain-tiles/),
using the [Skadi N35E033 tile](https://elevation-tiles-prod.s3.amazonaws.com/skadi/N35/N35E033.hgt.gz).
Terrain data courtesy of USGS; European terrain produced using Copernicus data
and information funded by the European Union (EU-DEM). See the provider's
[source attribution and licensing](https://github.com/tilezen/joerd/blob/master/docs/attribution.md).
Local source records are in `data/engomi/elevation/heightfield.json`.

Optional imagery: Cyprus Department of Lands and Surveys,
[Orthoimagery 2019 WebMercator service](https://eservices.dls.moi.gov.cy/arcgis/rest/services/BASEMAPS/Orthoimagery_2019_WebMercator/MapServer).
Redistribution permission for the local imagery pack has not been established;
omit `data/engomi/aerial` from distributions unless authorised.

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

- [ ] Let's make gear changes manual: A shifts up, Z shifts down, stop using WASD for movement, keep the arrows. Add a neutral between 1 and R.
- [ ] Braking feels like a full wheel lock. How does NFS5 solve this?
- [ ] The physics feels off, not entirely realistic. Can you describe the difference from NFS5?
- [ ] Is handbrake side-sliding realistic?
- [ ] Let's add Mazda Demio.

### HUD

- [ ] Need a full map view (M to open, esc to return back to the game).

### Sound

- [ ] What would it take to have realistic sounds?
