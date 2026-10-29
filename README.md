# Cars LOP

A driving game inspired by NFS5: beautiful cars, pleasant roads, readable handling
and mechanical personality.

## Contents

- [Build and play](#build-and-play)
- [Add a city step by step](#add-a-city-step-by-step)
- [Nicosia map](#nicosia-map)
- [Optional source tools](#optional-source-tools)
- [Development](#development)
- [Credits and distribution](#credits-and-distribution)
- [Backlog](#backlog)

## Build and play

Requires Rust and the platform's SDL3/OpenGL libraries. The game and map tools
are one executable; Python is not required.

```sh
cargo build --release
./target/release/nfs5like
```

No arguments opens the menu. To select the bundled city directly:

```sh
./target/release/nfs5like play --location nicosia
```

Keep the complete `data/engomi` asset folder beside the executable when moving
the game. Map assets remain separate from the binary. Play works offline.
Use `./target/release/nfs5like --help` for command syntax.

## Add a city step by step

Run these commands from the project root. Map downloads are manual; all other
preparation uses the game executable.

### 1. Build the executable

```sh
cargo build --release
```

### 2. Prepare an OSM XML extract

Choose a city-sized bounding box in decimal degrees, ordered **south latitude,
west longitude, north latitude, east longitude**. Start with a small neighbourhood
to check the result before importing a whole city.

If you already have a complete `.osm` XML file, use it directly in step 3.
Otherwise, obtain a regional `.osm.pbf` containing your city and convert the
selected region with `extract-region`. For example, this uses the saved Cyprus
PBF to prepare the original Engomi neighbourhood:

```sh
./target/release/nfs5like extract-region data/engomi/sources/cyprus.osm.pbf data/city-import/engomi.osm --bbox 35.156 33.312 35.168 33.328
```

For another city, replace the PBF path and all four bounds. Always supply
`--bbox` for another region; the extractor's default is the Nicosia urban crop.
`add-location` accepts OSM XML, not PBF or Overpass JSON. An XML export must
contain the nodes referenced by its ways; an incomplete or timed-out export
cannot be used as a complete map.

### 3. Import and bake the city

Choose a unique location name, such as `engomi-demo`. Names must start with an
ASCII letter or digit and contain only ASCII letters, digits, hyphens or
underscores, up to 64 characters. `nicosia` is reserved for the bundled city.

For a first import, use flat terrain:

```sh
./target/release/nfs5like add-location engomi-demo --source data/city-import/engomi.osm --bbox 35.156 33.312 35.168 33.328 --flat-terrain --locations-dir data/locations
```

For real elevation instead, replace `--flat-terrain` with
`--elevation /path/to/TILE.hgt.gz`. For the Engomi example, the saved tile is
`data/engomi/elevation/N35E033.hgt.gz`. Uncompressed `.hgt` files also work.
Exactly one terrain option is required. The single HGT tile must cover the
entire crop plus 80 metres of padding, with no void samples; multi-tile terrain
is not supported. Flat terrain explicitly places the city at zero elevation.

Use the same bounds as in step 2. With a direct XML input, provide the desired
crop here; omitting `--bbox` uses the XML bounds or its full node extent, which
may be larger than the intended city.

Wait for `Added engomi-demo` before launching it. The import prepares everything
needed to play. Whole-city imports can take substantial time, memory and disk
space. Existing locations are never overwritten.

### 4. List the city and drive

```sh
./target/release/nfs5like list-locations --locations-dir data/locations
./target/release/nfs5like play --location engomi-demo --locations-dir data/locations
```

These examples store the package at `data/locations/engomi-demo`. Keep that
folder: it contains the assets the game loads when you select the city.
Use the same `--locations-dir` on subsequent commands, from the same working
directory, or use an absolute path. You can also set `NFS_LOCATIONS_DIR`.
Omit the directory option throughout to use the default user data directory
(`~/.local/share/nfs5like/locations` on Linux, or
`$XDG_DATA_HOME/nfs5like/locations` when configured).

### 5. Rebuild or update later

After renderer changes, rebuild the city’s visuals with:

```sh
./target/release/nfs5like bake-city --location engomi-demo --locations-dir data/locations
```

To change the map, crop or terrain, repeat the import with a new location name.
New cities use generated road and building materials. Aerial imagery is optional;
the current imagery tool supports the original Engomi area only.

## Nicosia map

The bundled map covers about 289 km² of Nicosia, including Engomi, Strovolos,
Lakatamia, Aglandjia and Gonyeli. Its asset folder keeps the historical name
`data/engomi`.

Roads, building footprints, green spaces and street furniture come from
OpenStreetMap. Building heights and architectural details are partly estimated.
The Engomi starting area has additional balconies, windows and rooftop details
inspired by local architecture. You can drive across low curbs onto pavements,
parking and open ground; buildings remain solid.

The map is an approximation: bridges, tunnels and some complex buildings are
omitted. Some roads remain disconnected, one-way rules are not enforced, and
there is no city traffic AI yet.

An optional local aerial pack covers about 445 × 445 m of Engomi. The game
loads it automatically from `data/engomi/aerial` when available and works without
it. Set `NFS_NO_AERIAL=1` to disable it. See the redistribution limits below.

## Optional source tools

The city guide covers PBF extraction. Two other tools are included:

```sh
# Combine saved OSM XML snapshots; gzip inputs are also accepted.
./target/release/nfs5like merge-osm original.osm.gz extension.osm merged.osm

# Prepare the saved Engomi image for use in the game.
./target/release/nfs5like build-aerial-tiles --image data/aerial-preview/engomi-2019.jpg --export data/aerial-preview/export.json --output data/engomi/aerial
```

Merging preserves newer known object versions and favours the original when
versions tie or the extension has no version metadata. Image preparation needs
a saved JPEG/PNG and its Web Mercator export metadata; it does not download imagery.

## Development

Run the regular tests with:

```sh
cargo test
```

After changing city rendering, rebuild the bundled city's visuals with:

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
