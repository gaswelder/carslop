# Aerial surfaces: runtime integration and previews

Aerial projection is part of the normal playable city renderer.

optional local Engomi tile pack is loaded automatically.

Roads retain have 0.65 colour multiplier.


## Tile loading

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
See [source-tool options](tools/README.md).

```sh
cargo run --release -- build-aerial-tiles --image artifacts/aerial-preview/engomi-2019.jpg --export artifacts/aerial-preview/export.json --output data/engomi/aerial
```

No scene-pack rebake is needed when changing only image tiles. The shader reads
world positions already stored in the mesh. The source export and provider notes
are retained beside the tile catalog. The existing image is a local development
input; redistribution permission has not been established. Do not treat it as an
openly licensed part of a distributable game. The engine also works without it.

## Source and alignment

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

## Views

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

## Reproduce

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
