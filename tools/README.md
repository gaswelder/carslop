# Offline source preparation

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

After importing the full Nicosia crop described in the map README, run the
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
before removal because this workspace has no Git history.

The bundled `data/engomi` text assets retain their historical coordinates and
provenance. New imports use their crop centre and can produce a different
triangulation. Retiring the Python compiler does not regenerate bundled text data.
