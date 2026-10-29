# Optional source preparation

The Rust executable owns the playable map pipeline. Use `nfs5like add-location`
to compile OSM XML, streets, furniture, landscape, elevation and the render archive.
See the [project README](../README.md) for examples.

Only these optional Python utilities remain:

| Tool | Purpose | Dependency |
| --- | --- | --- |
| `extract_region.py` | Convert a regional PBF into reference-complete OSM XML | Osmium |
| `merge_osm.py` | Merge saved XML snapshots, preserving newer object versions | Python standard library |
| `build_aerial_tiles.py` | Prepare the optional saved Engomi aerial export | Pillow |

They are developer utilities and are not invoked by the executable. The aerial
utility currently projects around the historical Engomi origin; it is not an
arbitrary-location imagery importer.

```sh
python3 -m venv .venv-tools
.venv-tools/bin/pip install -r tools/requirements-optional.txt
.venv-tools/bin/python -m unittest discover -s tools -p 'test_*.py'
cargo test
```

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
