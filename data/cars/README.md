# Car assets

Kestrel assets live together under `data/cars`: `kestrel.mesh` is the body and
`kestrel/audio/` holds its engine loops, original recordings, and sound credits.

`kestrel.mesh` is the current Kestrel body, baked from the original procedural
artwork, with its exact normals, colors and paint/specular material values.
The game loads it from `data/cars/kestrel.mesh` relative to the working directory
when building scene geometry. Restart to pick up edits. Missing or invalid files
produce a warning and fall back to the procedural artwork. Wheels and physics
remain shared code.

Audio loads the three mono 48 kHz signed 16-bit little-endian `.pcm` files from
`kestrel/audio/` once at startup, relative to the working directory. Restart the
game after replacing them; rebuilding is unnecessary. If any loop is missing
or invalid, the game warns and uses its procedural engine voice. A valid loop
has an even byte count, more than twice its overlap in samples (2242 idle,
2068 load, 2064 coast), and at most 9,600,000 bytes. Raw PCM has no header, so
the sample rate and encoding must match. Keep all of `data/cars` with packaged
builds. Mesh and audio fallbacks operate independently.

See [Kestrel sound credits and conversion commands](kestrel/audio/README.md).

Format version 1: eight ASCII bytes `NFSCAR01`, then a little-endian u32 vertex
count, followed by that many records of eleven little-endian f32 values:
position XYZ, color RGB, normal XYZ, material XY. Each three records form a
triangle. Coordinates are meters, Y up, +Z forward. No indices or textures.

To regenerate after changing the procedural source:

```sh
cargo test export_kestrel_mesh -- --ignored
```
