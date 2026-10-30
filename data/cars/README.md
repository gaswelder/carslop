# Car artwork

`kestrel.mesh` is the current Kestrel body, baked from the original procedural
artwork, with its exact normals, colors and paint/specular material values.
The game loads it from `data/cars/kestrel.mesh` relative to the working directory
when building scene geometry. Restart to pick up edits. Missing or invalid files
produce a warning and fall back to the procedural artwork. Wheels and physics
remain shared code.

Format version 1: eight ASCII bytes `NFSCAR01`, then a little-endian u32 vertex
count, followed by that many records of eleven little-endian f32 values:
position XYZ, color RGB, normal XYZ, material XY. Each three records form a
triangle. Coordinates are meters, Y up, +Z forward. No indices or textures.

To regenerate after changing the procedural source:

```sh
cargo test export_kestrel_mesh -- --ignored
```
