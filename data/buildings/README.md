# Architectural parts

These static glTF 2.0 GLBs supply windows, entrances, and two balcony variants.
`parts.json` selects the models, declares their native dimensions and allowed
scale axes, and controls spacing and balcony placement. The building shell still
follows its mapped footprint and height. Models are loaded once per building
layer during location baking; play uses the resulting `scene.pack`.

## Editing

Edit a GLB in Blender or another glTF editor and export static triangle meshes
with one embedded buffer. Use metres, Y up, local X along the facade and local
+Z outward. Put X=0 at the left edge and Z=0 at the facade attachment plane.
For windows and entrances, Y=0 is the bottom; for balconies, Y=0 is the slab's
walking surface (the slab extends below it). Native `size` describes the intended
fitting dimensions, including slab thickness for balcony height. Models may have
small projections beyond the fitting box, such as the top rail.

`scale_axes` controls whether each X/Y/Z dimension can be stretched to fit a
placement. Windows allow width and height scaling; entrances retain native size;
balconies allow width and depth scaling while retaining their authored height.
Changes to placement rules apply to the next bake.

Nodes named `wall`, `frame`, `glass`, `trim`, and `shutter` multiply their authored
colors by the corresponding building tint. Use white for full tint replacement.
Other node names retain their authored colors (for example `stone`, `rail`, and
`railing_glass`). Name each mesh node directly; names on parent groups do not
propagate. Node transforms are supported. Use outward normals/winding, opaque
materials, and solid base colors or vertex colors. Textures, transparency,
animation, skinning, morph targets, and required extensions are unsupported.
The material extra `{"carslop_material": [0, gloss]}` preserves world geometry's
fixed color with specular strength from 0 to 1; use it for opaque glazing.

The initial assets are original geometry, reproducible with:

```sh
python3 tools/build_building_parts.py
```

This overwrites the four GLBs; it leaves `parts.json` untouched. Regenerating is
optional: the GLBs are the inputs to the location build and can be edited directly.
After editing, rebuild a location with:

```sh
cargo run -- bake-scene --location nicosia
```
