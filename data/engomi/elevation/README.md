# Nicosia urban-area elevation

Source: [Mapzen Terrain Tiles](https://registry.opendata.aws/terrain-tiles/),
[Skadi N35E033.hgt.gz](https://elevation-tiles-prod.s3.amazonaws.com/skadi/N35/N35E033.hgt.gz).
The saved tile is a 3601 × 3601 signed big-endian HGT raster, north to south,
in metres above EGM96 sea level. Native spacing is one arc-second.

Attribution: Mapzen; terrain data courtesy of USGS; European terrain produced
using Copernicus data and information funded by the European Union (EU-DEM).
This is a composite dataset; individual source coverage is not inferred here.
See the provider's [source attribution and licensing](https://github.com/tilezen/joerd/blob/master/docs/attribution.md)
and [format documentation](https://github.com/tilezen/joerd/blob/master/docs/formats.md).

`heightfield.json` records the downloaded tile SHA-256, source URL, local datum,
processing, and generated data hashes. `heightfield.txt` is the offline game
asset. Rebuild a location with the native `add-location` command and
`--elevation data/engomi/elevation/N35E033.hgt.gz`; see the [map instructions](../README.md).
Nothing is downloaded when importing or running the game.

Heights receive a 3 × 3 Gaussian filter at native resolution, then resampling
onto a 20 m local grid. Heights are not exaggerated. Void samples cause import
failure. The heightfield is bound to the city database by CRC to reject stale
packages. Roads are clipped to the terrain triangles, so junctions and curbs
share precisely the same ground surface as vehicle contact.
