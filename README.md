# Cars LOP

Experimental project: have an AI create a decent game and not even read the code.

Initially, every commit message was a one-line "ask" to the coding agent.
Everything else, including README.md outside of this section, was generated.

After 148 initial commits, the result is pretty decent, but one-line asks don't achieve their goals anymore.
I had to switch to longer negotiations to get improvements.
Also, I had to start looking into the code because generated technical decisions became idiotic as the slop level skyrocketed.

<img src="screenshot.20261104.jpg">

In-game screenshot at [Engomi, Nicosia](https://www.google.com/maps/search/?api=1&query=35.1670537,33.3169556)

## Controls

| Key                 | Action                               |
| ------------------- | ------------------------------------ |
| Up / Down           | Accelerate / brake                   |
| Left / Right        | Steer                                |
| A / Z               | Shift up / down                      |
| Space               | Handbrake                            |
| C                   | Cycle cameras                        |
| Esc                 | Pause / resume                       |
| R                   | Restart                              |
| H                   | Recover the car                      |
| M                   | Open / close the full map            |
| Mouse wheel / + / - | Zoom the full map in / out           |
| L                   | Return to location and car selection |
| V                   | Toggle audio mute                    |

## GLB models of the cars

Export static triangulated glTF 2.0 GLB: metres, Y up, +Z forward, one embedded
buffer and opaque materials. Name parts `body`, `wheel_rl`, `wheel_rr`,
`wheel_fl`, `wheel_fr`. Left/right wheels use negative/positive X.
Wheel origins are axle mounts; wheels spin about X and steer about Y.
Match JSON wheelbase and rolling radius to the artwork.

Use vertex colours or solid PBR materials; textures, transparency, animation,
skinning, morphs, sparse accessors and external buffers are unsupported.
Material extras `{"carslop_material": [CODE, GLOSS]}` use gloss 0–1 and codes:
0 fixed colour, 1 paint, 2 glass, 3 metal, 4 optical lens, 5 brake lens,
6 reverse lens. Normals and winding must face outward.

## Credits and distribution

- **Maps:** © [OpenStreetMap contributors](https://www.openstreetmap.org/copyright),
  ODbL 1.0; extracts from Geofabrik ([Cyprus](https://download.geofabrik.de/europe/cyprus.html),
  [Rheinland-Pfalz](https://download.geofabrik.de/europe/germany/rheinland-pfalz.html)).
  Preserve attribution and source provenance when sharing derived map databases.
- **Elevation:** [Mapzen/Skadi](https://registry.opendata.aws/terrain-tiles/),
  USGS and EU-funded Copernicus EU-DEM; [source attribution and licenses](https://github.com/tilezen/joerd/blob/master/docs/attribution.md).
- **Imagery:** [Esri World Imagery](https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer)
  — Esri, Vantor, Earthstar Geographics and the GIS User Community;
  pilot imagery from [Cyprus DLS Orthoimagery 2019](https://eservices.dls.moi.gov.cy/arcgis/rest/services/BASEMAPS/Orthoimagery_2019_WebMercator/MapServer).
  Redistribution permission is unconfirmed; do not redistribute imagery or
  derived assets without authorization.
- **Engine audio:** Pole Position Production's [1981 Porsche 911 SC recording](https://pole.se/the-iconic-sound-of-a-porsche-911-sc-from-1981/),
  `Porsche_911_SC_-81_t5_onboard_driving_constant_rpms_exhausts.wav`,
  from [Sonniss GDC 2016](https://sonniss.com/gameaudiogdc/), under the
  [Sonniss GDC bundle license](https://sonniss.com/gdc-bundle-license/).
  Modified load/coast excerpts are licensed game content, separate from the
  code license; distribute within the game, never as standalone recordings,
  sample packs, sound libraries or project templates.
