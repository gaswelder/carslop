# Code review findings

Reviewed: 2026-10-30

Scope: runtime location selection/loading, geometry streaming and archives,
SQLite asset packages, car discovery/configuration, and related vehicle/audio
paths. This is a focused correctness and reliability review, not an exhaustive
audit of the importer, graphics output, or vehicle tuning.

Priority: P1 = high-impact failure; P2 = normal-priority reliability issue;
P3 = low-priority edge case. No application code was changed for this review.

## 1. [P1] A broken optional car mesh prevents the game from opening

**Location:** `src/cars.rs:514` (mesh overrides and successful catalog insertion);
`src/scene.rs:161` (geometry construction for every car);
`src/scene.rs:540` (panic on mesh loading failure).

`load_extra` validates the JSON and builds asset paths, but never checks whether
the referenced body and wheel meshes exist or can be decoded. The extra car is
therefore accepted by `extras`, bypassing its warn-and-skip handling for invalid
cars. Scene construction subsequently loads every catalog entry, including cars
the user has never selected, and `load_design` panics on the bad asset.

**Trigger:** Add a custom `car.json` with a valid base and
`"mesh": "missing.mesh"`, then start the game. A truncated or invalid mesh has
the same failure path. One optional car can disable all playable cars.

**Evidence:** Reproduced using an isolated temporary car directory and the
existing `cars::runtime_tests::runtime_catalog_probe` test. Catalog assertions
passed before geometry construction panicked at `src/scene.rs:542` with
`Cannot load required car design data/cars/new/missing.mesh` (exit 101).

**Suggested fix:** Validate both meshes before admitting an optional car to the
catalog, returning an error through the existing skip-and-warn path. Preserve
strict failure for genuinely required base-car assets.

**Regression check:** A missing body mesh, missing wheel mesh, and corrupt mesh
should each exclude only the optional car; building the remaining geometry
should succeed.

## 2. [P2] Location loading turns recoverable asset errors into application panics

**Location:** `src/loading.rs:77`; related paths: `src/city.rs:96`,
`src/city_assets.rs:75`, and `src/main.rs:333`.

The loading worker returns an error when the scene archive cannot be opened,
but the main thread calls `result.expect("failed to load location")`. Map parsing
and missing map assets also panic inside `City::engomi`, and the join handler
explicitly resumes those panics. The loading function's boolean result cannot
represent an ordinary load failure.

Discovery admits any non-hidden `.sqlite` file, or a folder containing
`city.txt`, without checking that all runtime assets are present. Consequently,
incomplete or damaged user locations can appear in the menu and terminate the
game when selected.

**Trigger:** Select a discovered location with a missing/invalid `scene.pack`,
or missing required map assets.

**Evidence:** Confirmed by tracing discovery, worker error propagation, and the
main-thread join handler. The SDL/OpenGL interaction was not executed.

**Suggested fix:** Propagate a `Result` through map construction and loading;
show the failure and return to a usable menu. Keep the current location and its
mesh until the replacement has loaded successfully. Merely catching the worker
panic is insufficient: construction currently occurs while the shared track
cache mutex is held, so that panic can poison the cache.

**Regression check:** Selecting an incomplete location should report an error,
then permit starting a valid location in the same process.

## 3. [P2] Visiting different cities permanently retains every loaded city

**Location:** `src/city_assets.rs:14`; related paths: `src/track.rs:90`
and `src/main.rs:341`.

The shared cache stores a separate `&'static Track` for each location path using
`Box::leak`. Each track owns its city geometry inputs, collision surfaces,
spatial indexes, elevation data, and landscape. Switching locations releases
the old city GPU mesh, but never releases those CPU allocations. Memory grows
with the total size of all distinct cities visited during the session, rather
than the active city.

**Trigger:** Start several different large imported locations without restarting
the application. Returning to a built-in route does not free those city maps.

**Evidence:** Confirmed from ownership and lifetime analysis; no memory benchmark
was performed. Repeated visits to the same path reuse its entry; growth is per
distinct loaded path.

**Suggested fix:** Use owned or `Arc<Track>` session references with a bounded
cache, allowing inactive cities to be evicted once no session uses them.

**Regression check:** After switching through more cities than the cache limit,
inactive city allocations should be reclaimable while the active route remains
valid.

## 4. [P3] Accepted gearbox size conflicts with the neutral gear sentinel

**Location:** `src/cars.rs:202` and `src/cars.rs:489`; related paths:
`src/vehicle.rs:6`, `src/vehicle.rs:151`, and `src/vehicle.rs:176`.

Both original and custom car parsers accept up to 255 forward ratios. Vehicle
state reserves the value 255 (`u8::MAX`) for neutral. With 255 descending,
positive ratios, shifting up from gear 254 selects 255, which the rest of the
vehicle code interprets as neutral rather than the final forward gear. Another
upshift then selects first gear.

**Trigger:** A valid car configuration with exactly 255 descending ratios;
shift up from gear 254. This is an unusual configuration but explicitly accepted
by both parsers.

**Evidence:** Confirmed by comparing parser bounds with the shift state machine
and neutral-dependent engine/gear-label branches; not exercised dynamically.

**Suggested fix:** Reject ratio counts above 254 in both parsers, or represent
neutral separately from the forward gear number.

**Regression check:** Exercise the accepted maximum ratio count and ensure the
next count is rejected, or verify that the final forward gear remains distinct
from neutral with a revised representation.

## Validation and limits

- Confirmed the current test target builds with `cargo test --no-run`.
- Reproduced finding 1 in a temporary fixture without modifying repository data.
- Findings 2–4 are supported by code-path/lifetime analysis as described above.
- The preceding test-cleanup run passed 196 tests with 11 ignored; the full suite
  was not rerun for this documentation-only review.
- Graphics smoke tests, interactive loading, and memory profiling were not run.
