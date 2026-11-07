#!/usr/bin/env bash
# Build locations and their runtime assets.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

cargo build --release --offline --target-dir ./target
./target/release/nfs5like bake-collision --location nicosia
./target/release/nfs5like circuit-only --location nurburgring
./target/release/nfs5like bake-collision --location nurburgring

if [[ ! -f data/locations/scenic/city.txt ]]; then
    ./target/release/nfs5like create-track data/locations/scenic.track.json
else
    ./target/release/nfs5like bake-city --location scenic
fi
