#!/usr/bin/env bash
# Commands used to rebuild the required runtime collision data for both maps.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

cargo build --release --offline --target-dir ./target
./target/release/nfs5like bake-collision --location nicosia
./target/release/nfs5like circuit-only --location nurburgring
./target/release/nfs5like bake-collision --location nurburgring
