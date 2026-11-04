#!/bin/bash
# The multi-object tracker with the pedestrian persistence patch (pedtrack.patch), built for the Autoware image on the
# Autoware PC: ~/mot_build/install/autoware_multi_object_tracker/lib, mounted by run_autoware_carla.sh when PEDTRACK=1.
# The source is the upstream commit the image's tracker came from (its headers match it byte for byte).
#   bash ~/aw_overrides/mot_pedtrack/build.sh        (about a minute; idempotent)
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
COMMIT=3629ec2e44e84aceb3584dad8c248f7c280c38da
IMG=${IMG:-ghcr.io/autowarefoundation/autoware:universe-cuda-jazzy}
W=$HOME/mot_build
mkdir -p "$W/src"
STAMP="$COMMIT $(sha256sum "$HERE/pedtrack.patch" | cut -c1-16)"
LIB=$W/install/autoware_multi_object_tracker/lib/libautoware_multi_object_tracker.so
if [ -f "$LIB" ] && [ "$(cat "$W/stamp" 2>/dev/null)" = "$STAMP" ]; then echo "up to date: $LIB"; exit 0; fi
# the upstream source at that commit (a sparse clone, once), copied fresh and patched at every build
if [ ! -d "$W/universe/.git" ]; then
  rm -rf "$W/universe"
  git clone -q --filter=blob:none --no-checkout https://github.com/autowarefoundation/autoware_universe.git "$W/universe"
  git -C "$W/universe" sparse-checkout set perception/autoware_multi_object_tracker
fi
git -C "$W/universe" checkout -q -f $COMMIT
rm -rf "$W/src/autoware_multi_object_tracker" "$W/build" "$W/install"
cp -r "$W/universe/perception/autoware_multi_object_tracker" "$W/src/"
(cd "$W/src/autoware_multi_object_tracker" && patch -s -p1 < "$HERE/pedtrack.patch")
cd "$W"
docker run --rm --cpus=4 -v "$W:/ws" -w /ws "$IMG" bash -c "source /opt/autoware/setup.bash && MAKEFLAGS=-j4 nice -n 19 colcon build --base-paths src --packages-select autoware_multi_object_tracker --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF 2>&1 | tail -3; chown -R $(id -u):$(id -g) /ws"
ls -la "$LIB" && echo "$STAMP" > "$W/stamp"
