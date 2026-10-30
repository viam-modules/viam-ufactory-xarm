#!/usr/bin/env bash
# Starts viam-server on localhost:18080 with a stand-in of the workcell: fake xArm6 on the
# mounting plate, the gripper's default collision boxes, table, walls, and ceiling.
# Usage: cmd/scenecheck/replica/run.sh [joint degrees x6]   (default 0 -30 -30 0 60 0)
set -euo pipefail
root=$(git rev-parse --show-toplevel)
work=$(mktemp -d)
go build -C "$root" -o "$work/viam-xarm" .
go build -C "$root" -o "$work/setjoints" ./cmd/scenecheck/replica/setjoints
sed "s|MODULE_PATH|$work/viam-xarm|" "$root/cmd/scenecheck/replica/replica.json" > "$work/replica.json"
viam-server -config "$work/replica.json" > "$work/server.log" 2>&1 &
server=$!
trap 'kill $server 2>/dev/null' EXIT
until grep -q serving "$work/server.log" 2>/dev/null; do sleep 1; done
sleep 3
if [ $# -eq 0 ]; then set -- 0 -30 -30 0 60 0; fi
"$work/setjoints" "$@"
echo "replica up (log: $work/server.log); ctrl-c to stop"
wait $server
