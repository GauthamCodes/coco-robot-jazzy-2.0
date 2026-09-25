#!/usr/bin/env bash
# boot_probe.sh --out DIR --reps N IMAGE [IMAGE...] — how often does the
# appliance come up with every ros2_control controller ACTIVE?
#
# Each boot is a fresh container in the appliance's own mode, hardened as
# compose runs it (no capabilities, no-new-privileges), images alternated
# boot by boot so host load drifts over all of them alike. After the
# spawners have had their chance it records `ros2 control
# list_controllers`, whether the controller manager logged "Switch
# controller timed out", and the wall-clock order of the activation
# request and the Ogre2 (software rendering) initialisation, then stops.
# Refuses to start a boot while any gz sim runs on the host.
set -uo pipefail
read -r -a D <<< "${DOCKER:-docker}"
OUT=""; REPS=3; IMAGES=(); SETTLE=90
while [ $# -gt 0 ]; do
  case "$1" in
    --out) OUT=$2; shift ;;
    --reps) REPS=$2; shift ;;
    --settle) SETTLE=$2; shift ;;
    *) IMAGES+=("$1") ;;
  esac
  shift
done
[ -n "$OUT" ] && [ "${#IMAGES[@]}" -gt 0 ] || { echo "usage: --out DIR --reps N IMAGE..." >&2; exit 2; }
mkdir -p "$OUT"
echo "image,rep,diff_drive,all_active,switch_timeouts,activate_t,ogre_t,host_load1" > "$OUT/boots.csv"
for rep in $(seq 1 "$REPS"); do
  for img in "${IMAGES[@]}"; do
    if pgrep -f 'g[z] sim' >/dev/null; then echo "REFUSE: gz sim on host"; exit 3; fi
    tag=$(echo "$img" | tr '/:' '__'); d="$OUT/$tag-rep$rep"; mkdir -p "$d"
    load=$(cut -d' ' -f1 /proc/loadavg)
    cid=$("${D[@]}" run -d --network none --shm-size 2g --cap-drop ALL \
          --security-opt no-new-privileges:true "$img")
    sleep "$SETTLE"
    "${D[@]}" exec "$cid" bash -c 'source /opt/ros/jazzy/setup.bash; source /opt/coco_ws/install/setup.bash; timeout 30 ros2 control list_controllers' \
      > "$d/controllers.txt" 2>&1
    "${D[@]}" cp "$cid:/tmp/coco_sim.log" "$d/coco_sim.log" >/dev/null 2>&1
    "${D[@]}" rm -f "$cid" >/dev/null
    dd=$(awk '/^diff_drive_controller/{print $NF}' "$d/controllers.txt")
    n_active=$(grep -c ' active' "$d/controllers.txt")
    to=$(grep -c 'Switch controller timed out' "$d/coco_sim.log")
    act=$(grep -m1 'Activating controllers: \[ diff_drive_controller' "$d/coco_sim.log" | grep -oE '\[[0-9]+\.[0-9]+\]' | tr -d '[]')
    ogre=$(grep -n -m1 'Ogre2RenderEngine' "$d/coco_sim.log" | cut -d: -f1)
    actl=$(grep -n -m1 'Activating controllers: \[ diff_drive_controller' "$d/coco_sim.log" | cut -d: -f1)
    order=$([ -n "$ogre" ] && [ -n "$actl" ] && { [ "$ogre" -gt "$actl" ] && echo render_after_activate || echo render_before_activate; } || echo unknown)
    echo "$img,$rep,${dd:-missing},$([ "$n_active" = 4 ] && echo yes || echo no),$to,$act,$order,$load" | tee -a "$OUT/boots.csv"
  done
done
