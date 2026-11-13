#!/usr/bin/env bash
# Save a debug bundle of the latest robot runs, to analyse later (e.g. on a laptop via GitHub).
#
#   ./scripts/save_debug_bundle.sh            # last 6 launch logs
#   ./scripts/save_debug_bundle.sh 12 "robot spun at goal near door"   # N logs + a note
#
# Writes two copies:
#   $AME505_DATA/debug/<time>/   FULL logs (SSD, never in git)
#   debug_logs/<time>/           TRIMMED text for git: per log a summary of warnings/errors
#                                (deduplicated, counted) + the last 300 lines; the 2D maps,
#                                checkpoints, homes and a system snapshot (live topics/rates if
#                                the robot software is running).
# Then:  git add debug_logs && git commit -m "debug: ..." && git push
set -o pipefail   # (no -u: ROS setup scripts use unset variables)
cd "$(dirname "$0")/.." || exit 1
source scripts/env.sh >/dev/null 2>&1
N="${1:-6}"
NOTE="${2:-}"
TS="$(date +%Y%m%d_%H%M%S)"
FULL="$AME505_DATA/debug/$TS"
GIT="debug_logs/$TS"
mkdir -p "$FULL" "$GIT/logs" "$GIT/maps_2d"

LOGS=$(ls -t "$AME505_DATA"/logs/*.log 2>/dev/null | head -n "$N")
for f in $LOGS; do cp "$f" "$FULL/"; done

python3 - "$GIT/logs" $LOGS <<'EOF'
import collections, os, re, sys
out = sys.argv[1]
ansi = re.compile(r'\x1b\[[0-9;]*m')
num = re.compile(r'\d+(\.\d+)?')
for path in sys.argv[2:]:
    lines = [ansi.sub('', l.rstrip('\n')) for l in open(path, errors='replace')]
    keys = collections.OrderedDict()
    for l in lines:
        if re.search(r'ERROR|WARN|FATAL|Traceback|Exception|abort|fail|Timed out|died|exit code', l, re.I):
            k = num.sub('#', l)[:220]                    # same message with other numbers = one line
            if k not in keys:
                keys[k] = [0, l[:300]]
            keys[k][0] += 1
    base = os.path.basename(path)[:-4]
    with open(os.path.join(out, base + '.summary.txt'), 'w') as f:
        f.write(f'{base}: {len(lines)} lines, {len(keys)} distinct warning/error messages\n\n')
        for k, (n, ex) in sorted(keys.items(), key=lambda kv: -kv[1][0])[:150]:
            f.write(f'{n:6d} x  {ex}\n')
    with open(os.path.join(out, base + '.tail.txt'), 'w') as f:
        f.write('\n'.join(l[:400] for l in lines[-300:]) + '\n')
EOF

cp "$AME505_DATA"/maps_2d/*.yaml "$AME505_DATA"/maps_2d/*.pgm "$GIT/maps_2d/" 2>/dev/null
for d in checkpoints home; do
  ls "$AME505_DATA/$d"/*.yaml >/dev/null 2>&1 && mkdir -p "$GIT/$d" && cp "$AME505_DATA/$d"/*.yaml "$GIT/$d/"
done

{
  echo "time: $(date)"; echo "note: $NOTE"
  echo "git: $(git rev-parse --short HEAD 2>/dev/null) $(git status --short | wc -l) changed files"
  # Which Jetson and host system this is. Run inside the container, so the host-only files may be hidden.
  echo; echo "== board"
  m=$(cat /proc/device-tree/model 2>/dev/null | tr -d '\0'); echo "model:   ${m:-not visible here}"
  echo "kernel:  $(uname -r) $(uname -m)   (4.9-tegra = JetPack 4 / host Ubuntu 18.04, 5.10 = JetPack 5, 5.15 = JetPack 6)"
  l=$(head -1 /etc/nv_tegra_release 2>/dev/null); echo "l4t:     ${l:-not visible here (host file)}"
  echo "os here: $(. /etc/os-release 2>/dev/null && echo "$PRETTY_NAME")"
  echo "cpu:     $(nproc) cores, $(grep -m1 -iE 'model name|CPU part' /proc/cpuinfo | sed 's/.*: //')"
  echo; echo "== memory"; free -m
  echo; echo "== disk"; df -h "$AME505_DATA" | tail -1
  echo; echo "== robot processes"; pgrep -af 'ros2 launch|dashboard|amcl|slam_toolbox|rtabmap|component_container' | cut -c1-200
  if pgrep -f 'ros2 launch' >/dev/null; then
    echo; echo "== topics"; timeout 10 ros2 topic list --no-daemon 2>/dev/null
    for t in /scan /odom /odom_raw /imu /amcl_pose /map /cmd_vel /depth_cam/rgb/image_raw; do
      printf '%-28s ' "$t"; timeout 6 ros2 topic hz "$t" 2>/dev/null | grep -m1 'average rate' || echo 'no data'
    done
    echo; echo "== TF map -> base_footprint"
    timeout 5 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -m3 -E 'Translation|Rotation|RPY|Invalid|exist'
  fi
} > "$GIT/system.txt" 2>&1
cp "$GIT/system.txt" "$FULL/"
[ -n "$NOTE" ] && echo "$NOTE" > "$GIT/NOTE.txt"

du -sh "$GIT" "$FULL" | sed 's/^/[ame505] /'
echo "[ame505] Full logs on SSD: $FULL"
echo "[ame505] For GitHub:       git add $GIT && git commit -m 'debug: $TS' && git push"
