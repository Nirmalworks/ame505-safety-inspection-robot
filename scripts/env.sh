# Source this in every terminal:  source ~/ame505-safety-inspection-robot/scripts/env.sh
# (zsh or bash). Loads ROS 2 + Hiwonder env + this workspace, and fixes device nodes
# that Docker loses: /dev/lidar and the Xbox controller (USB or Bluetooth).
export AME505_ROOT="${AME505_ROOT:-$HOME/ame505-safety-inspection-robot}"
# Large data on the SSD bind-mount (survives container rebuilds)
export AME505_DATA="${AME505_DATA:-/home/ubuntu/share/tmp/ame505_data}"
# Depth camera lives in Hiwonder's ROS 1 container; this bridge republishes it in ROS 2
export AME505_ASTRA_BRIDGE="${AME505_ASTRA_BRIDGE:-$HOME/jetauto_toolkit/scripts/camera/astra_bridge.py}"
# SDL fix: over Bluetooth the Xbox Series pad's LB is otherwise read as BACK
export SDL_GAMECONTROLLERCONFIG='050000005e040000130b000009050000,Xbox Series X Controller,a:b0,b:b1,x:b3,y:b4,leftshoulder:b6,rightshoulder:b7,back:b10,start:b11,guide:b12,leftstick:b13,rightstick:b14,leftx:a0,lefty:a1,rightx:a2,righty:a3,lefttrigger:a5,righttrigger:a4,dpup:h0.1,dpright:h0.2,dpdown:h0.4,dpleft:h0.8,misc1:b15,platform:Linux,'

[ -z "$need_compile" ] && source "$HOME/ros2_ws/.robotrc" >/dev/null
if [ -n "$ZSH_VERSION" ]; then
  [ -f "$AME505_ROOT/install/local_setup.zsh" ] && source "$AME505_ROOT/install/local_setup.zsh"
else
  [ -f "$AME505_ROOT/install/local_setup.bash" ] && source "$AME505_ROOT/install/local_setup.bash"
fi

ame505_fix_devices() {
  # Gamepads paired/plugged after the container started exist in /sys but not /dev.
  local s n mm
  for s in /sys/class/input/js* /sys/class/input/event*; do
    [ -e "$s/dev" ] || continue
    n="/dev/input/$(basename "$s")"
    if [ ! -e "$n" ]; then
      mm=$(cat "$s/dev"); sudo -n mknod -m 666 "$n" c "${mm%%:*}" "${mm##*:}" 2>/dev/null
    fi
  done
  sudo -n chmod a+rw /dev/input/js* /dev/input/event* 2>/dev/null
  # /dev/lidar: the RPLidar is the CH340 on hub port 4 (path ends .4:1.0)
  if [ ! -e /dev/lidar ]; then
    for s in /sys/class/tty/ttyUSB*; do
      if readlink -f "$s/device" | grep -qE '\.4:1\.0/ttyUSB'; then
        sudo -n ln -sf "$(basename "$s")" /dev/lidar && echo "[ame505] linked /dev/lidar -> $(basename "$s")"
        break
      fi
    done
  fi
}
ame505_fix_devices

# VS Code / tools need many file watches; the kernel default (8192) makes VS Code's
# Explorer stop refreshing ("ENOSPC"). Resets on reboot, so re-apply here.
if [ "$(cat /proc/sys/fs/inotify/max_user_watches 2>/dev/null || echo 0)" -lt 524288 ]; then
  sudo -n sysctl -qw fs.inotify.max_user_watches=524288 fs.inotify.max_user_instances=512 2>/dev/null || true
fi

# VS Code (+ Pylance + Claude) on the robot takes ~1.3 GB RAM and ~50% CPU and runs at normal
# priority, so Nav2/AMCL lose CPU to the editor (load average ~20 seen). Lower the editor's
# priority so the robot software wins. Harmless: the editor just gets a bit slower.
ame505_deprioritize_editor() {
  local p
  for p in $(pgrep -f '/usr/share/code/|claude-code.*/claude|vscode-pylance' 2>/dev/null); do
    renice -n 10 -p "$p" >/dev/null 2>&1
  done
}
true
