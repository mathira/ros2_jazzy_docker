#!/usr/bin/env bash
set -e
export DISPLAY=:1
SCREEN_WIDTH="${SCREEN_WIDTH:-1440}"
SCREEN_HEIGHT="${SCREEN_HEIGHT:-900}"
SCREEN_SIZE="${SCREEN_WIDTH}x${SCREEN_HEIGHT}x24"

if ! pgrep -f 'Xvfb :1' >/dev/null 2>&1; then
  Xvfb :1 -screen 0 "${SCREEN_SIZE}" -ac &
  sleep 2
fi

mkdir -p /home/ros/.vnc
if [ ! -f /home/ros/.vnc/passwd ]; then
  echo 'ros' | x11vnc -storepasswd ros /home/ros/.vnc/passwd
fi
chmod 600 /home/ros/.vnc/passwd

pgrep -f xfwm4 >/dev/null 2>&1 || xfwm4 &
pgrep -f xfce4-panel >/dev/null 2>&1 || xfce4-panel &
pgrep -x x11vnc >/dev/null 2>&1 || x11vnc -display :1 -forever -shared -rfbport 5901 -rfbauth /home/ros/.vnc/passwd -o /tmp/x11vnc.log &
pgrep -f websockify >/dev/null 2>&1 || websockify --web=/usr/share/novnc/ 6080 localhost:5901 >/tmp/novnc.log 2>&1 &

# Open a real TTY for keyboard-driven ROS nodes inside the noVNC desktop.
if ! pgrep -u "$(id -u)" -f 'xfce4-terminal' >/dev/null 2>&1; then
  xfce4-terminal --disable-server --geometry=120x35 \
    --title='ROS 2 Jazzy - project terminal' \
    --command='/home/ros/setup/web-terminal.sh' >/tmp/xfce4-terminal.log 2>&1 &
fi
