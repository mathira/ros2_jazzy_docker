#!/usr/bin/env bash
# Starts an X server (Xvfb) with a lightweight desktop, x11vnc and noVNC.
set -e

export DISPLAY=:1

# Start a virtual X server if not already running
if ! pgrep -f "Xvfb :1" > /dev/null 2>&1; then
    Xvfb :1 -screen 0 1920x1080x24 -ac &
    sleep 2
fi

# Ensure VNC password exists (default: ros)
mkdir -p /home/ros/.vnc
if [ ! -f /home/ros/.vnc/passwd ]; then
    echo "ros" | x11vnc -storepasswd "ros" /home/ros/.vnc/passwd
fi
chmod 600 /home/ros/.vnc/passwd

# Start a window manager so the desktop is usable
if ! pgrep -f "xfwm4" > /dev/null 2>&1; then
    xfwm4 &
    xfce4-panel &
fi

# Start x11vnc (VNC server on 5901)
if ! pgrep -f "x11vnc" > /dev/null 2>&1; then
    x11vnc -display :1 -forever -shared -rfbport 5901 -rfbauth /home/ros/.vnc/passwd -o /tmp/x11vnc.log &
fi

# Start noVNC (browser access on 6080)
if ! pgrep -f "websockify" > /dev/null 2>&1; then
    websockify --web=/usr/share/novnc/ 6080 localhost:5901 > /tmp/novnc.log 2>&1 &
fi

sleep 2
echo "VNC ready on :1 | x11vnc on 5901 | noVNC browser on http://localhost:6080/vnc.html"

