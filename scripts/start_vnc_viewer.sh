#!/usr/bin/env bash
set -euo pipefail

DISPLAY_NUM="${1:-1}"
GEOMETRY="${VNC_GEOMETRY:-1440x900}"
DEPTH="${VNC_DEPTH:-24}"

mkdir -p "$HOME/.vnc"

cat > "$HOME/.vnc/xstartup" <<'EOF'
#!/usr/bin/env bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
xsetroot -solid '#202020'
openbox-session &
xterm -geometry 120x32+30+30 -fa Monospace -fs 11 &
wait
EOF
chmod +x "$HOME/.vnc/xstartup"

vncserver ":${DISPLAY_NUM}" \
  -localhost yes \
  -geometry "${GEOMETRY}" \
  -depth "${DEPTH}" \
  -SecurityTypes VncAuth

cat <<EOF
VNC desktop started on :${DISPLAY_NUM}.

From your Mac, open a separate local terminal and run:
  ssh -N -L 590${DISPLAY_NUM}:localhost:590${DISPLAY_NUM} thor@192.168.71.19

Then connect your VNC client to:
  localhost:590${DISPLAY_NUM}
EOF
