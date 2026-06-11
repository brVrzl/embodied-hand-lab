#!/usr/bin/env bash
set -euo pipefail

IFACE="${IFACE:-enp8s0}"
PC_IP="${PC_IP:-192.168.71.100}"
JAKA_IP="${JAKA_IP:-192.168.71.50}"
EXECUTE=0

for arg in "$@"; do
  case "${arg}" in
    --execute)
      EXECUTE=1
      ;;
    --help|-h)
      cat <<'EOF'
Usage:
  IFACE=enp8s0 PC_IP=192.168.71.100 JAKA_IP=192.168.71.50 \
    ./scripts/setup_jaka_lan2_route.sh --execute

This configures a /32 address and /32 host route for JAKA MiniCab LAN2.
It intentionally avoids adding a 192.168.71.0/24 route to Ethernet, because
that can steal SSH traffic from Wi-Fi when Wi-Fi is on the same subnet.
EOF
      exit 0
      ;;
    *)
      echo "Unknown argument: ${arg}" >&2
      exit 2
      ;;
  esac
done

commands=(
  "sudo ip addr flush dev ${IFACE}"
  "sudo ip addr add ${PC_IP}/32 dev ${IFACE}"
  "sudo ip link set ${IFACE} up"
  "sudo ip route replace ${JAKA_IP}/32 dev ${IFACE} src ${PC_IP}"
  "ip route get ${JAKA_IP}"
)

if [[ "${EXECUTE}" -ne 1 ]]; then
  printf '[setup_jaka_lan2_route] Dry run. Add --execute to run:\n'
  printf '  %s\n' "${commands[@]}"
  exit 0
fi

for command in "${commands[@]}"; do
  echo "[setup_jaka_lan2_route] ${command}"
  eval "${command}"
done
