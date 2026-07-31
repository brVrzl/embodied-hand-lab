#!/usr/bin/env bash
set -euo pipefail

IFACE="${IFACE:-enp8s0}"
PC_IP="${PC_IP:-192.168.71.100}"
JAKA_IP="${JAKA_IP:-192.168.71.50}"
EXECUTE=0
ADDRESS_REPLACEMENT_ACK=0

for arg in "$@"; do
  case "${arg}" in
    --execute)
      EXECUTE=1
      ;;
    --acknowledge-interface-address-replacement)
      ADDRESS_REPLACEMENT_ACK=1
      ;;
    --help|-h)
      cat <<'EOF'
Usage:
  IFACE=enp8s0 PC_IP=192.168.71.100 JAKA_IP=192.168.71.50 \
    ./scripts/setup_jaka_lan2_route.sh \
      --execute \
      --acknowledge-interface-address-replacement

This configures a /32 address and /32 host route for JAKA MiniCab LAN2.
It intentionally avoids adding a 192.168.71.0/24 route to Ethernet, because
that can steal SSH traffic from Wi-Fi when Wi-Fi is on the same subnet.

WARNING: execution removes every existing IP address from IFACE before adding
the selected /32 address. Verify that IFACE is the dedicated robot interface
and keep another management connection available. The acknowledgement is
required only for execution; without --execute this script prints a dry run.
EOF
      exit 0
      ;;
    *)
      echo "Unknown argument: ${arg}" >&2
      exit 2
      ;;
  esac
done

if [[ ! "${IFACE}" =~ ^[A-Za-z0-9_.:-]+$ ]]; then
  echo "Invalid interface name: ${IFACE}" >&2
  exit 2
fi

validate_ipv4() {
  local value="$1"
  local first second third fourth extra
  IFS=. read -r first second third fourth extra <<<"${value}"
  if [[ -n "${extra:-}" ]]; then
    return 1
  fi
  for octet in "${first:-}" "${second:-}" "${third:-}" "${fourth:-}"; do
    if [[ ! "${octet}" =~ ^[0-9]{1,3}$ ]] \
      || (( 10#${octet} < 0 || 10#${octet} > 255 )); then
      return 1
    fi
  done
}

if ! validate_ipv4 "${PC_IP}"; then
  echo "Invalid PC_IP: ${PC_IP}" >&2
  exit 2
fi
if ! validate_ipv4 "${JAKA_IP}"; then
  echo "Invalid JAKA_IP: ${JAKA_IP}" >&2
  exit 2
fi

if [[ "${EXECUTE}" -ne 1 ]]; then
  printf '[setup_jaka_lan2_route] Dry run; no network state was changed.\n'
  printf '  sudo ip addr flush dev %q\n' "${IFACE}"
  printf '  sudo ip addr add %q dev %q\n' "${PC_IP}/32" "${IFACE}"
  printf '  sudo ip link set %q up\n' "${IFACE}"
  printf '  sudo ip route replace %q dev %q src %q\n' \
    "${JAKA_IP}/32" "${IFACE}" "${PC_IP}"
  printf '  ip route get %q\n' "${JAKA_IP}"
  exit 0
fi

if [[ "${ADDRESS_REPLACEMENT_ACK}" -ne 1 ]]; then
  echo "Execution requires --acknowledge-interface-address-replacement." >&2
  exit 2
fi
if ! command -v ip >/dev/null 2>&1; then
  echo "The Linux ip command is required." >&2
  exit 2
fi

echo "[setup_jaka_lan2_route] replacing addresses on dedicated interface ${IFACE}"
sudo ip addr flush dev "${IFACE}"
sudo ip addr add "${PC_IP}/32" dev "${IFACE}"
sudo ip link set "${IFACE}" up
sudo ip route replace "${JAKA_IP}/32" dev "${IFACE}" src "${PC_IP}"
ip route get "${JAKA_IP}"
