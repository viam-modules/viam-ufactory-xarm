#!/usr/bin/env bash
#
# setup-arm-link.sh — configure a persistent static IP on the host's NIC
# so it can reach the UFactory xArm controller across reboots.
#
# Supports hosts managed by NetworkManager (Ubuntu Desktop, Raspberry Pi OS
# Bookworm+, Fedora/RHEL, Debian Desktop) and by netplan/systemd-networkd
# (Ubuntu Server). The resulting profile is bound to the NIC's MAC address
# so it survives across interface-name changes (e.g. enp86s0 vs eno1).

set -euo pipefail

HOST_IP="192.168.1.10"
PREFIX="24"
IFACE=""
DRY_RUN=0
CON_NAME="xarm-link"
NETPLAN_FILE="/etc/netplan/99-xarm-link.yaml"

log() { printf '[setup-arm-link] %s\n' "$*"; }
err() { printf '[setup-arm-link] ERROR: %s\n' "$*" >&2; }

usage() {
    cat <<'EOF'
Usage: sudo bash setup-arm-link.sh [options]

Configure a persistent static IP on the host's NIC for the xArm link.

Options:
  --iface NAME     Interface to configure (default: auto-detect)
  --host-ip IP     Host IP on the arm subnet (default: 192.168.1.10)
  --prefix N       Subnet prefix length (default: 24)
  --dry-run        Print what would happen, don't apply
  -h, --help       Show this help

Remove later:
  NetworkManager:  sudo nmcli connection delete xarm-link
  netplan:         sudo rm /etc/netplan/99-xarm-link.yaml && sudo netplan apply
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --iface)   IFACE="${2:-}";   shift 2 ;;
        --host-ip) HOST_IP="${2:-}"; shift 2 ;;
        --prefix)  PREFIX="${2:-}";  shift 2 ;;
        --dry-run) DRY_RUN=1;        shift ;;
        -h|--help) usage; exit 0 ;;
        *) err "unknown argument: $1"; usage >&2; exit 2 ;;
    esac
done

require_root() {
    if [[ $EUID -ne 0 ]]; then
        err "must run as root (use sudo)"
        exit 1
    fi
}

detect_iface() {
    local candidates=() name
    for path in /sys/class/net/*; do
        name=$(basename "$path")
        case "$name" in
            lo|docker*|virbr*|veth*|br-*|tun*|tap*|bond*|wl*|wlx*|wwan*|ppp*) continue ;;
        esac
        [[ -e "$path/device" ]] || continue
        candidates+=("$name")
    done

    if [[ ${#candidates[@]} -eq 0 ]]; then
        err "no physical ethernet interfaces found; pass --iface explicitly"
        exit 1
    fi

    local preferred=()
    for name in "${candidates[@]}"; do
        local has_ip4
        has_ip4=$(ip -o -4 addr show dev "$name" 2>/dev/null | wc -l)
        if [[ "$has_ip4" -eq 0 ]]; then
            preferred+=("$name")
        fi
    done

    local -a pool
    if [[ ${#preferred[@]} -gt 0 ]]; then
        pool=("${preferred[@]}")
    else
        pool=("${candidates[@]}")
    fi

    if [[ ${#pool[@]} -eq 1 ]]; then
        printf '%s' "${pool[0]}"
        return 0
    fi

    err "multiple candidate interfaces found: ${pool[*]}"
    err "pass --iface to pick one explicitly"
    exit 1
}

get_mac() {
    cat "/sys/class/net/$1/address"
}

detect_stack() {
    if command -v nmcli >/dev/null 2>&1 && systemctl is-active --quiet NetworkManager 2>/dev/null; then
        echo "nm"
    elif command -v netplan >/dev/null 2>&1 && [[ -d /etc/netplan ]]; then
        echo "netplan"
    else
        echo "unknown"
    fi
}

apply_nm() {
    local iface="$1" mac="$2" host_ip="$3" prefix="$4"

    if [[ $DRY_RUN -eq 1 ]]; then
        cat <<EOF
[dry-run] would run:
  nmcli connection delete $CON_NAME      # if it already exists
  nmcli connection add type ethernet con-name $CON_NAME mac $mac \\
      ipv4.method manual ipv4.addresses $host_ip/$prefix \\
      ipv4.never-default yes ipv4.ignore-auto-dns yes \\
      ipv6.method disabled connection.autoconnect yes
  nmcli connection up $CON_NAME
EOF
        return 0
    fi

    log "configuring NetworkManager profile '$CON_NAME' on $iface (MAC $mac) → $host_ip/$prefix"

    if nmcli -t -f NAME connection show | grep -qx "$CON_NAME"; then
        log "replacing existing profile '$CON_NAME'"
        nmcli connection delete "$CON_NAME" >/dev/null
    fi

    nmcli connection add \
        type ethernet \
        con-name "$CON_NAME" \
        mac "$mac" \
        ipv4.method manual \
        ipv4.addresses "$host_ip/$prefix" \
        ipv4.never-default yes \
        ipv4.ignore-auto-dns yes \
        ipv6.method disabled \
        connection.autoconnect yes >/dev/null

    nmcli connection up "$CON_NAME" >/dev/null
}

apply_netplan() {
    local iface="$1" mac="$2" host_ip="$3" prefix="$4"

    local yaml
    yaml=$(cat <<EOF
network:
  version: 2
  ethernets:
    xarm-link:
      match:
        macaddress: $mac
      dhcp4: false
      dhcp6: false
      link-local: []
      accept-ra: false
      addresses: [$host_ip/$prefix]
      optional: true
EOF
    )

    if [[ $DRY_RUN -eq 1 ]]; then
        echo "[dry-run] would write $NETPLAN_FILE (mode 0600, root:root):"
        printf '%s\n' "$yaml"
        echo "[dry-run] then run: netplan apply"
        return 0
    fi

    log "writing $NETPLAN_FILE for $iface (MAC $mac) → $host_ip/$prefix"
    printf '%s\n' "$yaml" > "$NETPLAN_FILE"
    chmod 0600 "$NETPLAN_FILE"
    chown root:root "$NETPLAN_FILE"
    netplan apply
}

verify() {
    local host_ip="$1"
    local deadline=$((SECONDS + 10))
    while (( SECONDS < deadline )); do
        if ip -o -4 addr show | grep -q "inet $host_ip/"; then
            log "OK: $host_ip is assigned"
            return 0
        fi
        sleep 1
    done
    err "$host_ip did not appear on any interface within 10s"
    err "check 'ip -4 addr' and 'journalctl -u NetworkManager -u systemd-networkd -n 50'"
    return 1
}

main() {
    require_root

    if [[ -z "$IFACE" ]]; then
        IFACE=$(detect_iface)
        log "auto-detected interface: $IFACE"
    elif [[ ! -e "/sys/class/net/$IFACE" ]]; then
        err "interface '$IFACE' not found"
        exit 1
    fi

    local mac
    mac=$(get_mac "$IFACE")
    if [[ -z "$mac" || "$mac" == "00:00:00:00:00:00" ]]; then
        err "interface $IFACE has no MAC address; is it up and cabled?"
        exit 1
    fi

    local stack
    stack=$(detect_stack)
    case "$stack" in
        nm)      apply_nm      "$IFACE" "$mac" "$HOST_IP" "$PREFIX" ;;
        netplan) apply_netplan "$IFACE" "$mac" "$HOST_IP" "$PREFIX" ;;
        *)
            err "unsupported network stack; this script supports NetworkManager and netplan"
            err "configure the interface manually — see README"
            exit 1
            ;;
    esac

    if [[ $DRY_RUN -eq 0 ]]; then
        verify "$HOST_IP"
    fi
}

main "$@"
