#!/bin/bash
# Optional local hotspot alongside the existing vehicle Wi-Fi uplink.
# Run on the DEEPBOT only; does not start motor control.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
STATE="$ROOT/runtime"
AP=vision-ap
UPLINK=wlp58s0
SSID=DEEPBOT-Vision
SUBNET=192.168.77.0/24
GATEWAY=192.168.77.1
PASSWORD="$STATE/vision-hotspot.key"
HOSTAPD="$ROOT/staging/hotspot-local/extracted/usr/sbin/hostapd"
AP_UNIT=visual-race-hotspot-ap
DNS_UNIT=visual-race-hotspot-dns
MARKER="$STATE/vision-ap-owned"
FORWARD_BEFORE="$STATE/vision-ipforward-before"

[[ $(id -u) == 0 ]] || { echo '请使用 sudo 运行'; exit 1; }
mkdir -p "$STATE"

remove_rules() {
  iptables -w 3 -t nat -D POSTROUTING -s "$SUBNET" -o "$UPLINK" -m comment --comment visual-race-hotspot -j MASQUERADE 2>/dev/null || true
  iptables -w 3 -D FORWARD -i "$AP" -o "$UPLINK" -s "$SUBNET" -m comment --comment visual-race-hotspot -j ACCEPT 2>/dev/null || true
  iptables -w 3 -D FORWARD -i "$UPLINK" -o "$AP" -d "$SUBNET" -m conntrack --ctstate ESTABLISHED,RELATED -m comment --comment visual-race-hotspot -j ACCEPT 2>/dev/null || true
}

stop_hotspot() {
  systemctl stop "$DNS_UNIT.service" "$AP_UNIT.service" 2>/dev/null || true
  remove_rules
  if [[ -f "$FORWARD_BEFORE" ]]; then
    before="$(cat "$FORWARD_BEFORE")"
    if [[ "$before" == 0 || "$before" == 1 ]]; then sysctl -q -w "net.ipv4.ip_forward=$before" || true; fi
    rm -f "$FORWARD_BEFORE"
  fi
  if [[ -f "$MARKER" ]]; then
    iw dev "$AP" del 2>/dev/null || true
    rm -f "$MARKER"
  fi
}

case "${1:-status}" in
  start)
    [[ -f "$PASSWORD" && -x "$HOSTAPD" ]] || { echo '热点密码或 hostapd 不存在'; exit 1; }
    # Transient services and virtual interfaces vanish on reboot; discard only
    # state known to have belonged to an earlier boot.
    if [[ -f "$MARKER" ]] && [[ "$(cat "$MARKER")" != "$(cat /proc/sys/kernel/random/boot_id)" ]] &&
       ! ip link show "$AP" >/dev/null 2>&1 &&
       ! systemctl is-active --quiet "$AP_UNIT.service" && ! systemctl is-active --quiet "$DNS_UNIT.service"; then
      rm -f "$MARKER" "$FORWARD_BEFORE"
    fi
    [[ ! -e "$MARKER" && ! -e "$FORWARD_BEFORE" ]] || { echo '热点已启动或需要先清理'; exit 1; }
    if ip link show "$AP" >/dev/null 2>&1; then echo '热点网卡名称已被使用'; exit 1; fi
    if systemctl is-active --quiet "$AP_UNIT.service" || systemctl is-active --quiet "$DNS_UNIT.service"; then
      echo '热点服务名称已被使用'; exit 1
    fi
    read -r password < "$PASSWORD"
    [[ "$password" =~ ^[A-Za-z0-9]{8,63}$ ]] || { echo '热点密码格式无效'; exit 1; }
    umask 077
    cat > "$STATE/vision-hostapd.conf" <<EOF
interface=$AP
driver=nl80211
ssid=$SSID
hw_mode=g
channel=11
wpa=2
wpa_key_mgmt=WPA-PSK
wpa_passphrase=$password
rsn_pairwise=CCMP
EOF
    unset password
    cat > "$STATE/vision-dnsmasq.conf" <<EOF
interface=$AP
bind-interfaces
listen-address=$GATEWAY
port=53
dhcp-range=192.168.77.20,192.168.77.200,255.255.255.0,12h
dhcp-option=3,$GATEWAY
dhcp-option=6,$GATEWAY
dhcp-leasefile=$STATE/vision-hotspot.leases
user=deepbot
group=deepbot
EOF
    trap 'echo "热点启动失败，正在恢复"; stop_hotspot' ERR
    iw dev "$UPLINK" interface add "$AP" type __ap
    cat /proc/sys/kernel/random/boot_id > "$MARKER"
    # NetworkManager sees the new interface asynchronously and otherwise resets
    # it to managed mode, which conflicts with the existing Wi-Fi connection.
    found=0
    for _ in $(seq 1 30); do
      if nmcli device set "$AP" managed no 2>/dev/null; then found=1; break; fi
      sleep .2
    done
    [[ "$found" == 1 ]] || { echo '热点网卡未被识别'; false; }
    ip link set "$AP" down
    iw dev "$AP" set type __ap
    ip address add "$GATEWAY/24" dev "$AP"
    ip link set "$AP" up
    systemd-run --quiet --unit="$AP_UNIT" "$HOSTAPD" "$STATE/vision-hostapd.conf"
    sleep 2
    systemctl is-active --quiet "$AP_UNIT.service"
    systemd-run --quiet --unit="$DNS_UNIT" /usr/sbin/dnsmasq --keep-in-foreground --conf-file="$STATE/vision-dnsmasq.conf"
    sleep 1
    systemctl is-active --quiet "$DNS_UNIT.service"
    cat /proc/sys/net/ipv4/ip_forward > "$FORWARD_BEFORE"
    sysctl -q -w net.ipv4.ip_forward=1
    iptables -w 3 -t nat -I POSTROUTING -s "$SUBNET" -o "$UPLINK" -m comment --comment visual-race-hotspot -j MASQUERADE
    iptables -w 3 -I FORWARD -i "$AP" -o "$UPLINK" -s "$SUBNET" -m comment --comment visual-race-hotspot -j ACCEPT
    iptables -w 3 -I FORWARD -i "$UPLINK" -o "$AP" -d "$SUBNET" -m conntrack --ctstate ESTABLISHED,RELATED -m comment --comment visual-race-hotspot -j ACCEPT
    trap - ERR
    echo "热点 $SSID 已启动；小车地址 $GATEWAY"
    ;;
  stop) stop_hotspot; echo '小车热点已停止';;
  status)
    ip -br address show "$AP" 2>/dev/null || true
    systemctl is-active "$AP_UNIT.service" "$DNS_UNIT.service" 2>/dev/null || true
    ;;
  *) echo "用法: sudo bash $0 {start|stop|status}"; exit 2;;
esac
