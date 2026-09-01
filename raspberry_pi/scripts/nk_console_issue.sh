#!/bin/bash
# Write /etc/issue so the HDMI/console login screen shows how to reach this Pi
# (primary IPv4, interface, MAC, and the web URL) BEFORE anyone logs in — or
# says so when no IP has been assigned yet. Run by nk-console-issue.timer at the
# lowest priority. Never touches the IRIG-H sender.

name="$(hostname)"
ip="$(ip -4 route get 192.0.2.1 2>/dev/null | sed -n 's/.* src \([0-9.]\+\).*/\1/p')"
iface="$(ip -4 route get 192.0.2.1 2>/dev/null | sed -n 's/.* dev \([^ ]\+\).*/\1/p')"
mac=""
[ -n "$iface" ] && mac="$(cat "/sys/class/net/${iface}/address" 2>/dev/null)"
# MAC fallback when no IP yet: prefer wired, then Wi-Fi (a headless sender is
# often Wi-Fi-only). Skip the AP interface if the setup hotspot is up.
[ -z "$mac" ] && mac="$(cat /sys/class/net/e*/address /sys/class/net/wlan*/address 2>/dev/null | head -n1)"

# Is the "NeuroKairos-Setup" onboarding hotspot currently active?
ap_active=""
if command -v nmcli >/dev/null 2>&1; then
  nmcli -t -f NAME connection show --active 2>/dev/null | grep -qx "nk-setup" && ap_active=1
fi

{
  echo ""
  echo "  NeuroKairos appliance  (\\l)"
  if [ -n "$ap_active" ]; then
    echo "  Wi-Fi setup mode: no network configured yet."
    echo "  1. On a phone/laptop, join Wi-Fi:  NeuroKairos-Setup  (password: neurokairos)"
    echo "  2. Open:  http://10.42.0.1   and pick your network."
    echo "  MAC: ${mac:-unknown}"
  elif [ -n "$ip" ]; then
    echo "  Web:  http://neurokairos-sender.local   or   http://${ip}"
    echo "  IP:   ${ip}  (${iface})     MAC: ${mac:-unknown}"
  else
    echo "  Network: NO IP YET (waiting for DHCP — check the cable, or configure"
    echo "           Wi-Fi via neurokairos-wifi.txt / the NeuroKairos-Setup hotspot)"
    echo "  MAC: ${mac:-unknown}"
  fi
  echo "  Login: user 'neurokairos' (you'll be asked to set a new password)"
  echo ""
} > /etc/issue
