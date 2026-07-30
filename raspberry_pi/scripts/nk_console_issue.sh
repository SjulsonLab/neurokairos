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

{
  echo ""
  echo "  NeuroKairos appliance  (\\l)"
  if [ -n "$ip" ]; then
    echo "  Web:  http://neurokairos.local   or   http://${ip}"
    echo "  IP:   ${ip}  (${iface})     MAC: ${mac:-unknown}"
  else
    echo "  Network: NO IP ASSIGNED YET (waiting for DHCP / check the cable)"
    echo "  MAC: $(cat /sys/class/net/e*/address 2>/dev/null | head -n1)"
  fi
  echo "  Login: user 'neurokairos' (you'll be asked to set a new password)"
  echo ""
} > /etc/issue
