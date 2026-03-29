#!/usr/bin/env bash
set -euo pipefail

iface=eth0
capture_count=200
do_capture=1
run_pmc=0
run_ptp4l=0

usage(){
  cat <<EOF
Usage: $0 [-i iface] [-c capture_count] [--no-capture] [--pmc] [--ptp4l]
  -i iface        Network interface to check (default: eth0)
  -c count        Number of PTP packets to capture (default: 200)
  --no-capture    Skip tcpdump/tshark capture
  --pmc           Run pmc GET CURRENT_DATA_SET (requires pmc)
  --ptp4l         Run ptp4l in monitor mode (interactive; prints to stdout)
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -i) iface="$2"; shift 2;;
    -c) capture_count="$2"; shift 2;;
    --no-capture) do_capture=0; shift;;
    --pmc) run_pmc=1; shift;;
    --ptp4l) run_ptp4l=1; shift;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown arg: $1"; usage; exit 2;;
  esac
done

echo "PTP check starting on interface: $iface"

if ! command -v ethtool >/dev/null 2>&1; then
  echo "ethtool not found; please install ethtool." >&2
else
  echo "\n== ethtool -T $iface (timestamp capabilities) =="
  sudo ethtool -T "$iface" || true
  echo "\n== ethtool -i $iface (driver info) =="
  sudo ethtool -i "$iface" || true
fi

echo "\n== /dev/ptp* devices =="
ls -l /dev/ptp* 2>/dev/null || echo "(no /dev/ptp devices found)"

echo "\n== multicast groups (ip maddr) =="
ip maddr show dev "$iface" || true

echo "\n== listening UDP sockets on 319/320 =="
if sudo -n true >/dev/null 2>&1; then
  sudo -n ss -u -a | grep -E ':319|:320' || echo "(no local UDP listeners on 319/320)"
else
  ss -u -a | grep -E ':319|:320' || echo "(no local UDP listeners on 319/320; sudo unavailable, showing current user sockets only)"
fi

if [[ $do_capture -eq 1 ]]; then
  if ! command -v tcpdump >/dev/null 2>&1; then
    echo "tcpdump not found; skipping capture" >&2
  else
    echo "\nAbout to capture up to $capture_count PTP packets (UDP 319/320)."
    read -r -p "Proceed with capture on $iface? [y/N] " ans
    if [[ "$ans" =~ ^[Yy]$ ]]; then
      out=/tmp/ptp_capture_$(date +%s).pcap
      echo "Capturing to $out (ctrl-c to abort)..."
      sudo tcpdump -i "$iface" -n -s 256 'udp port 319 or udp port 320' -c "$capture_count" -w "$out" || true
      if command -v tshark >/dev/null 2>&1; then
        echo "\n== tshark decoded PTP messages (messageType, grandmasterIdentity, correctionField) =="
        sudo tshark -r "$out" -Y ptp -T fields -e ptp.messageType -e ptp.grandmasterIdentity -e ptp.clockIdentity -e ptp.correctionField -E header=y -E separator=, || true
      else
        echo "tshark not available; saved pcap at $out" >&2
      fi
    else
      echo "Skipping capture by user choice."
    fi
  fi
else
  echo "Capture skipped (--no-capture)."
fi

if [[ $run_pmc -eq 1 ]]; then
  if ! command -v pmc >/dev/null 2>&1; then
    echo "pmc not found; please install linuxptp package to use pmc." >&2
  else
    echo "\n== pmc GET CURRENT_DATA_SET =="
    sudo pmc -u -b 0 'GET CURRENT_DATA_SET' || true
    echo "\n== pmc GET PARENT_DATA_SET =="
    sudo pmc -u -b 0 'GET PARENT_DATA_SET' || true
    echo "\n== pmc GET TIME_PROPERTIES_DATA_SET =="
    sudo pmc -u -b 0 'GET TIME_PROPERTIES_DATA_SET' || true
  fi
fi

if [[ $run_ptp4l -eq 1 ]]; then
  if ! command -v ptp4l >/dev/null 2>&1; then
    echo "ptp4l not found; please install linuxptp to run ptp4l." >&2
  else
    echo "\n== Running ptp4l -i $iface -m (interactive; press Ctrl-C to stop) =="
    sudo ptp4l -i "$iface" -m
  fi
fi

echo "\nPTP check finished. Interpret results: look for Announce/Sync messages and grandmasterIdentity values; pmc/ptp4l output shows selected master." 
