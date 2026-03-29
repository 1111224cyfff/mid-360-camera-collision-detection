# PTP detection helper

This folder contains a small helper script to check whether a PTP/gPTP grandmaster (master clock) is present on the network and to gather diagnostics.

Files
- `check_ptp.sh`: main script. Copy to a convenient location and make executable:

```bash
chmod +x scripts/check_ptp.sh
```

Quick usage

Run the script (requires sudo for some checks):

```bash
sudo scripts/check_ptp.sh -i eth0
```

Common options
- `-i <iface>`: network interface (default `eth0`)
- `-c <count>`: number of PTP packets to capture (default `200`)
- `--no-capture`: skip tcpdump/tshark capture
- `--pmc`: run `pmc` dataset queries (requires `pmc` from `linuxptp`)
- `--ptp4l`: run `ptp4l -i <iface> -m` (interactive monitor; requires `linuxptp`)

What it checks
- `ethtool -T` and `ethtool -i` for NIC timestamping support
- presence of `/dev/ptp*` devices
- multicast group membership on the interface
- UDP listeners on ports 319/320
- optional tcpdump capture of PTP messages (UDP 319/320) and `tshark` decoding
- optional `pmc` queries to inspect PTP datasets

Interpretation
- If you see `Announce` messages and a non-empty `grandmasterIdentity`, there is a master on the wire.
- `ptp4l` logs showing a selected grandmaster are evidence of a usable master.
- `correctionField != 0` in captured packets suggests transparent clocks (switches) modifying timestamps.

Notes
- Script checks for presence of commands (`ethtool`, `tcpdump`, `tshark`, `pmc`, `ptp4l`) and skips steps if missing.
- Capture may require network span/mirror if IGMP snooping or VLAN filtering hides multicast traffic.
