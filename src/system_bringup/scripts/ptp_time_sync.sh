#!/usr/bin/env bash

set -euo pipefail

print_help() {
  cat <<'EOF'
Usage: ptp_time_sync.sh [options]

Options:
  --primary-iface <iface>        Network interface used for PTP. Default: eth0
  --primary-mode <mode>          auto|hardware|software. Default: auto
  --sync-system-clock <bool>     true|false, whether to run phc2sys. Default: true
  --clock-sync-direction <mode>  system-to-phc|phc-to-system. Default: system-to-phc
  --log-level <level>            debug|info|warn|error. Default: info
  --use-sudo <bool>              true|false, re-exec with sudo -n when not root. Default: true
  --help                         Show this help message.
EOF
}

primary_iface="eth0"
primary_mode="auto"
sync_system_clock="true"
clock_sync_direction="system-to-phc"
log_level="info"
use_sudo="true"
effective_sync_system_clock="true"
ptp4l_cmd=()
phc2sys_cmd=()
kill_with_sudo="false"
is_shutting_down="false"
ptp4l_match_pattern=""
phc2sys_match_pattern=""
ready_message_printed="false"
phc2sys_stable_threshold_ns=10000000
phc2sys_required_stable_samples=5
phc2sys_stable_sample_count=0
phc_sanity_wait_seconds=2
phc_sanity_max_offset_ns=5000000
state_dir="$(mktemp -d -t ptp_time_sync.XXXXXX)"
ptp4l_ready_file="$state_dir/ptp4l.ready"
phc2sys_ready_file="$state_dir/phc2sys.ready"

log() {
  local level="$1"
  shift
  printf '[ptp_time_sync][%s] %s\n' "$level" "$*"
}

mark_component_ready() {
  local ready_file="$1"
  if [[ ! -e "$ready_file" ]]; then
    : > "$ready_file"
  fi
}

clear_component_ready() {
  local ready_file="$1"
  rm -f "$ready_file"
}

abs_int() {
  local value="$1"
  if (( value < 0 )); then
    echo $(( -value ))
  else
    echo "$value"
  fi
}

maybe_log_ready() {
  if [[ "$ready_message_printed" == "true" ]]; then
    return
  fi

  if [[ -e "$ptp4l_ready_file" && -e "$phc2sys_ready_file" ]]; then
    ready_message_printed="true"
    if [[ "$effective_sync_system_clock" == "true" ]]; then
      log info "PTP 已就绪：ptp4l/phc2sys 已进入稳定状态，可以启动传感器采集。"
    else
      log info "PTP 已就绪：ptp4l 已进入稳定状态，可以启动传感器采集。"
    fi
  fi
}

handle_ptp4l_line() {
  local line="$1"
  printf '%s\n' "$line"

  if [[ "$line" == *"master offset"* ||
        "$line" == *"selected local clock"* ||
        "$line" == *"assuming the grand master role"* ||
        "$line" == *"UNCALIBRATED to SLAVE"* ||
        "$line" == *"LISTENING to MASTER"* ]]; then
    mark_component_ready "$ptp4l_ready_file"
  fi
}

handle_phc2sys_line() {
  local line="$1"
  printf '%s\n' "$line"

  if [[ "$line" =~ [[:space:]]offset[[:space:]]+(-?[0-9]+)[[:space:]] ]]; then
    local offset_ns="${BASH_REMATCH[1]}"
    local abs_offset_ns
    abs_offset_ns="$(abs_int "$offset_ns")"

    if (( abs_offset_ns <= phc2sys_stable_threshold_ns )); then
      phc2sys_stable_sample_count=$((phc2sys_stable_sample_count + 1))
      if (( phc2sys_stable_sample_count >= phc2sys_required_stable_samples )); then
        mark_component_ready "$phc2sys_ready_file"
      fi
    else
      if (( phc2sys_stable_sample_count > 0 )); then
        log warn "phc2sys offset left stable range; reset readiness window (offset_ns=$offset_ns threshold_ns=$phc2sys_stable_threshold_ns)"
      fi
      phc2sys_stable_sample_count=0
      clear_component_ready "$phc2sys_ready_file"
    fi
  fi
}

to_bool() {
  case "$1" in
    true|TRUE|1|yes|YES|on|ON) echo "true" ;;
    false|FALSE|0|no|NO|off|OFF) echo "false" ;;
    *)
      log error "invalid boolean value: $1"
      exit 2
      ;;
  esac
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    __name:=*|__log:=*|__ns:=*|*:=*)
      shift
      ;;
    --primary-iface)
      primary_iface="$2"
      shift 2
      ;;
    --primary-mode)
      primary_mode="$2"
      shift 2
      ;;
    --sync-system-clock)
      sync_system_clock="$(to_bool "$2")"
      shift 2
      ;;
    --clock-sync-direction)
      clock_sync_direction="$2"
      shift 2
      ;;
    --log-level)
      log_level="$2"
      shift 2
      ;;
    --use-sudo)
      use_sudo="$(to_bool "$2")"
      shift 2
      ;;
    --help|-h)
      print_help
      exit 0
      ;;
    *)
      log error "unknown argument: $1"
      print_help
      exit 2
      ;;
  esac
done

case "$primary_mode" in
  auto|hardware|software)
    ;;
  *)
    log error "invalid --primary-mode: $primary_mode"
    exit 2
    ;;
esac

case "$clock_sync_direction" in
  system-to-phc|phc-to-system)
    ;;
  *)
    log error "invalid --clock-sync-direction: $clock_sync_direction"
    exit 2
    ;;
esac

case "$log_level" in
  debug|info|warn|error)
    ;;
  *)
    log error "invalid --log-level: $log_level"
    exit 2
    ;;
esac

require_command() {
  local command_name="$1"
  if ! command -v "$command_name" >/dev/null 2>&1; then
    log error "required command not found: $command_name"
    exit 127
  fi
}

get_operstate() {
  local iface="$1"
  local operstate_path="/sys/class/net/$iface/operstate"
  if [[ -r "$operstate_path" ]]; then
    cat "$operstate_path"
    return 0
  fi
  echo "unknown"
}

get_carrier() {
  local iface="$1"
  local carrier_path="/sys/class/net/$iface/carrier"
  if [[ -r "$carrier_path" ]]; then
    cat "$carrier_path"
    return 0
  fi
  echo "-1"
}

run_phc_ctl() {
  if [[ "$(id -u)" -eq 0 ]]; then
    phc_ctl "$@"
  elif [[ "$use_sudo" == "true" ]]; then
    sudo -n "$(command -v phc_ctl)" "$@"
  else
    phc_ctl "$@"
  fi
}

extract_phc_offset_ns() {
  local cmp_output="$1"
  awk '/offset from CLOCK_REALTIME is/ {
    value = $6
    sub(/ns$/, "", value)
    print value
    exit
  }' <<< "$cmp_output"
}

phc_passes_sanity_check() {
  local phc_device="$1"
  local cmp_output=""
  local offset_ns=""
  local abs_offset_ns=""

  if ! command -v phc_ctl >/dev/null 2>&1; then
    log warn "phc_ctl is unavailable; skipping PHC sanity check for $phc_device"
    return 0
  fi

  if ! run_phc_ctl "$phc_device" set >/dev/null 2>&1; then
    log warn "failed to set $phc_device from CLOCK_REALTIME before sanity check; skipping hardware validation"
    return 0
  fi

  sleep "$phc_sanity_wait_seconds"
  cmp_output="$(run_phc_ctl "$phc_device" cmp 2>&1 || true)"
  offset_ns="$(extract_phc_offset_ns "$cmp_output")"
  if [[ -z "$offset_ns" ]]; then
    log warn "failed to parse PHC sanity check output for $phc_device: $cmp_output"
    return 0
  fi

  abs_offset_ns="$(abs_int "$offset_ns")"
  if (( abs_offset_ns > phc_sanity_max_offset_ns )); then
    log warn "PHC sanity check failed for $phc_device: offset_ns=$offset_ns after ${phc_sanity_wait_seconds}s exceeds threshold_ns=$phc_sanity_max_offset_ns"
    return 1
  fi

  log "$log_level" "PHC sanity check passed for $phc_device: offset_ns=$offset_ns after ${phc_sanity_wait_seconds}s"
  return 0
}

warn_if_system_ntp_active() {
  local ntp_active=""
  if ! command -v timedatectl >/dev/null 2>&1; then
    return
  fi

  ntp_active="$(timedatectl show -p NTP --value 2>/dev/null || true)"
  if [[ "$ntp_active" == "yes" ]]; then
    log warn "system NTP service is active; disable it with 'timedatectl set-ntp false' before running this host as a PTP master"
  fi
}

require_command ip
require_command ethtool
require_command ptp4l

warn_if_system_ntp_active

if ! ip link show "$primary_iface" >/dev/null 2>&1; then
  log error "network interface does not exist: $primary_iface"
  exit 1
fi

operstate="$(get_operstate "$primary_iface")"
carrier="$(get_carrier "$primary_iface")"
if [[ "$carrier" == "0" || "$operstate" == "down" || "$operstate" == "dormant" || "$operstate" == "lowerlayerdown" ]]; then
  log error "interface $primary_iface has no carrier (operstate=$operstate, carrier=$carrier). Connect the LiDAR/switch link before starting PTP."
  exit 1
fi

if [[ "$sync_system_clock" == "true" ]]; then
  require_command phc2sys
fi
effective_sync_system_clock="$sync_system_clock"

if [[ "$(id -u)" -ne 0 ]]; then
  if [[ "$use_sudo" == "true" ]]; then
    if ! sudo -n /usr/local/sbin/ptp4l -v >/dev/null 2>&1; then
      log error "passwordless sudo for ptp4l is not available; configure NOPASSWD or launch as root."
      exit 1
    fi
    if [[ "$sync_system_clock" == "true" ]] && ! sudo -n /usr/local/sbin/phc2sys -v >/dev/null 2>&1; then
      log error "passwordless sudo for phc2sys is not available; configure NOPASSWD or launch as root."
      exit 1
    fi
    if ! sudo -n /usr/bin/pkill -V >/dev/null 2>&1; then
      log error "passwordless sudo for pkill is not available; cleanup after Ctrl-C would leak ptp4l/phc2sys."
      exit 1
    fi
    kill_with_sudo="true"
    ptp4l_cmd=(sudo -n /usr/local/sbin/ptp4l)
    phc2sys_cmd=(sudo -n /usr/local/sbin/phc2sys)
    log "$log_level" "using sudo for ptp4l/phc2sys while keeping this script under roslaunch supervision"
  else
    log error "root privileges are required; rerun as root or enable passwordless sudo"
    exit 1
  fi
else
  ptp4l_cmd=(ptp4l)
  phc2sys_cmd=(phc2sys)
fi

ptp_hardware_clock="$(ethtool -T "$primary_iface" 2>/dev/null | awk -F': ' '/PTP Hardware Clock/ {print $2; exit}')"
has_hardware_clock="false"
if [[ -n "$ptp_hardware_clock" && "$ptp_hardware_clock" != "none" ]]; then
  has_hardware_clock="true"
fi

resolved_mode="$primary_mode"
if [[ "$resolved_mode" == "auto" ]]; then
  if [[ "$has_hardware_clock" == "true" ]]; then
    resolved_mode="hardware"
  else
    resolved_mode="software"
  fi
fi

if [[ "$resolved_mode" == "hardware" && "$has_hardware_clock" == "true" ]]; then
  phc_device="/dev/ptp${ptp_hardware_clock}"
  if ! phc_passes_sanity_check "$phc_device"; then
    if [[ "$primary_mode" == "auto" ]]; then
      log warn "hardware PHC on $primary_iface is unstable; falling back to software timestamp mode"
      resolved_mode="software"
    else
      log error "hardware PHC on $primary_iface is unstable; rerun with --primary-mode software"
      exit 1
    fi
  fi
fi

log "$log_level" "primary interface: $primary_iface"
log "$log_level" "requested mode: $primary_mode"
log "$log_level" "resolved mode: $resolved_mode"
log "$log_level" "clock sync direction: $clock_sync_direction"

ptp4l_pid=""
phc2sys_pid=""

terminate_pattern() {
  local signal_name="$1"
  local pattern="$2"
  if [[ -z "$pattern" ]]; then
    return
  fi

  if [[ "$kill_with_sudo" == "true" ]]; then
    sudo -n /usr/bin/pkill "-$signal_name" -f "$pattern" >/dev/null 2>&1 || true
  else
    /usr/bin/pkill "-$signal_name" -f "$pattern" >/dev/null 2>&1 || true
  fi
}

cleanup() {
  if [[ "$is_shutting_down" == "true" ]]; then
    return
  fi
  is_shutting_down="true"
  local exit_code=$?
  trap - EXIT INT TERM

  if [[ -n "$phc2sys_pid" ]] && kill -0 "$phc2sys_pid" >/dev/null 2>&1; then
    log "$log_level" "stopping phc2sys pid=$phc2sys_pid"
    terminate_pattern TERM "$phc2sys_match_pattern"
    kill "$phc2sys_pid" >/dev/null 2>&1 || true
    sleep 0.2
    terminate_pattern KILL "$phc2sys_match_pattern"
    kill -KILL "$phc2sys_pid" >/dev/null 2>&1 || true
    wait "$phc2sys_pid" >/dev/null 2>&1 || true
  else
    terminate_pattern TERM "$phc2sys_match_pattern"
    sleep 0.2
    terminate_pattern KILL "$phc2sys_match_pattern"
  fi

  if [[ -n "$ptp4l_pid" ]] && kill -0 "$ptp4l_pid" >/dev/null 2>&1; then
    log "$log_level" "stopping ptp4l pid=$ptp4l_pid"
    terminate_pattern TERM "$ptp4l_match_pattern"
    kill "$ptp4l_pid" >/dev/null 2>&1 || true
    sleep 0.2
    terminate_pattern KILL "$ptp4l_match_pattern"
    kill -KILL "$ptp4l_pid" >/dev/null 2>&1 || true
    wait "$ptp4l_pid" >/dev/null 2>&1 || true
  else
    terminate_pattern TERM "$ptp4l_match_pattern"
    sleep 0.2
    terminate_pattern KILL "$ptp4l_match_pattern"
  fi

  rm -rf "$state_dir"

  exit "$exit_code"
}

handle_signal() {
  local signal_name="$1"
  log warn "received $signal_name, terminating ptp4l/phc2sys"
  cleanup
}

trap cleanup EXIT
trap 'handle_signal INT' INT
trap 'handle_signal TERM' TERM
trap 'handle_signal HUP' HUP
trap 'handle_signal QUIT' QUIT

ptp4l_args=(-i "$primary_iface" -m)
if [[ "$resolved_mode" == "hardware" ]]; then
  ptp4l_args=(-H "${ptp4l_args[@]}")
elif [[ "$resolved_mode" == "software" ]]; then
  ptp4l_args=(-S "${ptp4l_args[@]}")
fi
ptp4l_match_pattern="^/usr/local/sbin/ptp4l ${ptp4l_args[*]}$"

log "$log_level" "starting: ptp4l ${ptp4l_args[*]}"
{
  "${ptp4l_cmd[@]}" "${ptp4l_args[@]}" 2>&1 | while IFS= read -r line; do
    handle_ptp4l_line "$line"
  done
} &
ptp4l_pid=$!

if [[ "$sync_system_clock" == "true" ]]; then
  if [[ "$resolved_mode" == "hardware" && "$has_hardware_clock" == "true" ]]; then
    while [[ ! -e "$ptp4l_ready_file" ]]; do
      if ! kill -0 "$ptp4l_pid" >/dev/null 2>&1; then
        log error "ptp4l exited before reaching a ready state; phc2sys will not start"
        wait "$ptp4l_pid"
        exit $?
      fi
      sleep 0.2
    done

    phc_device="/dev/ptp${ptp_hardware_clock}"
    if [[ "$clock_sync_direction" == "system-to-phc" ]]; then
      phc2sys_match_pattern="^/usr/local/sbin/phc2sys -s CLOCK_REALTIME -c ${primary_iface} -O 0 -m$"
      log "$log_level" "starting: phc2sys -s CLOCK_REALTIME -c $primary_iface -O 0 -m"
      {
        "${phc2sys_cmd[@]}" -s CLOCK_REALTIME -c "$primary_iface" -O 0 -m 2>&1 | while IFS= read -r line; do
          handle_phc2sys_line "$line"
        done
      } &
    else
      phc2sys_match_pattern="^/usr/local/sbin/phc2sys -s ${phc_device} -c CLOCK_REALTIME -w -m$"
      log "$log_level" "starting: phc2sys -s $phc_device -c CLOCK_REALTIME -w -m"
      {
        "${phc2sys_cmd[@]}" -s "$phc_device" -c CLOCK_REALTIME -w -m 2>&1 | while IFS= read -r line; do
          handle_phc2sys_line "$line"
        done
      } &
    fi
    phc2sys_pid=$!
  else
    log warn "system clock sync requested, but no PHC is available on $primary_iface; skipping phc2sys"
    effective_sync_system_clock="false"
    mark_component_ready "$phc2sys_ready_file"
  fi
else
  effective_sync_system_clock="false"
  mark_component_ready "$phc2sys_ready_file"
fi

while true; do
  maybe_log_ready

  if [[ -n "$phc2sys_pid" ]] && ! kill -0 "$phc2sys_pid" >/dev/null 2>&1; then
    wait "$phc2sys_pid"
  fi

  if ! kill -0 "$ptp4l_pid" >/dev/null 2>&1; then
    break
  fi

  sleep 0.2
done

wait "$ptp4l_pid"
