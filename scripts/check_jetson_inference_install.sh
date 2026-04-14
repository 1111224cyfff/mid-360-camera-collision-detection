#!/usr/bin/env bash
set -euo pipefail

repo_dir="/home/nvidia/jetson-inference"
run_cpp_demo=0
run_python_demo=0
sample_image=""

usage() {
  cat <<EOF
Usage: $0 [--repo DIR] [--run-cpp-demo] [--run-python-demo] [--sample-image PATH]

Checks whether jetson-inference is installed and whether its C++ and Python
artifacts are usable on this machine.

Options:
  --repo DIR          jetson-inference source/build directory
  --run-cpp-demo      run the official C++ detectnet demo on a sample image
  --run-python-demo   run the official Python detectnet demo on a sample image
  --sample-image PATH override the sample image used by demo checks
  -h, --help          show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo)
      repo_dir="$2"
      shift 2
      ;;
    --run-cpp-demo)
      run_cpp_demo=1
      shift
      ;;
    --run-python-demo)
      run_python_demo=1
      shift
      ;;
    --sample-image)
      sample_image="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown arg: $1" >&2
      usage
      exit 2
      ;;
  esac
done

failures=0

check_path() {
  local label="$1"
  local path="$2"

  if [[ -e "$path" ]]; then
    echo "[OK] $label: $path"
  else
    echo "[FAIL] $label: $path"
    failures=$((failures + 1))
  fi
}

run_check() {
  local label="$1"
  shift

  echo
  echo "== $label =="
  if "$@"; then
    echo "[OK] $label"
  else
    echo "[FAIL] $label"
    failures=$((failures + 1))
  fi
}

echo "jetson-inference verification starting"
echo "repo_dir=$repo_dir"

echo
echo "== Platform =="
uname -m || true
if [[ -f /etc/nv_tegra_release ]]; then
  cat /etc/nv_tegra_release
else
  echo "/etc/nv_tegra_release not found"
fi

echo
echo "== Installation Artifacts =="
check_path "repo directory" "$repo_dir"
check_path "shared library" "/usr/local/lib/libjetson-inference.so"
check_path "shared utility library" "/usr/local/lib/libjetson-utils.so"
check_path "header directory" "/usr/local/include/jetson-inference"
check_path "utility header directory" "/usr/local/include/jetson-utils"
check_path "detectnet binary" "$repo_dir/build/aarch64/bin/detectnet"
check_path "python detectnet example" "$repo_dir/python/examples/detectnet.py"

echo
echo "== Linker Visibility =="
ldconfig_output="$(ldconfig -p || true)"
if grep -q 'libjetson-inference.so' <<< "$ldconfig_output"; then
  grep -E 'libjetson-inference|libjetson-utils|libnvinfer' <<< "$ldconfig_output" || true
else
  echo "[FAIL] libjetson-inference.so not visible to ldconfig"
  failures=$((failures + 1))
fi

if [[ -z "$sample_image" ]]; then
  if [[ -f "$repo_dir/data/images/peds_4.jpg" ]]; then
    sample_image="$repo_dir/data/images/peds_4.jpg"
  elif [[ -f "$repo_dir/data/images/humans_5.jpg" ]]; then
    sample_image="$repo_dir/data/images/humans_5.jpg"
  fi
fi

python_bind_dir=""
if [[ -d "$repo_dir/build/aarch64/lib/python" ]]; then
  python_bind_dir="$(find "$repo_dir/build/aarch64/lib/python" -maxdepth 2 -type f -name 'jetson_inference_python.so' -printf '%h\n' | head -n 1)"
fi

echo
echo "== Python Binding Discovery =="
if [[ -n "$python_bind_dir" ]]; then
  echo "[OK] build python bindings found in $python_bind_dir"
else
  echo "[FAIL] build python bindings not found under $repo_dir/build/aarch64/lib/python"
  failures=$((failures + 1))
fi

if /usr/bin/python3 -c "import jetson_inference, jetson_utils" >/dev/null 2>&1; then
  echo "[OK] system python imports jetson_inference and jetson_utils directly"
elif [[ -n "$python_bind_dir" ]] && PYTHONPATH="$python_bind_dir" /usr/bin/python3 -c "import jetson_inference, jetson_utils" >/dev/null 2>&1; then
  echo "[OK] system python imports succeed with PYTHONPATH=$python_bind_dir"
else
  echo "[FAIL] python imports failed for jetson_inference and jetson_utils"
  failures=$((failures + 1))
fi

if [[ $run_cpp_demo -eq 1 ]]; then
  run_check "C++ detectnet demo" bash -lc '
    set -euo pipefail
    repo_dir="$1"
    sample_image="$2"
    output_image="/tmp/jetson_inference_cpp_check.jpg"
    [[ -f "$sample_image" ]]
    cd "$repo_dir/build/aarch64/bin"
    ./detectnet "$sample_image" "$output_image" >/tmp/jetson_inference_cpp_check.log 2>&1
    [[ -f "$output_image" ]]
    tail -n 20 /tmp/jetson_inference_cpp_check.log
  ' _ "$repo_dir" "$sample_image"
fi

if [[ $run_python_demo -eq 1 ]]; then
  if [[ -z "$python_bind_dir" ]]; then
    echo "[FAIL] Python demo skipped because no binding directory was found"
    failures=$((failures + 1))
  else
    run_check "Python detectnet demo" bash -lc '
      set -euo pipefail
      repo_dir="$1"
      sample_image="$2"
      python_bind_dir="$3"
      output_image="/tmp/jetson_inference_python_check.jpg"
      [[ -f "$sample_image" ]]
      cd "$repo_dir"
      PYTHONPATH="$python_bind_dir" /usr/bin/python3 python/examples/detectnet.py "$sample_image" "$output_image" >/tmp/jetson_inference_python_check.log 2>&1
      [[ -f "$output_image" ]]
      tail -n 20 /tmp/jetson_inference_python_check.log
    ' _ "$repo_dir" "$sample_image" "$python_bind_dir"
  fi
fi

echo
echo "== Summary =="
if [[ $failures -eq 0 ]]; then
  echo "All checks passed."
  exit 0
fi

echo "$failures check(s) failed."
exit 1