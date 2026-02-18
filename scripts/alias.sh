#!/usr/bin/env bash
set -euo pipefail

usage() {
  local script_name
  script_name="$(basename "$0")"
  cat <<'EOF'
Usage:
  scripts/<script>.sh <device> <alias> [--install]

Examples:
  scripts/<script>.sh /dev/ttyACM1 str1ker_laser
  scripts/<script>.sh /dev/ttyACM0 str1ker_arm --install

Notes:
  - Generates one udev rule that creates /dev/<alias>.
  - Uses vendor/product + serial (when available) so names stay stable.
  - With --install, writes to /etc/udev/rules.d/99-str1ker-usb.rules,
    reloads rules, and triggers udev.
EOF
  echo
  echo "This script is currently: scripts/$script_name"
}

if [[ $# -lt 2 || $# -gt 3 ]]; then
  usage
  exit 1
fi

device="$1"
alias_name="$2"
install_flag="${3:-}"
rules_file="/etc/udev/rules.d/99-str1ker-usb.rules"

if [[ ! -e "$device" ]]; then
  echo "Device not found: $device" >&2
  exit 1
fi

if [[ ! "$alias_name" =~ ^[a-zA-Z0-9._-]+$ ]]; then
  echo "Alias must contain only letters, numbers, dot, underscore, or dash." >&2
  exit 1
fi

props="$(udevadm info --query=property --name="$device")"
vendor_id="$(awk -F= '/^ID_VENDOR_ID=/{print $2}' <<<"$props")"
product_id="$(awk -F= '/^ID_MODEL_ID=/{print $2}' <<<"$props")"
serial_short="$(awk -F= '/^ID_SERIAL_SHORT=/{print $2}' <<<"$props")"
id_path="$(awk -F= '/^ID_PATH=/{print $2}' <<<"$props")"

if [[ -z "$vendor_id" || -z "$product_id" ]]; then
  echo "Unable to read ID_VENDOR_ID/ID_MODEL_ID from $device" >&2
  exit 1
fi

rule_prefix="SUBSYSTEM==\"tty\", ENV{ID_VENDOR_ID}==\"$vendor_id\", ENV{ID_MODEL_ID}==\"$product_id\""
rule_suffix=", GROUP=\"dialout\", MODE=\"0666\", SYMLINK+=\"$alias_name\""

if [[ -n "$serial_short" ]]; then
  rule="$rule_prefix, ENV{ID_SERIAL_SHORT}==\"$serial_short\"$rule_suffix"
elif [[ -n "$id_path" ]]; then
  # Fallback when serial is unavailable. Path ties to a specific USB port.
  rule="$rule_prefix, ENV{ID_PATH}==\"$id_path\"$rule_suffix"
else
  # Last-resort fallback; may collide if identical devices are connected.
  rule="$rule_prefix$rule_suffix"
fi

if [[ "$install_flag" == "--install" ]]; then
  if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
    if command -v sudo >/dev/null 2>&1; then
      exec sudo "$0" "$device" "$alias_name" --install
    fi
    echo "Root privileges required for --install. Run with sudo." >&2
    exit 1
  fi

  tmp_file="$(mktemp)"
  if [[ -f "$rules_file" ]]; then
    grep -v "SYMLINK+=\"$alias_name\"" "$rules_file" >"$tmp_file" || true
  fi
  echo "$rule" >>"$tmp_file"
  install -m 0644 "$tmp_file" "$rules_file"
  rm -f "$tmp_file"

  udevadm control --reload-rules
  udevadm trigger

  echo "Installed rule in $rules_file"
  echo "New alias should appear as /dev/$alias_name (replug device if needed)."
else
  echo "$rule"
  cat <<EOF

Run the following to install it:
  sudo mkdir -p /etc/udev/rules.d
  echo '$rule' | sudo tee -a $rules_file >/dev/null
  sudo udevadm control --reload-rules
  sudo udevadm trigger

Then replug $device and use:
  /dev/$alias_name
EOF
fi
