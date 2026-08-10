#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
sudo install -m 0644 "${SCRIPT_DIR}/99-k4a.rules" /etc/udev/rules.d/99-k4a.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb
echo "Azure Kinect udev rules installed. Replug the camera if it is not detected."
