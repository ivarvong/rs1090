#!/usr/bin/env bash
# Install (or reinstall) the rs1090-serve systemd unit on a Pi / Linux box.
#
# Usage:
#   sudo ./install.sh
#
# What it does:
#   - copies rs1090-serve.service to /etc/systemd/system/
#   - daemon-reloads
#   - enables the unit so it starts on boot (autostart after reboot)
#   - (re)starts it so it picks up any edits
#
# What it does NOT do:
#   - copy or build the binary (do that yourself; the unit's ExecStart
#     defaults to /usr/local/bin/rs1090-serve)
#   - edit the ExecStart line for your receiver location / outputs;
#     either edit the .service file before running this, or use
#     `sudo systemctl edit rs1090-serve` afterwards for a drop-in override
#
# Reapply at any time; existing state is preserved by enable/restart.

set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
unit="${here}/rs1090-serve.service"
dest="/etc/systemd/system/rs1090-serve.service"

if [[ $EUID -ne 0 ]]; then
    echo "error: must run as root (try \`sudo $0\`)" >&2
    exit 1
fi

if [[ ! -f "${unit}" ]]; then
    echo "error: ${unit} not found" >&2
    exit 1
fi

echo "installing unit -> ${dest}"
install -m 0644 "${unit}" "${dest}"

echo "systemctl daemon-reload"
systemctl daemon-reload

echo "systemctl enable rs1090-serve  (autostart on boot)"
systemctl enable rs1090-serve

echo "systemctl restart rs1090-serve  (apply current unit)"
systemctl restart rs1090-serve

echo
echo "done. status:"
systemctl --no-pager --full status rs1090-serve || true

cat <<'TIP'

Tail logs:           sudo journalctl -u rs1090-serve -f
Override ExecStart:  sudo systemctl edit rs1090-serve
Disable autostart:   sudo systemctl disable rs1090-serve
Stop now:            sudo systemctl stop rs1090-serve
TIP
