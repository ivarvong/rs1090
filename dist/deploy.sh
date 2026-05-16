#!/usr/bin/env bash
# Deploy rs1090-serve to a remote Linux host (typically a Raspberry Pi)
# and start it as a systemd service.
#
# Reads dist/.env for site-specific values (Pi hostname, user, receiver
# location, output flags). See dist/.env.example for the template.
#
# Steps:
#   1. Cross-compile rs1090-serve for $PI_TARGET via cargo-zigbuild.
#   2. scp the binary to /tmp on the Pi, then `sudo install` into
#      /usr/local/bin (atomic replace, sets 0755).
#   3. Render dist/systemd/rs1090-serve.service with the .env values
#      substituted into ExecStart, ship it to the Pi.
#   4. Run dist/systemd/install.sh on the Pi: daemon-reload, enable
#      (autostart on boot), restart (apply the new unit).
#   5. Wait for /healthz to come green over the network.
#
# Idempotent — rerun any time to push a new build or unit change.
#
# Usage:
#   dist/deploy.sh             # full deploy
#   dist/deploy.sh --no-build  # use whatever's already in target/$PI_TARGET/release/

set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
repo_root="$(cd "${here}/.." && pwd)"

if [[ ! -f "${here}/.env" ]]; then
    echo "error: ${here}/.env not found. Copy ${here}/.env.example and fill it in." >&2
    exit 1
fi

# shellcheck source=/dev/null
set -a; source "${here}/.env"; set +a

: "${PI_HOST:?PI_HOST not set in dist/.env}"
: "${PI_USER:?PI_USER not set in dist/.env}"
: "${PI_TARGET:?PI_TARGET not set in dist/.env}"
: "${RS1090_BIND:?RS1090_BIND not set in dist/.env}"
: "${RS1090_REFERENCE:=}"
: "${RS1090_EXTRA_FLAGS:=}"
: "${RS1090_SOURCE:?RS1090_SOURCE not set in dist/.env}"

ssh_target="${PI_USER}@${PI_HOST}"
do_build=1
for arg in "$@"; do
    case "$arg" in
        --no-build) do_build=0 ;;
        -h|--help)
            sed -n '2,/^$/p' "$0" | sed 's/^# //; s/^#//'
            exit 0
            ;;
        *) echo "unknown arg: $arg" >&2; exit 2 ;;
    esac
done

binary_path="${repo_root}/target/${PI_TARGET}/release/rs1090-serve"

if [[ $do_build -eq 1 ]]; then
    echo "==> cargo zigbuild --release --target ${PI_TARGET}"
    (cd "${repo_root}" && cargo zigbuild --release -p rs1090-serve --target "${PI_TARGET}")
fi

if [[ ! -x "${binary_path}" ]]; then
    echo "error: ${binary_path} not found (run without --no-build first)" >&2
    exit 1
fi

echo "==> scp binary -> ${ssh_target}:/tmp/rs1090-serve.new"
scp -q "${binary_path}" "${ssh_target}:/tmp/rs1090-serve.new"

echo "==> render unit with ExecStart from .env"
unit_src="${here}/systemd/rs1090-serve.service"
unit_rendered="$(mktemp)"
trap 'rm -f "${unit_rendered}"' EXIT

# Build the ExecStart line from .env values. Spaces inside RS1090_SOURCE
# / RS1090_EXTRA_FLAGS are intentional — they expand to multiple argv
# tokens, which is what we want for `--auto-gain` etc.
exec_start="/usr/local/bin/rs1090-serve --bind ${RS1090_BIND}"
if [[ -n "${RS1090_REFERENCE}" ]]; then
    exec_start+=" --reference ${RS1090_REFERENCE}"
fi
if [[ -n "${RS1090_EXTRA_FLAGS}" ]]; then
    exec_start+=" ${RS1090_EXTRA_FLAGS}"
fi
exec_start+=" ${RS1090_SOURCE}"

# Replace the multi-line ExecStart=... \ block in the template with one
# rendered single-line ExecStart=. Awk handles the line-continuation
# parsing so we don't need to know how many lines it currently spans.
awk -v repl="ExecStart=${exec_start}" -v user="${PI_USER}" '
    BEGIN { in_exec = 0 }
    /^ExecStart=/ { print repl; if ($0 ~ /\\$/) in_exec = 1; next }
    in_exec {
        if ($0 !~ /\\$/) in_exec = 0
        next
    }
    /^User=/ { print "User=" user; next }
    { print }
' "${unit_src}" > "${unit_rendered}"

echo "==> rsync unit + installer -> ${ssh_target}:/tmp/rs1090-systemd/"
# `mktemp` gives us 0600; bump to 0644 so the unit is world-readable
# once it lands on the Pi (rsync preserves mode by default).
chmod 0644 "${unit_rendered}"
ssh "${ssh_target}" 'mkdir -p /tmp/rs1090-systemd'
rsync -azh "${unit_rendered}" "${ssh_target}:/tmp/rs1090-systemd/rs1090-serve.service"
rsync -azh "${here}/systemd/install.sh" "${ssh_target}:/tmp/rs1090-systemd/install.sh"

echo "==> install binary + unit on ${PI_HOST}"
ssh "${ssh_target}" 'sudo install -m 0755 /tmp/rs1090-serve.new /usr/local/bin/rs1090-serve && rm -f /tmp/rs1090-serve.new'
ssh "${ssh_target}" 'sudo /tmp/rs1090-systemd/install.sh'

echo "==> wait for /healthz on http://${PI_HOST}${RS1090_BIND/0.0.0.0/}"
# Strip the bind-IP, keep the port. If RS1090_BIND is 0.0.0.0:8080,
# we hit ${PI_HOST}:8080 from the Mac side.
port="${RS1090_BIND##*:}"
for _ in $(seq 1 20); do
    if curl -sf --max-time 2 "http://${PI_HOST}:${port}/healthz" >/dev/null; then
        echo "    /healthz OK"
        break
    fi
    sleep 1
done

echo
echo "deployed. quick checks:"
echo "  curl http://${PI_HOST}:${port}/healthz       # should print 'ok'"
echo "  curl http://${PI_HOST}:${port}/metrics | head"
echo "  curl -sN http://${PI_HOST}:${port}/stream | head -5"
echo "  ssh ${ssh_target} 'sudo journalctl -u rs1090-serve -f'"
