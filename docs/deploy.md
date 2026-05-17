# Deploying rs1090-serve to a Raspberry Pi

A field-ready deployment runbook for putting `rs1090-serve` on a Pi
behind your antenna and keeping it running across reboots and USB
hiccups.

The reference target is a **Raspberry Pi Zero 2 W** running 64-bit
Raspberry Pi OS (Debian 13 "Trixie"). The same procedure works on a
Pi 3 / 4 / 5; for an older 32-bit Pi swap `PI_TARGET` to
`armv7-unknown-linux-gnueabihf`.

## What this runbook gives you

- A binary at `/usr/local/bin/rs1090-serve`.
- A systemd unit at `/etc/systemd/system/rs1090-serve.service` that
  **comes back automatically after a reboot** (`WantedBy=multi-user.target`)
  **and after a crash** (`Restart=on-failure`, `RestartSec=5s`,
  `StartLimitIntervalSec=0` so flap-protection never gives up).
- `/healthz` for load-balancer / Caddy probes.
- `/metrics` for Prometheus scraping.
- A one-command redeploy path for new builds.

## Prerequisites (one-time setup)

### On the Mac

```sh
# Cross-compile toolchain.
brew install rustup zig
rustup target add aarch64-unknown-linux-gnu
cargo install cargo-zigbuild

# Tailscale (or any other way to reach the Pi by name). The deploy
# script doesn't care how SSH resolves PI_HOST — Tailscale MagicDNS,
# /etc/hosts, an ~/.ssh/config alias all work.
```

### On the Pi

```sh
# RTL-SDR userspace + udev rules so non-root processes can open the
# dongle. Adds an `rtl-sdr` group; users who need device access should
# be in `plugdev` (the group the udev rules grant).
sudo apt-get install -y rtl-sdr
sudo usermod -aG plugdev "$USER"
# Unplug + replug the dongle (or reboot) so the new udev rules apply.
```

A receiver location helps decoding (local-CPR fallback when no
even/odd pair is available) — capture it once, put it in `dist/.env`.

## Per-deployment config

The deploy script reads from `dist/.env`, which is gitignored. Copy
the example and fill in your values:

```sh
cp dist/.env.example dist/.env
$EDITOR dist/.env
```

The fields:

| Variable | Example | Notes |
|---|---|---|
| `PI_HOST` | `zulu-1` | Anything `ssh` accepts: Tailscale name, IP, `~/.ssh/config` alias |
| `PI_USER` | `ivar` | Account on the Pi; must be in `plugdev` |
| `RS1090_BIND` | `0.0.0.0:8080` | `0.0.0.0` for LAN/Tailnet, `127.0.0.1` for loopback-only |
| `RS1090_REFERENCE` | `40.70214,-73.98262` | `lat,lon` decimal degrees; empty = disable local CPR |
| `RS1090_EXTRA_FLAGS` | `--gdl90 --avr --beast` | Output protocol enables, min-confidence, etc. |
| `RS1090_SOURCE` | `live --auto-gain` | Subcommand: `live …` (RTL-SDR) or `file PATH …` (replay) |
| `PI_TARGET` | `aarch64-unknown-linux-gnu` | Rust target triple |

## Deploy

```sh
dist/deploy.sh
```

This:

1. Builds `rs1090-serve` for `$PI_TARGET` (skip with `--no-build` if
   you already have a fresh binary in `target/$PI_TARGET/release/`).
2. `scp`s the binary to `/tmp/rs1090-serve.new`, then `sudo install`s
   it into `/usr/local/bin/rs1090-serve` (atomic replace; surviving
   journalctl IDs stay coherent).
3. Renders the unit at `dist/systemd/rs1090-serve.service` with the
   `ExecStart` and `User=` lines substituted from `.env`, ships it +
   the installer to the Pi.
4. Runs `dist/systemd/install.sh` over SSH: `daemon-reload`, `enable`
   (so it starts on the next boot), `restart` (so it picks up this
   build).
5. Polls `http://$PI_HOST:$PORT/healthz` until it answers `ok`.

Rerun any time. Idempotent.

## Verify it'll survive a reboot

Two failure modes worth testing in this order, because the second is
the one you can only catch in advance.

**Crash recovery** — the `LivenessGuard` in
`crates/rs1090-serve/src/main.rs` turns any abnormal decoder exit
(panic, returned `Err`, dropped on unwind) into a non-zero process
exit; systemd's `Restart=on-failure` brings it back inside
`RestartSec=5s`. The same path covers a yanked / failed RTL-SDR
dongle: `rs-rtl` gives up after five consecutive bulk-transfer
errors, the live source returns `Ok(0)`, the decoder loop turns
that into an `anyhow::bail!` (live only — file replay still EOFs
cleanly), and systemd restarts.

```sh
ssh $PI_USER@$PI_HOST 'sudo pkill -SIGABRT rs1090-serve'
sleep 8
curl -sS http://$PI_HOST:8080/healthz       # should print 'ok'
ssh $PI_USER@$PI_HOST 'systemctl is-active rs1090-serve'   # active
```

**Boot autostart** — the actual question from "I just rebooted the
Pi." `systemctl enable` (run by the installer) is what makes this
work; the reboot test proves it.

```sh
ssh $PI_USER@$PI_HOST 'sudo reboot'
# Pi Zero 2 W is back inside ~45s. Give it a minute.
sleep 75
curl -sS http://$PI_HOST:8080/healthz       # should print 'ok'
ssh $PI_USER@$PI_HOST 'systemctl is-active rs1090-serve'   # active
ssh $PI_USER@$PI_HOST 'systemctl is-enabled rs1090-serve'  # enabled
```

## Day-2 operations

```sh
# Tail logs (structured via tracing).
ssh $PI_USER@$PI_HOST 'sudo journalctl -u rs1090-serve -f'

# Crank verbosity without changing the binary.
ssh $PI_USER@$PI_HOST \
    'sudo systemctl set-environment RUST_LOG=debug && sudo systemctl restart rs1090-serve'

# Site-specific override that won't get clobbered by the next deploy.
ssh -t $PI_USER@$PI_HOST 'sudo systemctl edit rs1090-serve'
# Empty ExecStart= first, then your replacement:
#   [Service]
#   ExecStart=
#   ExecStart=/usr/local/bin/rs1090-serve --bind 0.0.0.0:8080 …

# Scrape /metrics by hand.
curl -s http://$PI_HOST:8080/metrics | grep -E '^rs1090_'

# Stop / disable.
ssh $PI_USER@$PI_HOST 'sudo systemctl stop rs1090-serve'
ssh $PI_USER@$PI_HOST 'sudo systemctl disable rs1090-serve'
```

## Prometheus metrics

`/metrics` exposes five low-cardinality series — point a scraper at
`http://$PI_HOST:8080/metrics` and you get:

| Metric | Type | Purpose |
|---|---|---|
| `rs1090_frames_total{outcome=clean\|corrected\|failed}` | counter | per-frame throughput, broken out by CRC outcome |
| `rs1090_state_events_total{kind=acquired\|identification\|position\|velocity\|address_recovered\|orphan\|lost}` | counter | per-event tracker output |
| `rs1090_aircraft_tracked` | gauge | current tracker table size |
| `rs1090_sse_subscribers` | gauge | live SSE clients on `/stream` |
| `rs1090_decoder_alive` | gauge | 1 while the decoder thread is alive, 0 once it has died |

Plenty for a Grafana panel — frames/sec rate, corrected-frame ratio
(noise-floor proxy), aircraft trend, alert on
`rs1090_decoder_alive == 0` (or just `up == 0`, since the LivenessGuard
exits the process and the scrape goes silent).

There are no per-ICAO labels on purpose — that would blow up the
time-series count for no operational gain. Aircraft state lives in
the tracker, not in metrics.

## Troubleshooting

**`/healthz` returns 503 "decoder dead" right after deploy.** Almost
always the dongle isn't accessible — udev rules didn't apply, the user
isn't in `plugdev`, or the cable came loose. `journalctl -u rs1090-serve`
will have the specific error from the decoder thread.

**Builds OOM on the Pi.** Don't build on the Pi. The Mac-side
`cargo zigbuild` path in `deploy.sh` is the supported route — the Pi
Zero 2 W's 512 MB isn't enough to link `rs1090-serve` with LTO. If
you must build on-device, use `--profile pi` (LTO off; see
`Cargo.toml`).

**Decode rate looks low.** Check `rs1090_frames_total{outcome="failed"}`
— if it dominates `clean`, the receiver is hearing noise more than
signal. Antenna placement and amp/filter matter way more than any
software knob; see `docs/sdr.md`.
