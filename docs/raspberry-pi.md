# Running rs1090 on a Raspberry Pi

A practical walkthrough for deploying `rs1090-serve` onto a Raspberry Pi
with an RTL-SDR dongle. The numbers below are from a live Pi Zero 2 W
on the LAN, validated against real ADS-B traffic over New York airspace.

## Hardware

This guide is validated on:

| Item | Detail |
|------|--------|
| Board | Raspberry Pi Zero 2 W (Rev 1.0) — quad-core Cortex-A53 @ 1 GHz, ARMv8 / `aarch64` |
| OS | Debian 13 (trixie), kernel 6.12 (Raspberry Pi OS 64-bit) |
| RAM | 512 MB (≈416 MB usable after kernel + GPU split) |
| SDR | Generic Realtek RTL2832U + R820T tuner (`0bda:2838`) |
| Antenna | Any 1090 MHz monopole on the device's SMA jack |
| Network | Tailscale (no public exposure) |

**A note on the name.** This guide says "Pi Zero" but the validated
hardware is the **Pi Zero 2 W**, not the original ARMv6 Pi Zero W. The
Zero 2 W has a 64-bit quad-core SoC and is dramatically more capable;
the original Zero W (ARMv6, single-core, no NEON) is theoretically a
target for rs1090 but has not been measured. See the "Original Pi
Zero W (ARMv6)" section below for the current status.

## Quick start

1. **Cross-compile the binary on your dev machine.**

   `cargo-zigbuild` uses Zig as a cross-linker so any host can target
   any Linux. No need to install a separate GCC cross-toolchain.

   ```sh
   brew install zig                                       # macOS host
   cargo install cargo-zigbuild
   rustup target add aarch64-unknown-linux-gnu
   cargo zigbuild --release -p rs1090-serve \
       --target aarch64-unknown-linux-gnu
   ```

   Output: `target/aarch64-unknown-linux-gnu/release/rs1090-serve`
   (about 11 MB, dynamically linked against glibc). The Pi runs
   glibc 2.38 (Debian 13); Zig's glibc shim emits backward-compatible
   symbol versions so the same binary runs on any Debian-family Pi
   OS from Debian 12 (bookworm) onward.

2. **Copy it to the Pi.**

   ```sh
   scp target/aarch64-unknown-linux-gnu/release/rs1090-serve \
       ivar@<pi-tailscale-addr>:~/rs1090-serve
   ssh ivar@<pi-tailscale-addr> chmod +x ~/rs1090-serve
   ```

3. **Verify the SDR is talking.**

   On the Pi:

   ```sh
   sudo apt-get install -y rtl-sdr   # one-time, for `rtl_test`
   rtl_test -t                       # should find the device
   ```

   Output should include `Found 1 device(s):` and your tuner type.
   If permission errors appear, log out and back in (the standard
   `rtl-sdr` package installs udev rules that take effect on next
   login).

4. **Run.**

   ```sh
   ~/rs1090-serve --bind 0.0.0.0:8080 --reference <lat>,<lon> live
   ```

   `--bind 0.0.0.0:8080` makes the server reachable over Tailscale
   from any other machine on your tailnet. `--reference` lets the
   tracker fall back to local CPR decode when only an even-or-odd
   fragment is available (faster first fix on cold start).

5. **Watch it work, from your dev machine.**

   ```sh
   open http://<pi-tailscale-addr>:8080      # live map (UI)
   curl -s http://<pi-tailscale-addr>:8080/aircraft | jq
   curl -sN http://<pi-tailscale-addr>:8080/stream
   ```

## Observed performance

From a one-minute live run on the Pi Zero 2 W, antenna indoors near a
window in Brooklyn, looking at JFK / LGA / Newark traffic:

| Metric | Value |
|--------|-------|
| CPU | 13-17% of one core (≈3-4% of total quad-core capacity) |
| Resident memory (RSS) | ~5 MB |
| Virtual memory (VSZ) | ~344 MB (mostly mmap, not resident) |
| Aircraft tracked | 9 in 80 s, 4 with resolved positions |
| Frame throughput | ~100-200 Mode S messages/sec at this site |
| Cross-compile time | ~26 s incremental on M-series Mac |
| SoC temperature | 43.5°C under load, indoor ambient |

The pipeline ran comfortably below any resource ceiling. The headline
"designed for Pi Zero W" claim is concretely validated for the Zero 2 W
target with this much headroom to spare.

## Capturing IQ for offline replay

The same binary can record a raw I/Q stream while it decodes, useful
for differential testing, sharing reproducible captures, or replaying
a specific signal against a future decoder version. The CLI lives in
`rs1090-cli`:

```sh
# On the Pi
ssh ivar@<pi> ~/rs1090-cli live --record /tmp/capture.iq --duration-secs 60
# Then pull it back
scp ivar@<pi>:/tmp/capture.iq corpus/
```

Capture is signed-8-bit interleaved I/Q at 2 MS/s (≈4 MB/s, ≈240 MB/min).
The Pi Zero 2 W's microSD bandwidth handles this without dropping
samples; for longer captures consider a USB-attached fast storage
device, since 1 GB/h fills the system card quickly.

## Differential test using the captured file

From a dev machine:

```sh
scripts/diff_pymodes.py corpus/capture.iq
```

This runs `rs1090 replay` over the file and diffs every CRC-clean frame
against pyModeS. The Pi capture is now part of the differential-test
corpus, validating that the same binary produces identical decoded
output on the Pi's signal chain as the dev host's.

## Field-ready deployment (systemd, restart-on-failure, `/metrics`)

The "manual scp + run in the foreground" flow above is for first-light
exploration. Once it's working and you want it to survive reboots and
dongle hiccups, switch to the **scripted deploy + systemd path** in
[`docs/deploy.md`](deploy.md):

- Shipped systemd unit at `dist/systemd/rs1090-serve.service` (hardened:
  `Restart=on-failure`, `StartLimitIntervalSec=0`, `ProtectSystem=strict`,
  the usual sandboxing knobs).
- One-command remote deploy via `dist/deploy.sh` driven by
  `dist/.env` (gitignored, per-deployment values: PI host, user,
  receiver coordinates, output flags).
- Prometheus `/metrics` scrape table + Grafana / alerting playbook.
- End-to-end verification recipes for both crash-recovery and
  reboot-autostart.

## BLE peripheral (optional)

The `ble` feature on `rs1090-serve` exposes a Bluetooth Low Energy GATT
peripheral so iPhone / Android BLE debug apps (nRF Connect, LightBlue)
can subscribe to live aircraft data without any custom mobile code.
Three characteristics under one custom service:

- **count** (`u16` LE) — aircraft currently tracked. Read + notify.
- **nearest** (15 bytes packed binary: ICAO, alt/25, lat×1e6, lon×1e6,
  track×10) — the lowest-altitude aircraft with a position fix.
- **summary** (UTF-8) — short human-readable one-liner that fits the
  default 20-byte ATT MTU.

### Prerequisites on the Pi

```sh
sudo apt-get install libdbus-1-dev pkg-config
sudo usermod -aG bluetooth $USER
# log out and back in for the group change to take effect
```

### Build

Cross-compiling from a Mac via `cargo-zigbuild` fails on the libdbus
dependency unless you set up a sysroot — easier to build on the Pi:

```sh
# On the dev machine: push the source over
rsync -azh --exclude target --exclude .git --exclude corpus \
    rs1090/ ivar@<pi>:~/rs1090/

# On the Pi: install rustup, then build
ssh ivar@<pi>
curl --proto =https --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal
source ~/.cargo/env
cd ~/rs1090
nice -n 10 cargo build --profile pi -p rs1090-serve --features ble -j 1
```

Use `-j 1` and `nice` — the Zero 2 W has 416 MB usable RAM, and parallel
rustc instances will thrash swap. The `--profile pi` profile inherits
the release settings but disables LTO; the workspace `release` profile's
LTO link step OOMs the Zero 2 W even with `-j 1` (peak rustc RSS during
the link is ~700 MB). Expect ~30 min for a clean build.

### Run

```sh
~/rs1090/target/pi/rs1090-serve --ble --bind 0.0.0.0:8080 live
```

### Verify from iPhone

Install [nRF Connect for Mobile](https://apps.apple.com/us/app/nrf-connect-for-mobile/id1054362403),
open it, tap **Scanner**, look for the device named `rs1090`. Tap it to
connect. The custom service UUID starts `10901090-…` — expand it to see
the three characteristics. Tap the **notify** icon on any of them to
subscribe; values update live as aircraft come and go on the SDR.

### Caveats

- BLE range is ~10 m — same room as the Pi works, "across the apartment" doesn't.
- iOS only scans BLE while the app is foregrounded. Background updates
  would need iBeacon region monitoring + a custom app — not the
  debug-app path.
- The Pi Zero 2 W's BCM43436 chip handles WiFi and BT on one antenna;
  coexistence is fine at our data rates but heavy WiFi traffic can
  briefly delay BLE advertisements.

## Original Pi Zero W (ARMv6) — status

The original Raspberry Pi Zero W (BCM2835, single-core ARMv6 @ 1 GHz,
no NEON) is the design target named in `DESIGN.md`. It has **not** yet
been validated end-to-end. To attempt it:

```sh
rustup target add arm-unknown-linux-gnueabihf
cargo zigbuild --release -p rs1090-serve --target arm-unknown-linux-gnueabihf
```

Expected concerns to verify:
- The `alpha-max-beta-min` magnitude function is integer-only and
  should run fine without NEON.
- Allocation patterns are already validated (the `FrameDetector`
  preallocates its scratch buffer; see the
  `process_is_zero_allocation_in_steady_state` test).
- Single-core CPU budget: on the Zero 2 W's quad A53 we see ~14% of
  one core. Translating to the Zero W's ARMv6 the single core is ~3-4×
  slower per-clock — so expect 40-60% of one Zero W core.
- 512 MB RAM is plenty; current measured RSS is ~5 MB.

When validated, replace this section with the measured numbers and
update DESIGN.md accordingly.

## Troubleshooting

**`rs1090-serve: error while loading shared libraries`**
The cross-compiled binary depends on glibc on the Pi. Make sure the
Pi is running Debian 12 (bookworm) or later, or rebuild against
musl with `--target aarch64-unknown-linux-musl`.

**`rs1090-serve: opening RTL-SDR device: usb error`**
The user isn't in the `plugdev` group, or the udev rules from the
`rtl-sdr` package didn't trigger. Quick check:
`ls -l /dev/bus/usb/$(lsusb | grep RTL | awk '{print $2"/"$4}' | tr -d ':')`

**Server starts but `/aircraft` always empty**
Antenna placement — 1090 MHz is line-of-sight. Move the antenna near
a window and away from electronics. `rtl_test -t` will not show RF
activity (it just tests the tuner); to confirm RF reception, run
`rtl_adsb` and watch hex messages flow.

**High CPU on Pi Zero W (the original ARMv6)**
Not validated yet — see the section above. If you hit a ceiling
there, the most promising lever is swapping
`magnitude::alpha_max_beta_min` for `magnitude::lut` (lookup table:
one indexed load per sample, vs. the current two abs + max + min +
shift sequence). The table is 128 KiB which fits the Zero W's L2.
