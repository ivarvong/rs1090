# SkyAR

An iOS app that overlays callsigns on real aircraft in your camera view, in real time, fed by your own ADS-B receiver running `rs1090-serve`. End-to-end: SDR → decoder → SSE → AR scene, no cloud aggregator in the middle.

## What this app does

- Connects to `rs1090-serve` over SSE.
- Asks CoreLocation for your position; tells the server to filter to aircraft within 60 nm.
- For each aircraft with a known position, computes the bearing and elevation from you to it; places a billboarded text label at that direction in the AR scene.
- One-tap **sun calibration** at startup to correct the iPhone's magnetometer bias against the sun's known true azimuth.

## Setup

### One-time, on this Mac

1. **Create the Xcode project.** Open Xcode → *File → New → Project → iOS → App*. Use these settings:
   - **Product Name:** `SkyAR`
   - **Interface:** SwiftUI
   - **Language:** Swift
   - **Storage:** None
   - **Include Tests:** Yes (optional but nice for the geo-math tests)
   - **Save in:** `apps/ios/` of this repo. Xcode will create `apps/ios/SkyAR/SkyAR.xcodeproj` and a `SkyAR/` source folder next to the source files this README ships alongside.
2. **Replace the auto-generated `SkyARApp.swift` and `ContentView.swift`** with the versions already in `apps/ios/SkyAR/SkyAR/`. The simplest way: drag the existing six files (`SkyARApp.swift`, `ContentView.swift`, `ARScene.swift`, `CalibrationView.swift`, `Geo.swift`, `AircraftEvent.swift`, `StreamClient.swift`, `AircraftStore.swift`, `CalibrationStore.swift`) into the *Project Navigator* in Xcode and choose **Copy items if needed: off**, **Added folders: Create groups**, **Add to targets: SkyAR**. They live on disk where they already are; Xcode just learns about them.
3. **Info.plist usage descriptions** are required for camera + location. In Xcode's project settings → *Info → Custom iOS Target Properties*, add:
   - `NSCameraUsageDescription` → `"Show aircraft labels over the live camera view."`
   - `NSLocationWhenInUseUsageDescription` → `"Used to compute bearing and elevation to each aircraft from your current position."`
4. **App Transport Security exception** for the LAN/Tailscale server (plaintext HTTP). In *Custom iOS Target Properties*, add `App Transport Security Settings` (dictionary) → `Allow Arbitrary Loads` → `YES`. Acceptable for personal use on a tailnet; if you ever expose the server to the open internet, terminate TLS at a reverse proxy and remove this exception.
5. **Signing.** Project settings → *Signing & Capabilities* → tick *Automatically manage signing* and pick your Apple ID as the team. Free dev profile is enough for personal sideloading.

### Per-deployment

- The server URL is hard-coded in `ContentView.swift` (look for `serverURL`). Default is `http://zulu-1:8080`, matching the example `dist/.env`. Edit if your receiver is named differently.

## Running it

1. Plug the iPhone into the Mac with USB-C.
2. In Xcode, pick your phone in the device dropdown (top centre).
3. Cmd-R to build + install.
4. On the phone, grant the camera + location permissions when prompted.

## Calibrating

The compass on every iPhone is approximate — usually ±5° outdoors, much worse indoors or near anything ferrous. For AR labels to land on the right aircraft, we need to know how wrong the compass is right now. The sun is the best outdoor reference: we know exactly where it should be from your latitude / longitude / UTC.

1. **Outside**, with the sun visible (or visible recently — refraction means it can be calibrated a degree below the horizon at dawn/dusk).
2. Tap **Calibrate** in the top-right corner.
3. Aim the phone roughly at the sun — *do not look directly at the sun*. Look at the screen's reticle, not at the sky.
4. When the brightest spot in the camera view sits in the yellow reticle, tap **Sun is centered**.
5. The HUD now shows the corrective offset (e.g., `+3.2°`) and a sun-fill icon turns yellow.

Calibration goes stale over time as you walk past metal objects, get in a car, etc. A grey sun icon means the last calibration was over 10 minutes ago and you should re-do it.

## Architecture notes

```
ContentView ──► ARScene ─────────► ARKit (ARWorldTrackingConfig, gravityAndHeading)
     │             ▲
     │             │
     ▼          places labels by bearing+elevation, rotated by compass offset
AircraftStore     /
     ▲           /
     │          /
StreamClient ──┘  (Last-Event-ID resumes via replay buffer)
     ▲
     │
rs1090-serve   ──►  /stream?origin_lat=…&origin_lon=…&max_distance_nm=60
```

Aircraft labels are placed at a fixed *near* distance (~80 m) regardless of true range, so far-away aircraft don't shrink to nothing and near-overhead aircraft don't pop past the camera's near plane. The whole scene rotates by `compassOffsetDeg` from `CalibrationStore` so a calibrated app shows aircraft above the right rooftops.

## Limitations

- **iOS 17+** only.
- **Magnetometer drift is real.** Even with sun calibration, walking past a parked car can knock the heading by a degree or two. Recalibrate when an aircraft label visibly lags the actual aircraft.
- **No occlusion.** Labels appear in front of buildings; we don't depth-test against the environment. Acceptable for a v1 demo; an `ARMeshManager`-based occlusion pass is a natural follow-up.
- **No flight enrichment.** We show callsign / altitude / track from what's broadcast; we don't yet cross-reference against an airline / flight-number database to show origin/destination. Easy follow-up via OpenSky's network API.
