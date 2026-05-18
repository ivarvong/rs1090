// CalibrationView.swift — the one-tap sun sighting that grounds the
// AR overlay to true north. The user opens this view, lines the sun
// up with the on-screen reticle, and taps. We capture ARKit's
// current camera heading, compute the sun's true azimuth from time
// and location, and store the difference as `compassOffsetDeg` in
// CalibrationStore. From then on every aircraft label is placed in
// a frame that's been rotated to compensate for the magnetometer's
// bias.
//
// The user is told *not* to look directly at the sun: they aim the
// phone at it (the camera doesn't care), keep their eyes on the
// reticle, and tap when the bright spot in the camera frame is
// centered.

import SwiftUI
import ARKit
import RealityKit
import CoreLocation

struct CalibrationView: View {
    @ObservedObject var calibration: CalibrationStore
    let observerLocation: () -> CLLocation?
    let onDone: () -> Void

    // We attach to the same AR session as the main scene by using a
    // dedicated ARView here; it shares the world frame because both
    // configurations use gravityAndHeading anchored at the same pose.
    @State private var arViewBox = ARViewBox()
    @State private var status: String = "Aim the phone at the sun"

    private var sunPrediction: (az: Double, el: Double)? {
        guard let here = observerLocation() else { return nil }
        let p = solarPosition(
            date: Date(),
            latitudeDeg: here.coordinate.latitude,
            longitudeDeg: here.coordinate.longitude
        )
        return (p.azimuthDeg, p.elevationDeg)
    }

    var body: some View {
        ZStack {
            CalibrationARView(box: arViewBox)
                .ignoresSafeArea()
            VStack {
                Spacer()
                Text(status)
                    .font(.headline)
                    .padding(.horizontal, 18)
                    .padding(.vertical, 8)
                    .background(.ultraThinMaterial, in: Capsule())
                Spacer()
                // Crosshair reticle.
                ZStack {
                    Circle().stroke(.yellow, lineWidth: 3).frame(width: 64, height: 64)
                    Rectangle().fill(.yellow).frame(width: 24, height: 2)
                    Rectangle().fill(.yellow).frame(width: 2, height: 24)
                }
                Spacer()
                if let p = sunPrediction {
                    Text(String(
                        format: "Sun expected at %@%.0f° / %@%.0f° up",
                        compassQuadrant(p.az), p.az, p.el >= 0 ? "+" : "-",
                        abs(p.el)
                    ))
                    .font(.footnote)
                    .foregroundStyle(.secondary)
                } else {
                    Text("Waiting for location fix…")
                        .font(.footnote)
                        .foregroundStyle(.secondary)
                }
                Button(action: capture) {
                    Text("Sun is centered — tap to calibrate")
                        .font(.headline)
                        .padding(.horizontal, 20).padding(.vertical, 12)
                        .background(.yellow, in: Capsule())
                        .foregroundStyle(.black)
                }
                .padding(.bottom, 8)

                Button("Cancel", action: onDone)
                    .padding(.bottom, 16)
            }
            .padding()
        }
    }

    private func capture() {
        guard let here = observerLocation() else {
            status = "No location fix yet — wait a moment"
            return
        }
        guard let frame = arViewBox.view?.session.currentFrame else {
            status = "AR frame not ready"
            return
        }
        // ARKit's camera transform: column 3 = position, the third
        // column (z axis) points OUT OF the back of the phone (so
        // -z is where the camera is looking, in camera-local
        // coordinates). The transform is camera→world, so the
        // world-frame forward direction is -transform.columns.2.
        let t = frame.camera.transform
        let forwardWorld = SIMD3<Float>(-t.columns.2.x, -t.columns.2.y, -t.columns.2.z)
        // In gravityAndHeading: +x east, +y up, -z north.
        //   bearing from north (clockwise) = atan2(east_component, north_component)
        //                                 = atan2(forwardX, -forwardZ)
        let bearingRad = atan2(Double(forwardWorld.x), Double(-forwardWorld.z))
        var bearingDeg = bearingRad * 180 / .pi
        if bearingDeg < 0 { bearingDeg += 360 }

        calibration.calibrate(
            cameraBearingDeg: bearingDeg,
            latitudeDeg: here.coordinate.latitude,
            longitudeDeg: here.coordinate.longitude
        )
        onDone()
    }

    private func compassQuadrant(_ az: Double) -> String {
        switch az {
        case 0..<22.5, 337.5...360: return "N "
        case 22.5..<67.5: return "NE "
        case 67.5..<112.5: return "E "
        case 112.5..<157.5: return "SE "
        case 157.5..<202.5: return "S "
        case 202.5..<247.5: return "SW "
        case 247.5..<292.5: return "W "
        case 292.5..<337.5: return "NW "
        default: return ""
        }
    }
}

// Small box so SwiftUI's view can hand the ARView back to capture().
@MainActor
final class ARViewBox {
    var view: ARView?
}

private struct CalibrationARView: UIViewRepresentable {
    let box: ARViewBox
    func makeUIView(context: Context) -> ARView {
        let v = ARView(frame: .zero)
        let cfg = ARWorldTrackingConfiguration()
        cfg.worldAlignment = .gravityAndHeading
        cfg.planeDetection = []
        v.session.run(cfg)
        box.view = v
        return v
    }
    func updateUIView(_ uiView: ARView, context: Context) {}
}
