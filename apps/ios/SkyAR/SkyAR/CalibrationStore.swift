// CalibrationStore.swift — compass-bias state.
//
// The iPhone's magnetometer is good to maybe ±5° outdoors with a
// clean horizon, much worse near buildings, cars, or any
// ferromagnetic mass (yes, including some balcony railings). For an
// AR overlay where 1° = ~14 m of placement error at 1 km range, we
// want an external reference.
//
// Solution: at app launch, ask the user to point the phone at the
// sun. We know exactly where the sun should be (lat/lon + UTC →
// closed-form solar position; see Geo.swift). The phone reports
// where it thinks it's pointing. The difference is the compass
// bias. Store that bias and rotate every subsequent aircraft
// placement by it.
//
// This is the same trick aviators used pre-GPS to fix inertial-
// platform heading drift: shoot the sun, compare to expected,
// correct. We're doing it for an AR phone instead of an INS.

import Foundation
import Combine

@MainActor
final class CalibrationStore: ObservableObject {
    /// Degrees to add to ARKit's reported heading to get true
    /// heading. `nil` means the user hasn't calibrated yet, in
    /// which case the AR scene falls back to raw ARKit heading
    /// (it's usable; just less accurate).
    @Published private(set) var compassOffsetDeg: Double?

    /// When the calibration was captured. Older calibrations are
    /// less trustworthy — the magnetometer can drift over time,
    /// especially after walking past a parked car or other big
    /// metal object. The UI nudges for a refresh when the offset
    /// is stale.
    @Published private(set) var calibratedAt: Date?

    /// Sun's predicted azimuth at the moment of the last sighting,
    /// for the calibration view's display.
    @Published private(set) var lastSunAzimuthDeg: Double?

    /// Apply a sun sighting. `cameraBearingDeg` is what ARKit
    /// thought the camera was pointing at (true-north heading,
    /// already passed through ARKit's `gravityAndHeading` frame);
    /// the receiver location + UTC give us the sun's actual
    /// azimuth, and the difference is the bias.
    func calibrate(
        cameraBearingDeg: Double,
        latitudeDeg: Double, longitudeDeg: Double,
        at date: Date = Date()
    ) {
        let (az, _) = solarPosition(
            date: date, latitudeDeg: latitudeDeg, longitudeDeg: longitudeDeg
        )
        self.lastSunAzimuthDeg = az
        self.compassOffsetDeg = compassOffsetDeg(
            cameraBearingDeg: cameraBearingDeg, sunAzimuthDeg: az
        )
        self.calibratedAt = date
    }

    /// Forget the calibration. Used by the UI's "reset" button.
    func reset() {
        compassOffsetDeg = nil
        calibratedAt = nil
        lastSunAzimuthDeg = nil
    }

    /// Convenience: have we calibrated, and was it recent?
    func isFresh(maxAgeSeconds: TimeInterval = 600) -> Bool {
        guard let at = calibratedAt else { return false }
        return Date().timeIntervalSince(at) <= maxAgeSeconds
    }
}
