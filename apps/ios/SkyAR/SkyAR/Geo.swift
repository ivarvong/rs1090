// Geo.swift — everything that converts between world coordinates,
// observer-relative angles, and ARKit's right-handed gravity-aligned
// world frame.
//
// ARKit world frame (when ARWorldTrackingConfiguration uses
// `gravityAndHeading`):
//   +x = east, +y = up, -z = north
//
// All angles in this file are in *degrees* unless the variable name
// says otherwise. Conversion to radians is a one-line property, and
// keeping the public surface in degrees matches the wire format
// (lat/lon, bearing, elevation) so callers don't accidentally pass
// the wrong unit.

import Foundation
import simd

/// Earth radius for haversine. Mean radius rather than equatorial —
/// at the receive radii we care about (≤ ~250 nm), the 0.3 %
/// difference between equatorial and mean is well below GPS noise.
private let earthRadiusMeters: Double = 6_371_000.0

/// Convert a feet altitude (ADS-B native) to meters.
private let feetToMeters: Double = 0.3048

/// Distance in meters between two `(lat, lon)` points on the sphere.
/// Standard haversine; accurate to <0.5 % anywhere on Earth, more
/// than good enough for placing AR labels.
func haversineMeters(
    lat1Deg: Double, lon1Deg: Double, lat2Deg: Double, lon2Deg: Double
) -> Double {
    let lat1 = lat1Deg * .pi / 180
    let lat2 = lat2Deg * .pi / 180
    let dLat = (lat2Deg - lat1Deg) * .pi / 180
    let dLon = (lon2Deg - lon1Deg) * .pi / 180
    let a = sin(dLat / 2) * sin(dLat / 2)
        + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2)
    let c = 2 * asin(min(1, sqrt(a)))
    return earthRadiusMeters * c
}

/// Initial bearing (true-north heading) from point 1 to point 2,
/// degrees clockwise from north, in `[0, 360)`.
///
/// "Initial" matters: along a great-circle route the bearing changes,
/// but for the short distances relevant to AR (line-of-sight to
/// aircraft), the initial bearing is what aligns with what the phone
/// is pointing at right now.
func initialBearingDegrees(
    lat1Deg: Double, lon1Deg: Double, lat2Deg: Double, lon2Deg: Double
) -> Double {
    let lat1 = lat1Deg * .pi / 180
    let lat2 = lat2Deg * .pi / 180
    let dLon = (lon2Deg - lon1Deg) * .pi / 180
    let y = sin(dLon) * cos(lat2)
    let x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon)
    let bearingRad = atan2(y, x)
    let deg = bearingRad * 180 / .pi
    return deg.truncatingRemainder(dividingBy: 360).positiveMod(360)
}

/// Angle above the horizon (positive = up) from observer to a target
/// at the given altitude difference and ground distance.
///
/// `groundDistanceMeters` is the great-circle distance; for line-of-
/// sight to aircraft within ~50 nm we ignore Earth's curvature (the
/// drop is ≤ 0.06° at 50 nm). For very-long-range aircraft this
/// underestimates slightly — acceptable for an AR overlay.
func elevationAngleDegrees(
    altitudeDifferenceMeters: Double, groundDistanceMeters: Double
) -> Double {
    // Guard against the asymptote when the aircraft is directly
    // overhead (ground distance → 0). atan2 handles it naturally.
    let rad = atan2(altitudeDifferenceMeters, max(groundDistanceMeters, 1))
    return rad * 180 / .pi
}

/// Convenience: full observer→target geometry given the receiver's
/// position (`obs`) and the aircraft's position + altitude.
struct AircraftGeometry {
    /// Great-circle ground distance, meters.
    let groundDistanceMeters: Double
    /// Bearing from observer to aircraft, degrees clockwise from
    /// true north, `[0, 360)`.
    let bearingDeg: Double
    /// Angle above horizon, degrees, positive = up.
    let elevationDeg: Double
}

func geometry(
    observer obsLat: Double, _ obsLon: Double, observerAltitudeMeters: Double,
    aircraft acLat: Double, _ acLon: Double, aircraftAltitudeFeet: Int?
) -> AircraftGeometry {
    let d = haversineMeters(
        lat1Deg: obsLat, lon1Deg: obsLon,
        lat2Deg: acLat, lon2Deg: acLon
    )
    let b = initialBearingDegrees(
        lat1Deg: obsLat, lon1Deg: obsLon,
        lat2Deg: acLat, lon2Deg: acLon
    )
    let acAltitudeMeters = Double(aircraftAltitudeFeet ?? 0) * feetToMeters
    let altDelta = acAltitudeMeters - observerAltitudeMeters
    let e = elevationAngleDegrees(
        altitudeDifferenceMeters: altDelta, groundDistanceMeters: d
    )
    return AircraftGeometry(
        groundDistanceMeters: d, bearingDeg: b, elevationDeg: e
    )
}

/// Position in ARKit world space at a fixed near distance from the
/// observer, given a bearing + elevation. We render labels at a
/// fixed distance (~80 m) and scale them by actual range so the
/// camera's near/far planes don't clip them and labels for far-away
/// aircraft don't shrink to nothing.
///
/// `compassOffsetDeg` is the calibration constant from
/// [`CalibrationStore`]: positive values rotate the world to the right
/// (clockwise) to correct a leftward compass bias.
func arPosition(
    bearingDeg: Double,
    elevationDeg: Double,
    nearDistanceMeters: Double = 80,
    compassOffsetDeg: Double = 0
) -> SIMD3<Float> {
    let theta = (bearingDeg - compassOffsetDeg) * .pi / 180
    let phi = elevationDeg * .pi / 180
    let r = nearDistanceMeters
    // ARKit world frame: +x east, +y up, -z north.
    let x = r * cos(phi) * sin(theta)
    let y = r * sin(phi)
    let z = -r * cos(phi) * cos(theta)
    return SIMD3<Float>(Float(x), Float(y), Float(z))
}

// MARK: - Solar position

/// Sun's apparent position in the sky at a given UTC instant and
/// observer location. Returns (azimuth °, elevation °).
///
/// Algorithm: NOAA Solar Position Algorithm, simplified to the
/// terms that matter at our precision needs (we just need to know
/// where the sun is in the sky to ≈0.1° so we can use it as a
/// compass reference). Accurate to ~0.01° in elevation and ~0.05°
/// in azimuth between 1900 and 2100, refraction-corrected at the
/// horizon level. Good enough that the limiting error in any
/// sun-based calibration will be how well the user centers the sun
/// in their reticle, not this math.
///
/// References: Reda & Andreas, "Solar Position Algorithm for Solar
/// Radiation Applications", NREL/TP-560-34302 (2008); Astronomical
/// Almanac §C, low-precision formulas.
func solarPosition(date: Date, latitudeDeg: Double, longitudeDeg: Double)
    -> (azimuthDeg: Double, elevationDeg: Double)
{
    // Julian Day from a Date (Unix epoch is 2440587.5 JD).
    let secondsSinceEpoch = date.timeIntervalSince1970
    let jd = secondsSinceEpoch / 86400 + 2_440_587.5
    let n = jd - 2_451_545.0 // days since J2000.0

    // Mean longitude (deg), mean anomaly (deg).
    let L = (280.460 + 0.985_647_4 * n).positiveMod(360)
    let g = ((357.528 + 0.985_600_3 * n).positiveMod(360)) * .pi / 180

    // Ecliptic longitude (deg).
    let lambda = L + 1.915 * sin(g) + 0.020 * sin(2 * g)
    let lambdaRad = lambda * .pi / 180

    // Obliquity of the ecliptic (deg).
    let epsilon = (23.439 - 0.000_000_4 * n) * .pi / 180

    // Right ascension & declination (rad).
    let alpha = atan2(cos(epsilon) * sin(lambdaRad), cos(lambdaRad))
    let delta = asin(sin(epsilon) * sin(lambdaRad))

    // Greenwich Mean Sidereal Time (hours, then to degrees).
    let gmstHours = (18.697_374_558 + 24.065_709_824_419_08 * n)
        .truncatingRemainder(dividingBy: 24)
    let gmstDeg = (gmstHours < 0 ? gmstHours + 24 : gmstHours) * 15.0

    // Local hour angle (rad).
    let H = ((gmstDeg + longitudeDeg) * .pi / 180) - alpha

    // Altitude (elevation) and azimuth.
    let phi = latitudeDeg * .pi / 180
    let sinAlt = sin(phi) * sin(delta) + cos(phi) * cos(delta) * cos(H)
    let alt = asin(sinAlt)
    let cosAz = (sin(delta) - sinAlt * sin(phi)) / (cos(alt) * cos(phi))
    // sinAz is needed only for quadrant; from the standard form below.
    let sinAz = -cos(delta) * sin(H) / cos(alt)
    let azRad = atan2(sinAz, cosAz)

    var altitudeDeg = alt * 180 / .pi
    var azimuthDeg = (azRad * 180 / .pi).positiveMod(360)

    // Approximate atmospheric refraction at the horizon (Saemundsson
    // formula), so a sun "centered" by eye matches the math.
    if altitudeDeg > -1.0 {
        let h = altitudeDeg
        let refractionArcmin =
            1.02 / tan(((h + 10.3 / (h + 5.11)) * .pi) / 180.0)
        altitudeDeg += refractionArcmin / 60.0
    }
    // Stay tidy with the azimuth wrap.
    if azimuthDeg < 0 { azimuthDeg += 360 }

    return (azimuthDeg: azimuthDeg, elevationDeg: altitudeDeg)
}

// MARK: - Compass offset from a sun sighting

/// Solve for the compass offset given a single sighting: the user
/// pointed the phone at the sun, ARKit's compass-aligned heading
/// reported `cameraBearingDeg`, and we computed the sun's true
/// azimuth as `sunAzimuthDeg`.
///
/// Returns the offset to add to ARKit's reported heading to get
/// true heading, in degrees, wrapped to `(-180, 180]`.
///
/// Example: if the sun is truly at 200° but ARKit's camera reports
/// it at 210°, the compass reads 10° too high; this returns -10°
/// (so subsequent placements compensate by rotating left by 10°).
func compassOffsetDeg(cameraBearingDeg: Double, sunAzimuthDeg: Double)
    -> Double
{
    let raw = sunAzimuthDeg - cameraBearingDeg
    return wrapPlusMinus180(raw)
}

/// Wrap `deg` to `(-180, 180]`.
func wrapPlusMinus180(_ deg: Double) -> Double {
    var d = deg.truncatingRemainder(dividingBy: 360)
    if d > 180 { d -= 360 }
    if d <= -180 { d += 360 }
    return d
}

// MARK: - Helpers

extension Double {
    /// `self mod m`, always non-negative. Stdlib's
    /// `truncatingRemainder(dividingBy:)` can return negative values.
    fileprivate func positiveMod(_ m: Double) -> Double {
        let r = self.truncatingRemainder(dividingBy: m)
        return r < 0 ? r + m : r
    }
}
