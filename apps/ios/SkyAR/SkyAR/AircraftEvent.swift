// AircraftEvent.swift — `Codable` models for the rs1090-serve SSE
// wire format. One enum (`AircraftEvent`) per event tag, plus a
// flat `LiveAircraft` struct that the store maintains across events.

import Foundation

/// Tagged-union of every SSE event the server emits today. The
/// `event:` line in the SSE wire format selects the variant; the
/// `data:` line is one of these struct payloads as JSON.
enum AircraftEvent {
    case acquired(Acquired)
    case identification(Identification)
    case position(Position)
    case velocity(Velocity)
    case lost(Lost)
    case addressRecovered(AddressRecovered)

    /// Build an event from `(event: tag, data: jsonPayload)` as read
    /// from the SSE stream. Returns `nil` if the tag isn't one we
    /// know about — rs1090-serve may emit new event types over time
    /// (heartbeats, dropped notifications, etc.) and we'd rather
    /// skip an unknown event than crash a client.
    static func decode(tag: String, json: Data) -> AircraftEvent? {
        do {
            let d = JSONDecoder()
            switch tag {
            case "acquired":
                return try .acquired(d.decode(Acquired.self, from: json))
            case "identification":
                return try .identification(d.decode(Identification.self, from: json))
            case "position":
                return try .position(d.decode(Position.self, from: json))
            case "velocity":
                return try .velocity(d.decode(Velocity.self, from: json))
            case "lost":
                return try .lost(d.decode(Lost.self, from: json))
            case "address_recovered":
                return try .addressRecovered(d.decode(AddressRecovered.self, from: json))
            default:
                return nil
            }
        } catch {
            return nil
        }
    }

    var icao: String {
        switch self {
        case .acquired(let e): return e.icao
        case .identification(let e): return e.icao
        case .position(let e): return e.icao
        case .velocity(let e): return e.icao
        case .lost(let e): return e.icao
        case .addressRecovered(let e): return e.icao
        }
    }

    struct Acquired: Decodable {
        let v: Int
        let t: String
        let icao: String
    }

    struct Identification: Decodable {
        let v: Int
        let t: String
        let icao: String
        let callsign: String
    }

    struct Position: Decodable {
        let v: Int
        let t: String
        let icao: String
        let lat: Double
        let lon: Double
        /// Aircraft altitude in feet. Absent for surface positions
        /// and for airborne frames where the aircraft is not
        /// reporting altitude.
        let alt_ft: Int?
        /// `"baro"` or `"gnss"`. Absent when `alt_ft` is absent.
        let alt_source: String?
        /// `"global"`, `"local"`, or `"surface"`.
        let source: String
    }

    struct Velocity: Decodable {
        let v: Int
        let t: String
        let icao: String
        let gs_mps: Double?
        let track_deg: Double?
        let ias_mps: Double?
        let heading_deg: Double?
        let heading_magnetic: Bool?
        let vr_mps: Double?
        let vr_source: String?
    }

    struct Lost: Decodable {
        let v: Int
        let t: String
        let icao: String
    }

    struct AddressRecovered: Decodable {
        let v: Int
        let t: String
        let icao: String
        let df: Int
    }
}

/// Flattened per-aircraft state the store maintains over time. Built
/// up by folding the event stream; consumed by the AR scene to
/// place + label each known aircraft.
struct LiveAircraft: Identifiable {
    let icao: String
    var callsign: String?
    var latitude: Double?
    var longitude: Double?
    var altitudeFeet: Int?
    var groundSpeedMps: Double?
    var trackDeg: Double?
    var lastSeen: Date
    /// Position source tag from the most recent position event —
    /// "global", "local", or "surface".
    var positionSource: String?

    var id: String { icao }

    /// True if we have enough to place a label in the AR scene.
    /// Aircraft we've only heard `acquired` for don't get drawn.
    var hasPosition: Bool { latitude != nil && longitude != nil }
}
