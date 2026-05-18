// AircraftStore.swift — observable aircraft state. Folds the SSE
// event stream into a dictionary of `LiveAircraft` keyed by ICAO;
// the AR scene observes this and (re)places labels on each tick.

import Foundation
import Combine

@MainActor
final class AircraftStore: ObservableObject {
    @Published private(set) var byIcao: [String: LiveAircraft] = [:]
    @Published private(set) var lastUpdate: Date = .distantPast

    func apply(_ event: AircraftEvent) {
        let icao = event.icao
        var a = byIcao[icao] ?? LiveAircraft(
            icao: icao, callsign: nil, latitude: nil, longitude: nil,
            altitudeFeet: nil, groundSpeedMps: nil, trackDeg: nil,
            lastSeen: Date(), positionSource: nil
        )
        a.lastSeen = Date()

        switch event {
        case .acquired:
            break // nothing more to record
        case .identification(let e):
            a.callsign = e.callsign
        case .position(let e):
            a.latitude = e.lat
            a.longitude = e.lon
            a.altitudeFeet = e.alt_ft
            a.positionSource = e.source
        case .velocity(let e):
            a.groundSpeedMps = e.gs_mps
            a.trackDeg = e.track_deg ?? e.heading_deg
        case .lost:
            byIcao.removeValue(forKey: icao)
            lastUpdate = Date()
            return
        case .addressRecovered:
            break
        }

        byIcao[icao] = a
        lastUpdate = Date()
    }

    /// Drop aircraft we haven't heard from in a while. The server's
    /// state tracker emits `lost` events on eviction, so this is a
    /// belt-and-suspenders catch for clients that disconnected
    /// during the eviction.
    func evictStale(olderThan seconds: TimeInterval = 90) {
        let cutoff = Date().addingTimeInterval(-seconds)
        byIcao = byIcao.filter { $0.value.lastSeen >= cutoff }
    }

    /// Aircraft with a known position, sorted by ground distance to
    /// the observer (used for the on-screen "nearest" indicator).
    func placeable(observerLat: Double, observerLon: Double) -> [LiveAircraft] {
        byIcao.values
            .filter { $0.hasPosition }
            .sorted { lhs, rhs in
                let dl = haversineMeters(
                    lat1Deg: observerLat, lon1Deg: observerLon,
                    lat2Deg: lhs.latitude!, lon2Deg: lhs.longitude!
                )
                let dr = haversineMeters(
                    lat1Deg: observerLat, lon1Deg: observerLon,
                    lat2Deg: rhs.latitude!, lon2Deg: rhs.longitude!
                )
                return dl < dr
            }
    }
}
