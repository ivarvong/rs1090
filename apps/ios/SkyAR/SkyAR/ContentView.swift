// ContentView.swift — top-level app surface. Holds the singleton
// stores, owns the CoreLocation manager, drives the SSE stream, and
// composes the AR scene with the calibration overlay.

import SwiftUI
import CoreLocation

struct ContentView: View {
    @StateObject private var aircraft = AircraftStore()
    @StateObject private var calibration = CalibrationStore()
    @StateObject private var location = LocationProvider()
    @State private var showingCalibration: Bool = false

    /// Server URL. Hard-coded for v1 to keep the surface area
    /// minimal; flip to a `@AppStorage` text field if/when we want
    /// to support multiple receivers.
    private let serverURL = URL(string: "http://zulu-1:8080")!

    /// Max distance to ask the server to filter on. 60 nm is the
    /// rough practical line-of-sight horizon for 1090 MHz at
    /// realistic antenna heights; bumping past it just gets you
    /// aircraft that can't be in the camera frame regardless.
    private let maxNm: Double = 60

    var body: some View {
        ZStack(alignment: .top) {
            ARScene(
                aircraft: aircraft,
                calibration: calibration,
                observerLocation: { location.current }
            )
            .ignoresSafeArea()

            // HUD overlay.
            VStack {
                HStack {
                    countBadge
                    Spacer()
                    calibrationBadge
                }
                .padding(.horizontal)
                .padding(.top, 8)
                Spacer()
            }
        }
        .sheet(isPresented: $showingCalibration) {
            CalibrationView(
                calibration: calibration,
                observerLocation: { location.current },
                onDone: { showingCalibration = false }
            )
        }
        .task {
            // Prompt for location, then start the SSE stream once
            // we have a fix.
            location.start()
            while location.current == nil {
                try? await Task.sleep(nanoseconds: 200_000_000)
                if Task.isCancelled { return }
            }
            let here = location.current!
            let client = StreamClient(baseURL: serverURL)
            let events = await client.events(
                originLat: here.coordinate.latitude,
                originLon: here.coordinate.longitude,
                maxNm: maxNm
            )
            for await event in events {
                await MainActor.run { aircraft.apply(event) }
            }
        }
        .task {
            // Periodically prune the in-memory store. The server
            // emits explicit `lost` events but a reconnect window
            // can drop one; this is cheap insurance.
            while !Task.isCancelled {
                try? await Task.sleep(nanoseconds: 30_000_000_000)
                aircraft.evictStale()
            }
        }
    }

    private var countBadge: some View {
        Text("\(aircraft.byIcao.count) aircraft")
            .font(.callout.monospacedDigit())
            .padding(.horizontal, 12).padding(.vertical, 6)
            .background(.ultraThinMaterial, in: Capsule())
    }

    @ViewBuilder
    private var calibrationBadge: some View {
        Button {
            showingCalibration = true
        } label: {
            HStack(spacing: 6) {
                Image(systemName: calibration.isFresh()
                      ? "sun.max.fill" : "sun.max")
                if let off = calibration.compassOffsetDeg {
                    Text(String(format: "%+.1f°", off))
                        .font(.caption.monospacedDigit())
                } else {
                    Text("Calibrate")
                }
            }
            .padding(.horizontal, 12).padding(.vertical, 6)
            .background(.ultraThinMaterial, in: Capsule())
        }
    }
}

/// Thin wrapper around `CLLocationManager` that publishes the most
/// recent fix. CoreLocation's delegate-callback API doesn't play
/// nicely with SwiftUI; this adapter does.
@MainActor
final class LocationProvider: NSObject, ObservableObject, CLLocationManagerDelegate {
    @Published var current: CLLocation?
    private let manager = CLLocationManager()

    override init() {
        super.init()
        manager.delegate = self
        manager.desiredAccuracy = kCLLocationAccuracyBest
        manager.distanceFilter = 5
    }

    func start() {
        manager.requestWhenInUseAuthorization()
        manager.startUpdatingLocation()
    }

    nonisolated func locationManager(
        _ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]
    ) {
        guard let loc = locations.last else { return }
        Task { @MainActor in self.current = loc }
    }
}
