// ARScene.swift — SwiftUI wrapper around an ARKit/RealityKit
// `ARView`. The scene reads observed aircraft + the compass
// calibration offset, places one billboarded text entity per
// aircraft at the right bearing/elevation, and updates them in
// place as new SSE events arrive.

import SwiftUI
import ARKit
import RealityKit
import CoreLocation

struct ARScene: UIViewRepresentable {
    @ObservedObject var aircraft: AircraftStore
    @ObservedObject var calibration: CalibrationStore
    let observerLocation: () -> CLLocation?

    func makeCoordinator() -> Coordinator {
        Coordinator(
            aircraft: aircraft,
            calibration: calibration,
            observerLocation: observerLocation
        )
    }

    func makeUIView(context: Context) -> ARView {
        let view = ARView(frame: .zero)
        let config = ARWorldTrackingConfiguration()
        // gravityAndHeading puts +y up and -z towards true north
        // (subject to the magnetometer's compass accuracy, which is
        // exactly what CalibrationStore exists to correct).
        config.worldAlignment = .gravityAndHeading
        config.planeDetection = []
        config.environmentTexturing = .none
        view.session.run(config)

        // Anchor everything to a single world-fixed anchor at the
        // origin (the phone's startup pose). Per-aircraft entities
        // are children of this so they all rotate together when we
        // apply the calibration offset.
        let root = AnchorEntity(world: .zero)
        view.scene.anchors.append(root)
        context.coordinator.root = root

        // 60 fps refresh on the scene update tick is overkill — we
        // only need to reposition labels on new SSE events. We
        // observe the aircraft store and react in `updateUIView`.
        return view
    }

    func updateUIView(_ view: ARView, context: Context) {
        context.coordinator.refresh()
    }

    @MainActor
    final class Coordinator {
        let aircraft: AircraftStore
        let calibration: CalibrationStore
        let observerLocation: () -> CLLocation?
        var root: AnchorEntity?
        // ICAO → entity for in-place updates.
        var entities: [String: Entity] = [:]

        init(
            aircraft: AircraftStore,
            calibration: CalibrationStore,
            observerLocation: @escaping () -> CLLocation?
        ) {
            self.aircraft = aircraft
            self.calibration = calibration
            self.observerLocation = observerLocation
        }

        func refresh() {
            guard let root = root, let here = observerLocation() else { return }
            let placeable = aircraft.placeable(
                observerLat: here.coordinate.latitude,
                observerLon: here.coordinate.longitude
            )
            let nowIcaos = Set(placeable.map(\.icao))

            // Remove entities for aircraft that have aged out of the
            // store (lost or evicted).
            for (icao, entity) in entities where !nowIcaos.contains(icao) {
                entity.removeFromParent()
                entities.removeValue(forKey: icao)
            }

            let offset = calibration.compassOffsetDeg ?? 0
            for a in placeable {
                guard let lat = a.latitude, let lon = a.longitude else { continue }
                let geom = geometry(
                    observer: here.coordinate.latitude, here.coordinate.longitude,
                    observerAltitudeMeters: here.altitude,
                    aircraft: lat, lon,
                    aircraftAltitudeFeet: a.altitudeFeet
                )
                // Hide aircraft below the horizon — they can't be in
                // the camera frame even if we placed a label there.
                if geom.elevationDeg < -2 { continue }
                let pos = arPosition(
                    bearingDeg: geom.bearingDeg,
                    elevationDeg: geom.elevationDeg,
                    nearDistanceMeters: 80,
                    compassOffsetDeg: offset
                )

                let label = a.callsign?.trimmingCharacters(in: .whitespaces).nilIfEmpty
                    ?? a.icao
                let entity = entities[a.icao] ?? newLabel(text: label)
                if entity.parent == nil {
                    root.addChild(entity)
                    entities[a.icao] = entity
                }
                entity.position = pos
                // Update text if the callsign learned/changed.
                if let model = entity.children.first as? ModelEntity,
                   let _ = model.model
                {
                    rebuildLabelText(model: model, text: label)
                }
            }
        }

        /// Build a billboarded text label rendered at a consistent
        /// physical size regardless of distance (we render at fixed
        /// near-distance; see `arPosition`).
        private func newLabel(text: String) -> Entity {
            let model = ModelEntity()
            rebuildLabelText(model: model, text: text)
            // Billboard so the label always faces the camera.
            model.components.set(BillboardComponent())
            let parent = Entity()
            parent.addChild(model)
            return parent
        }

        private func rebuildLabelText(model: ModelEntity, text: String) {
            let mesh = MeshResource.generateText(
                text,
                extrusionDepth: 0.02,
                font: .systemFont(ofSize: 6, weight: .bold),
                containerFrame: .zero,
                alignment: .center,
                lineBreakMode: .byTruncatingTail
            )
            let material = UnlitMaterial(color: .white)
            model.model = ModelComponent(mesh: mesh, materials: [material])
        }
    }
}

private extension String {
    var nilIfEmpty: String? { isEmpty ? nil : self }
}
