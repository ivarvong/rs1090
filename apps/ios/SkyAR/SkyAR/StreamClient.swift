// StreamClient.swift — minimal SSE client over `URLSession.bytes`.
// Reads `event: …` and `data: …` lines from the server's
// `text/event-stream` response, packs them into `AircraftEvent`s,
// and yields each one through an `AsyncStream`. Reconnects on
// disconnection with `Last-Event-ID` to ride out network blips
// without losing events (rs1090-serve has a replay buffer we hook
// into automatically by sending the last id we saw).

import Foundation

actor StreamClient {
    private let baseURL: URL
    private var task: Task<Void, Never>?
    private var lastEventId: UInt64?

    init(baseURL: URL) {
        self.baseURL = baseURL
    }

    /// Open a long-lived connection to `/stream` with the given
    /// radius filter centered on the observer; yields `AircraftEvent`s
    /// as they arrive. The continuation finishes if the caller cancels
    /// the parent task.
    ///
    /// On network error or end-of-stream, retries after a short delay
    /// and sends `Last-Event-ID` so the server's replay buffer
    /// resends anything we missed (up to 4096 events of history).
    func events(
        originLat: Double, originLon: Double, maxNm: Double
    ) -> AsyncStream<AircraftEvent> {
        AsyncStream { continuation in
            let task = Task { [self] in
                while !Task.isCancelled {
                    do {
                        try await self.runOne(
                            originLat: originLat,
                            originLon: originLon,
                            maxNm: maxNm,
                            yield: { event in continuation.yield(event) }
                        )
                    } catch {
                        // Connection died (server restart, Wi-Fi
                        // handoff, etc.). Brief backoff, then
                        // reconnect with whatever Last-Event-ID we
                        // last saw.
                        try? await Task.sleep(nanoseconds: 1_500_000_000)
                    }
                }
                continuation.finish()
            }
            continuation.onTermination = { @Sendable _ in task.cancel() }
            Task { await self.setTask(task) }
        }
    }

    private func setTask(_ t: Task<Void, Never>) { self.task = t }

    private func runOne(
        originLat: Double, originLon: Double, maxNm: Double,
        yield: @Sendable (AircraftEvent) -> Void
    ) async throws {
        var components = URLComponents(
            url: baseURL.appendingPathComponent("stream"),
            resolvingAgainstBaseURL: false
        )!
        components.queryItems = [
            URLQueryItem(name: "origin_lat", value: String(originLat)),
            URLQueryItem(name: "origin_lon", value: String(originLon)),
            URLQueryItem(name: "max_distance_nm", value: String(maxNm)),
        ]
        var request = URLRequest(url: components.url!)
        request.setValue("text/event-stream", forHTTPHeaderField: "Accept")
        if let id = lastEventId {
            request.setValue(String(id), forHTTPHeaderField: "Last-Event-ID")
        }

        let (bytes, response) = try await URLSession.shared.bytes(for: request)
        guard let http = response as? HTTPURLResponse, http.statusCode == 200 else {
            throw URLError(.badServerResponse)
        }

        var currentEventTag: String? = nil
        var currentData = Data()
        var currentId: UInt64? = nil

        for try await line in bytes.lines {
            if line.isEmpty {
                // Blank line terminates an event. Flush.
                if let tag = currentEventTag,
                   let event = AircraftEvent.decode(tag: tag, json: currentData)
                {
                    yield(event)
                }
                if let id = currentId {
                    self.lastEventId = id
                }
                currentEventTag = nil
                currentData = Data()
                currentId = nil
                continue
            }
            if line.hasPrefix(":") {
                // SSE comment / keep-alive heartbeat; ignore.
                continue
            }
            if let value = strip(line, prefix: "event:") {
                currentEventTag = value
            } else if let value = strip(line, prefix: "data:") {
                if !currentData.isEmpty { currentData.append(UInt8(ascii: "\n")) }
                currentData.append(value.data(using: .utf8) ?? Data())
            } else if let value = strip(line, prefix: "id:"),
                      let id = UInt64(value)
            {
                currentId = id
            }
            // Anything else (retry: …, custom fields) is ignored.
        }
        // Server closed the connection. Loop reconnects.
    }

    /// SSE lines look like `field: value` with an optional single
    /// leading space after the colon. Strip the prefix and any
    /// one leading space.
    private func strip(_ line: String, prefix: String) -> String? {
        guard line.hasPrefix(prefix) else { return nil }
        var s = String(line.dropFirst(prefix.count))
        if s.hasPrefix(" ") { s.removeFirst() }
        return s
    }
}
