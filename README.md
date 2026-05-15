# rs1090

A Mode S / ADS-B decoder in Rust, designed to run real-time on a Raspberry Pi Zero W.

**Status:** pre-alpha, M1 in progress. See [DESIGN.md](./DESIGN.md) for the full design.

## Layout

- `crates/rs1090` — the library: sample sources, magnitude, demod, CRC, frame, decode, state.
- `crates/rs1090-cli` — command-line tool for decoding and replaying captures.
- `crates/rs1090-serve` — *(future)* HTTP/SSE server.

## Quick start

```sh
cargo test --workspace
```

## License

MIT. See [LICENSE](./LICENSE).
