//! Convert rs1090's signed-8 `.iq` recording format to `UC8` (unsigned-8,
//! RTL-SDR raw-byte format) on stdin → stdout.
//!
//! rs1090 stores bias-subtracted samples as `i8` (each byte was the raw
//! RTL byte minus 128 at capture time). Most third-party Mode-S tools —
//! `dump1090`, GNU Radio source blocks, `FlightAware` feeders — expect
//! the original `UC8` stream centred around 127.5. The bias is restored
//! per-byte by flipping the sign bit, i.e. `b ^ 0x80`, which is
//! identical to `(b + 128) mod 256`.
//!
//! Streaming and stateless: a single fixed-size buffer, no
//! [`std::io::BufReader`]/[`std::io::BufWriter`] indirection because we
//! own buffering directly and the per-read cost dominates only at very
//! small chunk sizes.

use std::io::{self, Read, Write};

/// 64 KiB — far above the OS pipe buffer (16 KiB on macOS / Linux), so
/// every `read`/`write` corresponds to a single syscall in the common
/// case, and well below L1 working set on any platform we target.
const CHUNK_BYTES: usize = 64 * 1024;

fn main() -> io::Result<()> {
    let mut reader = io::stdin().lock();
    let mut writer = io::stdout().lock();
    // Direct heap allocation — `Box::new([0; N])` materialises the array
    // on the stack first, which trips `clippy::large_stack_arrays` at
    // this size. One allocation for the lifetime of the process.
    let mut buf: Box<[u8]> = vec![0; CHUNK_BYTES].into_boxed_slice();

    loop {
        let n = reader.read(&mut buf)?;
        if n == 0 {
            return Ok(());
        }
        for byte in &mut buf[..n] {
            *byte ^= 0x80;
        }
        writer.write_all(&buf[..n])?;
    }
}
