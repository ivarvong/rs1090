//! Sample sources: the boundary between the SDR and the demodulator.
//!
//! `std`-only because every SDR backend ultimately calls into `std::io`.
//! The trait is intentionally tiny — a sample rate, a center frequency, and
//! a blocking read of `Iq` samples into a caller-supplied buffer.
//!
//! Backends live in their own modules (or feature-gated crates) and
//! implement [`SampleSource`] mechanically. The only one in this crate is
//! [`IqFileSource`], which replays a raw signed-8-bit interleaved I/Q file.
//!
//! Lint exemption: technical terms (SDR, RTL-SDR, ARMv6) aren't Rust items.

#![allow(clippy::doc_markdown)]

use std::io::{self, Read};

use crate::Iq;

/// Read I/Q samples from some upstream source.
///
/// Implementations are synchronous; the Pi Zero W has no useful async story
/// for this workload and `async fn` in the trait would color the library
/// without buying us anything. SDR backends that need their own threads run
/// them internally and feed a ring buffer that [`read`](Self::read) pops from.
pub trait SampleSource {
    /// Sample rate in Hz. For ADS-B at 1090 MHz the canonical rate is 2 MS/s.
    fn sample_rate(&self) -> u32;

    /// Center frequency in Hz. For ADS-B this is `1_090_000_000`.
    fn center_freq(&self) -> u32;

    /// Read up to `out.len()` samples into `out` and return the number written.
    ///
    /// Blocks until at least one sample is available or the source has
    /// permanently failed. A return of `Ok(0)` means end-of-stream (file
    /// exhausted, device disconnected).
    fn read(&mut self, out: &mut [Iq]) -> io::Result<usize>;
}

/// Replay a file of interleaved signed-8-bit I/Q samples.
///
/// File layout: `i0 q0 i1 q1 i2 q2 ...`, each byte interpreted as `i8`.
/// This is the canonical post-bias-subtraction RTL-SDR format. The file's
/// sample rate and center frequency are not embedded in the format itself,
/// so the caller supplies them.
#[derive(Debug)]
pub struct IqFileSource<R: Read> {
    reader: R,
    sample_rate: u32,
    center_freq: u32,
    /// Scratch buffer for byte-level reads. Sized to one cache line per
    /// sample pair.
    byte_buf: [u8; 4096],
}

impl<R: Read> IqFileSource<R> {
    /// Wrap a reader. Caller is responsible for supplying the sample rate
    /// and center frequency that match the file's contents.
    pub fn new(reader: R, sample_rate: u32, center_freq: u32) -> Self {
        Self {
            reader,
            sample_rate,
            center_freq,
            byte_buf: [0; 4096],
        }
    }
}

impl<R: Read> SampleSource for IqFileSource<R> {
    fn sample_rate(&self) -> u32 {
        self.sample_rate
    }

    fn center_freq(&self) -> u32 {
        self.center_freq
    }

    fn read(&mut self, out: &mut [Iq]) -> io::Result<usize> {
        // Each sample is 2 bytes (i, q). Read as many full samples as we
        // can fit in either the caller's buffer or our byte scratch,
        // whichever is smaller.
        let max_samples = out.len().min(self.byte_buf.len() / 2);
        if max_samples == 0 {
            return Ok(0);
        }
        let max_bytes = max_samples * 2;

        // Read may return fewer bytes than requested; loop until we have
        // a whole number of samples or hit EOF.
        let mut got = 0usize;
        while got < max_bytes {
            match self.reader.read(&mut self.byte_buf[got..max_bytes]) {
                Ok(0) => break, // EOF
                Ok(n) => got += n,
                Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
                Err(e) => return Err(e),
            }
        }

        // Round down to whole samples; if we got a trailing half-sample at
        // EOF, drop it on the floor (the caller can detect EOF on a future
        // read returning 0).
        let pairs = got / 2;
        // The cast from u8 to i8 is the explicit intent of the file format:
        // raw bytes are signed 8-bit samples after the receiver's bias
        // subtraction. We use `i8::from_ne_bytes` so the conversion is
        // documented rather than a lint-suppressed `as`.
        for (k, slot) in out.iter_mut().enumerate().take(pairs) {
            let i = i8::from_ne_bytes([self.byte_buf[k * 2]]);
            let q = i8::from_ne_bytes([self.byte_buf[k * 2 + 1]]);
            *slot = Iq::new(i, q);
        }
        Ok(pairs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn file_source_reads_samples() {
        // Three samples: (1,2), (-3,-4), (127,-128).
        let bytes: &[u8] = &[1, 2, 0xFD, 0xFC, 0x7F, 0x80];
        let mut src = IqFileSource::new(bytes, 2_000_000, 1_090_000_000);
        let mut out = [Iq::default(); 4];
        let n = src.read(&mut out).unwrap();
        assert_eq!(n, 3);
        assert_eq!(out[0], Iq::new(1, 2));
        assert_eq!(out[1], Iq::new(-3, -4));
        assert_eq!(out[2], Iq::new(127, -128));
        // Subsequent read sees EOF.
        let n = src.read(&mut out).unwrap();
        assert_eq!(n, 0);
    }

    #[test]
    fn file_source_handles_short_reads() {
        // Drip-feed two samples one byte at a time via a custom reader.
        struct OneByteAtATime<'a>(&'a [u8]);
        impl Read for OneByteAtATime<'_> {
            fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
                if self.0.is_empty() || buf.is_empty() {
                    return Ok(0);
                }
                buf[0] = self.0[0];
                self.0 = &self.0[1..];
                Ok(1)
            }
        }
        let bytes: &[u8] = &[10, 20, 30, 40];
        let mut src = IqFileSource::new(OneByteAtATime(bytes), 2_000_000, 1_090_000_000);
        let mut out = [Iq::default(); 4];
        let n = src.read(&mut out).unwrap();
        assert_eq!(n, 2);
        assert_eq!(out[0], Iq::new(10, 20));
        assert_eq!(out[1], Iq::new(30, 40));
    }

    #[test]
    fn file_source_drops_trailing_half_sample() {
        // Odd-byte file: last byte is half a sample, must be ignored.
        let bytes: &[u8] = &[1, 2, 3];
        let mut src = IqFileSource::new(bytes, 2_000_000, 1_090_000_000);
        let mut out = [Iq::default(); 4];
        let n = src.read(&mut out).unwrap();
        assert_eq!(n, 1);
        assert_eq!(out[0], Iq::new(1, 2));
    }

    #[test]
    fn file_source_reports_rate_and_freq() {
        let bytes: &[u8] = &[];
        let src = IqFileSource::new(bytes, 2_400_000, 1_090_000_000);
        assert_eq!(src.sample_rate(), 2_400_000);
        assert_eq!(src.center_freq(), 1_090_000_000);
    }
}
