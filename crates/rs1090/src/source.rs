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

// --- RTL-SDR live backend ---------------------------------------------------

#[cfg(feature = "rtl-sdr")]
pub use rtl_sdr_backend::{RtlSdrSource, RtlSdrSourceBuilder};

#[cfg(feature = "rtl-sdr")]
mod rtl_sdr_backend {
    //! Live `SampleSource` backed by a real RTL-SDR dongle via `rs-rtl`.
    //!
    //! `rs-rtl` runs its own USB I/O thread (15 in-flight bulk transfers,
    //! 32 KiB each by default) and delivers unsigned-8-bit interleaved I/Q
    //! chunks through a bounded channel. We block on `recv()` and convert
    //! each byte to signed by subtracting 128 — the same bias subtraction
    //! that the offline `python3` pipeline used to do.
    //!
    //! Lint exemption: technical terms (RTL-SDR, USB, librtlsdr, AGC) aren't
    //! Rust items.

    #![allow(clippy::doc_markdown)]

    use std::io;

    use rs_rtl::rtlsdr::{AsyncReadHandle, RtlSdr};

    use super::{Iq, SampleSource};

    /// Frequency in Hz that ADS-B uses. Exposed as a const so callers don't
    /// have to remember `1_090_000_000`.
    pub const ADS_B_FREQ_HZ: u32 = 1_090_000_000;

    /// Canonical ADS-B sample rate. The receiver code assumes exactly 2 MS/s
    /// throughout; setting a different rate here makes the decoder produce
    /// wrong results, not just slower ones.
    pub const ADS_B_SAMPLE_RATE_HZ: u32 = 2_000_000;

    /// Builder for an [`RtlSdrSource`].
    ///
    /// Default settings match the configuration that worked for our live
    /// captures: 2 MS/s, 1090 MHz, manual gain at the device's maximum.
    /// The `rs-rtl` driver auto-clamps gain to the nearest supported value.
    #[derive(Debug, Clone)]
    pub struct RtlSdrSourceBuilder {
        index: usize,
        sample_rate: u32,
        center_freq: u32,
        /// Gain in tenths of dB, or `None` for AGC.
        gain_tenth_db: Option<i32>,
        bias_t: bool,
    }

    impl Default for RtlSdrSourceBuilder {
        fn default() -> Self {
            Self {
                index: 0,
                sample_rate: ADS_B_SAMPLE_RATE_HZ,
                center_freq: ADS_B_FREQ_HZ,
                // 40 dB worked well in our outdoor LGA-proximate captures.
                // Adjustable per environment.
                gain_tenth_db: Some(400),
                bias_t: false,
            }
        }
    }

    impl RtlSdrSourceBuilder {
        #[must_use]
        pub fn new() -> Self {
            Self::default()
        }

        /// Open device by 0-based index.
        #[must_use]
        pub fn device_index(mut self, index: usize) -> Self {
            self.index = index;
            self
        }

        #[must_use]
        pub fn sample_rate(mut self, hz: u32) -> Self {
            self.sample_rate = hz;
            self
        }

        #[must_use]
        pub fn center_freq(mut self, hz: u32) -> Self {
            self.center_freq = hz;
            self
        }

        /// Manual gain in tenths of dB. Common useful values: 197 (19.7 dB),
        /// 297 (29.7 dB), 408 (40.8 dB), 496 (49.6 dB). The driver clamps
        /// to the nearest supported step.
        #[must_use]
        pub fn gain_tenth_db(mut self, g: i32) -> Self {
            self.gain_tenth_db = Some(g);
            self
        }

        /// Use the tuner's automatic gain control.
        #[must_use]
        pub fn auto_gain(mut self) -> Self {
            self.gain_tenth_db = None;
            self
        }

        /// Enable bias-T (phantom power on the antenna port). Off by default
        /// because feeding 5V into an antenna that doesn't expect it is bad.
        #[must_use]
        pub fn bias_t(mut self, enable: bool) -> Self {
            self.bias_t = enable;
            self
        }

        /// Open the device and start streaming.
        ///
        /// # Errors
        ///
        /// Returns the underlying USB/driver error mapped onto `io::Error`.
        pub fn open(self) -> io::Result<RtlSdrSource> {
            let mut dev = RtlSdr::open(self.index).map_err(map_err)?;
            dev.set_sample_rate(self.sample_rate).map_err(map_err)?;
            dev.set_center_freq(self.center_freq).map_err(map_err)?;
            if let Some(g) = self.gain_tenth_db {
                dev.set_gain_manual(g).map_err(map_err)?;
            } else {
                dev.set_gain_auto().map_err(map_err)?;
            }
            if self.bias_t {
                dev.set_bias_t(true).map_err(map_err)?;
            }
            let actual_sr = dev.sample_rate();
            let actual_cf = dev.center_freq();
            let handle = dev.start_streaming().map_err(map_err)?;
            Ok(RtlSdrSource {
                _device: Box::new(dev),
                handle,
                sample_rate: actual_sr,
                center_freq: actual_cf,
                leftover: Vec::new(),
                cursor: 0,
            })
        }
    }

    /// Live RTL-SDR sample source. Created via [`RtlSdrSourceBuilder`].
    pub struct RtlSdrSource {
        /// Kept alive so the streaming thread isn't stopped by Drop until
        /// the source is dropped. The handle borrows internal channel state
        /// rooted in the device, but rs-rtl exposes them by-value with
        /// internal Arc'd channels, so this is just defensive ownership.
        _device: Box<RtlSdr>,
        handle: AsyncReadHandle,
        sample_rate: u32,
        center_freq: u32,
        /// Bytes from the most recent USB chunk that haven't been
        /// converted into the caller's `Iq` buffer yet.
        leftover: Vec<u8>,
        /// Read cursor within `leftover`.
        cursor: usize,
    }

    impl std::fmt::Debug for RtlSdrSource {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            f.debug_struct("RtlSdrSource")
                .field("sample_rate", &self.sample_rate)
                .field("center_freq", &self.center_freq)
                .field("leftover_len", &(self.leftover.len() - self.cursor))
                .finish_non_exhaustive()
        }
    }

    impl SampleSource for RtlSdrSource {
        fn sample_rate(&self) -> u32 {
            self.sample_rate
        }

        fn center_freq(&self) -> u32 {
            self.center_freq
        }

        fn read(&mut self, out: &mut [Iq]) -> io::Result<usize> {
            if out.is_empty() {
                return Ok(0);
            }
            // If we have no buffered bytes, block on the next USB chunk.
            if self.cursor >= self.leftover.len() {
                match self.handle.recv() {
                    Some(chunk) => {
                        self.leftover = chunk;
                        self.cursor = 0;
                    }
                    None => {
                        // Streaming thread exited (device disconnected
                        // or stop requested).
                        return Ok(0);
                    }
                }
            }
            let available_bytes = self.leftover.len() - self.cursor;
            // Each sample is 2 bytes (I, Q).
            let max_pairs = out.len().min(available_bytes / 2);
            for (k, slot) in out.iter_mut().enumerate().take(max_pairs) {
                let lo = self.cursor + k * 2;
                // RTL-SDR delivers unsigned-biased-at-128. Subtract 128
                // with wrapping_sub and reinterpret via from_ne_bytes so
                // the u8→i8 mapping is explicit instead of a lint-silenced
                // `as` cast.
                let i = i8::from_ne_bytes([self.leftover[lo].wrapping_sub(128)]);
                let q = i8::from_ne_bytes([self.leftover[lo + 1].wrapping_sub(128)]);
                *slot = Iq::new(i, q);
            }
            self.cursor += max_pairs * 2;
            Ok(max_pairs)
        }
    }

    impl Drop for RtlSdrSource {
        fn drop(&mut self) {
            self.handle.stop();
        }
    }

    // Take the error by value so the resulting `io::Error` can own its
    // formatted message without borrowing.
    #[allow(clippy::needless_pass_by_value)]
    fn map_err(e: rs_rtl::error::Error) -> io::Error {
        io::Error::other(e.to_string())
    }
}
