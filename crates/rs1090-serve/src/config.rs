//! Persistent TOML config for `rs1090-serve`.
//!
//! Schema mirrors the CLI flags so anything that can be passed on the
//! command line has a typed home in `/etc/rs1090/serve.toml`. The
//! deploy story is:
//!
//! - **Both present**: explicit CLI flags win, the config file fills in
//!   the rest, clap defaults catch anything neither sets.
//! - **Config only**: `rs1090-serve --config /etc/rs1090/serve.toml`
//!   takes everything from the file (typical systemd install).
//! - **CLI only**: same as before — the binary needs no config file.
//!
//! Precedence is resolved in `main::apply_config` using clap's
//! [`ValueSource`] so we can tell `--bind 0.0.0.0:8080` (explicit)
//! from a clap-supplied default with the same string value.

use std::fs;
use std::net::SocketAddr;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use clap::parser::ValueSource;
use clap::ArgMatches;
use serde::Deserialize;

use rs1090::cpr::LatLon;

/// Top-level config. Every field is optional so any subset of the
/// schema is a valid config — missing keys fall through to the CLI
/// flag's clap default.
#[derive(Debug, Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub bind: Option<String>,
    /// Receiver reference as `"lat,lon"`. Same form as `--reference`
    /// on the CLI so operators can copy-paste without thinking about
    /// it. Parsed by [`parse_latlon_string`] during merge.
    pub reference: Option<String>,
    pub min_confidence: Option<u8>,

    #[serde(default)]
    pub outputs: Outputs,

    pub source: Option<SourceConfig>,
}

#[derive(Debug, Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct Outputs {
    #[serde(default)]
    pub avr: NetOutput,
    #[serde(default)]
    pub beast: NetOutput,
    #[serde(default)]
    pub gdl90: Gdl90Output,
    #[serde(default)]
    pub ble: BoolOutput,
}

/// Inbound TCP output (AVR-text, Beast). Listens on `bind` when
/// enabled; the binary defaults the port if `bind` is omitted.
#[derive(Debug, Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct NetOutput {
    #[serde(default)]
    pub enabled: bool,
    pub bind: Option<SocketAddr>,
}

/// Outbound UDP output. `target` is who we send to (typically the
/// LAN broadcast `255.255.255.255:4000` or a unicast EFB device).
#[derive(Debug, Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct Gdl90Output {
    #[serde(default)]
    pub enabled: bool,
    pub target: Option<SocketAddr>,
}

/// Toggle-only output (BLE peripheral has no per-instance config
/// here — the GATT service shape is fixed).
#[derive(Debug, Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct BoolOutput {
    #[serde(default)]
    pub enabled: bool,
}

/// `[source]` section. Tagged on `kind = "live" | "file"` so the
/// shape mirrors the CLI subcommands one-to-one.
#[derive(Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "lowercase", deny_unknown_fields)]
pub enum SourceConfig {
    Live {
        #[serde(default)]
        device: Option<usize>,
        #[serde(default)]
        gain_tenth_db: Option<i32>,
        #[serde(default)]
        auto_gain: Option<bool>,
        #[serde(default)]
        bias_t: Option<bool>,
    },
    File {
        path: PathBuf,
        #[serde(default)]
        sample_rate: Option<u32>,
        #[serde(default)]
        center_freq: Option<u32>,
        #[serde(default)]
        realtime: Option<bool>,
    },
}

impl Config {
    /// Read + parse a TOML file. Surface its path in any error so
    /// operators don't have to guess which config was wrong.
    pub fn load(path: &Path) -> Result<Self> {
        let raw = fs::read_to_string(path)
            .with_context(|| format!("reading config file {}", path.display()))?;
        toml::from_str(&raw).with_context(|| format!("parsing config file {}", path.display()))
    }
}

/// Re-parse `"lat,lon"` from a TOML string. Duplicates the CLI's
/// `parse_latlon` to keep the modules decoupled; not on the hot path.
pub fn parse_latlon_string(s: &str) -> Result<LatLon> {
    let (lat, lon) = s
        .split_once(',')
        .with_context(|| format!("expected `lat,lon`, got `{s}`"))?;
    let lat_deg: f64 = lat
        .trim()
        .parse()
        .with_context(|| format!("bad latitude in `{s}`"))?;
    let lon_deg: f64 = lon
        .trim()
        .parse()
        .with_context(|| format!("bad longitude in `{s}`"))?;
    if !(-90.0..=90.0).contains(&lat_deg) || !(-180.0..=180.0).contains(&lon_deg) {
        anyhow::bail!("reference out of range: {lat_deg},{lon_deg}");
    }
    Ok(LatLon { lat_deg, lon_deg })
}

/// Returns `true` if the user explicitly passed `--<name>` on the
/// command line (vs. the value coming from a clap default or env).
///
/// `matches.value_source(name)` returns `None` for boolean flags that
/// weren't set (their `default_value` is `false` implicitly); we want
/// to treat that as "not from CLI" so the config can override.
pub fn cli_explicit(matches: &ArgMatches, name: &str) -> bool {
    matches!(matches.value_source(name), Some(ValueSource::CommandLine))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_full_config() {
        let toml_src = r#"
            bind = "0.0.0.0:8080"
            reference = "40.7, -74.0"
            min_confidence = 50

            [outputs.avr]
            enabled = true

            [outputs.beast]
            enabled = true
            bind = "0.0.0.0:30099"

            [outputs.gdl90]
            enabled = true
            target = "192.168.1.100:4000"

            [outputs.ble]
            enabled = false

            [source]
            kind = "live"
            auto_gain = true
            bias_t = false
        "#;
        let cfg: Config = toml::from_str(toml_src).expect("parse");
        assert_eq!(cfg.bind.as_deref(), Some("0.0.0.0:8080"));
        assert_eq!(cfg.reference.as_deref(), Some("40.7, -74.0"));
        assert!(cfg.outputs.avr.enabled);
        assert_eq!(
            cfg.outputs.beast.bind,
            Some("0.0.0.0:30099".parse().unwrap())
        );
        assert_eq!(
            cfg.outputs.gdl90.target,
            Some("192.168.1.100:4000".parse().unwrap())
        );
        match cfg.source.expect("source set") {
            SourceConfig::Live { auto_gain, .. } => assert_eq!(auto_gain, Some(true)),
            SourceConfig::File { .. } => panic!("expected live"),
        }
    }

    #[test]
    fn empty_config_is_valid() {
        let cfg: Config = toml::from_str("").expect("parse");
        assert!(cfg.bind.is_none());
        assert!(cfg.source.is_none());
        assert!(!cfg.outputs.avr.enabled);
    }

    #[test]
    fn file_source_requires_path() {
        let err = toml::from_str::<Config>("[source]\nkind = \"file\"\n");
        assert!(err.is_err(), "file source without path should fail");
    }

    #[test]
    fn unknown_keys_rejected() {
        let err = toml::from_str::<Config>("nonexistent_field = 42\n");
        assert!(err.is_err(), "unknown top-level key should fail");
    }

    #[test]
    fn parse_latlon_accepts_canonical_form() {
        let p = parse_latlon_string("40.5, -74.0").expect("valid");
        assert!((p.lat_deg - 40.5).abs() < 1e-9);
        assert!((p.lon_deg - -74.0).abs() < 1e-9);
    }

    #[test]
    fn parse_latlon_rejects_out_of_range() {
        assert!(parse_latlon_string("91.0,-74.0").is_err());
        assert!(parse_latlon_string("40.0,-181.0").is_err());
    }
}
