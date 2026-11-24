#[path = "../../platform/build_time.rs"]
mod build_time;

use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

fn timestamp(seconds: i64, nanos: u32) -> SystemTime {
    let whole = if seconds < 0 {
        UNIX_EPOCH - Duration::from_secs(seconds.unsigned_abs())
    } else {
        UNIX_EPOCH + Duration::from_secs(seconds as u64)
    };
    whole + Duration::from_nanos(u64::from(nanos))
}

#[test]
fn utc_calendar_matches_known_epoch_leap_day_and_century_boundaries() {
    // Unix seconds are fixed independently of the formatter under test.
    let cases = [
        (0, "1970-01-01T00:00:00+00:00"),
        (-1, "1969-12-31T23:59:59+00:00"),
        (-2_203_891_201, "1900-02-28T23:59:59+00:00"),
        (-2_203_891_200, "1900-03-01T00:00:00+00:00"),
        (946_684_799, "1999-12-31T23:59:59+00:00"),
        (946_684_800, "2000-01-01T00:00:00+00:00"),
        (951_782_400, "2000-02-29T00:00:00+00:00"),
        (951_868_800, "2000-03-01T00:00:00+00:00"),
        (1_709_251_199, "2024-02-29T23:59:59+00:00"),
        (1_709_251_200, "2024-03-01T00:00:00+00:00"),
        (4_107_542_399, "2100-02-28T23:59:59+00:00"),
        (4_107_542_400, "2100-03-01T00:00:00+00:00"),
        (13_574_563_200, "2400-02-29T00:00:00+00:00"),
    ];
    for (seconds, expected) in cases {
        assert_eq!(build_time::format(timestamp(seconds, 0)), expected);
    }
}

#[test]
fn utc_expanded_years_preserve_chrono_sign_and_padding() {
    let cases = [
        (-62_198_755_200_i128, "-0001-01-01T00:00:00+00:00"),
        (-62_167_219_200, "0000-01-01T00:00:00+00:00"),
        (-62_135_596_800, "0001-01-01T00:00:00+00:00"),
        (253_402_300_800, "+10000-01-01T00:00:00+00:00"),
    ];
    for (seconds, expected) in cases {
        assert_eq!(build_time::format_unix_nanos(seconds * 1_000_000_000), expected);
    }
}

#[test]
fn utc_format_preserves_chrono_auto_si_precision_and_explicit_offset() {
    // chrono 0.4.44 to_rfc3339 uses AutoSi and use_z=false: 0/3/6/9 digits,
    // including trailing zeroes, and +00:00 rather than Z for UTC.
    let cases = [
        (0, "1970-01-01T00:00:00+00:00"),
        (1, "1970-01-01T00:00:00.000000001+00:00"),
        (1_000, "1970-01-01T00:00:00.000001+00:00"),
        (1_000_000, "1970-01-01T00:00:00.001+00:00"),
        (120_000_000, "1970-01-01T00:00:00.120+00:00"),
        (123_400_000, "1970-01-01T00:00:00.123400+00:00"),
        (123_456_700, "1970-01-01T00:00:00.123456700+00:00"),
        (999_999_999, "1970-01-01T00:00:00.999999999+00:00"),
    ];
    for (nanos, expected) in cases {
        // Windows SystemTime has 100ns resolution; test finer formatting directly.
        assert_eq!(build_time::format_unix_nanos(nanos), expected);
    }
    assert_eq!(build_time::format_unix_nanos(-1), "1969-12-31T23:59:59.999999999+00:00");
    assert_eq!(
        build_time::format(UNIX_EPOCH - Duration::from_nanos(100)),
        "1969-12-31T23:59:59.999999900+00:00"
    );
    assert_eq!(
        build_time::format(timestamp(0, 123_456_700)),
        "1970-01-01T00:00:00.123456700+00:00"
    );
    // The instant expressed as 2025-01-01T00:30:00.000000001+01:00 is UTC here.
    assert_eq!(
        build_time::format_unix_nanos(1_735_687_800_000_000_001),
        "2024-12-31T23:30:00.000000001+00:00"
    );
}

struct Scratch(PathBuf);

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn run_script(binary: &Path, manifest: &Path, target_os: &str, timezone: &str) -> Vec<String> {
    let before = SystemTime::now();
    let output = Command::new(binary)
        .env("CARGO_CFG_TARGET_OS", target_os)
        .env("CARGO_MANIFEST_DIR", manifest)
        .env("TZ", timezone)
        .output()
        .unwrap();
    let after = SystemTime::now();
    assert!(output.status.success(), "{output:?}");
    let lines: Vec<String> = String::from_utf8(output.stdout)
        .unwrap()
        .lines()
        .map(str::to_owned)
        .collect();
    for line in &lines {
        if let Some(value) = line.strip_prefix("cargo:rustc-env=BUILD_TIME=") {
            let emitted = parse_emitted_utc(value);
            assert!(before <= emitted && emitted <= after, "{value}: {before:?}..{after:?}");
        }
    }
    lines
}

fn parse_emitted_utc(value: &str) -> SystemTime {
    // Independent inverse calendar calculation for current build timestamps.
    assert!(value.ends_with("+00:00"), "{value}");
    assert_eq!(&value[4..5], "-");
    assert_eq!(&value[7..8], "-");
    assert_eq!(&value[10..11], "T");
    assert_eq!(&value[13..14], ":");
    assert_eq!(&value[16..17], ":");
    let year: u64 = value[..4].parse().unwrap();
    let month: usize = value[5..7].parse().unwrap();
    let day: u64 = value[8..10].parse().unwrap();
    let hour: u64 = value[11..13].parse().unwrap();
    let minute: u64 = value[14..16].parse().unwrap();
    let second: u64 = value[17..19].parse().unwrap();
    assert!(year >= 1970 && (1..=12).contains(&month));
    assert!(hour < 24 && minute < 60 && second < 60);
    let leap = |y: u64| y % 4 == 0 && (y % 100 != 0 || y % 400 == 0);
    let months = [
        31,
        if leap(year) { 29 } else { 28 },
        31,
        30,
        31,
        30,
        31,
        31,
        30,
        31,
        30,
        31,
    ];
    assert!((1..=months[month - 1]).contains(&day));
    let days: u64 = (1970..year).map(|y| if leap(y) { 366 } else { 365 }).sum::<u64>()
        + months[..month - 1].iter().sum::<u64>()
        + day
        - 1;
    let fraction = &value[19..value.len() - 6];
    let nanos = if fraction.is_empty() {
        0
    } else {
        assert!(matches!(fraction.len(), 4 | 7 | 10), "{fraction}");
        assert!(fraction.starts_with('.'));
        assert!(fraction[1..].bytes().all(|digit| digit.is_ascii_digit()));
        fraction[1..].parse::<u32>().unwrap() * 10_u32.pow(10 - fraction.len() as u32)
    };
    UNIX_EPOCH + Duration::new(days * 86_400 + hour * 3600 + minute * 60 + second, nanos)
}

#[test]
fn actual_build_scripts_preserve_timestamp_conditions_and_linker_directives() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).parent().unwrap();
    let scratch = Scratch(std::env::temp_dir().join(format!(
        "secure-build-time-{}-{}",
        std::process::id(),
        SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_nanos()
    )));
    std::fs::create_dir(&scratch.0).unwrap();
    for platform in ["qemu-sp", "ihv1-sp"] {
        let manifest = root.join("platform").join(platform);
        let binary = scratch.0.join(format!("{platform}{}", std::env::consts::EXE_SUFFIX));
        let output = Command::new(std::env::var_os("RUSTC").unwrap_or_else(|| "rustc".into()))
            .arg("--edition=2021")
            .arg(manifest.join("build.rs"))
            .arg("-o")
            .arg(&binary)
            .output()
            .unwrap();
        assert!(output.status.success(), "{output:?}");

        for target_os in ["none", "linux", "windows"] {
            for timezone in ["UTC0", "EST5EDT"] {
                let mut lines = run_script(&binary, &manifest, target_os, timezone);
                if platform == "ihv1-sp" || target_os == "none" {
                    let time = lines.remove(0);
                    let value = time.strip_prefix("cargo:rustc-env=BUILD_TIME=").unwrap();
                    assert!(value.ends_with("+00:00"), "{value}");
                    assert!(value.contains('T'), "{value}");
                }
                let expected = if target_os != "none" {
                    Vec::new()
                } else if platform == "qemu-sp" {
                    vec![
                        "cargo:rustc-link-arg=-Timage.ld".to_owned(),
                        "cargo:rustc-link-arg=-Tplatform/qemu-sp/linker/qemu.ld".to_owned(),
                        "cargo:rerun-if-changed=platform/qemu-sp/linker/qemu.ld".to_owned(),
                        "cargo:rerun-if-changed=linker/image.ld".to_owned(),
                    ]
                } else {
                    let image = manifest.join("linker").join("image.ld");
                    let ihv1 = manifest.join("linker").join("ihv1.ld");
                    vec![
                        format!("cargo:rustc-link-arg=-T{}", image.display()),
                        format!("cargo:rustc-link-arg=-T{}", ihv1.display()),
                        format!("cargo:rerun-if-changed={}", image.display()),
                        format!("cargo:rerun-if-changed={}", ihv1.display()),
                    ]
                };
                assert_eq!(lines, expected, "{platform}, {target_os}, {timezone}");
            }
        }
    }
}
