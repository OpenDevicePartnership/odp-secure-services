// Copyright 2026 Microsoft Corporation
// This project is dual-licensed under Apache 2.0 and MIT terms.
// See LICENSE-APACHE and LICENSE-MIT for details.

use std::time::{SystemTime, UNIX_EPOCH};

pub(crate) fn format(time: SystemTime) -> String {
    let nanos = match time.duration_since(UNIX_EPOCH) {
        Ok(duration) => duration.as_nanos() as i128,
        Err(error) => -(error.duration().as_nanos() as i128),
    };
    format_unix_nanos(nanos)
}

pub(crate) fn format_unix_nanos(nanos: i128) -> String {
    // Euclidean division keeps the fractional second positive before the epoch.
    let seconds = nanos.div_euclid(1_000_000_000);
    let fraction = nanos.rem_euclid(1_000_000_000);
    let days = seconds.div_euclid(86_400);
    let day_seconds = seconds.rem_euclid(86_400);

    // 2000-01-01 is 10,957 days after the epoch and starts a Gregorian
    // 400-year cycle. Each cycle has exactly 146,097 days, including leap days.
    let relative_days = days - 10_957;
    let mut year = 2000 + relative_days.div_euclid(146_097) * 400;
    let mut day = relative_days.rem_euclid(146_097);
    loop {
        let year_days = if leap_year(year) { 366 } else { 365 };
        if day < year_days {
            break;
        }
        day -= year_days;
        year += 1;
    }
    let month_days = [
        31,
        if leap_year(year) { 29 } else { 28 },
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
    let mut month = 1;
    for length in month_days {
        if day < length {
            break;
        }
        day -= length;
        month += 1;
    }

    let year = if (0..=9999).contains(&year) {
        format!("{year:04}")
    } else {
        format!("{year:+05}")
    };
    let subsecond = match fraction {
        0 => String::new(),
        n if n % 1_000_000 == 0 => format!(".{:03}", n / 1_000_000),
        n if n % 1_000 == 0 => format!(".{:06}", n / 1_000),
        n => format!(".{n:09}"),
    };
    format!(
        "{year}-{month:02}-{:02}T{:02}:{:02}:{:02}{subsecond}+00:00",
        day + 1,
        day_seconds / 3600,
        day_seconds / 60 % 60,
        day_seconds % 60,
    )
}

fn leap_year(year: i128) -> bool {
    year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)
}
