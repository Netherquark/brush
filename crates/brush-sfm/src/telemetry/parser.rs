use std::collections::HashMap;

use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RawTelemetryRecord {
    /// Seconds from the beginning of the capture or Unix epoch seconds if the
    /// source provides absolute timestamps.
    pub timestamp: f64,
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
    pub yaw: Option<f64>,
    pub pitch: Option<f64>,
    pub roll: Option<f64>,
    pub gimbal_pitch: Option<f64>,
    pub vel_n: Option<f64>,
    pub vel_e: Option<f64>,
    pub vel_d: Option<f64>,
    pub hdop: Option<f64>,
    pub num_sats: Option<u32>,
}

#[derive(Debug, Error, PartialEq)]
pub enum ParseTelemetryError {
    #[error("telemetry input is empty")]
    Empty,
    #[error("unsupported telemetry format")]
    UnsupportedFormat,
    #[error("missing required column `{0}`")]
    MissingColumn(&'static str),
    #[error("line {line}: missing required value `{field}`")]
    MissingValue { line: usize, field: &'static str },
    #[error("line {line}: invalid `{field}` value `{value}`")]
    InvalidNumber {
        line: usize,
        field: &'static str,
        value: String,
    },
}

pub fn parse_telemetry(input: &str) -> Result<Vec<RawTelemetryRecord>, ParseTelemetryError> {
    let trimmed = input.trim_start();
    if trimmed.is_empty() {
        return Err(ParseTelemetryError::Empty);
    }

    if looks_like_srt(trimmed) {
        parse_dji_srt(trimmed)
    } else if trimmed.lines().next().is_some_and(|line| line.contains(',')) {
        parse_litchi_csv(trimmed)
    } else {
        Err(ParseTelemetryError::UnsupportedFormat)
    }
}

pub fn parse_litchi_csv(input: &str) -> Result<Vec<RawTelemetryRecord>, ParseTelemetryError> {
    let mut lines = input.lines().enumerate().filter_map(|(idx, line)| {
        let line = line.trim();
        (!line.is_empty()).then_some((idx + 1, line))
    });

    let Some((_, header_line)) = lines.next() else {
        return Err(ParseTelemetryError::Empty);
    };

    let headers: Vec<_> = split_csv_line(header_line)
        .into_iter()
        .map(|h| canonical_key(&h))
        .collect();
    let header_map: HashMap<_, _> = headers
        .iter()
        .enumerate()
        .map(|(idx, name)| (name.as_str(), idx))
        .collect();

    for required in ["timestamp", "lat", "lon", "alt"] {
        if !header_map.contains_key(required) {
            return Err(ParseTelemetryError::MissingColumn(required));
        }
    }

    lines
        .map(|(line, row)| {
            let values = split_csv_line(row);
            parse_row(line, &header_map, &values)
        })
        .collect()
}

fn parse_dji_srt(input: &str) -> Result<Vec<RawTelemetryRecord>, ParseTelemetryError> {
    let mut records = Vec::new();

    for block in input.split("\n\n") {
        let mut timestamp = None;
        let mut fields = HashMap::new();

        for line in block.lines().map(str::trim).filter(|line| !line.is_empty()) {
            if line.contains("-->") {
                timestamp = parse_srt_start_timestamp(line);
                continue;
            }
            collect_srt_fields(line, &mut fields);
        }

        if fields.is_empty() {
            continue;
        }

        let line = records.len() + 1;
        records.push(RawTelemetryRecord {
            timestamp: timestamp.unwrap_or(records.len() as f64),
            lat: required_srt_value(line, &fields, "lat")?,
            lon: required_srt_value(line, &fields, "lon")?,
            alt: required_srt_value(line, &fields, "alt")?,
            yaw: optional_value(&fields, &["yaw"]),
            pitch: optional_value(&fields, &["pitch"]),
            roll: optional_value(&fields, &["roll"]),
            gimbal_pitch: optional_value(&fields, &["gimbalpitch", "gimbal_pitch"]),
            vel_n: optional_value(&fields, &["veln", "vel_n"]),
            vel_e: optional_value(&fields, &["vele", "vel_e"]),
            vel_d: optional_value(&fields, &["veld", "vel_d"]),
            hdop: optional_value(&fields, &["hdop"]),
            num_sats: optional_value(&fields, &["numsats", "num_sats", "sats"])
                .map(|value| value as u32),
        });
    }

    if records.is_empty() {
        Err(ParseTelemetryError::UnsupportedFormat)
    } else {
        Ok(records)
    }
}

fn parse_row(
    line: usize,
    header_map: &HashMap<&str, usize>,
    values: &[String],
) -> Result<RawTelemetryRecord, ParseTelemetryError> {
    Ok(RawTelemetryRecord {
        timestamp: required_csv_value(line, header_map, values, "timestamp")?,
        lat: required_csv_value(line, header_map, values, "lat")?,
        lon: required_csv_value(line, header_map, values, "lon")?,
        alt: required_csv_value(line, header_map, values, "alt")?,
        yaw: optional_csv_value(line, header_map, values, &["yaw"])?,
        pitch: optional_csv_value(line, header_map, values, &["pitch"])?,
        roll: optional_csv_value(line, header_map, values, &["roll"])?,
        gimbal_pitch: optional_csv_value(line, header_map, values, &["gimbalpitch"])?,
        vel_n: optional_csv_value(line, header_map, values, &["veln"])?,
        vel_e: optional_csv_value(line, header_map, values, &["vele"])?,
        vel_d: optional_csv_value(line, header_map, values, &["veld"])?,
        hdop: optional_csv_value(line, header_map, values, &["hdop"])?,
        num_sats: optional_csv_value(line, header_map, values, &["numsats", "sats"])?
            .map(|value| value as u32),
    })
}

fn required_csv_value(
    line: usize,
    header_map: &HashMap<&str, usize>,
    values: &[String],
    field: &'static str,
) -> Result<f64, ParseTelemetryError> {
    let idx = *header_map
        .get(field)
        .ok_or(ParseTelemetryError::MissingColumn(field))?;
    let value = values
        .get(idx)
        .map(String::as_str)
        .unwrap_or("")
        .trim();
    if value.is_empty() {
        return Err(ParseTelemetryError::MissingValue { line, field });
    }
    value
        .parse()
        .map_err(|_| ParseTelemetryError::InvalidNumber {
            line,
            field,
            value: value.to_owned(),
        })
}

fn optional_csv_value(
    line: usize,
    header_map: &HashMap<&str, usize>,
    values: &[String],
    aliases: &[&'static str],
) -> Result<Option<f64>, ParseTelemetryError> {
    let Some((field, idx)) = aliases
        .iter()
        .find_map(|field| header_map.get(field).map(|idx| (*field, *idx)))
    else {
        return Ok(None);
    };
    let value = values
        .get(idx)
        .map(String::as_str)
        .unwrap_or("")
        .trim();
    if value.is_empty() {
        return Ok(None);
    }
    value
        .parse()
        .map(Some)
        .map_err(|_| ParseTelemetryError::InvalidNumber {
            line,
            field,
            value: value.to_owned(),
        })
}

fn required_srt_value(
    line: usize,
    fields: &HashMap<String, f64>,
    field: &'static str,
) -> Result<f64, ParseTelemetryError> {
    fields
        .get(field)
        .copied()
        .ok_or(ParseTelemetryError::MissingValue { line, field })
}

fn optional_value(fields: &HashMap<String, f64>, aliases: &[&str]) -> Option<f64> {
    aliases.iter().find_map(|alias| fields.get(*alias).copied())
}

fn split_csv_line(line: &str) -> Vec<String> {
    let mut fields = Vec::new();
    let mut current = String::new();
    let mut in_quotes = false;
    let mut chars = line.chars().peekable();

    while let Some(ch) = chars.next() {
        match ch {
            '"' if in_quotes && chars.peek() == Some(&'"') => {
                current.push('"');
                chars.next();
            }
            '"' => in_quotes = !in_quotes,
            ',' if !in_quotes => {
                fields.push(current.trim().to_owned());
                current.clear();
            }
            _ => current.push(ch),
        }
    }
    fields.push(current.trim().to_owned());
    fields
}

fn collect_srt_fields(line: &str, fields: &mut HashMap<String, f64>) {
    for chunk in line.split(['[', ']']) {
        let Some((key, value)) = chunk.split_once(':') else {
            continue;
        };
        if let Some(number) = first_number(value) {
            fields.insert(canonical_key(key), number);
        }
    }
}

fn first_number(value: &str) -> Option<f64> {
    let start = value.find(|ch: char| ch == '-' || ch == '+' || ch.is_ascii_digit())?;
    let number: String = value[start..]
        .chars()
        .take_while(|ch| ch.is_ascii_digit() || matches!(ch, '-' | '+' | '.' | 'e' | 'E'))
        .collect();
    number.parse().ok()
}

fn looks_like_srt(input: &str) -> bool {
    input.lines().take(4).any(|line| line.contains("-->"))
}

fn parse_srt_start_timestamp(line: &str) -> Option<f64> {
    let start = line.split("-->").next()?.trim();
    let mut parts = start.split([':', ',']);
    let hours: f64 = parts.next()?.parse().ok()?;
    let minutes: f64 = parts.next()?.parse().ok()?;
    let seconds: f64 = parts.next()?.parse().ok()?;
    let millis: f64 = parts.next()?.parse().ok()?;
    Some(hours * 3600.0 + minutes * 60.0 + seconds + millis / 1000.0)
}

fn normalize_key(key: &str) -> String {
    key.chars()
        .filter(|ch| ch.is_ascii_alphanumeric())
        .flat_map(char::to_lowercase)
        .collect()
}

fn canonical_key(key: &str) -> String {
    match normalize_key(key).as_str() {
        "latitude" => "lat".to_owned(),
        "longitude" => "lon".to_owned(),
        "altitude" | "altitudem" | "altitudefeet" | "height" => "alt".to_owned(),
        "time" | "datetime" | "timestampms" => "timestamp".to_owned(),
        "gimbalpitchangle" => "gimbalpitch".to_owned(),
        "velocityn" | "velocitynorth" => "veln".to_owned(),
        "velocitye" | "velocityeast" => "vele".to_owned(),
        "velocityd" | "velocitydown" => "veld".to_owned(),
        "satellites" | "satellitecount" | "numsatellites" => "numsats".to_owned(),
        other => other.to_owned(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_litchi_style_csv() {
        let records = parse_litchi_csv(
            "timestamp,latitude,longitude,altitude(m),yaw,pitch,roll,gimbalpitch,veln,vele,veld,hdop,numsats\n\
             0.000,12.971600,77.594600,920.5,45.0,-1.0,0.2,-35.0,1.2,0.1,-0.3,0.7,18\n\
             0.500,12.971610,77.594620,921.0,46.0,-1.1,0.1,-36.0,1.3,0.2,-0.2,0.8,17\n",
        )
        .unwrap();

        assert_eq!(records.len(), 2);
        assert_eq!(records[0].timestamp, 0.0);
        assert_eq!(records[0].lat, 12.9716);
        assert_eq!(records[0].lon, 77.5946);
        assert_eq!(records[0].alt, 920.5);
        assert_eq!(records[0].yaw, Some(45.0));
        assert_eq!(records[0].gimbal_pitch, Some(-35.0));
        assert_eq!(records[0].num_sats, Some(18));
    }

    #[test]
    fn parses_dji_srt_blocks() {
        let records = parse_telemetry(
            "1\n\
             00:00:01,250 --> 00:00:01,750\n\
             [latitude: 12.971600] [longitude: 77.594600] [altitude: 920.5] [yaw: 45.0] [pitch: -1.0] [roll: 0.2]\n\
             [gimbal_pitch: -35.0] [hdop: 0.7] [num_sats: 18]\n\n\
             2\n\
             00:00:01,750 --> 00:00:02,250\n\
             [lat: 12.971610] [lon: 77.594620] [alt: 921.0]\n",
        )
        .unwrap();

        assert_eq!(records.len(), 2);
        assert_eq!(records[0].timestamp, 1.25);
        assert_eq!(records[0].hdop, Some(0.7));
        assert_eq!(records[1].lat, 12.97161);
        assert_eq!(records[1].alt, 921.0);
    }

    #[test]
    fn rejects_csv_without_required_columns() {
        let error = parse_litchi_csv("timestamp,lat,alt\n0.0,12.0,920.0\n").unwrap_err();
        assert_eq!(error, ParseTelemetryError::MissingColumn("lon"));
    }
}
