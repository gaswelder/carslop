use super::*;

#[test]
fn rejects_nonfinite_degenerate_and_unknown_barriers() {
    for record in [
        "1 wall 0 0 0 0 2 0.2",
        "1 wall NaN 0 10 0 2 0.2",
        "1 wall 0 0 10 0 inf 0.2",
        "1 wall 0 0 10 0 2 0",
        "1 wall 0 0 10 0 20 0.2",
        "1 unknown 0 0 10 0 2 0.2",
        "1 wall 0 0 10 0 2",
        "1 wall 0 0 100001 0 2 0.2",
    ] {
        assert!(parse(&format!("{record}\n")).is_err(), "{record}");
    }
    assert!(parse("BARRIERS 2\n").is_err());
    assert!(parse("").unwrap().barriers.is_empty());
    for kind in ["wall", "fence", "chain_link", "wood_fence"] {
        let records = parse(&format!("1 {kind} -5.1234565 0 10 0 1.8765432 0.1234567\n")).unwrap();
        let mut bytes = Vec::new();
        write(&mut bytes, &[], &records.barriers).unwrap();
        assert_eq!(
            parse(std::str::from_utf8(&bytes).unwrap())
                .unwrap()
                .barriers,
            records.barriers
        );
    }
}

#[test]
fn mixed_records_keep_control_fields_and_barrier_geometry() {
    let text = concat!(
        "10 signal 1.234 -2.345 0 1 0 49\n",
        "11 hedge 0 0 10 0 1.5 0.6\n",
        "12 speed_limit 3 4 1 0 50 0\n",
        "13 guard_rail 0 2 10 2 0.75 0.08\n",
        "14 stop 5 6 -1 0 0 0\n",
        "15 retaining_wall 0 4 10 4 2 0.25\n",
        "16 yield 7 8 0 -1 0 0\n",
    );
    let parsed = parse(text).unwrap();
    assert_eq!(parsed.controls.len(), 4);
    assert_eq!(parsed.barriers.len(), 3);
    let records: Vec<_> = parsed
        .controls
        .iter()
        .map(|f| FurnitureRecord {
            source_id: f._source_id as i64,
            kind: f.kind,
            position: XZ {
                x: f.coords.x as f64,
                z: f.coords.z as f64,
            },
            travel: XZ {
                x: f.travel.x as f64,
                z: f.travel.z as f64,
            },
            value: f.value,
            phase: f.phase,
        })
        .collect();
    let mut bytes = Vec::new();
    write(&mut bytes, &records, &parsed.barriers).unwrap();
    let output = std::str::from_utf8(&bytes).unwrap();
    assert_eq!(output.lines().count(), 7);
    let reread = parse(output).unwrap();
    assert_eq!(reread.barriers, parsed.barriers);
    for (actual, expected) in reread.controls.iter().zip(&parsed.controls) {
        assert_eq!(actual._source_id, expected._source_id);
        assert_eq!(actual.kind, expected.kind);
        assert_eq!(actual.coords, expected.coords);
        assert_eq!(actual.travel, expected.travel);
        assert_eq!(actual.value, expected.value);
        assert_eq!(actual.phase, expected.phase);
    }
    let empty = parse("").unwrap();
    assert!(empty.controls.is_empty() && empty.barriers.is_empty());
    for invalid in [
        "1 signal NaN 0 1 0 0 0",
        "1 stop 0 0 0 0 0 0",
        "1 yield 0 0 1 0 0 50",
        "1 speed_limit 0 0 1 0 4 0",
        "1 speed_limit 0 0 1 0 151 0",
    ] {
        assert!(parse(&format!("{text}{invalid}\n")).is_err(), "{invalid}");
    }
}
