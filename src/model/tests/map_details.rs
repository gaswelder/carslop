use super::*;

const RECORDS: &str = concat!(
    "DETAILS 1\n",
    "building 0 gabled 2.25 0.3 0.1 0.2 0.3 -\n",
    "building 1 flat 0 0 - 0.5 0.6 0.7\n",
    "tree 18446744073709551615 olive true\n",
    "tree 2 broadleaf false\n",
    "object turn_sign -2.1234567 3.5 0.4 1.2 0.8 true \"Πάρκο \\\"East\\\"\\nnext\\tstop\\r\\\\end\"\n",
    "object bench 0 0 0 1 1 false \"\"\n",
    "line power 5 2 0 0 10 20\n",
    "line cliff 2 3 0 0 10 20 30 40\n",
    "deck 0 -1 6 concrete 2 0 -3.1234567 0 10 -4 20\n",
    "restriction -1 2 3 \"no left turn\"\n",
);

#[test]
fn all_records_round_trip_without_losing_fields() {
    let data = parse(RECORDS).unwrap();
    validate(&data, 2, 1).unwrap();
    assert_eq!(data.buildings[0].wall_color, Some([0.1, 0.2, 0.3]));
    assert_eq!(data.buildings[0].roof_color, None);
    assert_eq!(data.trees[0].id, u64::MAX);
    assert_eq!(data.objects[0].text, "Πάρκο \"East\"\nnext\tstop\r\\end");
    assert_eq!(data.objects[1].text, "");
    assert_eq!(data.lines[1].points.len(), 3);
    assert_eq!(data.decks[0].grade, -1);
    assert_eq!(data.restrictions[0].from, -1);
    let directory = tempfile::tempdir().unwrap();
    write(&data, directory.path()).unwrap();
    assert!(directory.path().join("details.txt").is_file());
    assert_eq!(read(directory.path(), 2, 1).unwrap(), data);
}

#[test]
fn missing_file_and_empty_records_have_default_details() {
    let directory = tempfile::tempdir().unwrap();
    assert_eq!(read(directory.path(), 0, 0).unwrap(), Data::default());
    assert_eq!(parse("DETAILS 1\n\n").unwrap(), Data::default());
}

#[test]
fn malformed_records_are_rejected_with_line_numbers() {
    for record in [
        "unknown 1",
        "tree 1 unknown false",
        "tree 1 palm",
        "tree 1 palm false extra",
        "tree 1 palm 1",
        "building 0 flat 0 0 -",
        "building 0 flat 0 0 1 2 -",
        "line power 5 3 0 0 10 20",
        "line power 5 1 0 0 10 20",
        "line power 5 18446744073709551615",
        "deck 0 1 6 asphalt 2 0 0 0",
        "restriction 1 2 3 \"unterminated",
        "restriction 1 2 3 \"bad\\q\"",
        "restriction 1 2 3 \"text\"junk",
        "restriction 1 2 3 bad\"quote",
    ] {
        let error = parse(&format!("DETAILS 1\n{record}\n")).unwrap_err();
        assert!(error.starts_with("details.txt:2:"), "{record}: {error}");
    }
    for header in ["", "DETAILS 2\n", "tree 1 palm true\n"] {
        assert!(parse(header).is_err());
    }
}

#[test]
fn reading_still_validates_geometry_and_references() {
    let directory = tempfile::tempdir().unwrap();
    for record in [
        "building 1 flat 0 0 - -",
        "building 0 flat 0 0 2 0 0 -",
        "object bench NaN 0 0 1 1 false \"\"",
        "line cliff 1 2 0 0 inf 1",
        "deck 0 0 6 asphalt 2 0 0 0 10 0 0",
        "deck 1 1 6 asphalt 2 0 0 0 10 0 0",
    ] {
        fs::write(
            directory.path().join("details.txt"),
            format!("DETAILS 1\n{record}\n"),
        )
        .unwrap();
        assert!(read(directory.path(), 1, 1).is_err(), "{record}");
    }
}
