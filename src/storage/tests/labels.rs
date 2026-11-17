use super::*;

#[test]
fn labels_round_trip_unicode_spaces_and_shared_buildings() {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join(FILE_NAME);
    let text = concat!(
        "street -12 28th October\n",
        "building 18446744073709551615 Town Hall\n",
        "storefront 42 Εν ελλαδι\n",
        "storefront 42 Pahit & Ice\n",
        "storefront 42  Space  preserved \n",
    );
    fs::write(&path, text).unwrap();
    let labels = parse(directory.path()).unwrap();
    assert_eq!(labels.streets[0].0, -12);
    assert_eq!(labels.buildings[0].0, u64::MAX);
    assert_eq!(labels.storefronts.len(), 3);
    assert_eq!(labels.storefronts[0].1, "Εν ελλαδι");
    write(directory.path(), &labels).unwrap();
    assert_eq!(fs::read_to_string(&path).unwrap(), text);
    assert_eq!(parse(directory.path()).unwrap(), labels);
    fs::write(&path, text.replace('\n', "\r\n")).unwrap();
    assert_eq!(parse(directory.path()).unwrap(), labels);
    write(directory.path(), &Labels::default()).unwrap();
    assert_eq!(fs::read_to_string(&path).unwrap(), "");
    assert_eq!(parse(directory.path()).unwrap(), Labels::default());
}

#[test]
fn malformed_labels_report_the_line_and_invalid_names_cannot_be_written() {
    let directory = tempfile::tempdir().unwrap();
    for line in [
        "unknown 1 Name",
        "street",
        "street 1",
        "street 1 ",
        "building -1 Name",
        "storefront nope Name",
        "building 18446744073709551616 Name",
        "street 1 Bad\rName",
        "",
    ] {
        fs::write(
            directory.path().join(FILE_NAME),
            format!("street 1 Valid\n{line}\n"),
        )
        .unwrap();
        let error = parse(directory.path()).unwrap_err();
        assert_eq!(error.kind(), io::ErrorKind::InvalidData);
        assert!(error.to_string().starts_with("line 2:"), "{error}");
    }
    for name in ["", "   ", "First\nSecond", "First\rSecond"] {
        let labels = Labels {
            storefronts: vec![(1, name.into())],
            ..Labels::default()
        };
        assert_eq!(
            write(directory.path(), &labels).unwrap_err().kind(),
            io::ErrorKind::InvalidInput
        );
    }
}

#[test]
fn file_errors_are_propagated() {
    let directory = tempfile::tempdir().unwrap();
    assert_eq!(
        parse(directory.path()).unwrap_err().kind(),
        io::ErrorKind::NotFound
    );
    let missing = directory.path().join("missing");
    assert_eq!(
        write(&missing, &Labels::default()).unwrap_err().kind(),
        io::ErrorKind::NotFound
    );
    fs::write(directory.path().join(FILE_NAME), b"street 1 \xff\n").unwrap();
    assert_eq!(
        parse(directory.path()).unwrap_err().kind(),
        io::ErrorKind::InvalidData
    );
}
