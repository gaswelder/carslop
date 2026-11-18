use super::run;
use crate::util::error::invalid;
use std::cell::Cell;
use std::fs;

#[test]
fn pipeline_reuses_completed_stages_and_rebuilds_dependents() {
    let root = tempfile::tempdir().unwrap();
    let source = root.path().join("source");
    let intermediate = root.path().join("intermediate");
    let output = root.path().join("output");
    let first = Cell::new(0);
    let second = Cell::new(0);
    fs::write(&source, "original").unwrap();
    let build = || {
        run(
            &root.path().join("first.json"),
            &[source.clone()],
            &[intermediate.clone()],
            "",
            || {
                first.set(first.get() + 1);
                fs::copy(&source, &intermediate)?;
                Ok(())
            },
        )
        .unwrap();
        run(
            &root.path().join("second.json"),
            &[intermediate.clone()],
            &[output.clone()],
            "",
            || {
                second.set(second.get() + 1);
                fs::copy(&intermediate, &output)?;
                Ok(())
            },
        )
        .unwrap();
    };
    build();
    build();
    assert_eq!(first.get(), 1);
    assert_eq!(second.get(), 1);
    fs::write(&source, "changed source").unwrap();
    build();
    assert_eq!(first.get(), 2);
    assert_eq!(second.get(), 2);
    assert_eq!(fs::read_to_string(&output).unwrap(), "changed source");
    fs::remove_file(&output).unwrap();
    build();
    assert_eq!(first.get(), 2);
    assert_eq!(second.get(), 3);
}

#[test]
fn failures_invalidate_receipts_even_when_inputs_are_restored() {
    let root = tempfile::tempdir().unwrap();
    let receipt = root.path().join("stage.json");
    let output = root.path().join("output");
    let outputs = [output.clone()];
    run(&receipt, &[], &outputs, "old", || {
        fs::write(&output, "complete")?;
        Ok(())
    })
    .unwrap();
    assert!(run(&receipt, &[], &outputs, "new", || {
        fs::write(&output, "partial")?;
        Err(invalid("interrupted"))
    })
    .is_err());
    assert!(!receipt.exists());
    run(&receipt, &[], &outputs, "old", || {
        fs::write(&output, "rebuilt")?;
        Ok(())
    })
    .unwrap();
    assert_eq!(fs::read_to_string(output).unwrap(), "rebuilt");
}

#[test]
fn tracks_directory_members_and_output_changes() {
    let root = tempfile::tempdir().unwrap();
    let inputs = root.path().join("inputs");
    let outputs = root.path().join("outputs");
    fs::create_dir(&inputs).unwrap();
    fs::create_dir(&outputs).unwrap();
    fs::write(inputs.join("first"), "input").unwrap();
    let calls = Cell::new(0);
    let build = || {
        run(
            &root.path().join("stage.json"),
            &[inputs.clone()],
            &[outputs.clone()],
            "",
            || {
                calls.set(calls.get() + 1);
                fs::write(outputs.join("result"), "complete")?;
                Ok(())
            },
        )
        .unwrap();
    };
    build();
    build();
    assert_eq!(calls.get(), 1);
    fs::write(inputs.join("second"), "new input").unwrap();
    build();
    assert_eq!(calls.get(), 2);
    fs::remove_file(inputs.join("first")).unwrap();
    build();
    assert_eq!(calls.get(), 3);
    fs::write(outputs.join("result"), "damaged").unwrap();
    build();
    assert_eq!(calls.get(), 4);
}
