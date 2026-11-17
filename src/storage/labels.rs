//! One label per line: kind, source ID, then the name verbatim.
use crate::model::location::Labels;
use std::fs;
use std::io;
use std::io::BufRead;
use std::io::Write;
use std::path::Path;

const FILE_NAME: &str = "labels.txt";

pub fn write(directory: &Path, labels: &Labels) -> io::Result<()> {
    let mut writer = io::BufWriter::new(fs::File::create(directory.join(FILE_NAME))?);
    for (id, name) in &labels.streets {
        write_label(&mut writer, "street", *id, name)?;
    }
    for (id, name) in &labels.buildings {
        write_label(&mut writer, "building", *id, name)?;
    }
    for (id, name) in &labels.storefronts {
        write_label(&mut writer, "storefront", *id, name)?;
    }
    writer.flush()
}

fn write_label(
    writer: &mut impl io::Write,
    kind: &str,
    id: impl std::fmt::Display,
    name: &str,
) -> io::Result<()> {
    if name.trim().is_empty() || name.contains(['\n', '\r']) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "label name must occupy one nonempty line",
        ));
    }
    writeln!(writer, "{kind} {id} {name}")
}

pub fn parse(directory: &Path) -> io::Result<Labels> {
    let reader = fs::File::open(directory.join(FILE_NAME))?;
    let mut labels = Labels::default();
    for (index, line) in io::BufReader::new(reader).lines().enumerate() {
        let line = line?;
        parse_line(&line, &mut labels).map_err(|e| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!("line {}: {e}", index + 1),
            )
        })?;
    }
    Ok(labels)
}

fn parse_line(line: &str, labels: &mut Labels) -> Result<(), &'static str> {
    let mut fields = line.splitn(3, ' ');
    let kind = fields.next().ok_or("missing label kind")?;
    let id = fields.next().ok_or("missing label ID")?;
    let name = fields.next().ok_or("missing label name")?;
    if name.trim().is_empty() || name.contains('\r') {
        return Err("invalid label name");
    }
    match kind {
        "street" => labels
            .streets
            .push((id.parse().map_err(|_| "invalid street ID")?, name.into())),
        "building" => labels
            .buildings
            .push((id.parse().map_err(|_| "invalid building ID")?, name.into())),
        "storefront" => labels.storefronts.push((
            id.parse().map_err(|_| "invalid storefront ID")?,
            name.into(),
        )),
        _ => return Err("unknown label kind"),
    }
    Ok(())
}

#[cfg(test)]
#[path = "tests/labels.rs"]
mod tests;

pub fn read_optional(directory: &Path) -> Result<Labels, String> {
    match parse(directory) {
        Ok(labels) => Ok(labels),
        Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(Labels::default()),
        Err(e) => Err(format!("{FILE_NAME}: {e}")),
    }
}
