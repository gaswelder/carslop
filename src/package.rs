//! Versioned SQLite location container. Assets are independently compressed in
//! 256 KiB blocks, so even multi-gigabyte scene archives support bounded seeks.
use crate::location::{invalid, Result};
use rusqlite::{params, Connection, OpenFlags, OptionalExtension};
use sha2::{Digest, Sha256};
use std::{
    fs,
    io::{self, Read, Seek, SeekFrom, Write},
    path::{Path, PathBuf},
};
const APP: i32 = 0x4e46534c;
const VERSION: i32 = 1;
const BLOCK: usize = 256 * 1024;
fn err(e: impl std::fmt::Display) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, e.to_string())
}
fn missing(name: &str) -> io::Error {
    io::Error::new(
        io::ErrorKind::NotFound,
        format!("missing packaged asset: {name}"),
    )
}
fn safe(name: &str) -> bool {
    !name.is_empty()
        && !name.contains('\\')
        && Path::new(name)
            .components()
            .all(|c| matches!(c, std::path::Component::Normal(_)))
}
fn connect(path: &Path, write: bool) -> io::Result<Connection> {
    let flags = if write {
        OpenFlags::SQLITE_OPEN_READ_WRITE
    } else {
        OpenFlags::SQLITE_OPEN_READ_ONLY
    };
    let db =
        Connection::open_with_flags(path, flags | OpenFlags::SQLITE_OPEN_NO_MUTEX).map_err(err)?;
    db.busy_timeout(std::time::Duration::from_secs(10))
        .map_err(err)?;
    db.execute_batch("PRAGMA cache_size=-2048; PRAGMA mmap_size=0;")
        .map_err(err)?;
    let app: i32 = db
        .pragma_query_value(None, "application_id", |r| r.get(0))
        .map_err(err)?;
    let version: i32 = db
        .pragma_query_value(None, "user_version", |r| r.get(0))
        .map_err(err)?;
    if app != APP || version != VERSION {
        return Err(err("unsupported location package format/version"));
    }
    Ok(db)
}
pub fn is_package(path: &Path) -> bool {
    path.is_file() && path.extension().is_some_and(|x| x == "sqlite")
}
fn address(path: &Path) -> io::Result<(PathBuf, String)> {
    for parent in path.ancestors().skip(1) {
        if is_package(parent) {
            let name = path
                .strip_prefix(parent)
                .map_err(err)?
                .to_str()
                .ok_or_else(|| err("asset names must be UTF-8"))?
                .replace('\\', "/");
            if !safe(&name) {
                return Err(err("invalid asset path"));
            }
            return Ok((parent.to_owned(), name));
        }
    }
    Err(missing(&path.to_string_lossy()))
}
pub enum Reader {
    File(fs::File),
    Database {
        db: Connection,
        id: i64,
        length: u64,
        position: u64,
        block: Option<u64>,
        buffer: Vec<u8>,
    },
}
impl Reader {
    pub fn open(path: &Path) -> io::Result<Self> {
        if path.is_file() {
            return Ok(Self::File(fs::File::open(path)?));
        }
        let (package, name) = address(path)?;
        Self::asset(&package, &name)
    }
    pub fn asset(package: &Path, name: &str) -> io::Result<Self> {
        if !safe(name) {
            return Err(err("invalid asset name"));
        }
        let db = connect(package, false)?;
        db.execute_batch("BEGIN").map_err(err)?;
        let info: Option<(i64, i64)> = db
            .query_row("SELECT id,size FROM assets WHERE name=?1", [name], |r| {
                Ok((r.get(0)?, r.get(1)?))
            })
            .optional()
            .map_err(err)?;
        let (id, length) = info.ok_or_else(|| missing(name))?;
        if length < 0 {
            return Err(err("invalid asset size"));
        }
        Ok(Self::Database {
            db,
            id,
            length: length as u64,
            position: 0,
            block: None,
            buffer: vec![],
        })
    }
    pub fn len(&self) -> io::Result<u64> {
        match self {
            Self::File(f) => Ok(f.metadata()?.len()),
            Self::Database { length, .. } => Ok(*length),
        }
    }
}
impl Read for Reader {
    fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
        match self {
            Self::File(file) => file.read(out),
            Self::Database {
                db,
                id,
                length,
                position,
                block,
                buffer,
            } => {
                if *position >= *length || out.is_empty() {
                    return Ok(0);
                }
                let next = *position / BLOCK as u64;
                if *block != Some(next) {
                    let (codec,raw_size,crc,data):(i32,u32,u32,Vec<u8>)=db.query_row("SELECT codec,raw_size,crc32,data FROM blocks WHERE asset_id=?1 AND block_no=?2",params![*id,next as i64],|r|Ok((r.get(0)?,r.get(1)?,r.get(2)?,r.get(3)?))).map_err(err)?;
                    let expected = (*length - next * BLOCK as u64).min(BLOCK as u64) as usize;
                    if raw_size as usize != expected || data.len() > BLOCK + 1024 {
                        return Err(err("invalid package block size"));
                    }
                    let decoded = match codec {
                        0 => data,
                        1 => {
                            let mut raw = vec![];
                            flate2::read::ZlibDecoder::new(data.as_slice())
                                .take(BLOCK as u64 + 1)
                                .read_to_end(&mut raw)?;
                            raw
                        }
                        _ => return Err(err("unsupported package block codec")),
                    };
                    if decoded.len() != raw_size as usize || crc32fast::hash(&decoded) != crc {
                        return Err(err("corrupt location package block"));
                    }
                    *buffer = decoded;
                    *block = Some(next);
                }
                let offset = (*position % BLOCK as u64) as usize;
                let n = out.len().min(buffer.len() - offset);
                out[..n].copy_from_slice(&buffer[offset..offset + n]);
                *position += n as u64;
                Ok(n)
            }
        }
    }
}
impl Seek for Reader {
    fn seek(&mut self, from: SeekFrom) -> io::Result<u64> {
        match self {
            Self::File(file) => file.seek(from),
            Self::Database {
                position, length, ..
            } => {
                let target = match from {
                    SeekFrom::Start(n) => n as i128,
                    SeekFrom::Current(n) => *position as i128 + n as i128,
                    SeekFrom::End(n) => *length as i128 + n as i128,
                };
                if target < 0 || target > u64::MAX as i128 {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidInput,
                        "invalid asset seek",
                    ));
                }
                *position = target as u64;
                Ok(*position)
            }
        }
    }
}
pub fn exists(path: &Path) -> bool {
    if path.is_file() {
        return true;
    }
    let Ok((package, name)) = address(path) else {
        return false;
    };
    let Ok(db) = connect(&package, false) else {
        return false;
    };
    db.query_row(
        "SELECT EXISTS(SELECT 1 FROM assets WHERE name=?1)",
        [name],
        |r| r.get::<_, bool>(0),
    )
    .unwrap_or(false)
}
pub fn read(path: &Path) -> io::Result<Vec<u8>> {
    let mut reader = Reader::open(path)?;
    let mut bytes = Vec::new();
    let length = usize::try_from(reader.len()?).map_err(err)?;
    bytes.try_reserve_exact(length).map_err(err)?;
    reader.read_to_end(&mut bytes)?;
    Ok(bytes)
}
pub fn read_to_string(path: impl AsRef<Path>) -> io::Result<String> {
    String::from_utf8(read(path.as_ref())?).map_err(err)
}
pub fn size(path: &Path) -> io::Result<u64> {
    Reader::open(path)?.len()
}
fn write_asset(db: &Connection, name: &str, path: &Path) -> Result<()> {
    if !safe(name) {
        return Err(invalid("invalid asset name"));
    }
    db.execute(
        "DELETE FROM blocks WHERE asset_id IN (SELECT id FROM assets WHERE name=?1)",
        [name],
    )?;
    db.execute("DELETE FROM assets WHERE name=?1", [name])?;
    let mut file = fs::File::open(path)?;
    let length = file.metadata()?.len();
    db.execute(
        "INSERT INTO assets(name,size,sha256) VALUES(?1,?2,'')",
        params![name, length as i64],
    )?;
    let id = db.last_insert_rowid();
    let mut insert = db.prepare(
        "INSERT INTO blocks(asset_id,block_no,codec,raw_size,crc32,data) VALUES(?1,?2,?3,?4,?5,?6)",
    )?;
    let mut buffer = vec![0; BLOCK];
    let mut index = 0i64;
    let mut sha = Sha256::new();
    let mut total = 0u64;
    loop {
        let mut n = 0;
        while n < BLOCK {
            let got = file.read(&mut buffer[n..])?;
            if got == 0 {
                break;
            }
            n += got;
        }
        if n == 0 {
            break;
        }
        let raw = &buffer[..n];
        sha.update(raw);
        total += n as u64;
        let mut encoder = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::fast());
        encoder.write_all(raw)?;
        let compressed = encoder.finish()?;
        let (codec, data) = if compressed.len() + 32 < raw.len() {
            (1, compressed.as_slice())
        } else {
            (0, raw)
        };
        insert.execute(params![
            id,
            index,
            codec,
            n as i64,
            crc32fast::hash(raw),
            data
        ])?;
        index += 1;
    }
    if total != length {
        return Err(invalid("source asset changed while packaging"));
    }
    let hash = sha
        .finalize()
        .iter()
        .map(|v| format!("{v:02x}"))
        .collect::<String>();
    db.execute("UPDATE assets SET sha256=?1 WHERE id=?2", params![hash, id])?;
    Ok(())
}
fn inventory(root: &Path, path: &Path, out: &mut Vec<(String, PathBuf)>) -> Result<()> {
    for entry in fs::read_dir(path)? {
        let entry = entry?;
        let p = entry.path();
        let kind = entry.file_type()?;
        if kind.is_symlink() {
            return Err(invalid("location packages cannot include symlinks"));
        }
        if kind.is_dir() {
            inventory(root, &p, out)?;
        } else if kind.is_file() {
            let name = p
                .strip_prefix(root)?
                .to_str()
                .ok_or_else(|| invalid("asset names must be UTF-8"))?
                .replace('\\', "/");
            if !safe(&name) {
                return Err(invalid("invalid location asset path"));
            }
            out.push((name, p));
        }
    }
    Ok(())
}
pub fn validate(path: &Path) -> Result<()> {
    let db = connect(path, false)?;
    for name in [
        "manifest.json",
        "city.txt",
        "streets.txt",
        "furniture.txt",
        "landscape.txt",
        "elevation/heightfield.txt",
        "scene.pack",
    ] {
        let found: bool = db.query_row(
            "SELECT EXISTS(SELECT 1 FROM assets WHERE name=?1)",
            [name],
            |r| r.get(0),
        )?;
        if !found {
            return Err(invalid(format!("missing packaged asset: {name}")));
        }
    }
    Ok(())
}
pub fn pack(source: &Path, output: &Path, replace: bool) -> Result<()> {
    if output.extension().is_none_or(|e| e != "sqlite") {
        return Err(invalid("location packages must use the .sqlite extension"));
    }
    if !source.is_dir() {
        return Err(invalid("pack-location requires a source folder"));
    }
    if output.exists() && !replace {
        return Err(invalid("package already exists"));
    }
    let parent = output
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    fs::create_dir_all(parent)?;
    let absolute_parent = parent.canonicalize()?;
    if absolute_parent.starts_with(source.canonicalize()?) {
        return Err(invalid("package output must be outside its source folder"));
    }
    let mut files = vec![];
    inventory(source, source, &mut files)?;
    files.sort_by(|a, b| a.0.cmp(&b.0));
    let temp = tempfile::Builder::new()
        .prefix(".location-")
        .suffix(".sqlite")
        .tempfile_in(parent)?;
    let mut db = Connection::open(temp.path())?;
    db.execute_batch(&format!("PRAGMA journal_mode=OFF; PRAGMA synchronous=OFF; PRAGMA cache_size=-8192; PRAGMA application_id={APP}; PRAGMA user_version={VERSION}; CREATE TABLE assets(id INTEGER PRIMARY KEY,name TEXT NOT NULL UNIQUE,size INTEGER NOT NULL CHECK(size>=0),sha256 TEXT NOT NULL); CREATE TABLE blocks(asset_id INTEGER NOT NULL REFERENCES assets(id),block_no INTEGER NOT NULL,codec INTEGER NOT NULL,raw_size INTEGER NOT NULL,crc32 INTEGER NOT NULL,data BLOB NOT NULL,PRIMARY KEY(asset_id,block_no)) WITHOUT ROWID;"))?;
    let tx = db.transaction()?;
    for (i, (name, path)) in files.iter().enumerate() {
        if i % 500 == 0 || name == "scene.pack" || name == "streets.txt" {
            println!("Packing asset {}/{}: {name}", i + 1, files.len());
        }
        write_asset(&tx, name, path)?;
    }
    tx.commit()?;
    db.close().map_err(|(_, e)| e)?;
    validate(temp.path())?;
    temp.as_file().sync_all()?;
    if replace {
        temp.persist(output)?;
    } else {
        temp.persist_noclobber(output)?;
    }
    println!("Packed {} assets into {}", files.len(), output.display());
    Ok(())
}
pub fn verify(path: &Path) -> Result<()> {
    validate(path)?;
    let db = connect(path, false)?;
    let check: String = db.query_row("PRAGMA integrity_check", [], |r| r.get(0))?;
    if check != "ok" {
        return Err(invalid(check));
    }
    let files = db
        .prepare("SELECT name,size,sha256 FROM assets ORDER BY name")?
        .query_map([], |r| {
            Ok((
                r.get::<_, String>(0)?,
                r.get::<_, i64>(1)? as u64,
                r.get::<_, String>(2)?,
            ))
        })?
        .collect::<rusqlite::Result<Vec<_>>>()?;
    let mut total = 0;
    let mut buffer = vec![0; BLOCK];
    for (name, size, expected) in &files {
        let mut reader = Reader::asset(path, name)?;
        let mut hash = Sha256::new();
        let mut count = 0;
        loop {
            let n = reader.read(&mut buffer)?;
            if n == 0 {
                break;
            }
            hash.update(&buffer[..n]);
            count += n as u64;
        }
        if count != *size
            || hash
                .finalize()
                .iter()
                .map(|v| format!("{v:02x}"))
                .collect::<String>()
                != *expected
        {
            return Err(invalid(format!(
                "package asset verification failed: {name}"
            )));
        }
        total += count;
    }
    println!(
        "Verified {} assets, {:.2} GiB of original data",
        files.len(),
        total as f64 / 1024f64.powi(3)
    );
    Ok(())
}
pub fn unpack(path: &Path, output: &Path) -> Result<()> {
    if output.exists() {
        return Err(invalid("unpack output must not exist"));
    }
    validate(path)?;
    fs::create_dir_all(output)?;
    let result = (|| -> Result<()> {
        let db = connect(path, false)?;
        let names = db
            .prepare("SELECT name FROM assets ORDER BY name")?
            .query_map([], |r| r.get::<_, String>(0))?
            .collect::<rusqlite::Result<Vec<_>>>()?;
        for name in names {
            if !safe(&name) {
                return Err(invalid("unsafe asset path in package"));
            }
            let dest = output.join(&name);
            fs::create_dir_all(dest.parent().unwrap())?;
            let mut file = fs::File::create(dest)?;
            io::copy(&mut Reader::asset(path, &name)?, &mut file)?;
        }
        Ok(())
    })();
    if result.is_err() {
        let _ = fs::remove_dir_all(output);
    }
    result
}
pub fn replace_asset(path: &Path, name: &str, source: &Path) -> Result<()> {
    let mut db = connect(path, true)?;
    db.execute_batch("PRAGMA journal_mode=DELETE; PRAGMA synchronous=FULL;")?;
    let tx = db.transaction()?;
    write_asset(&tx, name, source)?;
    tx.commit()?;
    Ok(())
}
pub fn edit(path: &Path, operation: impl FnOnce(&Path) -> Result<()>) -> Result<()> {
    if path.is_dir() {
        return operation(path);
    }
    validate(path)?;
    let temp = tempfile::Builder::new()
        .prefix(".location-edit-")
        .tempdir_in(path.parent().unwrap_or(Path::new(".")))?;
    let folder = temp.path().join("assets");
    unpack(path, &folder)?;
    operation(&folder)?;
    pack(&folder, path, true)
}
#[cfg(test)]
mod tests {
    use super::*;
    fn fixture(folder: &Path) {
        fs::create_dir_all(folder.join("elevation")).unwrap();
        for name in [
            "manifest.json",
            "city.txt",
            "streets.txt",
            "furniture.txt",
            "landscape.txt",
            "elevation/heightfield.txt",
            "scene.pack",
        ] {
            fs::write(folder.join(name), name).unwrap();
        }
    }
    #[test]
    fn compressed_random_access_round_trip_and_mutation() {
        let t = tempfile::tempdir().unwrap();
        let folder = t.path().join("source");
        fixture(&folder);
        let bytes: Vec<_> = (0..BLOCK * 3 + 17).map(|i| (i % 251) as u8).collect();
        fs::write(folder.join("scene.pack"), &bytes).unwrap();
        let db = t.path().join("test.sqlite");
        pack(&folder, &db, false).unwrap();
        assert!(db.metadata().unwrap().len() < (bytes.len() / 2) as u64);
        verify(&db).unwrap();
        let mut r = Reader::open(&db.join("scene.pack")).unwrap();
        r.seek(SeekFrom::Start((BLOCK - 9) as u64)).unwrap();
        let mut got = vec![0; BLOCK + 23];
        r.read_exact(&mut got).unwrap();
        assert_eq!(got, bytes[BLOCK - 9..BLOCK * 2 + 14]);
        r.seek(SeekFrom::End(-17)).unwrap();
        let mut end = vec![];
        r.read_to_end(&mut end).unwrap();
        assert_eq!(end, bytes[bytes.len() - 17..]);
        assert!(r.seek(SeekFrom::End(-((bytes.len() + 1) as i64))).is_err());
        drop(r);
        let out = t.path().join("unpacked");
        unpack(&db, &out).unwrap();
        assert_eq!(fs::read(out.join("scene.pack")).unwrap(), bytes);
        fs::write(out.join("scene.pack"), b"replacement").unwrap();
        replace_asset(&db, "scene.pack", &out.join("scene.pack")).unwrap();
        assert_eq!(read(&db.join("scene.pack")).unwrap(), b"replacement");
        verify(&db).unwrap();
    }
    #[test]
    fn offline_edits_publish_only_on_success() {
        let t = tempfile::tempdir().unwrap();
        let source = t.path().join("source");
        fixture(&source);
        let path = t.path().join("location.sqlite");
        pack(&source, &path, false).unwrap();
        assert!(edit(&path, |folder| {
            fs::write(folder.join("city.txt"), "failed edit")?;
            Err(invalid("deliberate failure"))
        })
        .is_err());
        assert_eq!(read_to_string(path.join("city.txt")).unwrap(), "city.txt");
        edit(&path, |folder| {
            fs::write(folder.join("city.txt"), "successful edit")?;
            Ok(())
        })
        .unwrap();
        assert_eq!(
            read_to_string(path.join("city.txt")).unwrap(),
            "successful edit"
        );
        verify(&path).unwrap();
        assert!(!path.with_extension("sqlite-wal").exists());
    }
    #[test]
    fn rejects_corrupt_blocks_and_unsafe_names() {
        let t = tempfile::tempdir().unwrap();
        let folder = t.path().join("source");
        fixture(&folder);
        let db = t.path().join("test.sqlite");
        pack(&folder, &db, false).unwrap();
        assert!(!exists(&db.join("missing")));
        assert!(Reader::asset(&db, "../escape").is_err());
        let conn = connect(&db, true).unwrap();
        conn.execute("UPDATE blocks SET crc32=0 WHERE asset_id=(SELECT id FROM assets WHERE name='scene.pack')",[]).unwrap();
        drop(conn);
        assert!(read(&db.join("scene.pack")).is_err());
        assert!(verify(&db).is_err());
    }
}
