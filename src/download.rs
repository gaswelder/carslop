//! Download source files into the compiler cache.
use crate::error::{invalid, Result};
use std::{
    fs,
    io::{Read, Write},
    path::Path,
    time::Duration,
};

pub struct DownloadOptions {
    pub attempts: usize,
    pub(super) max_bytes: Option<u64>,
    pub(super) validate: fn(&Path) -> Result<bool>,
    pub(super) replace_invalid: bool,
}

impl Default for DownloadOptions {
    fn default() -> Self {
        Self {
            attempts: 1,
            max_bytes: None,
            validate: |path| Ok(fs::metadata(path)?.len() > 0),
            replace_invalid: false,
        }
    }
}

// Cache entries are published only after a complete, validated response has been flushed.
pub fn download(
    agent: &ureq::Agent,
    url: &str,
    path: &Path,
    options: &DownloadOptions,
) -> Result<()> {
    if options.attempts == 0 {
        return Err(invalid("download requires at least one attempt"));
    }
    if path.is_file()
        && options
            .max_bytes
            .is_none_or(|limit| fs::metadata(path).is_ok_and(|m| m.len() <= limit))
        && (options.validate)(path)?
    {
        return Ok(());
    }
    println!("Downloading {url}");
    let parent = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    fs::create_dir_all(parent)?;
    for attempt in 0..options.attempts {
        let result = (|| -> Result<()> {
            let mut file = tempfile::NamedTempFile::new_in(parent)?;
            let mut response = agent
                .get(url)
                .header("User-Agent", "carslop-location-import/0.1")
                .call()?;
            let expected = response
                .headers()
                .get("content-length")
                .and_then(|v| v.to_str().ok())
                .and_then(|v| v.parse::<u64>().ok());
            let mut reader = response.body_mut().as_reader();
            let count = if let Some(limit) = options.max_bytes {
                std::io::copy(
                    &mut reader.by_ref().take(limit.saturating_add(1)),
                    &mut file,
                )?
            } else {
                std::io::copy(&mut reader, &mut file)?
            };
            if options.max_bytes.is_some_and(|limit| count > limit) {
                return Err(invalid(format!("download exceeds size limit: {url}")));
            }
            if count == 0 || expected.is_some_and(|n| n != count) {
                return Err(invalid(format!("incomplete download: {url}")));
            }
            file.flush()?;
            if !(options.validate)(file.path())? {
                return Err(invalid(format!("invalid downloaded file: {url}")));
            }
            file.as_file().sync_all()?;
            if options.replace_invalid {
                file.persist(path).map_err(|e| e.error)?;
            } else {
                file.persist_noclobber(path).map_err(|e| e.error)?;
            }
            Ok(())
        })();
        match result {
            Ok(()) => return Ok(()),
            Err(error) if attempt + 1 == options.attempts => return Err(error),
            Err(_) => std::thread::sleep(Duration::from_secs(1 << attempt.min(6))),
        }
    }
    unreachable!("download attempts are nonzero")
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{
        io::{Read, Write},
        net::TcpListener,
    };
    fn serve(responses: Vec<(usize, &'static [u8])>) -> (String, std::thread::JoinHandle<()>) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}/file", listener.local_addr().unwrap());
        let server = std::thread::spawn(move || {
            for (length, body) in responses {
                let (mut stream, _) = listener.accept().unwrap();
                stream
                    .set_read_timeout(Some(Duration::from_secs(5)))
                    .unwrap();
                let mut request = Vec::new();
                while !request.ends_with(b"\r\n\r\n") {
                    let mut byte = [0];
                    stream.read_exact(&mut byte).unwrap();
                    request.push(byte[0]);
                }
                write!(
                    stream,
                    "HTTP/1.1 200 OK\r\nContent-Length: {length}\r\nConnection: close\r\n\r\n"
                )
                .unwrap();
                stream.write_all(body).unwrap();
            }
        });
        (url, server)
    }

    fn valid_payload(path: &Path) -> Result<bool> {
        Ok(fs::read(path)? == b"valid")
    }

    #[test]
    fn retries_invalid_responses_and_replaces_invalid_cache() {
        let temp = tempfile::tempdir().unwrap();
        let path = temp.path().join("cached.data");
        fs::write(&path, b"invalid").unwrap();
        let (url, server) = serve(vec![(3, b"bad"), (5, b"valid")]);
        let options = DownloadOptions {
            attempts: 2,
            validate: valid_payload,
            replace_invalid: true,
            ..DownloadOptions::default()
        };
        download(&ureq::Agent::new_with_defaults(), &url, &path, &options).unwrap();
        server.join().unwrap();
        assert_eq!(fs::read(&path).unwrap(), b"valid");
        download(
            &ureq::Agent::new_with_defaults(),
            "invalid-url",
            &path,
            &options,
        )
        .unwrap();
        assert_eq!(fs::read_dir(temp.path()).unwrap().count(), 1);
    }

    #[test]
    fn oversized_or_truncated_responses_do_not_replace_existing_files() {
        for (length, body, limit) in [
            (5, b"valid".as_slice(), Some(4)),
            (8, b"short".as_slice(), None),
        ] {
            let temp = tempfile::tempdir().unwrap();
            let path = temp.path().join("cached.data");
            fs::write(&path, b"invalid").unwrap();
            let (url, server) = serve(vec![(length, body)]);
            let options = DownloadOptions {
                max_bytes: limit,
                validate: valid_payload,
                replace_invalid: true,
                ..DownloadOptions::default()
            };
            assert!(download(&ureq::Agent::new_with_defaults(), &url, &path, &options).is_err());
            server.join().unwrap();
            assert_eq!(fs::read(&path).unwrap(), b"invalid");
            assert_eq!(fs::read_dir(temp.path()).unwrap().count(), 1);
        }
    }
}
