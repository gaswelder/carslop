//! Download source files into the compiler cache.
use crate::{
    imagery::{image_path, Block, CORE, PAD, SERVICE},
    location::{invalid, Result},
};
use serde_json::Value;
use std::{
    fs,
    io::{Read, Write},
    path::Path,
    sync::{
        atomic::{AtomicUsize, Ordering},
        mpsc,
    },
    time::Duration,
};

pub struct DownloadOptions {
    pub attempts: usize,
    pub max_bytes: Option<u64>,
    pub validate: fn(&Path) -> Result<bool>,
    pub replace_invalid: bool,
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

fn url(b: &Block) -> String {
    let e = b.extent;
    format!("{SERVICE}/export?bbox={:.9},{:.9},{:.9},{:.9}&bboxSR=3857&imageSR=3857&size={},{}&format=jpg&f=image",e[0],e[1],e[2],e[3],CORE+2*PAD,CORE+2*PAD)
}
fn valid_image(path: &Path) -> Result<bool> {
    let bytes = fs::read(path)?;
    Ok(image::load_from_memory(&bytes)
        .is_ok_and(|im| im.width() == CORE + 2 * PAD && im.height() == CORE + 2 * PAD))
}

fn valid_service(path: &Path) -> Result<bool> {
    Ok(serde_json::from_slice::<Value>(&fs::read(path)?)
        .is_ok_and(|value| value.get("error").is_none()))
}

pub fn download_tiles(work: &Path, service_path: &Path, all: &[Block]) -> Result<()> {
    fs::create_dir_all(work)?;
    let agent: ureq::Agent = ureq::Agent::config_builder()
        .timeout_global(Some(Duration::from_secs(60)))
        .build()
        .into();
    download(
        &agent,
        &format!("{SERVICE}?f=pjson"),
        service_path,
        &DownloadOptions {
            validate: valid_service,
            replace_invalid: true,
            ..DownloadOptions::default()
        },
    )?;
    let options = DownloadOptions {
        attempts: 3,
        max_bytes: Some(32 * 1024 * 1024),
        validate: valid_image,
        replace_invalid: true,
    };
    let next = AtomicUsize::new(0);
    let (send, recv) = mpsc::channel();
    std::thread::scope(|scope| {
        for _ in 0..4 {
            let send = send.clone();
            let agent = agent.clone();
            let next = &next;
            let options = &options;
            scope.spawn(move || loop {
                let i = next.fetch_add(1, Ordering::Relaxed);
                if i >= all.len() {
                    break;
                }
                let block = &all[i];
                let result = download(&agent, &url(block), &image_path(work, block), options)
                    .map_err(|e| format!("block {},{}: {e}", block.x, block.y));
                let failed = result.is_err();
                let _ = send.send(result);
                if failed {
                    next.store(all.len(), Ordering::Relaxed);
                    break;
                }
            });
        }
        drop(send);
        let mut count = 0;
        let mut error = None;
        for result in recv {
            match result {
                Ok(()) => {
                    count += 1;
                    if count % 10 == 0 || count == all.len() {
                        println!("Images ready: {count}/{}", all.len());
                    }
                }
                Err(e) => error = Some(e),
            }
        }
        if let Some(e) = error {
            Err(invalid(e))
        } else if count != all.len() {
            Err(invalid("incomplete image downloads"))
        } else {
            Ok(())
        }
    })
}
