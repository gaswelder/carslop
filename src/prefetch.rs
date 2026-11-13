//! Bounded background geometry reads. OpenGL uploads stay on the render thread.
use crate::{scene::Vertex, scene_archive::Archive};
use std::{
    collections::{HashMap, HashSet, VecDeque},
    sync::{Arc, Condvar, Mutex},
    thread::JoinHandle,
};
type Key = (usize, usize);
const DEFAULT_BUDGET: usize = 32 * 1024 * 1024;
#[derive(Default)]
struct State {
    order: Vec<Key>,
    queue: VecDeque<Key>,
    wanted: HashSet<Key>,
    inflight: Option<Key>,
    ready: HashMap<Key, Result<Vec<Vertex>, String>>,
    bytes: usize,
    peak: usize,
    read_bytes: usize,
    decode_peak: usize,
    hits: usize,
    misses: usize,
    stop: bool,
}
pub struct Prefetch {
    shared: Arc<(Mutex<State>, Condvar)>,
    worker: Option<JoinHandle<()>>,
}
impl Prefetch {
    pub fn new(archive: Archive) -> Self {
        Self::with_budget(archive, DEFAULT_BUDGET)
    }
    fn with_budget(mut archive: Archive, budget: usize) -> Self {
        let shared = Arc::new((Mutex::new(State::default()), Condvar::new()));
        let background = Arc::clone(&shared);
        let worker = std::thread::Builder::new()
            .name("city-prefetch".into())
            .spawn(move || {
                let (mutex, signal) = &*background;
                loop {
                    let key = {
                        let mut state = mutex.lock().unwrap();
                        loop {
                            if state.stop {
                                return;
                            }
                            let Some(&key) = state.queue.front() else {
                                state = signal.wait(state).unwrap();
                                continue;
                            };
                            let expected = archive.entries[key.0].layers[key.1].count
                                * std::mem::size_of::<Vertex>();
                            if expected > budget {
                                state.queue.pop_front();
                                state.wanted.remove(&key);
                                continue;
                            }
                            if state.bytes + expected > budget {
                                let rank = |k: &Key| {
                                    state
                                        .order
                                        .iter()
                                        .position(|q| q == k)
                                        .unwrap_or(usize::MAX)
                                };
                                let worst = state.ready.keys().max_by_key(|k| rank(k)).copied();
                                if let Some(worst) = worst.filter(|w| rank(w) > rank(&key)) {
                                    if let Some(Ok(v)) = state.ready.remove(&worst) {
                                        state.bytes -= v.len() * std::mem::size_of::<Vertex>();
                                    }
                                    continue;
                                }
                                state = signal.wait(state).unwrap();
                                continue;
                            }
                            state.queue.pop_front();
                            state.inflight = Some(key);
                            break key;
                        }
                    };
                    let before = archive.bytes_read;
                    let result = archive.read_layer(key.0, key.1).map_err(|e| e.to_string());
                    let mut state = mutex.lock().unwrap();
                    state.inflight = None;
                    state.read_bytes += archive.bytes_read - before;
                    state.decode_peak = state.decode_peak.max(archive.peak_decode_bytes);
                    if state.wanted.contains(&key) {
                        let bytes = result
                            .as_ref()
                            .map_or(0, |v| v.len() * std::mem::size_of::<Vertex>());
                        if state.bytes + bytes <= budget {
                            state.bytes += bytes;
                            state.peak = state.peak.max(state.bytes);
                            state.ready.insert(key, result);
                        }
                    }
                    signal.notify_all();
                }
            })
            .expect("start city prefetch worker");
        Self {
            shared,
            worker: Some(worker),
        }
    }
    pub fn plan(&self, keys: Vec<Key>) {
        let (mutex, signal) = &*self.shared;
        let mut state = mutex.lock().unwrap();
        let mut unique = HashSet::new();
        let order: Vec<_> = keys
            .into_iter()
            .take(64)
            .filter(|k| unique.insert(*k))
            .collect();
        let stale: Vec<_> = state
            .ready
            .keys()
            .filter(|k| !unique.contains(k))
            .copied()
            .collect();
        for key in stale {
            if let Some(Ok(v)) = state.ready.remove(&key) {
                state.bytes -= v.len() * std::mem::size_of::<Vertex>();
            }
        }
        state.wanted = unique;
        state.order = order.clone();
        state.queue = order
            .into_iter()
            .filter(|k| !state.ready.contains_key(k) && state.inflight != Some(*k))
            .collect();
        signal.notify_all();
    }
    pub fn take(&self, key: Key) -> Option<Result<Vec<Vertex>, String>> {
        let (mutex, signal) = &*self.shared;
        let mut state = mutex.lock().unwrap();
        state.wanted.remove(&key);
        state.queue.retain(|q| *q != key);
        let result = state.ready.remove(&key);
        if let Some(Ok(v)) = &result {
            state.bytes -= v.len() * std::mem::size_of::<Vertex>();
            state.hits += 1;
        } else if result.is_none() {
            state.misses += 1;
        }
        signal.notify_all();
        result
    }

    #[cfg(test)]
    fn stats(&self) -> (usize, usize, usize, usize, usize) {
        let state = self.shared.0.lock().unwrap();
        (
            state.hits,
            state.misses,
            state.peak,
            state.read_bytes,
            state.decode_peak,
        )
    }
}
impl Drop for Prefetch {
    fn drop(&mut self) {
        {
            let mut state = self.shared.0.lock().unwrap();
            state.stop = true;
            state.queue.clear();
            state.ready.clear();
            self.shared.1.notify_all();
        }
        if let Some(worker) = self.worker.take() {
            worker.join().expect("join city prefetch worker");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn background_reads_are_bounded_consumable_and_cancelled_when_no_longer_wanted() {
        let path = std::env::temp_dir().join(format!(
            "cars-prefetch-{}-{}.pack",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let tri = [[0., 0., 0.], [1., 0., 0.], [0., 0., 1.]]
            .map(|position| Vertex {
                position,
                normal: [0., 1., 0.],
                color: [0.5; 3],
                material: [-1., 0.],
            })
            .to_vec();
        crate::scene_archive::write(
            &path,
            std::array::from_fn(|_| crate::test_support::partition(tri.clone())),
        )
        .unwrap();
        let p = Prefetch::with_budget(
            Archive::open(&path).unwrap(),
            3 * std::mem::size_of::<Vertex>(),
        );
        p.plan(vec![(0, 0), (0, 1), (0, 2)]);
        for key in [(0, 0), (0, 1)] {
            let state = p.shared.0.lock().unwrap();
            let (state, _) = p
                .shared
                .1
                .wait_timeout_while(state, std::time::Duration::from_secs(3), |s| {
                    !s.ready.contains_key(&key)
                })
                .unwrap();
            assert!(
                state.ready.contains_key(&key),
                "background read did not complete"
            );
            drop(state);
            let loaded = p.take(key).unwrap().unwrap();
            assert_eq!(loaded.len(), 3);
            assert_eq!(loaded[1].position, tri[1].position);
        }
        p.plan(Vec::new());
        let state = p.shared.0.lock().unwrap();
        let (state, _) = p
            .shared
            .1
            .wait_timeout_while(state, std::time::Duration::from_secs(3), |s| {
                s.inflight.is_some()
            })
            .unwrap();
        assert!(state.ready.is_empty() && state.bytes == 0);
        drop(state);
        let (hits, _, peak, _, _) = p.stats();
        assert_eq!(hits, 2);
        assert!(peak <= 3 * std::mem::size_of::<Vertex>());
        drop(p);
        std::fs::remove_file(path).unwrap();
    }
}
