use super::*;
use crate::render;
use crate::test;
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
    render::scene_archive::write(
        &path,
        std::array::from_fn(|_| test::support::partition(tri.clone())),
    )
    .unwrap();
    let p = render::prefetch::with_budget(
        render::scene_archive::open(&path).unwrap(),
        3 * std::mem::size_of::<Vertex>(),
    );
    render::prefetch::plan(&p, vec![(0, 0), (0, 1), (0, 2)]);
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
        let loaded = render::prefetch::take(&p, key).unwrap().unwrap();
        assert_eq!(loaded.len(), 3);
        assert_eq!(loaded[1].position, tri[1].position);
    }
    render::prefetch::plan(&p, Vec::new());
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
    let (hits, _, peak, _, _) = render::prefetch::stats(&p);
    assert_eq!(hits, 2);
    assert!(peak <= 3 * std::mem::size_of::<Vertex>());
    drop(p);
    std::fs::remove_file(path).unwrap();
}
