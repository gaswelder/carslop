//! Prepared mapped details and their privately owned spatial indexes.

use crate::model::map_details::BuildingStyle;
use crate::model::map_details::Data;
use crate::model::map_details::Deck;
use crate::model::map_details::MapObject;
use crate::model::map_details::ObjectKind;
use crate::model::map_details::TreeStyle;
use crate::model::world::RoadRise;
use crate::util::geometry::XYZ;
use crate::util::geometry::XZ;
use crate::world;
use std::collections::HashMap;

#[derive(Debug)]
struct IndexedObject {
    index: usize,
    bounds: rstar::AABB<[f32; 2]>,
}
impl rstar::RTreeObject for IndexedObject {
    type Envelope = rstar::AABB<[f32; 2]>;
    fn envelope(&self) -> Self::Envelope {
        self.bounds
    }
}
#[derive(Debug)]
pub struct Prepared {
    data: Data,
    buildings: HashMap<usize, usize>,
    trees: HashMap<u64, TreeStyle>,
    objects: rstar::RTree<IndexedObject>,
    decks: rstar::RTree<IndexedObject>,
    deck_roads: HashMap<usize, usize>,
}

pub fn new(mut data: Data) -> Prepared {
    // Retain the legacy serialized variant, but never install gates from old imports.
    data.objects
        .retain(|object| object.kind != ObjectKind::Gate);
    let buildings = data
        .buildings
        .iter()
        .enumerate()
        .map(|(i, style)| (style.volume, i))
        .collect();
    let trees = data
        .trees
        .iter()
        .map(|style| (style.id, style.clone()))
        .collect();
    let objects = rstar::RTree::bulk_load(
        data.objects
            .iter()
            .enumerate()
            .map(|(index, object)| {
                let reach = object.width / 2. + 3.;
                IndexedObject {
                    index,
                    bounds: rstar::AABB::from_corners(
                        object.position.map(|v| v - reach),
                        object.position.map(|v| v + reach),
                    ),
                }
            })
            .collect(),
    );
    let decks = rstar::RTree::bulk_load(
        data.decks
            .iter()
            .enumerate()
            .map(|(index, deck)| {
                let reach = deck.width / 2. + 3.;
                let min = [
                    deck.path.iter().map(|p| p[0]).fold(f32::INFINITY, f32::min) - reach,
                    deck.path.iter().map(|p| p[2]).fold(f32::INFINITY, f32::min) - reach,
                ];
                let max = [
                    deck.path
                        .iter()
                        .map(|p| p[0])
                        .fold(f32::NEG_INFINITY, f32::max)
                        + reach,
                    deck.path
                        .iter()
                        .map(|p| p[2])
                        .fold(f32::NEG_INFINITY, f32::max)
                        + reach,
                ];
                IndexedObject {
                    index,
                    bounds: rstar::AABB::from_corners(min, max),
                }
            })
            .collect(),
    );
    let deck_roads = data
        .decks
        .iter()
        .enumerate()
        .map(|(i, deck)| (deck.road, i))
        .collect();
    Prepared {
        data,
        buildings,
        decks,
        deck_roads,
        trees,
        objects,
    }
}
pub fn road_deck(prepared: &Prepared, road: usize) -> Option<&Deck> {
    prepared
        .deck_roads
        .get(&road)
        .map(|i| &prepared.data.decks[*i])
}
pub fn nearby_decks(prepared: &Prepared, p: XZ<f32>) -> impl Iterator<Item = &Deck> {
    decks_near_segment(prepared, p, p)
}
pub fn decks_near_segment(
    prepared: &Prepared,
    a: XZ<f32>,
    b: XZ<f32>,
) -> impl Iterator<Item = &Deck> {
    prepared
        .decks
        .locate_in_envelope_intersecting(&rstar::AABB::from_corners(
            [a.x.min(b.x), a.z.min(b.z)],
            [a.x.max(b.x), a.z.max(b.z)],
        ))
        .map(|item| &prepared.data.decks[item.index])
}
pub fn height_on(deck: &Deck, p: XZ<f32>) -> Option<f32> {
    deck_triangles(deck)
        .into_iter()
        .filter_map(|triangle| world::raised_roads::road_rise_offset(&triangle, p))
        .max_by(f32::total_cmp)
}
pub fn deck_height(prepared: &Prepared, p: XZ<f32>, reference: f32) -> Option<f32> {
    nearby_decks(prepared, p)
        .filter_map(|deck| height_on(deck, p))
        .min_by(|a, b| (a - reference).abs().total_cmp(&(b - reference).abs()))
}
pub fn nearby_objects(prepared: &Prepared, p: XZ<f32>) -> impl Iterator<Item = &MapObject> {
    prepared
        .objects
        .locate_in_envelope_intersecting(&rstar::AABB::from_point([p.x, p.z]))
        .map(|item| &prepared.data.objects[item.index])
}
pub fn data(prepared: &Prepared) -> &Data {
    &prepared.data
}
pub fn building(prepared: &Prepared, index: usize) -> Option<&BuildingStyle> {
    prepared
        .buildings
        .get(&index)
        .map(|i| &prepared.data.buildings[*i])
}
pub fn tree(prepared: &Prepared, id: u64) -> Option<&TreeStyle> {
    prepared.trees.get(&id)
}

/// Shared deck footprint and height planes for contact and scene construction.
pub fn deck_triangles(deck: &Deck) -> Vec<RoadRise> {
    let points: Vec<_> = deck
        .path
        .iter()
        .map(|p| XYZ {
            x: p[0],
            y: p[1],
            z: p[2],
        })
        .collect();
    let sections: Vec<_> = points
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let a = points[i.saturating_sub(1)];
            let b = points[(i + 1).min(points.len() - 1)];
            let length = (b.x - a.x).hypot(b.z - a.z).max(0.0001);
            let normal = XZ {
                x: -(b.z - a.z) / length,
                z: (b.x - a.x) / length,
            };
            [-1., 1.].map(|side| XYZ {
                x: p.x + normal.x * side * deck.width / 2.,
                y: p.y,
                z: p.z + normal.z * side * deck.width / 2.,
            })
        })
        .collect();
    sections
        .windows(2)
        .flat_map(|p| {
            [
                RoadRise {
                    points: [p[0][0], p[0][1], p[1][1]],
                },
                RoadRise {
                    points: [p[0][0], p[1][1], p[1][0]],
                },
            ]
        })
        .collect()
}
