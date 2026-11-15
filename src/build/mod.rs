pub mod add_location;
pub mod authoring;
mod barriers;
mod buildings;
mod collision;
mod compile_location;
pub mod create_track;
mod crossings;
mod details;
mod download;
mod download_location;
mod furniture;
mod hgt;
mod imagery;
mod infer_trees;
mod landscape;
mod lane_details;
pub mod osm;
mod osm_extract;
pub mod osm_labels;
mod scene;
pub mod surfaces;
mod terrain;
mod transport;
mod tree_obstacles;
mod tree_proposals;

#[cfg(test)]
#[path = "../test/support.rs"]
pub mod test_support;
