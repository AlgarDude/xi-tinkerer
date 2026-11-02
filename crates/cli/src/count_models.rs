use std::path::PathBuf;

use anyhow::{Result, anyhow};
use dats::{context::DatContext, formats::zone_data::ChunkData, id_mapping::DatIdMapping};

/// Lists every collision model in a zone's data file with its placement count and height span.
pub fn count_models(ffxi_path: PathBuf, zone_id: u16) -> Result<()> {
    let dat_context = DatContext::from_ffxi_path(ffxi_path)?;
    let zone_data_dat = DatIdMapping::get().zone_data.get(&zone_id).ok_or(anyhow!("No zone data for {}", zone_id))?;
    let zone_data = dat_context.get_data_from_dat(zone_data_dat)?;
    for (index, chunk) in zone_data.dat.chunks.iter().enumerate() {
        if let ChunkData::ZoneModel { zone_model } = &chunk.data {
            let heights: Vec<f32> = zone_model.mesh.placements.iter().flat_map(|p| [p.min_y, p.max_y]).collect();
            let low = heights.iter().cloned().fold(f32::MAX, f32::min);
            let high = heights.iter().cloned().fold(f32::MIN, f32::max);
            println!("chunk {} '{}': {} placements, {} blocks, heights {:.1}..{:.1}; {} model instances; coll1 0x{:x} coll2end 0x{:x} coll3 0x{:x} coll4 count {} at 0x{:x}; flags 0x{:x}",
                index, chunk.four_char_code, zone_model.mesh.placements.len(), zone_model.mesh.blocks.len(), low, high, zone_model.model_block_instances.len(),
                zone_model.unk_coll1_offset, zone_model.unk_coll2_end_offset, zone_model.unk_coll3_offset, zone_model.unk_coll4_count, zone_model.unk_coll4_offset, zone_model.flags);
            if let (Ok(x), Ok(z)) = (std::env::var("NEAR_X"), std::env::var("NEAR_Z")) {
                let (x, z): (f32, f32) = (x.parse()?, z.parse()?);
                for instance in &zone_model.model_block_instances {
                    let t = &instance.translation;
                    if (t.x - x).abs() < 8.0 && (t.z - z).abs() < 8.0 {
                        println!("  instance '{}' at ({:.2}, {:.2}, {:.2}) rot ({:.2}, {:.2}, {:.2}) scale ({:.2}, {:.2}, {:.2}) unk {:?}", instance.id, t.x, t.y, t.z,
                            instance.rotation.x, instance.rotation.y, instance.rotation.z, instance.scale.x, instance.scale.y, instance.scale.z, instance.unk_vals);
                    }
                }
            }
        }
    }
    Ok(())
}
