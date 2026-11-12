use std::path::PathBuf;

use anyhow::Result;
use dats::{context::DatContext, formats::zone_data::zone_model::ZoneMesh, id_mapping::DatIdMapping};

/// For every zone, counts triangles whose stored normal is all zero or not finite (the client repairs those to (0, 0.000001, 0), a wall), and prints a sample triangle per zone.
pub fn zero_normals(ffxi_path: PathBuf) -> Result<()> {
    let dat_context = DatContext::from_ffxi_path(ffxi_path)?;
    let mut zones: Vec<u16> = (0..512).collect();
    zones.retain(|zone| DatIdMapping::get().zone_data.get(zone).is_some());
    let raw: Vec<u32> = std::env::var("RAW_DATS").map(|v| v.split(',').filter_map(|x| x.trim().parse().ok()).collect()).unwrap_or_default();
    let items: Vec<(u32, Option<dats::formats::zone_data::ZoneData>)> = if raw.is_empty() {
        zones.iter().map(|&zone| {
            let data = DatIdMapping::get().zone_data.get(&zone).and_then(|dat| dat_context.get_data_from_dat(dat).ok()).map(|d| d.dat);
            (zone as u32, data)
        }).collect()
    } else {
        raw.iter().map(|&id| (id, dat_context.get_data_from_dat_id(dats::base::DatId::from(id)).ok())).collect()
    };
    for (zone, data) in items {
        let Some(data) = data else { continue };
        let Ok(mesh) = ZoneMesh::parse_from_zone_data(&data) else { continue };
        let mut count = 0usize;
        let mut sample = None;
        for block in &mesh.mesh.blocks {
            for tri in &block.triangles {
                let Some(n) = block.normals.get(tri.normal_idx as usize) else {
                    count += 1;
                    continue;
                };
                let length = n.x * n.x + n.y * n.y + n.z * n.z;
                if !length.is_finite() || length < 1e-12 {
                    count += 1;
                    if sample.is_none() {
                        let v = &block.vertices[tri.vertex1_idx as usize];
                        sample = Some((v.x, v.y, v.z));
                    }
                }
            }
        }
        if count > 0 {
            println!("{}\t{}\tlocal sample {:?}", zone, count, sample);
        }
    }
    Ok(())
}
