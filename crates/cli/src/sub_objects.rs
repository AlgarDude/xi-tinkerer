use std::path::PathBuf;

use anyhow::{Result, anyhow};
use dats::{context::DatContext, formats::zone_data::zone_model::ZoneMesh, id_mapping::DatIdMapping};

/// Prints the zone's named sub-objects (name, the three dwords after it, transform, world box of the unit cube); RAW_DAT=<dat id> loads a DAT by id instead of the zone.
pub fn sub_objects(ffxi_path: PathBuf, zone_id: u16, prefixes: String) -> Result<()> {
    let dat_context = DatContext::from_ffxi_path(ffxi_path)?;
    let zone_data: dats::formats::zone_data::ZoneData = match std::env::var("RAW_DAT") {
        Ok(raw) => dat_context.get_data_from_dat_id(dats::base::DatId::from(raw.parse::<u32>()?))?,
        Err(_) => {
            let zone_data_dat = DatIdMapping::get()
                .zone_data
                .get(&zone_id)
                .ok_or(anyhow!("No zone data for {}", zone_id))?;
            dat_context.get_data_from_dat(zone_data_dat)?.dat
        }
    };
    let zone_mesh = ZoneMesh::parse_from_zone_data(&zone_data)?;
    for instance in &zone_mesh.model_block_instances {
        let first = instance.raw_id.first().copied().unwrap_or(0) as char;
        if !prefixes.is_empty() && !prefixes.contains(first) {
            continue;
        }
        let dword = |index: usize| u32::from_le_bytes(instance.raw_id[4 + index * 4..8 + index * 4].try_into().unwrap());
        let m = instance.to_world_matrix().into_raw();
        let mut low = [f32::MAX; 3];
        let mut high = [f32::MIN; 3];
        for corner in 0..8 {
            let local = [if corner & 1 > 0 { 0.5 } else { -0.5 }, if corner & 2 > 0 { 0.5 } else { -0.5 }, if corner & 4 > 0 { 0.5 } else { -0.5 }];
            for axis in 0..3 {
                let value = local[0] * m[0][axis] + local[1] * m[1][axis] + local[2] * m[2][axis] + m[3][axis];
                low[axis] = low[axis].min(value);
                high[axis] = high[axis].max(value);
            }
        }
        println!(
            "{}\t{}\t{}\t{}\tpos {:.2},{:.2},{:.2}\trot {:.3},{:.3},{:.3}\tscale {:.2},{:.2},{:.2}\tbox x {:.1}..{:.1} h {:.1}..{:.1} z {:.1}..{:.1}",
            String::from_utf8_lossy(&instance.raw_id[0..4]),
            dword(0), dword(1), dword(2),
            instance.translation.x, instance.translation.y, instance.translation.z,
            instance.rotation.x, instance.rotation.y, instance.rotation.z,
            instance.scale.x, instance.scale.y, instance.scale.z,
            low[0], high[0], low[1], high[1], low[2], high[2]
        );
    }
    Ok(())
}
