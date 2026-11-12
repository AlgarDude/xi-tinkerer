use std::{io::Write, path::PathBuf};

use anyhow::{Context, Result, anyhow};
use dats::{
    context::DatContext, formats::zone_data::zone_model::ZoneMesh, id_mapping::DatIdMapping,
};
use flate2::{Compression, write::ZlibEncoder};
use processor::ximesh::get_ximesh_bytes;
use tokio::{fs, task::JoinSet};

use crate::util::{ZoneInfo, get_zone_ids_from_dats};

pub fn get_ximesh_filename(zone_info: &ZoneInfo) -> String {
    format!("{}.ximesh", zone_info.filename)
}

pub async fn export_zone_meshes(ffxi_path: PathBuf, mut out_dir: PathBuf, interiors: bool) -> Result<()> {
    let dat_context = DatContext::from_ffxi_path(ffxi_path)?;

    let zone_infos = get_zone_ids_from_dats(&DatIdMapping::get().zone_data, &dat_context).await?;

    fs::create_dir_all(&out_dir).await?;
    out_dir = fs::canonicalize(out_dir).await?;

    let index: String = zone_infos
        .iter()
        .map(|zone_info| format!("{}\t{}\t{}\n", zone_info.id, zone_info.name, get_ximesh_filename(zone_info)))
        .collect();
    fs::write(out_dir.join("zones.tsv"), index).await?;

    let mut join_set: JoinSet<Result<()>> = JoinSet::new();

    for zone_info in zone_infos {
        let dat_context = dat_context.clone();

        let mut out_path = out_dir.clone();
        out_path.push(get_ximesh_filename(&zone_info));

        join_set.spawn(async move {
            let zone_data_dat = DatIdMapping::get()
                .zone_data
                .get(&zone_info.id)
                .ok_or(anyhow!("No zone data for {}", zone_info.id))?;

            let zone_data = dat_context
                .get_data_from_dat(zone_data_dat)
                .with_context(|| {
                    format!(
                        "Failed to get data for \"{}\" ({})",
                        zone_info.name, zone_info.id
                    )
                })?;

            let zone_model = ZoneMesh::parse_from_zone_data(&zone_data.dat).with_context(|| {
                format!(
                    "Failed to parse data for \"{}\" ({})",
                    zone_info.name, zone_info.id
                )
            })?;

            let merged;
            let zone_model = if interiors {
                merged = with_interiors(zone_model, &zone_data.dat, &dat_context, &zone_info.name);
                &merged
            } else {
                zone_model
            };
            let mesh_data = get_ximesh_bytes(zone_model);

            let mut encoder = ZlibEncoder::new(Vec::new(), Compression::default());
            encoder.write_all(&mesh_data)?;
            fs::write(out_path, encoder.finish()?).await?;

            Ok::<_, anyhow::Error>(())
        });
    }

    while let Some(res) = join_set.join_next().await {
        match res {
            Ok(Ok(_)) => {}
            Ok(Err(err)) => {
                eprintln!("Parse error: {}", err);
            }
            Err(err) => {
                eprintln!("Join error: {err:?}");
            }
        }
    }

    Ok(())
}

/// The zone's collision as a player inside its buildings sees it: every sub-area shell placement (unk_2 != 0) dropped, every sub-area model's collision added.
fn with_interiors(zone_model: &ZoneMesh, zone_data: &dats::formats::zone_data::ZoneData, dat_context: &DatContext, name: &str) -> ZoneMesh {
    let mut merged = zone_model.clone();
    let shells: std::collections::HashSet<u32> = (0..merged.mesh.placements.len() as u32).filter(|&i| merged.mesh.placements[i as usize].unk_2 != 0).collect();
    if shells.is_empty() {
        return merged;
    }
    let boxed = trigger_box_ids(zone_data);
    let shells: std::collections::HashSet<u32> = shells.into_iter().filter(|&i| boxed.contains(&merged.mesh.placements[i as usize].unk_2)).collect();
    let mut ids: Vec<u32> = shells.iter().map(|&i| merged.mesh.placements[i as usize].unk_2).collect();
    ids.sort();
    ids.dedup();
    for cell in merged.mesh.grid_cells.iter_mut() {
        cell.indices.retain(|entry| !shells.contains(&entry.placement_idx));
    }
    let mut added = vec![];
    for id in ids {
        let dat_id = if id < 600 { id + 100 } else { id + 0x144F7 };
        let interior: Result<dats::formats::zone_data::ZoneData, _> = dat_context.get_data_from_dat_id(dats::base::DatId::from(dat_id));
        let Ok(interior) = interior else {
            eprintln!("{name}: sub-area {id} (DAT {dat_id}) did not load");
            continue;
        };
        let Ok(interior_mesh) = ZoneMesh::parse_from_zone_data(&interior) else {
            eprintln!("{name}: sub-area {id} (DAT {dat_id}) has no zone model");
            continue;
        };
        let block_base = merged.mesh.blocks.len() as u32;
        let placement_base = merged.mesh.placements.len() as u32;
        merged.mesh.blocks.extend(interior_mesh.mesh.blocks.iter().cloned());
        merged.mesh.placements.extend(interior_mesh.mesh.placements.iter().cloned());
        let mut seen = std::collections::HashSet::new();
        for cell in &interior_mesh.mesh.grid_cells {
            for entry in &cell.indices {
                if seen.insert((entry.block_idx, entry.placement_idx)) {
                    let mut moved = entry.clone();
                    moved.block_idx += block_base;
                    moved.placement_idx += placement_base;
                    added.push(moved);
                }
            }
        }
        eprintln!("{name}: sub-area {id} (DAT {dat_id}) added {} entries", seen.len());
    }
    if let Some(cell) = merged.mesh.grid_cells.iter_mut().find(|cell| !cell.indices.is_empty()) {
        cell.indices.extend(added);
    }
    merged
}

/// Sub-area ids named by the zone's active 'M'/'m' trigger boxes (0x40-byte records in the RID chunk; a non-zero float at +0x0C drops the box).
fn trigger_box_ids(zone_data: &dats::formats::zone_data::ZoneData) -> std::collections::HashSet<u32> {
    let mut ids = std::collections::HashSet::new();
    for chunk in &zone_data.chunks {
        let dats::formats::zone_data::ChunkData::Unknown { data } = &chunk.data else { continue };
        if data.len() < 0x40 || &data[0..4] != b"RID\0" {
            continue;
        }
        let count = u32::from_le_bytes(data[0x30..0x34].try_into().unwrap()) as usize;
        for index in 0..count {
            let record = 0x40 + index * 0x40;
            if record + 0x40 > data.len() {
                break;
            }
            let first = data[record + 0x24];
            let disabled = f32::from_le_bytes(data[record + 0x0C..record + 0x10].try_into().unwrap()) != 0.0;
            if (first == b'M' || first == b'm') && !disabled {
                ids.insert(u32::from_le_bytes(data[record + 0x2C..record + 0x30].try_into().unwrap()));
            }
        }
    }
    ids
}
