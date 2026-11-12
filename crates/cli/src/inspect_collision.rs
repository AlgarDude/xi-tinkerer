use std::{collections::{HashMap, HashSet}, path::PathBuf};

use anyhow::{Result, anyhow};
use dats::{context::DatContext, formats::zone_data::zone_model::ZoneMesh, id_mapping::DatIdMapping};

/// Prints every raw field of the collision pieces with a triangle inside a world-space box (x, z ground plane, y height).
pub fn inspect_collision(ffxi_path: PathBuf, zone_id: u16, bounds: [f32; 6]) -> Result<()> {
    let [x1, z1, x2, z2, y1, y2] = bounds;
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
    let mesh = &zone_mesh.mesh;

    let mut cellInfos: HashMap<(u32, u32), Vec<u32>> = HashMap::new();
    for cell in &mesh.grid_cells {
        for entry in &cell.indices {
            cellInfos.entry((entry.block_idx, entry.placement_idx)).or_default().push(cell.info);
        }
    }
    let mut seen = HashSet::new();
    for cell in &mesh.grid_cells {
        for entry in &cell.indices {
            if !seen.insert((entry.block_idx, entry.placement_idx)) {
                continue;
            }
            let block = &mesh.blocks[entry.block_idx as usize];
            let placement = &mesh.placements[entry.placement_idx as usize];
            let rows: Vec<[f32; 3]> = placement.o2w.iter().cloned().collect();
            let world = |v: &dats::formats::zone_data::math::Vertex| {
                [
                    v.x * rows[0][0] + v.y * rows[1][0] + v.z * rows[2][0] + rows[3][0],
                    v.x * rows[0][1] + v.y * rows[1][1] + v.z * rows[2][1] + rows[3][1],
                    v.x * rows[0][2] + v.y * rows[1][2] + v.z * rows[2][2] + rows[3][2],
                ]
            };
            let rotate = |v: &dats::formats::zone_data::math::Vertex| {
                [
                    v.x * rows[0][0] + v.y * rows[1][0] + v.z * rows[2][0],
                    v.x * rows[0][1] + v.y * rows[1][1] + v.z * rows[2][1],
                    v.x * rows[0][2] + v.y * rows[1][2] + v.z * rows[2][2],
                ]
            };
            let mut header_printed = false;
            for (index, tri) in block.triangles.iter().enumerate() {
                let corners = [
                    world(&block.vertices[tri.vertex1_idx as usize]),
                    world(&block.vertices[tri.vertex2_idx as usize]),
                    world(&block.vertices[tri.vertex3_idx as usize]),
                ];
                let low = |axis: usize| corners.iter().map(|c| c[axis]).fold(f32::MAX, f32::min);
                let high = |axis: usize| corners.iter().map(|c| c[axis]).fold(f32::MIN, f32::max);
                let inside = low(0) <= x2 && high(0) >= x1 && low(2) <= z2 && high(2) >= z1 && low(1) <= y2 && high(1) >= y1;
                if !inside {
                    continue;
                }
                if !header_printed {
                    println!(
                        "block {} flags 0x{:04x} ({} tris) | placement {} data 0x{:08x} unk_floats {:?} unk_bytes {:?} unk_1 0x{:08x} unk_2 0x{:08x} coll4 0x{:x} y {:.2}..{:.2} o2w_opts {:?} cell infos {:?}",
                        entry.block_idx, block.flags, block.triangles.len(), entry.placement_idx, placement.data_field, placement.unk_floats,
                        placement.unk_bytes, placement.unk_1, placement.unk_2, placement.unk_coll4_offset, placement.min_y, placement.max_y, placement.o2w_opts,
                        cellInfos.get(&(entry.block_idx, entry.placement_idx)).map(|v| v.iter().map(|i| format!("0x{:08x}", i)).collect::<HashSet<_>>()).unwrap_or_default()
                    );
                    header_printed = true;
                }
                let stored = &block.normals[tri.normal_idx as usize];
                let normal = rotate(stored);
                let m = placement.unk_floats;
                let byRows = [m[0] * stored.x + m[1] * stored.y + m[2] * stored.z, m[3] * stored.x + m[4] * stored.y + m[5] * stored.z, m[6] * stored.x + m[7] * stored.y + m[8] * stored.z];
                let byColumns = [m[0] * stored.x + m[3] * stored.y + m[6] * stored.z, m[1] * stored.x + m[4] * stored.y + m[7] * stored.z, m[2] * stored.x + m[5] * stored.y + m[8] * stored.z];
                println!(
                    "  tri {:3} words 0x{:04x} 0x{:04x} 0x{:04x} 0x{:04x} mat {} barrier {} invalid {} normal ({:.2}, {:.2}, {:.2}) stored ({:.5}, {:.5}, {:.5}) unkRows ({:.5}, {:.5}, {:.5}) unkColumns ({:.5}, {:.5}, {:.5}) corners {:?}",
                    index,
                    tri.vertex1_idx | (((tri.material & 1) as u16) << 15),
                    tri.vertex2_idx | ((tri.is_invalid_triangle as u16) << 14) | ((((tri.material >> 1) & 1) as u16) << 15),
                    tri.vertex3_idx | ((tri.is_barrier as u16) << 14) | ((((tri.material >> 2) & 1) as u16) << 15),
                    tri.normal_idx | ((((tri.material >> 3) & 1) as u16) << 15),
                    tri.material, tri.is_barrier as u8, tri.is_invalid_triangle as u8, normal[0], normal[1], normal[2],
                    stored.x, stored.y, stored.z, byRows[0], byRows[1], byRows[2], byColumns[0], byColumns[1], byColumns[2],
                    corners.iter().map(|c| format!("({:.2}, {:.2}, {:.2})", c[0], c[1], c[2])).collect::<Vec<_>>()
                );
            }
        }
    }
    Ok(())
}
