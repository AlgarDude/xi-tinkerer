use std::{collections::HashMap, path::PathBuf};

use anyhow::{Result, anyhow};
use dats::{context::DatContext, formats::zone_data::{ZoneData, zone_model::ZoneMesh}, id_mapping::DatIdMapping};

/// World-space triangles of a zone model, optionally only those whose placement matches a filter.
fn world_triangles(mesh: &ZoneMesh, keep: impl Fn(u32) -> bool) -> Vec<(u32, [[f32; 3]; 3])> {
    let mut out = vec![];
    let mut seen = std::collections::HashSet::new();
    for cell in &mesh.mesh.grid_cells {
        for entry in &cell.indices {
            if !seen.insert((entry.block_idx, entry.placement_idx)) {
                continue;
            }
            let placement = &mesh.mesh.placements[entry.placement_idx as usize];
            if !keep(placement.unk_2) {
                continue;
            }
            let block = &mesh.mesh.blocks[entry.block_idx as usize];
            let rows: Vec<[f32; 3]> = placement.o2w.iter().cloned().collect();
            for tri in &block.triangles {
                let corners = [&block.vertices[tri.vertex1_idx as usize], &block.vertices[tri.vertex2_idx as usize], &block.vertices[tri.vertex3_idx as usize]].map(|v| {
                    [
                        v.x * rows[0][0] + v.y * rows[1][0] + v.z * rows[2][0] + rows[3][0],
                        v.x * rows[0][1] + v.y * rows[1][1] + v.z * rows[2][1] + rows[3][1],
                        v.x * rows[0][2] + v.y * rows[1][2] + v.z * rows[2][2] + rows[3][2],
                    ]
                });
                out.push((placement.unk_2, corners));
            }
        }
    }
    out
}

fn closest(p: [f32; 3], t: &[[f32; 3]; 3]) -> f32 {
    let sub = |a: [f32; 3], b: [f32; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    let dot = |a: [f32; 3], b: [f32; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let (a, b, c) = (t[0], t[1], t[2]);
    let mut best = f32::MAX;
    for i in 0..=8 {
        for j in 0..=(8 - i) {
            let (u, v) = (i as f32 / 8.0, j as f32 / 8.0);
            let q = [a[0] + (b[0] - a[0]) * u + (c[0] - a[0]) * v, a[1] + (b[1] - a[1]) * u + (c[1] - a[1]) * v, a[2] + (b[2] - a[2]) * u + (c[2] - a[2]) * v];
            let d = sub(p, q);
            best = best.min(dot(d, d).sqrt());
        }
    }
    best
}

/// Steep shell triangles (sub-area placements) with no collision of their interior model within 0.5: prints sub-area id and corners in world x, height, z.
pub fn shell_only(ffxi_path: PathBuf, zone_id: u16) -> Result<()> {
    let dat_context = DatContext::from_ffxi_path(ffxi_path)?;
    let zone_data_dat = DatIdMapping::get().zone_data.get(&zone_id).ok_or(anyhow!("No zone data for {}", zone_id))?;
    let zone_data = dat_context.get_data_from_dat(zone_data_dat)?;
    let zone_mesh = ZoneMesh::parse_from_zone_data(&zone_data.dat)?;
    let shells = world_triangles(zone_mesh, |id| id != 0);
    let mut by_id: HashMap<u32, Vec<[[f32; 3]; 3]>> = HashMap::new();
    for (id, t) in shells {
        by_id.entry(id).or_default().push(t);
    }
    for (id, tris) in by_id {
        let dat_id = if id < 600 { id + 100 } else { id + 0x144F7 };
        let Ok(interior) = dat_context.get_data_from_dat_id::<ZoneData>(dats::base::DatId::from(dat_id)) else { continue };
        let Ok(interior_mesh) = ZoneMesh::parse_from_zone_data(&interior) else { continue };
        let inner = world_triangles(interior_mesh, |_| true);
        let mut grid: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
        for (index, (_, t)) in inner.iter().enumerate() {
            let lo = |k: usize| t.iter().map(|c| c[k]).fold(f32::MAX, f32::min);
            let hi = |k: usize| t.iter().map(|c| c[k]).fold(f32::MIN, f32::max);
            for gx in ((lo(0) - 0.5).floor() as i32)..=((hi(0) + 0.5).floor() as i32) {
                for gz in ((lo(2) - 0.5).floor() as i32)..=((hi(2) + 0.5).floor() as i32) {
                    grid.entry((gx, gz)).or_default().push(index);
                }
            }
        }
        for t in &tris {
            let e1 = [t[1][0] - t[0][0], t[1][1] - t[0][1], t[1][2] - t[0][2]];
            let e2 = [t[2][0] - t[0][0], t[2][1] - t[0][1], t[2][2] - t[0][2]];
            let n = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]];
            let length = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            if length < 1e-9 || (n[1] / length).abs() > 0.708 {
                continue;
            }
            let samples = [(1.0 / 3.0, 1.0 / 3.0), (0.6, 0.2), (0.2, 0.6), (0.2, 0.2)].map(|(u, v): (f32, f32)| {
                [t[0][0] + e1[0] * u + e2[0] * v, t[0][1] + e1[1] * u + e2[1] * v, t[0][2] + e1[2] * u + e2[2] * v]
            });
            let uncovered = samples.iter().filter(|p| {
                let cell = grid.get(&(p[0].floor() as i32, p[2].floor() as i32));
                cell.map_or(true, |list| list.iter().all(|&i| closest(**p, &inner[i].1) > 0.5))
            }).count();
            if uncovered > 0 {
                println!("{}\t{}\t{:.2},{:.2},{:.2}\t{:.2},{:.2},{:.2}\t{:.2},{:.2},{:.2}", id, uncovered, t[0][0], t[0][1], t[0][2], t[1][0], t[1][1], t[1][2], t[2][0], t[2][1], t[2][2]);
            }
        }
    }
    Ok(())
}
