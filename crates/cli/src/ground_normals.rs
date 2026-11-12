use std::{collections::{HashMap, HashSet}, fs, path::PathBuf};

use anyhow::{Result, anyhow};
use dats::{context::DatContext, formats::zone_data::zone_model::ZoneMesh, id_mapping::DatIdMapping};

/// For each Windower point (x y z per line, z the height) prints the floor triangle under it: vertex normal, stored normal through o2w and through unk_floats, and unk_floats' determinant.
pub fn ground_normals(ffxi_path: PathBuf, zone_id: u16, points_path: PathBuf) -> Result<()> {
    let dat_context = DatContext::from_ffxi_path(ffxi_path)?;
    let zone_data_dat = DatIdMapping::get()
        .zone_data
        .get(&zone_id)
        .ok_or(anyhow!("No zone data for {}", zone_id))?;
    let zone_data = dat_context.get_data_from_dat(zone_data_dat)?;
    let zone_mesh = ZoneMesh::parse_from_zone_data(&zone_data.dat)?;
    let mesh = &zone_mesh.mesh;

    let mut triangles: Vec<([[f32; 3]; 3], f32, f32, f32, f32, f32, u8)> = Vec::new();
    let mut seen = HashSet::new();
    for cell in &mesh.grid_cells {
        for entry in &cell.indices {
            if !seen.insert((entry.block_idx, entry.placement_idx)) {
                continue;
            }
            let block = &mesh.blocks[entry.block_idx as usize];
            let placement = &mesh.placements[entry.placement_idx as usize];
            let rows: Vec<[f32; 3]> = placement.o2w.iter().cloned().collect();
            let m = placement.unk_floats;
            let determinant = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
            for tri in &block.triangles {
                if tri.is_invalid_triangle {
                    continue;
                }
                let corners = [&block.vertices[tri.vertex1_idx as usize], &block.vertices[tri.vertex2_idx as usize], &block.vertices[tri.vertex3_idx as usize]]
                    .map(|v| [
                        v.x * rows[0][0] + v.y * rows[1][0] + v.z * rows[2][0] + rows[3][0],
                        v.x * rows[0][1] + v.y * rows[1][1] + v.z * rows[2][1] + rows[3][1],
                        v.x * rows[0][2] + v.y * rows[1][2] + v.z * rows[2][2] + rows[3][2],
                    ]);
                let edge1 = [corners[1][0] - corners[0][0], corners[1][1] - corners[0][1], corners[1][2] - corners[0][2]];
                let edge2 = [corners[2][0] - corners[0][0], corners[2][1] - corners[0][1], corners[2][2] - corners[0][2]];
                let cross = [edge1[1] * edge2[2] - edge1[2] * edge2[1], edge1[2] * edge2[0] - edge1[0] * edge2[2], edge1[0] * edge2[1] - edge1[1] * edge2[0]];
                let length = (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt();
                if length < 1e-9 {
                    continue;
                }
                let stored = &block.normals[tri.normal_idx as usize];
                let rotatedUp = stored.x * rows[0][1] + stored.y * rows[1][1] + stored.z * rows[2][1];
                let unkUp = m[1] * stored.x + m[4] * stored.y + m[7] * stored.z;
                let unkUpRows = m[3] * stored.x + m[4] * stored.y + m[5] * stored.z;
                triangles.push((corners, (cross[1] / length).abs(), rotatedUp.abs(), unkUp.abs(), unkUpRows.abs(), determinant, tri.material));
            }
        }
    }

    let mut grid: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
    for (index, triangle) in triangles.iter().enumerate() {
        let low = |axis: usize| triangle.0.iter().map(|c| c[axis]).fold(f32::MAX, f32::min);
        let high = |axis: usize| triangle.0.iter().map(|c| c[axis]).fold(f32::MIN, f32::max);
        for gx in (low(0) / 2.0).floor() as i32..=(high(0) / 2.0).floor() as i32 {
            for gz in (low(2) / 2.0).floor() as i32..=(high(2) / 2.0).floor() as i32 {
                grid.entry((gx, gz)).or_default().push(index);
            }
        }
    }

    for line in fs::read_to_string(points_path)?.lines() {
        let values: Vec<f32> = line.split_whitespace().filter_map(|v| v.parse().ok()).collect();
        if values.len() < 3 {
            continue;
        }
        let (x, z, height) = (values[0], values[1], values[2]);
        let mut best: Option<(f32, usize)> = None;
        for &index in grid.get(&((x / 2.0).floor() as i32, (z / 2.0).floor() as i32)).map(|v| v.as_slice()).unwrap_or(&[]) {
            let [a, b, c] = triangles[index].0;
            let denominator = (b[2] - c[2]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[2] - c[2]);
            if denominator.abs() < 1e-9 {
                continue;
            }
            let u = ((b[2] - c[2]) * (x - c[0]) + (c[0] - b[0]) * (z - c[2])) / denominator;
            let v = ((c[2] - a[2]) * (x - c[0]) + (a[0] - c[0]) * (z - c[2])) / denominator;
            let w = 1.0 - u - v;
            if u < -1e-4 || v < -1e-4 || w < -1e-4 {
                continue;
            }
            let gap = (u * a[1] + v * b[1] + w * c[1] - height).abs();
            if gap < 1.0 && best.map_or(true, |(g, _)| gap < g) {
                best = Some((gap, index));
            }
        }
        match best {
            Some((gap, index)) => {
                let t = &triangles[index];
                println!("{:.2} {:.2} {:.2} gap {:.3} vertexUp {:.5} storedUp {:.5} unkUp {:.5} unkUpRows {:.5} det {:.4} mat {}", x, z, height, gap, t.1, t.2, t.3, t.4, t.5, t.6);
            }
            None => println!("{:.2} {:.2} {:.2} none", x, z, height),
        }
    }
    Ok(())
}
