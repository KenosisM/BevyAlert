mod blowfish_tables;
mod compress;
mod costgrid;
mod crowd;
mod orca;
mod orca_obstacles;
mod traffic;
mod recovery;
mod arrival;
mod formation;
mod collision;
mod debugview;
mod game;
mod hva;
mod ini;
mod lighting;
mod local_nav;
mod mapfile;
mod mat4;
mod mix;
mod nav;
mod nav_demo;
mod nav_headless;
mod nav_scenario;
mod png_out;
mod play;
mod ramp;
mod rules;
mod shp;
mod simulation;
mod spatial;
mod terrain;
mod theater;
mod tilefix;
mod tmp;
mod viewer;
mod voxel_mesh;
mod vpl;
mod vxl;
mod vxl_gpu;
mod vxl_normals;
mod world;

#[cfg(test)]
mod simulation_tests;
#[cfg(test)]
mod orca_integration_tests;
#[cfg(test)]
mod nav_properties;

use std::time::Instant;

use bevy::prelude::*;
use bevy::time::Fixed;

use ini::Ini;
use rules::{assemble, TypeTable};

const LOGIC_HZ: f64 = 15.0;
const TARGET_LOGIC_TICKS: u64 = 45;
const RUN_SECONDS: f32 = (TARGET_LOGIC_TICKS as f64 / LOGIC_HZ) as f32;
const SPEED_SCALE: f32 = 0.45;

#[derive(Resource)]
struct GameTypes(TypeTable);

#[derive(Resource, Default)]
struct Clock {
    logic_ticks: u64,
    frames: u64,
}

#[derive(Resource)]
struct Wall(Instant);

#[derive(Component)]
struct Unit {
    type_name: String,
    speed: f32,
}

#[derive(Component)]
struct CellPos {
    x: f32,
    y: f32,
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.get(1).map(|s| s.as_str()) == Some("nav-demo") {
        if let Err(message) = nav_demo::run(&args[2..]) {
            eprintln!("{message}");
            std::process::exit(2);
        }
        return;
    }
    if args.get(1).map(|s| s.as_str()) == Some("nav-headless") {
        if let Err(message) = nav_headless::run(&args[2..]) {
            eprintln!("{message}");
            std::process::exit(2);
        }
        return;
    }
    if args.get(1).map(|s| s.as_str()) == Some("view") {
        viewer::run(args[2..].to_vec());
        return;
    }
    if args.get(1).map(|s| s.as_str()) == Some("vxl") {
        for p in &args[2..] {
            probe_vxl(p);
        }
        return;
    }
    if args.get(1).map(|s| s.as_str()) == Some("vgpu") {
        cmd_vgpu(&args[2..]);
        return;
    }
    if args.get(1).map(|s| s.as_str()) == Some("map") {
        for p in &args[2..] {
            probe_map(p);
        }
        return;
    }
    if args.get(1).map(|s| s.as_str()) == Some("mapimg") {
        cmd_mapimg(&args[2..]);
        return;
    }
    if args.get(1).map(|s| s.as_str()) == Some("mixprobe") {
        cmd_mixprobe(&args[2..]);
        return;
    }
    if args.get(1).map(|s| s.as_str()) == Some("dump") {
        cmd_dump(&args[2..]);
        return;
    }
    if args.get(1).map(|s| s.as_str()) == Some("tmpimg") {
        cmd_tmpimg(&args[2..]);
        return;
    }
    if args.get(1).map(|s| s.as_str()) == Some("costgrid") {
        cmd_costgrid(&args[2..]);
        return;
    }
    if args.get(1).map(|s| s.as_str()) == Some("game") {
        game::run(args[2..].to_vec());
        return;
    }
    let rules_path = std::env::args().nth(1).unwrap_or_else(|| "../../rulesmd.ini".into());
    let art_path = std::env::args().nth(2).unwrap_or_else(|| "../../artmd.ini".into());

    let rules_text = std::fs::read_to_string(&rules_path).expect("cannot read rulesmd.ini");
    let art_text = std::fs::read_to_string(&art_path).expect("cannot read artmd.ini");
    let rules = Ini::parse(&rules_text);
    let art = Ini::parse(&art_text);
    let table = assemble(&rules, &art);

    println!("rules sections   = {}", rules.len());
    println!("art sections     = {}", art.len());
    println!("assembled types  = {}", table.types.len());
    for (section, kind) in rules::REGISTRIES {
        println!("  {:<16} {}", section, table.count_of_kind(kind));
    }
    if !table.unresolved.is_empty() {
        println!("registry names with no section = {}", table.unresolved.len());
    }

    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.insert_resource(Time::<Fixed>::from_hz(LOGIC_HZ));
    app.insert_resource(GameTypes(table));
    app.insert_resource(Clock::default());
    app.insert_resource(Wall(Instant::now()));
    app.add_systems(Startup, spawn_from_types);
    app.add_systems(FixedUpdate, logic_tick);
    app.add_systems(Update, (count_frame, finish_when_done));
    app.run();
}

fn spawn_from_types(mut commands: Commands, types: Res<GameTypes>) {    let mut y = 0.0f32;
    for def in types.0.of_kind("vehicle").into_iter().take(5) {
        let speed = def.speed() as f32 * SPEED_SCALE;
        commands.spawn((
            Unit { type_name: def.name.clone(), speed },
            CellPos { x: 0.0, y },
        ));
        println!(
            "  {:<8} image={:<8} hp={:<5.0} cost={:<5.0} speed={:<4.0} voxel={:<5} -> {:.2} cells/s",
            def.name,
            def.image,
            def.strength(),
            def.cost(),
            def.speed(),
            def.is_voxel(),
            speed
        );
        y += 2.0;
    }
}

fn logic_tick(time: Res<Time<Fixed>>, mut clock: ResMut<Clock>, mut q: Query<(&Unit, &mut CellPos)>) {
    clock.logic_ticks += 1;
    let dt = time.delta_secs();
    for (unit, mut pos) in &mut q {
        pos.x += unit.speed * dt;
    }
}

fn count_frame(mut clock: ResMut<Clock>) {
    clock.frames += 1;
}

fn finish_when_done(
    clock: Res<Clock>,
    wall: Res<Wall>,
    q: Query<(&Unit, &CellPos)>,
    mut exit: MessageWriter<AppExit>,
) {
    if clock.logic_ticks < TARGET_LOGIC_TICKS {
        return;
    }
    println!("---");
    println!("logic ticks   = {}  ({} Hz for {:.1}s of sim)", clock.logic_ticks, LOGIC_HZ as u64, RUN_SECONDS);
    println!("render frames = {}", clock.frames);
    println!("wall seconds  = {:.3}", wall.0.elapsed().as_secs_f32());
    println!("---");
    for (unit, pos) in &q {
        println!(
            "  {:<8} speed={:<5.2} cells/s  moved {:.3} cells (expected {:.3})",
            unit.type_name,
            unit.speed,
            pos.x,
            unit.speed * RUN_SECONDS
        );
    }
    exit.write(AppExit::Success);
}

fn probe_vxl(path: &str) {
    let Ok(bytes) = std::fs::read(path) else {
        println!("{path}: cannot read");
        return;
    };
    let Some(v) = vxl::parse_vxl(&bytes) else {
        println!("{path}: parse failed ({} bytes)", bytes.len());
        return;
    };
    println!(
        "{:<34} bytes={:<7} sections={} palette={}",
        path.rsplit(['/', '\\']).next().unwrap_or(path),
        bytes.len(),
        v.sections.len(),
        v.palette.len()
    );
    if v.palette.len() >= 9 {
        let c: Vec<String> = (0..3)
            .map(|i| format!("({},{},{})", v.palette[i * 3], v.palette[i * 3 + 1], v.palette[i * 3 + 2]))
            .collect();
        println!("    palette[0..3] = {}", c.join(" "));
    }
    for s in &v.sections {
        println!(
            "    sec {:<10} num={:<4} size={}x{}x{} normals={} hva={:.4} voxels={} bad={}",
            s.name,
            s.number,
            s.x_size,
            s.y_size,
            s.z_size,
            s.normals_type,
            s.hva_multiplier,
            s.voxels.len(),
            s.bad
        );
        if s.voxels.is_empty() {
            continue;
        }
        let (mut cmax, mut cmin) = (0u8, 255u8);
        let (mut nmin, mut nmax) = (255u8, 0u8);
        let (mut zmax, mut xmax, mut ymax) = (0u8, 0u8, 0u8);
        for vx in &s.voxels {
            cmax = cmax.max(vx.color);
            cmin = cmin.min(vx.color);
            nmin = nmin.min(vx.normal);
            nmax = nmax.max(vx.normal);
            xmax = xmax.max(vx.x);
            ymax = ymax.max(vx.y);
            zmax = zmax.max(vx.z);
        }
        let v0 = s.voxels[0];
        println!(
            "      first=({},{},{}) c={} n={} | color {}..{} normal {}..{} | extents {}x{}x{}",
            v0.x, v0.y, v0.z, v0.color, v0.normal, cmin, cmax, nmin, nmax,
            xmax + 1, ymax + 1, zmax + 1
        );
    }
}

/// `mapimg <in.map> <out.png>`
///
/// 把解析结果画成等距俯视图，用来肉眼核对。地形砖按高度层 `z` 上色，覆盖物/地形对象/建筑/单位
/// 各叠一种亮点。几何用的是上游那对换算的逆解：
///   `rx = (dx+dy)/2 + 1`、`ry = (dy-dx)/2 + width`，再投影 `sx=(rx-ry)*2`、`sy=(rx+ry)`。
/// 于是 `sx = (dx+1-width)*2`、`sy = dy+1+width` —— 换算写错的话，图会立刻缝不上或撕开。
fn cmd_mapimg(args: &[String]) {
    if args.len() < 2 {
        println!("usage: ra2engine mapimg <in.map> <out.png>");
        return;
    }
    let bytes = match std::fs::read(&args[0]) {
        Ok(b) => b,
        Err(e) => {
            println!("{}: {e}", args[0]);
            return;
        }
    };
    let map = match mapfile::parse_map(&bytes) {
        Ok(m) => m,
        Err(e) => {
            println!("{}: {e}", args[0]);
            return;
        }
    };
    let w = map.width;
    let h = map.height;
    let iw = (4 * w + 8) as u32;
    let ih = (2 * h + w + 8) as u32;
    let mut rgba = vec![0u8; (iw * ih * 4) as usize];
    for p in rgba.chunks_mut(4) {
        p[3] = 255;
    }

    // 高度层调色：低处深绿，高处亮黄褐，一眼能看出台地。
    const ZCOL: [[u8; 3]; 13] = [
        [26, 38, 26],
        [34, 52, 30],
        [40, 64, 34],
        [46, 74, 38],
        [52, 84, 42],
        [58, 92, 46],
        [66, 98, 50],
        [76, 104, 54],
        [88, 110, 58],
        [100, 114, 62],
        [112, 118, 66],
        [122, 122, 72],
        [134, 126, 80],
    ];

    let sx_of = |dx: i32| (dx + 1 - w) * 2 + 2 * w;
    let sy_of = |dy: i32| dy + 1 + w;

    for t in &map.tiles {
        let sx = sx_of(t.dx);
        let sy = sy_of(t.dy);
        let c = if t.empty {
            [18, 18, 20]
        } else {
            ZCOL[(t.z as usize).min(ZCOL.len() - 1)]
        };
        fill_rect(&mut rgba, iw, ih, sx, sy, 4, 2, c);
    }

    // 亮点叠上去：覆盖物红、地形对象蓝、建筑白、单位黄。
    for o in &map.overlays {
        fill_rect(&mut rgba, iw, ih, sx_of(o.dx), sy_of(o.dy), 2, 2, [200, 40, 40]);
    }
    for t in &map.terrains {
        let Some(cell) = map.tile_rc(t.rx, t.ry) else { continue };
        fill_rect(&mut rgba, iw, ih, sx_of(cell.dx), sy_of(cell.dy), 2, 2, [40, 90, 220]);
    }
    for m in &map.structures {
        let Some(cell) = map.tile_rc(m.rx, m.ry) else { continue };
        fill_rect(&mut rgba, iw, ih, sx_of(cell.dx), sy_of(cell.dy), 4, 4, [245, 245, 245]);
    }
    for m in &map.units {
        let Some(cell) = map.tile_rc(m.rx, m.ry) else { continue };
        fill_rect(&mut rgba, iw, ih, sx_of(cell.dx), sy_of(cell.dy), 3, 3, [250, 210, 40]);
    }
    for m in &map.infantries {
        let Some(cell) = map.tile_rc(m.rx, m.ry) else { continue };
        fill_rect(&mut rgba, iw, ih, sx_of(cell.dx), sy_of(cell.dy), 2, 2, [180, 250, 60]);
    }

    match png_out::write_rgba(&args[1], iw, ih, &rgba) {
        Ok(()) => println!("wrote {} ({}x{})", args[1], iw, ih),
        Err(e) => println!("write {}: {e}", args[1]),
    }
}

/// `mixprobe <gameDir> <name> [more names ...]`
///
/// 建一次 AssetIndex，然后按名字取文件并打印大小与前几个字节。用来确认 MIX 读取层通了。
/// `dump <gameDir> <name> [outPath]`
///
/// 把 MIX 里的一个文件原样导出。对照参考实现时天天要用（theater ini、TMP、SHP 都在
/// MIX 里，不进文件系统就没法用文本工具看）。
fn cmd_dump(args: &[String]) {
    if args.len() < 2 {
        println!("usage: ra2engine dump <gameDir> <name> [outPath]");
        return;
    }
    let mut idx = mix::AssetIndex::new(&args[0]);
    if let Err(e) = idx.build("", false) {
        println!("index: {e}");
        return;
    }
    let bytes = match idx.read(&args[1]) {
        Ok(b) => b,
        Err(e) => {
            println!("{}: {e}", args[1]);
            return;
        }
    };
    match args.get(2) {
        Some(out) => match std::fs::write(out, &bytes) {
            Ok(()) => println!("{} -> {} ({} bytes)", args[1], out, bytes.len()),
            Err(e) => println!("write {out}: {e}"),
        },
        None => print!("{}", String::from_utf8_lossy(&bytes)),
    }
}

fn cmd_mixprobe(args: &[String]) {
    if args.len() < 2 {
        println!("usage: ra2engine mixprobe <gameDir> <name> [more names ...]");
        return;
    }
    let t0 = std::time::Instant::now();
    let mut idx = mix::AssetIndex::new(&args[0]);
    match idx.build("", false) {
        Ok(()) => println!(
            "index: {} sources, {} assets, {} names ({:.1}s)",
            idx.source_count(),
            idx.asset_count(),
            idx.name_count(),
            t0.elapsed().as_secs_f32()
        ),
        Err(e) => {
            println!("build failed: {e}");
            return;
        }
    }
    for s in idx.skipped().iter().take(6) {
        println!("  skipped: {s}");
    }
    for name in &args[1..] {
        match idx.read(name) {
            Ok(b) => {
                let head: Vec<String> = b.iter().take(8).map(|x| format!("{x:02X}")).collect();
                println!("{:<30} {:>9} bytes  first8={}", name, b.len(), head.join(" "));
            }
            Err(e) => println!("{:<30} {e}", name),
        }
    }
}

/// `costgrid <in.map> <gameDir> [--zone Track] [--no-fixtiles]`
///
/// 走完整条"地图 → 可寻路网格"的链：MIX → theater ini → tile numbering → 每个 tileset 的
/// TMP（取 terrainType）→ LAT/ramp 修正 → cost grid。输出可与 C++ 侧对照的统计。
fn cmd_costgrid(args: &[String]) {
    if args.len() < 2 {
        println!("usage: ra2engine costgrid <in.map> <gameDir> [--zone Track] [--no-fixtiles]");
        return;
    }
    let map_path = &args[0];
    let game_dir = &args[1];
    let mut zone = "Track".to_string();
    let mut no_fix = false;
    let mut png: Option<String> = None;
    let mut probe: Option<String> = None;
    let mut i = 2usize;
    while i < args.len() {
        match args[i].as_str() {
            "--cell" => {
                i += 1;
                probe = args.get(i).cloned();
            }
            "--zone" => {
                i += 1;
                if let Some(z) = args.get(i) {
                    zone = z.clone();
                }
            }
            "--no-fixtiles" => no_fix = true,
            "--png" => {
                i += 1;
                png = args.get(i).cloned();
            }
            other => println!("unknown arg {other}"),
        }
        i += 1;
    }

    let bytes = match std::fs::read(map_path) {
        Ok(b) => b,
        Err(e) => {
            println!("{map_path}: {e}");
            return;
        }
    };
    let map = match mapfile::parse_map(&bytes) {
        Ok(m) => m,
        Err(e) => {
            println!("{map_path}: {e}");
            return;
        }
    };
    let Some(mut th) = theater::theater_info(&map.theater) else {
        println!("unknown theater '{}' (expected {})", map.theater, theater::theater_names());
        return;
    };
    println!("map         {map_path}  {}x{} theater={} ({})", map.width, map.height, th.name, th.ini_name);

    let t0 = std::time::Instant::now();
    let mut idx = mix::AssetIndex::new(game_dir);
    if let Err(e) = idx.build("", false) {
        println!("index failed: {e}");
        return;
    }
    println!(
        "index       {} sources, {} assets ({:.1}s)",
        idx.source_count(),
        idx.asset_count(),
        t0.elapsed().as_secs_f32()
    );

    let th_ini = match idx.read(&th.ini_name) {
        Ok(b) => Ini::parse(&String::from_utf8_lossy(&b)),
        Err(e) => {
            println!("cannot read {}: {e}", th.ini_name);
            return;
        }
    };
    if let Err(e) = theater::parse_theater(&th_ini, &mut th) {
        println!("theater parse: {e}");
        return;
    }
    let numbering = theater::build_tile_numbering(&th);
    println!(
        "theater     {} tilesets ({} enabled, {} disabled), {} global tiles",
        th.tile_sets.len(),
        th.enabled_count,
        th.disabled_count,
        numbering.tile_to_set.len()
    );
    println!(
        "fixup ids   rough={} sand={} green={} pave={} blueMold={} crystal={} swamp={} ramp={}/{}",
        th.fixup.rough_tile,
        th.fixup.sand_tile,
        th.fixup.green_tile,
        th.fixup.pave_tile,
        th.fixup.blue_mold_tile,
        th.fixup.crystal_tile,
        th.fixup.swamp_tile,
        th.fixup.ramp_base,
        th.fixup.ramp_smooth
    );

    let t1 = std::time::Instant::now();
    let sets = costgrid::build_set_images(&idx, &th);
    println!(
        "tilesets    {} loaded from MIX ({:.1}s)",
        sets.len(),
        t1.elapsed().as_secs_f32()
    );

    let tb = costgrid::build_terrain_cells(&map, &th, &numbering, &sets, no_fix);
    println!(
        "terrain     {} cells, {} missing, {} blank, {} touched [gap={} noVar={} size={}]",
        tb.cells.len(),
        tb.missing,
        tb.blank,
        tb.touched.iter().filter(|&&t| t != 0).count(),
        tb.miss_gap,
        tb.miss_no_var,
        tb.miss_size
    );

    let rules_ini = costgrid::read_rules_ini(&idx, &["rulesmd.ini", "rules.ini"]);
    let art_ini = costgrid::read_rules_ini(&idx, &["artmd.ini", "art.ini"]);
    println!(
        "inis        rules sections={} art sections={}",
        rules_ini.sections.len(),
        art_ini.sections.len()
    );

    let (grid, stats) = costgrid::build_cost_grid(&map, &rules_ini, &art_ini, &tb, &zone);
    println!(
        "costgrid    {}x{} cells, zone={} blocked={} plain={}",
        grid.w, grid.h, zone, stats.blocked, stats.plain
    );
    println!(
        "  breakdown terrain-blocked={}  building-covered={}  building-cleared={}",
        stats.terrain_blocked, stats.building_cells, stats.building_cleared
    );
    let mut fp_hist: Vec<((i32, i32), usize)> =
        stats.building_fp_hist.iter().map(|(k, v)| (*k, *v)).collect();
    fp_hist.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
    let fp_txt: Vec<String> = fp_hist.iter().map(|((w, h), n)| format!("{w}x{h}×{n}")).collect();
    println!("  foundation  {}", fp_txt.join(" "));

    let mut hist = [0usize; 8];
    for &c in &grid.cost {
        let b = match c {
            0 => 0,
            1 => 1,
            2 => 2,
            3 => 3,
            4..=7 => 4,
            8..=31 => 5,
            32..=127 => 6,
            _ => 7,
        };
        hist[b] += 1;
    }
    println!(
        "cost hist   0={} 1={} 2={} 3={} 4-7={} 8-31={} 32-127={} 128+={}",
        hist[0], hist[1], hist[2], hist[3], hist[4], hist[5], hist[6], hist[7]
    );

    // 采样：换算是错的话，这几格的 rx/ry 与地形类型会立刻不对。
    for c in tb.cells.iter().take(5) {
        println!(
            "  cell dx={} dy={} rx={} ry={} z={} ramp={} terrain={} bridge={}",
            c.dx, c.dy, c.rx, c.ry, c.z, c.ramp_type, c.terrain_type, c.bridge
        );
    }
    // 诊断：砖自带高度（TMP 头 h 字段）+ extra 形状 + 地图 z 的交叉分布。
    // 只读，不参与任何逻辑；用来找出「砖自己声明了什么高度」。
    {
        let mut m: std::collections::BTreeMap<(i32, i32, i32), usize> = Default::default();
        for c in &tb.cells {
            *m.entry((c.cell_x, c.cell_y, c.level)).or_insert(0) += 1;
        }
        for ((cx, cy, lv), n) in m {
            println!("  cellX={cx} cellY={cy} level={lv} cells={n}");
        }
    }

    let terrain_hist = {
        let mut h = [0usize; 16];
        for c in &tb.cells {
            h[(c.terrain_type & 15) as usize] += 1;
        }
        h
    };
    let tl: Vec<(usize, usize)> = (0..16)
        .filter(|&i| terrain_hist[i] > 0)
        .map(|i| (i, terrain_hist[i]))
        .collect();
    println!("terrain tt  {tl:?}");

    // `--cell rx,ry[;rx,ry...]`：打印这些格子在 [IsoMapPack5] 里的原始 11 字节定义，
    // 以及解析出来的砖集名 / set 内序号 / ramp / TMP height。
    if let Some(spec) = probe {
        for part in spec.split(';') {
            let mut it = part.split(',');
            let rx: i32 = it.next().unwrap_or("").trim().parse().unwrap_or(-1);
            let ry: i32 = it.next().unwrap_or("").trim().parse().unwrap_or(-1);
            let Some(t) = map.tile_rc(rx, ry) else {
                println!("  probe {rx},{ry}: no tile in IsoMapPack5");
                continue;
            };
            let k = t.dx as usize * map.height as usize + (t.dy / 2) as usize;
            let tn = tb.tile_num.get(k).copied().unwrap_or(-1);
            let set_idx = numbering.tile_to_set.get(tn.max(0) as usize).copied().unwrap_or(-1);
            let set_name = if set_idx >= 0 {
                th.tile_sets[set_idx as usize].file_name.clone()
            } else {
                "?".to_string()
            };
            let first = numbering
                .set_first_tile
                .get(set_idx.max(0) as usize)
                .copied()
                .unwrap_or(0);
            // 探针字段与游戏内 `debugview::console_line` 保持同一口径（cell 序号、cell 在砖内的
            // 偏移、extra 相对 cell 的偏移 + 换算到砖内的位置）。**这是无头验算的入口** ——
            // 查格子数据不要开窗口，跑 `costgrid --cell "x,y;x,y"` 直接读 stdout。
            let cell_info = sets.get(&set_idx).and_then(|si| {
                let ti = si.tiles.get((tn - first).max(0) as usize)?;
                let mut cell = t.sub_tile as usize;
                let f = *ti.variant_cells.first()?;
                if cell >= f && f > 0 {
                    cell = f - 1;
                }
                Some((ti, cell))
            });
            let info = match cell_info {
                Some((ti, cell)) => {
                    let g = |v: &Vec<i32>| v.get(cell).copied().unwrap_or(0);
                    let (cx, cy) = (g(&ti.xs), g(&ti.ys));
                    let (ex, ey, ew, eh) = (g(&ti.ex_x), g(&ti.ex_y), g(&ti.ex_w), g(&ti.ex_h));
                    // extra 相对 cell，砖内位置 = cell + extra（和 debugview::extra_in_tile 同一算法）
                    let ext = if ew > 0 && eh > 0 {
                        format!(
                            "extY {ew}x{eh}@cell({ex},{ey})->tile({},{})",
                            cx + ex,
                            cy + ey
                        )
                    } else {
                        "extN".to_string()
                    };
                    format!("cell{cell} cellXY({cx},{cy}) tile{}x{} {ext}", ti.w, ti.h)
                }
                None => "cell=? (not found)".to_string(),
            };
            println!(
                "  probe rx={rx} ry={ry} | raw tile_num={} sub={} z={} ice={} | fixed tile_num={tn} set={set_name}[{set_idx}] in_set={} | {info} | ramp={} tmph={}",
                t.tile_num,
                t.sub_tile,
                t.z,
                t.ice_growth,
                tn - first,
                tb.ramp_at.get(k).copied().unwrap_or(0),
                tb.height_at.get(k).copied().unwrap_or(0)
            );
        }
    }

    // `--png out.png`：把 cost grid 画成等距俯视图（不可通行近黑 / 全速绿 / 慢速黄红），
    // 投影与 `mapimg` 同一套（`sx ∝ rx-ry`、`sy ∝ rx+ry`，比例 2:1）。
    if let Some(path) = png {
        let w = grid.w;
        let h = grid.h;
        let iw = (4 * w + 8) as u32;
        let ih = (2 * h + 4 * w + 8) as u32;
        let mut rgba = vec![255u8; (iw * ih * 4) as usize];
        for p in rgba.chunks_mut(4) {
            p[3] = 255;
        }
        let sx_of = |rx: i32, ry: i32| (rx - ry) * 2 + 2 * w;
        let sy_of = |rx: i32, ry: i32| rx + ry;
        for ry in 0..h {
            for rx in 0..w {
                let c = grid.cost[ry as usize * w as usize + rx as usize];
                let col = match c {
                    0 => [16, 16, 16],
                    1 => [60, 170, 60],
                    2 => [150, 200, 60],
                    3..=7 => [220, 190, 60],
                    _ => [210, 110, 50],
                };
                fill_rect(&mut rgba, iw, ih, sx_of(rx, ry), sy_of(rx, ry), 4, 2, col);
            }
        }
        match png_out::write_rgba(&path, iw, ih, &rgba) {
            Ok(()) => println!("wrote {path} ({iw}x{ih})"),
            Err(e) => println!("write {path}: {e}"),
        }
    }
}

/// `tmpimg <gameDir> <palette.pal> <out.png> <name> [more names ...]`
///
/// 把 TMP 里每个 tile **照 TmpRenderer.Draw 的 2D 摆法**画出来：主菱形在 (x, y)、
/// extra 矩形在 (ExtraX − X, ExtraY − Y)（单位都是像素），按调色板转成 RGB 输出。
/// 目的：直接看一块砖的 extra 到底是什么形状 —— 不用再靠推。
fn cmd_tmpimg(args: &[String]) {
    if args.len() < 4 {
        println!("usage: ra2engine tmpimg <gameDir> <palette.pal> <out.png> <name.tem> [more ...]");
        return;
    }
    let game_dir = &args[0];
    let pal_name = &args[1];
    let out = &args[2];
    let names = &args[3..];

    let mut idx = mix::AssetIndex::new(game_dir);
    if let Err(e) = idx.build("", false) {
        println!("index: {e}");
        return;
    }
    let pal_raw = match idx.read(pal_name) {
        Ok(b) => b,
        Err(e) => {
            println!("{pal_name}: {e}");
            return;
        }
    };
    // 原始调色板（不做光照）：TMP 的像素索引直接查它
    let pal = lighting::recalculate_palette(&pal_raw, lighting::PaletteMults::default(), false);

    const CW: i32 = 176;
    const CH: i32 = 176;
    const COLS: i32 = 8;
    const OX: i32 = 44;
    const OY: i32 = 96;

    let mut canvases: Vec<Vec<u8>> = Vec::new();
    for n in names {
        let bytes = match idx.read(n) {
            Ok(b) => b,
            Err(e) => {
                println!("{n}: {e}");
                continue;
            }
        };
        let Some(t) = tmp::parse_tmp(&bytes) else {
            println!("{n}: parse failed");
            continue;
        };
        println!(
            "{n}: cblocks={}x{} cells={}x{} tiles={}",
            t.cblocks_x,
            t.cblocks_y,
            t.cx,
            t.cy,
            t.tiles.len()
        );
        for (i, c) in t.tiles.iter().enumerate() {
            // ExtraZData 的取值分布：参考实现里 extra 的深度是 `zBase - xzData[rIdx]`，
            // 即逐像素不同。这里把它打出来，看它到底是"几个离散层"还是连续值。
            let mut hist: std::collections::BTreeMap<u8, usize> = std::collections::BTreeMap::new();
            for v in &c.extra_z {
                *hist.entry(*v).or_insert(0) += 1;
            }
            let zt: Vec<String> = hist.iter().take(10).map(|(k, n)| format!("{k}×{n}")).collect();
            println!(
                "  [{i}] xy=({},{}) extra=({},{}) exWH={}x{} pix={} extraPix={} extraZ={} hasX={} ramp={} h={} tt={}",
                c.x,
                c.y,
                c.x_extra,
                c.y_extra,
                c.cx_extra,
                c.cy_extra,
                c.pixels.len(),
                c.extra.len(),
                c.extra_z.len(),
                c.has_extra_data,
                c.ramp_type,
                c.height,
                c.terrain_type
            );
            if !c.extra_z.is_empty() {
                println!("        extraZ hist: {}", zt.join(" "));
                // 逐行打印前几列，确认它是"每行一个值"（线性）还是随 x 也变化。
                let cols = c.cx_extra.min(8) as usize;
                for y in 0..c.cy_extra.min(4) as usize {
                    let row: Vec<String> = (0..cols)
                        .map(|x| {
                            let i = y * c.cx_extra as usize + x;
                            c.extra_z.get(i).copied().unwrap_or(0).to_string()
                        })
                        .collect();
                    println!("          extraZ y={y}: {}", row.join(" "));
                }
            }
            let mut buf = vec![0u8; (CW * CH * 4) as usize];
            for p in buf.chunks_mut(4) {
                p[0] = 20;
                p[1] = 20;
                p[2] = 24;
                p[3] = 255;
            }
            // 主菱形：`decode_diamond` 已经把 900 字节的行序展开成 `cx×cy` 的矩形布局
            // （菱形之外是 0），所以这里按矩形布局遍历菱形区域 —— 取 `pixels[y*cx + x]`，
            // 而不是拿行序去顺序索引（那样取到的是别的行、图案会错乱）。
            let half = t.cy / 2;
            let half_cx = t.cx / 2;
            for y in 0..t.cy {
                let (x0, w) = if y < half {
                    (half_cx - 2 - 2 * y, 4 + 4 * y)
                } else {
                    let j = y - half;
                    (2 + 2 * j, (4 * half - 4 - 4 * j).max(0))
                };
                for k in 0..w {
                    let sx2 = x0 + k;
                    if sx2 < 0 || sx2 >= t.cx {
                        continue;
                    }
                    let v = c
                        .pixels
                        .get((y * t.cx + sx2) as usize)
                        .copied()
                        .unwrap_or(0);
                    if v != 0 {
                        tmpimg_px(&mut buf, CW, CH, OX + sx2, OY + y, &pal, v);
                    }
                }
            }
            // extra：位置 = 主菱形位置 + (ExtraX − X, ExtraY − Y)（TmpRenderer.cs:136-137）
            let ex = OX + c.x_extra - c.x;
            let ey = OY + c.y_extra - c.y;
            for y in 0..c.cy_extra {
                for x in 0..c.cx_extra {
                    let i2 = (y * c.cx_extra + x) as usize;
                    let v = c.extra.get(i2).copied().unwrap_or(0);
                    if v != 0 {
                        tmpimg_px(&mut buf, CW, CH, ex + x, ey + y, &pal, v);
                    }
                }
            }
            canvases.push(buf);
        }
    }
    if canvases.is_empty() {
        println!("nothing to draw");
        return;
    }
    let n = canvases.len() as i32;
    let rows = (n + COLS - 1) / COLS;
    let (aw, ah) = (COLS * CW, rows * CH);
    let mut rgba = vec![0u8; (aw * ah * 4) as usize];
    for p in rgba.chunks_mut(4) {
        p[0] = 10;
        p[1] = 10;
        p[2] = 12;
        p[3] = 255;
    }
    for (i, c) in canvases.iter().enumerate() {
        let (ox, oy) = ((i as i32 % COLS) * CW, (i as i32 / COLS) * CH);
        for y in 0..CH {
            for x in 0..CW {
                let s = ((y * CW + x) * 4) as usize;
                let d = (((oy + y) * aw + ox + x) * 4) as usize;
                rgba[d..d + 4].copy_from_slice(&c[s..s + 4]);
            }
        }
    }
    match png_out::write_rgba(out, aw as u32, ah as u32, &rgba) {
        Ok(()) => println!("wrote {out} ({aw}x{ah}), {n} tiles"),
        Err(e) => println!("write {out}: {e}"),
    }
}

fn tmpimg_px(buf: &mut [u8], w: i32, h: i32, x: i32, y: i32, pal: &[u8], idx: u8) {
    if x < 0 || y < 0 || x >= w || y >= h {
        return;
    }
    let k = (idx as usize) * 3;
    if k + 2 >= pal.len() {
        return;
    }
    let i = ((y * w + x) * 4) as usize;
    buf[i] = pal[k];
    buf[i + 1] = pal[k + 1];
    buf[i + 2] = pal[k + 2];
    buf[i + 3] = 255;
}

fn put_px(rgba: &mut [u8], iw: u32, ih: u32, x: i32, y: i32, c: [u8; 3]) {
    if x < 0 || y < 0 || x as u32 >= iw || y as u32 >= ih {
        return;
    }
    let i = ((y as u32 * iw + x as u32) * 4) as usize;
    rgba[i] = c[0];
    rgba[i + 1] = c[1];
    rgba[i + 2] = c[2];
    rgba[i + 3] = 255;
}

fn fill_rect(rgba: &mut [u8], iw: u32, ih: u32, x: i32, y: i32, rw: i32, rh: i32, c: [u8; 3]) {
    for dy in 0..rh {
        for dx in 0..rw {
            put_px(rgba, iw, ih, x + dx, y + dy, c);
        }
    }
}

/// `map <file.map> [more.map ...]`
///
/// 只解析不渲染，输出可与 C++ 参考实现（`wwread map`）逐行对照的统计。
fn probe_map(path: &str) {
    let bytes = match std::fs::read(path) {
        Ok(b) => b,
        Err(e) => {
            println!("{path}: {e}");
            return;
        }
    };
    let map = match mapfile::parse_map(&bytes) {
        Ok(m) => m,
        Err(e) => {
            println!("{path}: {e}");
            return;
        }
    };
    let cols = 2 * map.width - 1;
    let cells = cols as usize * map.height as usize;
    let empty = map.tiles.iter().filter(|t| t.empty).count();
    println!("map         {path}  ({} bytes)", bytes.len());
    println!("size        {}x{} cells", map.width, map.height);
    println!("theater     {}", map.theater);
    println!("newTheater  {}", map.new_theater);
    println!(
        "localSize   {},{},{},{}",
        map.local_x, map.local_y, map.local_width, map.local_height
    );
    println!(
        "tiles       {} placed, {} out of bounds, maxZ={}",
        map.present_tiles, map.out_of_bounds, map.max_z
    );
    println!("grid        {cols} cols x {} rows = {cells} cells, empty={empty}", map.height);
    println!("overlays    {}", map.overlays.len());
    println!("terrain     {}", map.terrains.len());
    println!("structures  {}", map.structures.len());
    println!("units       {}", map.units.len());
    println!("infantry    {}", map.infantries.len());
    println!("aircraft    {}", map.aircrafts.len());
    let l = &map.lighting;
    println!(
        "lighting    ambient={:.3} rgb={:.3}/{:.3}/{:.3} ground={:.3} level={:.3}",
        l.ambient, l.red, l.green, l.blue, l.ground, l.level
    );

    // 采样：逐格坐标是"换算公式写错"最容易露馅的地方，所以把边界上的几格也摆出来。
    let mut shown = 0;
    for (i, t) in map.tiles.iter().enumerate() {
        if t.empty {
            continue;
        }
        if i % 37 == 0 && shown < 6 {
            println!(
                "  tile[{}] rx={} ry={} dx={} dy={} num={} sub={} z={}",
                i, t.rx, t.ry, t.dx, t.dy, t.tile_num, t.sub_tile, t.z
            );
            shown += 1;
        }
    }
    for o in map.overlays.iter().take(4) {
        println!("  overlay rx={} ry={} dx={} dy={} z={} id={} data={}", o.rx, o.ry, o.dx, o.dy, o.z, o.id, o.data);
    }
    for t in map.terrains.iter().take(4) {
        println!("  terrain {} ({},{})", t.name, t.rx, t.ry);
    }
    for m in map.structures.iter().take(3) {
        println!(
            "  structure {} {} hp={} ({},{}) facing={} bridge={}",
            m.owner, m.name, m.health, m.rx, m.ry, m.facing, m.on_bridge
        );
    }
    for m in map.units.iter().take(3) {
        println!("  unit {} {} ({},{}) facing={} bridge={}", m.owner, m.name, m.rx, m.ry, m.facing, m.on_bridge);
    }
    for m in map.infantries.iter().take(3) {
        println!(
            "  infantry {} {} ({},{}) sub={} facing={} bridge={}",
            m.owner, m.name, m.rx, m.ry, m.sub_cell, m.facing, m.on_bridge
        );
    }
    // 反查：tileRC 与 gridAt 必须互为逆（上游 mapfile.cpp:74-79）。
    let t0 = map.tiles.iter().find(|t| !t.empty).copied();
    if let Some(t0) = t0 {
        let back = map.tile_rc(t0.rx, t0.ry);
        println!(
            "  tileRC({},{}) -> {}",
            t0.rx,
            t0.ry,
            match back {
                Some(t) => format!("rx={} ry={} num={} z={}", t.rx, t.ry, t.tile_num, t.z),
                None => "None".to_string(),
            }
        );
    }
    let nulls = map.tiles.iter().filter(|t| map.tile_rc(t.rx, t.ry).is_none()).count();
    println!("  tileRC misses = {nulls} of {cells}");
}

/// `vgpu <out.png> <palette.pal> <file.vxl> [more.vxl ...] [--vpl voxels.vpl] [--facing N] [--no-vpl]`
///
/// 把若干 VXL 文件的段拼成一个模型，走 GPU 光线投射，输出 400x400 画布。
/// 多个部件（车体/炮塔/炮管）拼在一起时，各自 minBounds 会让它们自动对齐 —— 与原始引擎一致。
fn cmd_vgpu(args: &[String]) {
    if args.len() < 3 {
        println!(
            "usage: ra2engine vgpu <out.png> <palette.pal> <file.vxl> [more.vxl ...] \
             [--vpl voxels.vpl] [--facing N] [--no-vpl] [--shadow blob|sun|off]"
        );
        return;
    }
    let out = args[0].clone();
    let pal_path = args[1].clone();
    let mut facing = 0.0f32;
    let mut vpl_path: Option<String> = None;
    let mut use_vpl = true;
    let mut shadow_mode = vxl_gpu::ShadowMode::Blob;
    let mut files: Vec<String> = Vec::new();
    let mut i = 2usize;
    while i < args.len() {
        match args[i].as_str() {
            "--vpl" => {
                i += 1;
                vpl_path = args.get(i).cloned();
            }
            "--facing" => {
                i += 1;
                if let Some(v) = args.get(i) {
                    facing = v.parse().unwrap_or(0.0);
                }
            }
            "--no-vpl" => use_vpl = false,
            // 影子模式：blob = 原版固定偏移的足迹点；sun = 真的从太阳打光；off = 不画。
            "--shadow" => {
                i += 1;
                match args.get(i).map(|s| s.as_str()) {
                    Some("sun") => shadow_mode = vxl_gpu::ShadowMode::Sun,
                    Some("blob") => shadow_mode = vxl_gpu::ShadowMode::Blob,
                    Some("off") => shadow_mode = vxl_gpu::ShadowMode::Off,
                    other => println!("--shadow: unknown mode {other:?}, keeping blob"),
                }
            }
            other => files.push(other.to_string()),
        }
        i += 1;
    }

    let pal_raw = match std::fs::read(&pal_path) {
        Ok(b) => b,
        Err(e) => {
            println!("{pal_path}: {e}");
            return;
        }
    };
    // .pal 是六位值；引擎画体素走的是算过光照的物体调色板，所以这里做 Recalculate
    // 而不是位对齐展开（两者差最多 3，逐像素对照会全错）。
    let palette = lighting::recalculate_palette(&pal_raw, lighting::PaletteMults::default(), true);
    println!("palette: {} bytes -> recalculated, [0..3] = {:?}", pal_raw.len(), &palette[0..9]);

    let vpl_file = vpl_path.as_ref().and_then(|p| {
        let bytes = std::fs::read(p).ok()?;
        match vpl::parse_vpl(&bytes) {
            Ok(v) => {
                println!("vpl: {p} pages={} table={}", v.num_sections, v.lookup_sections.len());
                Some(v)
            }
            Err(e) => {
                println!("vpl {p}: {e}");
                None
            }
        }
    });

    let mut sections = Vec::new();
    let mut file_palette = Vec::new();
    let mut hva = hva::Hva::default();
    for f in &files {
        let bytes = match std::fs::read(f) {
            Ok(b) => b,
            Err(e) => {
                println!("{f}: {e}");
                continue;
            }
        };
        let Some(v) = vxl::parse_vxl(&bytes) else {
            println!("{f}: parse failed");
            continue;
        };
        if file_palette.is_empty() {
            file_palette = v.palette.clone();
        }
        // 同名的 .hva（.vxl 换成 .hva）；没有就留空 —— 每段的矩阵会退成单位阵，
        // 和参考实现在 HVA 缺失时的行为一致。
        let hpart = {
            let hpath = if f.to_lowercase().ends_with(".vxl") {
                format!("{}.hva", &f[..f.len() - 4])
            } else {
                format!("{f}.hva")
            };
            match std::fs::read(&hpath) {
                Ok(hb) => match hva::parse_hva(&hb) {
                    Ok(h) => {
                        println!(
                            "  hva {:<28} frames={} sections={}",
                            hpath,
                            h.num_frames,
                            h.section_names.len()
                        );
                        Some(h)
                    }
                    Err(e) => {
                        println!("  hva {hpath}: {e}");
                        None
                    }
                },
                Err(_) => None,
            }
        };
        for s in &v.sections {
            println!(
                "  {:<10} grid={}x{}x{} normals={} hva={:.4} voxels={} scale=({:.4},{:.4},{:.4}) minBounds=({:.2},{:.2},{:.2})",
                s.name,
                s.x_size,
                s.y_size,
                s.z_size,
                s.normals_type,
                s.hva_multiplier,
                s.voxels.len(),
                s.scale_of(0),
                s.scale_of(1),
                s.scale_of(2),
                s.bounds[0],
                s.bounds[1],
                s.bounds[2]
            );
        }
        let count = v.sections.len();
        sections.extend(v.sections);
        hva.append_part(hpart.as_ref(), count);
    }
    if sections.is_empty() {
        println!("no sections parsed");
        return;
    }
    println!("  hva total: {} sections, {} values", hva.section_names.len(), hva.values.len());
    let model = vxl::Vxl {
        sections,
        palette: file_palette,
    };

    let gpu = match vxl_gpu::VxlGpu::new() {
        Ok(g) => g,
        Err(e) => {
            println!("gpu init failed: {e}");
            return;
        }
    };
    if shadow_mode == vxl_gpu::ShadowMode::Sun && !gpu.sun_supported() {
        println!("sun shadow unsupported on this device (R32Float store read/write); falling back to blob");
    }
    let opt = vxl_gpu::RenderOptions {
        facing,
        use_vpl,
        shadow: vxl_gpu::ShadowSettings {
            mode: shadow_mode,
            ..Default::default()
        },
        ..Default::default()
    };
    let canvas = match gpu.render(&model, Some(&hva), vpl_file.as_ref(), &palette, &opt) {
        Ok(c) => c,
        Err(e) => {
            println!("render failed: {e}");
            return;
        }
    };
    println!("shadow: {shadow_mode:?} res={}", opt.shadow.resolution);
    println!("canvas: {} opaque pixels", canvas.opaque);
    let zb_hit = canvas.z_body.iter().filter(|&&v| v != 0).count();
    let sh_hit = canvas.shadow_mask.iter().filter(|&&v| v != 0).count();
    println!("zBody non-zero = {zb_hit}, shadow footprint = {sh_hit}");
    if let Some((x0, y0, x1, y1)) = canvas.bbox {
        println!("bbox x[{x0}..{x1}] y[{y0}..{y1}]  ({}x{})", x1 - x0 + 1, y1 - y0 + 1);
    }
    match png_out::write_rgba(&out, vxl_gpu::CANVAS, vxl_gpu::CANVAS, &canvas.rgba) {
        Ok(()) => println!("wrote {out}"),
        Err(e) => println!("write {out}: {e}"),
    }
}
