//! 指令链 —— 对等移植。
//!
//! 来源：`I:\bevy\RA2REASM\src\engine\play.cpp` 的 `nearestUsable`（149-160）与
//! `issueOrderAt`（166-230）。
//!
//! 上游 `play.cpp` 还含窗口 / DIB / blit / 相机 / PNG 输出 —— 那些属渲染与平台层，
//! 由 Bevy 侧承担，**不搬**。

use std::sync::Arc;

use bevy::prelude::*;

use crate::nav::{
    compute_flow_field_cpu_multi, path_to_nearest_source, well_floor_cells, Cell, CostGrid,
    FlowField,
};
use crate::world::{SimEnv, Unit};

/// 当前选中的单位。上游是 `Play::sel`（entities 的下标列表）。
#[derive(Resource, Default)]
pub struct Selection(pub Vec<Entity>);

/// 一次指令落在障碍上时，就近找一格可用的：点在障碍上仍然要让部队尽量靠近，而不是什么都不做。
/// 上游签名带一个用不上的 `Footprint`；这里省掉。
pub fn nearest_usable(g: &CostGrid, want: Cell) -> Cell {
    if g.in_(want.x, want.y) && g.walkable(want.x, want.y) {
        return want;
    }
    for r in 1..=64i32 {
        for dy in -r..=r {
            for dx in -r..=r {
                if dx.abs().max(dy.abs()) != r {
                    continue;
                }
                if g.in_(want.x + dx, want.y + dy) && g.walkable(want.x + dx, want.y + dy) {
                    return Cell::new(want.x + dx, want.y + dy);
                }
            }
        }
    }
    want
}

/// RA2 的移动指令：点一处，整群朝它掉下去。这一群是被丢进碗里的一把球 —— 点击处是碗底的中心，
/// 每辆车顺墙滚下来、横穿平地，直到平地满了。没有「到达」，也没有每车一条路线：只有那**一张**场。
///
/// 返回真正拿到路线的单位数（上游 `pathed`）。用 exclusive 形式（`&mut World`）是因为它要
/// 按选中表逐车写 `guide/goal/...`，而车位又是 ECS 实体。
pub fn issue_order_at(world: &mut World, dest: Cell) -> usize {
    let sel: Vec<Entity> = world.resource::<Selection>().0.clone();
    if sel.is_empty() {
        return 0;
    }

    // 取一份导航网格（侵蚀过、单位真正看到的那张）与足迹。指令不频繁，克隆一次无所谓。
    let (nav, footprint) = {
        let env = world.resource::<SimEnv>();
        (env.nav.clone(), env.footprint)
    };

    let dest = nearest_usable(&nav, dest);

    // 碗底的大小**取自这一群**：它恰好装下与车数相同的格，于是「填满」与「停下」是同一件事 ——
    // 最后一辆进来后没有空格剩下，整群就停下，没有任何人为的刹车。宽一格就永远静不下来。
    //
    // 场是从**整块碗底**播种的，不是只从点击那一格。每辆车读同一份箭头，于是它的路线是一张
    // 标量场的流线，两条不可能相交。每车一张私场正是当年产生交叉的原因。
    let n = sel.len() as i32;
    let mut sum_x = 0.0f64;
    let mut sum_y = 0.0f64;
    for &e in &sel {
        let u = world.get::<Unit>(e).unwrap();
        sum_x += u.cell_x as f64;
        sum_y += u.cell_y as f64;
    }
    let head_x = (dest.x as f64 - sum_x / sel.len() as f64) as f32;
    let head_y = (dest.y as f64 - sum_y / sel.len() as f64) as f32;
    let mut floor = well_floor_cells(&nav, &footprint, dest, n, head_x, head_y);
    if floor.is_empty() {
        floor.push(dest);
    }
    // 行动预算：一辆车愿意忍受被**挡住**多久（tick）。站着不动两秒足够它前面任何一条队清空。
    // 自由滚动不花钱，所以这与编队大小、碗底大小都无关。
    let ap_budget = (2.0f64 * 1200.0).ceil() as i32;

    let mut shared = FlowField::default();
    if !compute_flow_field_cpu_multi(&nav, &floor, &mut shared) {
        return 0;
    }
    let shared = Arc::new(shared);
    let command_id = world.resource_mut::<SimEnv>().runtime.allocate_command();
    let mut pathed = 0usize;
    for (i, &e) in sel.iter().enumerate() {
        let here = {
            let u = world.get::<Unit>(e).unwrap();
            Cell::new(u.cell_x.round() as i32, u.cell_y.round() as i32)
        };
        if !shared.in_(here.x, here.y) {
            continue;
        }
        let mut reached = Cell::default();
        let mut route: Vec<Cell> = Vec::new();
        let goal = if path_to_nearest_source(&nav, &shared, here, &mut reached, &mut route) {
            reached
        } else {
            dest
        };

        let mut u = world.get_mut::<Unit>(e).unwrap();
        u.guide = Some(shared.clone());
        // 它自己那一格碗底：顺着共享场最近的播种格。它只是标记指向哪儿，车的运动是跟场走的。
        u.goal = goal;
        u.command_id = Some(command_id);
        u.start_delay = if sel.len() == 1 {
            0.0
        } else {
            0.04 * ((i % 7) as f32)
        };
        u.ap_budget = ap_budget;
        u.still_ticks = 0;
        u.turret_idle = false;
        if u.path.len() != 2 {
            u.path.clear();
            u.leg_t = 0.0;
        }
        pathed += 1;
    }
    pathed
}
