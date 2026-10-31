//! Fixed-step continuous simulation shared by ECS and asset-free headless runs.
//!
//! Unit and SimEnv are data at this boundary; this module never queries ECS.
//! Legacy steering proposes motion; collision checks constrain the committed trajectories.

use std::time::{Duration, Instant};
use crate::crowd;
use crate::nav;
use crate::world::{SimEnv, Unit, aim_turret, turn_toward};

#[derive(Default)]
pub struct Buffers {
    pub agents:Vec<crowd::Agent>,pub snapshot:Vec<crowd::Agent>,
    pub velocity_index:crate::spatial::Index,pub contacts:crate::collision::Workspace,
    bounds:Vec<crate::spatial::Aabb>,neighbors:Vec<usize>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CommandId(pub u64);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandStatus { Idle, Active }

#[derive(Clone, Debug)]
pub struct UnitSample {
    pub id: i32,
    pub command: Option<CommandId>,
    pub status: CommandStatus,
    pub position: (f32, f32),
    pub velocity: (f32, f32),
}

#[derive(Clone, Debug, Default)]
pub struct PhaseTimes {
    pub snapshot: Duration,
    pub pressure: Duration,
    pub velocity: Duration,
    pub contact: Duration,
    pub writeback: Duration,
    pub total: Duration,
    pub spatial: Duration,
}

/// A sampled event, kept in memory. Sampling performs no file or console I/O.
#[derive(Clone, Debug)]
pub struct TickSample {
    pub tick: u64,
    pub seed: u64,
    pub units: Vec<UnitSample>,
    pub moved: usize,
    pub with_guide: usize,
    /// Old proximity heuristic, never a completed-command count.
    pub near_goal: usize,
    pub max_displacement: f32,
    /// None until the arrival state machine is available.
    pub arrived: Option<usize>,
    pub times: PhaseTimes,
}

#[derive(Clone, Debug)]
pub struct Runtime {
    fixed_step: Duration,
    pub seed: u64,
    next_command: u64,
    /// Zero disables samples. The caller chooses whether/how to persist them.
    pub sample_every: u64,
    pub last_sample: Option<TickSample>,
    /// Last completed contact check, including conservative stops and unresolved initial overlaps.
    pub contacts: crate::collision::ContactReport,
    pub spatial_stats: crate::spatial::Stats,
}

impl Default for Runtime {
    fn default() -> Self { Self::from_hz(60.0, 0) }
}

impl Runtime {
    pub fn from_hz(hz: f64, seed: u64) -> Self {
        assert!(hz.is_finite() && hz > 0.0, "simulation rate must be positive and finite");
        let fixed_step = Duration::from_secs_f64(1.0 / hz);
        assert!(!fixed_step.is_zero(), "simulation timestep must be nonzero");
        Self { fixed_step, seed, next_command: 1, sample_every: 0, last_sample: None,
            contacts: crate::collision::ContactReport::default(),spatial_stats:crate::spatial::Stats::default() }
    }
    pub fn fixed_step(&self) -> Duration { self.fixed_step }
    pub fn dt(&self) -> f32 { self.fixed_step.as_secs_f32() }
    pub fn allocate_command(&mut self) -> CommandId {
        let id = CommandId(self.next_command);
        self.next_command = self.next_command.checked_add(1).expect("command ID exhausted");
        id
    }
}

/// Advances exactly one configured timestep in stable unit-ID order.
/// Timings and samples are observations and never feed simulation decisions.
pub fn step_continuous(env: &mut SimEnv, units: &mut [Unit]) {
    if !env.use_crowd || env.nav.cost.is_empty() { return; }
    units.sort_by_key(|u| u.idx);
    assert!(units.windows(2).all(|p| p[0].idx != p[1].idx), "duplicate simulation unit ID");
    let dt = env.runtime.dt();
    let total = Instant::now();
    let phase = Instant::now();
    // ---- 1. 快照：位置 / 路权 / 半径 ----------------------------------------
    let mut buffers=std::mem::take(&mut env.buffers);
    buffers.velocity_index.begin_tick();buffers.contacts.begin_tick();
    let agents=&mut buffers.agents;agents.clear();agents.reserve(units.len());
    for u in units.iter() {
        // 路权：离自己那一格越近越优先（用场的剩余距离；没有场就用欧氏距离）
        let remaining = match (&u.guide, u.cell_x >= 0.0) {
            (Some(gf), true) => {
                let cx = u.cell_x.round() as i32;
                let cy = u.cell_y.round() as i32;
                if gf.in_(cx, cy) {
                    gf.dist_at(cx, cy)
                } else {
                    0.0
                }
            }
            _ => 0.0,
        };
        let settled = u.goal.x < 0
            || (u.goal.x >= 0
                && (u.cell_x - u.goal.x as f32).abs() < env.crowd_params.arrive_radius
                && (u.cell_y - u.goal.y as f32).abs() < env.crowd_params.arrive_radius);
        agents.push(crowd::Agent {
            x: u.cell_x,
            y: u.cell_y,
            speed: u.speed,
            priority: -remaining,
            radius: 0.45,
            settled: settled || !u.on_ground,
            vx: u.vx,
            vy: u.vy,
            max_accel: u.speed * 6.0,
        });
    }

    buffers.bounds.clear();buffers.bounds.extend(agents.iter().map(|a|crate::spatial::Aabb::point(bevy::math::Vec2::new(a.x,a.y))));
    buffers.velocity_index.rebuild(&buffers.bounds);
    let max_speed=agents.iter().map(|a|a.speed.abs()).fold(0.0f32,f32::max);
    let max_radius=agents.iter().map(|a|a.radius).fold(0.0f32,f32::max);

    let snapshot_time = phase.elapsed();
    let phase = Instant::now();

    // ---- 2. 压力场：每 4 tick 重建一次（低频，避免抖动）--------------------
    env.since_pressure += 1;
    if env.since_pressure >= 4 || env.pressure.field.is_empty() {
        env.pressure.rebuild(&env.nav, &agents);
        env.since_pressure = 0;
    }

    let pressure_time = phase.elapsed();
    let phase = Instant::now();

    // ---- 3. 速度层：算出每辆车的期望速度，把位置先推到快照里 ----------------
    buffers.snapshot.clear();buffers.snapshot.extend_from_slice(agents);let snapshot=&buffers.snapshot;
    let dt_safe = dt.max(1e-4);
    for (i, u) in units.iter().enumerate() {
        if !u.on_ground || u.goal.x < 0 || agents[i].settled {
            continue;
        }
        let goal = (u.goal.x as f32, u.goal.y as f32);
        let mut dir = long_range_dir(&env.nav, u, &snapshot[i], goal);
        // 保留原有切向逃逸，直到显式停滞恢复接管。
        if u.escape_ticks > 0 {
            dir = crowd::rotate90(dir, u.escape_side);
        }
        let range=crate::spatial::neighbor_range(snapshot[i].speed,snapshot[i].radius,max_speed,max_radius,
            dt,env.crowd_params.neighbor_radius.max(1.2));
        buffers.velocity_index.query_radius(bevy::math::Vec2::new(snapshot[i].x,snapshot[i].y),range,&mut buffers.neighbors);
        let (vx, vy) = crowd::step_agent(
            i,
            snapshot,
            &buffers.neighbors,
            dir,
            goal,
            &env.pressure,
            &env.crowd_params,
            dt,
        );
        let from = (agents[i].x, agents[i].y);
        agents[i].x = from.0+vx*dt;
        agents[i].y = from.1+vy*dt;
    }

    let velocity_time = phase.elapsed();
    let phase = Instant::now();

    // ---- 4. 位置修正层：把重叠的圆盘直接推开 --------------------------------
    // 速度层只能把稳态压到「软重叠」——期望速度一直在往里顶。这一层是硬约束。
    crate::collision::finish_motion_with_workspace(snapshot,agents,&env.nav,&mut buffers.contacts,&mut env.runtime.contacts);
    let mut spatial_stats=buffers.velocity_index.stats.clone();
    spatial_stats.merge(&buffers.contacts.projection_index.stats);spatial_stats.merge(&buffers.contacts.trajectory_index.stats);
    env.runtime.spatial_stats=spatial_stats;

    let contact_time = phase.elapsed();
    let phase = Instant::now();

    // ---- 5. 状态写回：位置、速度、朝向、贴地 --------------------------------
    let mut dbg_moved = 0usize;
    let mut dbg_guide = 0usize;
    let mut near_goal = 0usize;
    let mut dbg_max_disp = 0.0f32;
    for (i, u) in units.iter_mut().enumerate() {
        let dx = agents[i].x - u.cell_x;
        let dy = agents[i].y - u.cell_y;
        if u.guide.is_some() {
            dbg_guide += 1;
        }
        if agents[i].settled {
            near_goal += 1;
        }
        let disp_dbg = (dx * dx + dy * dy).sqrt();
        if disp_dbg > 1e-4 {
            dbg_moved += 1;
            dbg_max_disp = dbg_max_disp.max(disp_dbg);
        }
        u.cell_x = agents[i].x;
        u.cell_y = agents[i].y;
        u.vx = dx / dt_safe;
        u.vy = dy / dt_safe;
        u.path.clear();
        u.leg_t = 0.0;

        // 卡死判据：想走（不 settled）却几乎没动
        let want_move = !agents[i].settled;
        let disp = (dx * dx + dy * dy).sqrt();
        if disp > 1e-4 {
            u.still_ticks = 0;
        } else {
            u.still_ticks = u.still_ticks.saturating_add(1);
        }
        if want_move && disp < 0.02 * dt_safe {
            u.stall_ticks += 1;
        } else {
            u.stall_ticks = 0;
        }
        if u.escape_ticks > 0 {
            u.escape_ticks -= 1;
        }
        if u.stall_ticks > 30 && u.escape_ticks == 0 {
            u.escape_ticks = 20;
            u.escape_side = if (u.idx % 2) == 0 { 1.0 } else { -1.0 };
            u.stall_ticks = 0;
        }

        // 车体朝向跟着速度走；没速度就保持原朝向，别抽搐
        let sp = (u.vx * u.vx + u.vy * u.vy).sqrt();
        if sp > 0.15 {
            u.dir_target = nav::facing_angle_of(u.vx, u.vy);
        }
        u.dir_angle = turn_toward(u.dir_angle, u.dir_target, u.turn_rate * dt);

        // 闲置只影响炮塔显示；这里不宣称命令已到达或完成。
        u.turret_idle = u.still_ticks > 20;

        aim_turret(u, &env.nav, dt);

        // 贴地
        if env.ground.valid() {
            let rx = u.cell_x.round() as i32;
            let ry = u.cell_y.round() as i32;
            let z = env.ground.level_at(rx, ry);
            let ramp = env.ground.ramp_at(rx, ry);
            if u.ground_dirty || z != u.cell_z || ramp != u.ramp_type {
                u.ground_dirty = false;
                u.cell_z = z;
                u.ramp_type = ramp;
                let (p, y) = env.ground.tilt_at(rx, ry);
                u.tilt_pitch = p;
                u.tilt_yaw = y;
            }
        }
    }


    env.tick += 1;
    if env.runtime.sample_every > 0 && env.tick % env.runtime.sample_every == 0 {
        let times = PhaseTimes { snapshot: snapshot_time, pressure: pressure_time,
            velocity: velocity_time, contact: contact_time, writeback: phase.elapsed(),
            total: total.elapsed(),spatial:env.runtime.spatial_stats.build_time+env.runtime.spatial_stats.query_time };
        env.runtime.last_sample = Some(TickSample {
            tick: env.tick, seed: env.runtime.seed,
            units: units.iter().map(|u| UnitSample {
                id: u.idx, command: u.command_id,
                status: if u.goal.x < 0 { CommandStatus::Idle } else { CommandStatus::Active },
                position: (u.cell_x, u.cell_y), velocity: (u.vx, u.vy),
            }).collect(),
            moved: dbg_moved, with_guide: dbg_guide, near_goal,
            max_displacement: dbg_max_disp, arrived: None, times,
        });
    }
    env.buffers=buffers;
}

/// 长程方向：场里 8 邻域的最省格；没有场就直指目标。
fn long_range_dir(grid:&nav::CostGrid, u: &Unit, me: &crowd::Agent, goal: (f32, f32)) -> (f32, f32) {
    let fallback = {
        let dx = goal.0 - me.x;
        let dy = goal.1 - me.y;
        let len = (dx * dx + dy * dy).sqrt();
        if len <= 1e-6 {
            (0.0, 0.0)
        } else {
            (dx / len, dy / len)
        }
    };
    let Some(gf) = u.guide.as_ref() else {
        return fallback;
    };
    let from=nav::Cell::new(me.x.round() as i32,me.y.round() as i32);
    if !gf.valid_for(grid) || gf.dist_at(from.x,from.y)>=1e29 {return (0.0,0.0);}
    let Some(next)=nav::flow_step(grid,gf,from) else {
        let i=(from.y as usize)*gf.w as usize+from.x as usize;
        return if gf.is_source.get(i).copied()==Some(1) {fallback} else {(0.0,0.0)};
    };
    let (dx,dy)=(next.x as f32-me.x,next.y as f32-me.y);
    let len=dx.hypot(dy);
    if len>1e-6 {(dx/len,dy/len)}else{(0.0,0.0)}
}
