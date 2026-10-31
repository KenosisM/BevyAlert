//! 连续运动层 —— 取代「重力碗」的离散占格移动（路线 C）。
//!
//! 背景：`docs/调研-寻路替代-重力碗停摆机理与换法.md` 证明旧模型有三条硬闸门
//! （碗底格数=车数 / 平地只准往前 / 动量耗尽放弃）加一条不可恢复性（卡住后全工程
//! 没有任何重规划入口），因此「外周汇聚到一点」必然永久停摆。
//!
//! 本模块只负责**短程**：给定「我想往哪走」的单位方向，算出一个不和邻居打架的速度。
//! 长程仍由 `nav::FlowField` 提供方向 —— 那张共享场的好处（一条流线不相交）要保住。
//!
//! 三层，按你说的那三样来：
//!   1. **互斥力（separation）**：局部分离，允许部分重叠，不再「一格锁死一辆」。
//!   2. **路权（right of way）**：优先级 = 离终点还有多远（越近越高）。低优先级主动让，
//!      高优先级只做最小偏转。这一条消掉对头死锁的对称性。
//!   3. **压力场（pressure，解耦）**：单独的拥挤度标量场，只作用于速度，
//!      **绝不进 A*/流场的代价** —— 进了就会让箭头每帧翻转，单位原地抽搐。
//!
//! 速度提议交给 `collision` 执行带半径扫掠、接触约束与轨迹检查，再写回正式位置。

use crate::nav::CostGrid;

/// 一个 agent 的快照（每 tick 建一次，供邻居查询用）。
#[derive(Clone, Copy, Debug)]
pub struct Agent {
    /// Shared formation command, only for units with exclusive parking targets.
    pub group: Option<u64>,
    pub x: f32,
    pub y: f32,
    pub speed: f32,
    /// 路权：**越大越优先**。用「剩余路程的负数」或「已走路程」都行，只要单调。
    pub priority: f32,
    /// 半径（格）
    pub radius: f32,
    /// 是否已经到位（到位的不参与推挤，但仍占位）
    pub settled: bool,
    /// 上一帧速度（格/秒）—— 加速度限幅要用
    pub vx: f32,
    pub vy: f32,
    /// 加速度上限（格/秒²）
    pub max_accel: f32,
}

impl Default for Agent {
    fn default() -> Self {
        Agent {
            group: None,
            x: 0.0,
            y: 0.0,
            speed: 6.0,
            priority: 0.0,
            radius: 0.45,
            settled: false,
            vx: 0.0,
            vy: 0.0,
            max_accel: 24.0,
        }
    }
}

/// 把方向向量转 90°（切向逃逸用）。side = +1 逆时针，-1 顺时针。
pub fn rotate90(dir: (f32, f32), side: f32) -> (f32, f32) {
    (-dir.1 * side, dir.0 * side)
}

#[derive(Clone, Copy, Debug)]
pub struct Params {
    /// 邻居查询半径（格）
    pub neighbor_radius: f32,
    /// 分离力权重
    pub sep_weight: f32,
    /// 压力场权重
    pub press_weight: f32,
    /// 让路时的横向偏转强度（0..1）
    pub yield_strength: f32,
    /// 重叠判定的软半径倍数：距离小于 `radius * this` 才算「挤到了」
    pub overlap_scale: f32,
    /// 允许的最大速度（格/秒），用来兜住参数写错
    pub speed_cap: f32,
    /// 到位判定半径（格）
    pub arrive_radius: f32,
}

impl Params {
    /// 单位半径（格）。RTS 里和格子大小挂钩，先固定 0.45。
    pub fn radius_default(&self) -> f32 {
        0.45
    }
}

impl Default for Params {
    fn default() -> Self {
        Params {
            neighbor_radius: 2.2,
            sep_weight: 3.2,
            press_weight: 1.1,
            yield_strength: 0.85,
            overlap_scale: 1.05,
            speed_cap: 24.0,
            arrive_radius: 0.45,
        }
    }
}

/// 拥挤度压力场（解耦层）。低频重建：每隔若干 tick 用当前位置做一次密度 + 模糊。
#[derive(Default, Debug)]
pub struct Pressure {
    w: usize,
    h: usize,
    /// 0..1 的拥挤度
    pub field: Vec<f32>,
    scratch: Vec<f32>,
}

impl Pressure {
    pub fn rebuild(&mut self, grid: &CostGrid, agents: &[Agent]) {
        let (w, h) = (grid.w as usize, grid.h as usize);
        if w == 0 || h == 0 {
            return;
        }
        if self.w != w || self.h != h {
            self.w = w;
            self.h = h;
            self.field = vec![0.0; w * h];
            self.scratch = vec![0.0; w * h];
        }
        for v in &mut self.field {
            *v = 0.0;
        }
        for a in agents {
            let cx = a.x.round() as i32;
            let cy = a.y.round() as i32;
            if cx < 0 || cy < 0 || cx as usize >= w || cy as usize >= h {
                continue;
            }
            let i = (cy as usize) * w + (cx as usize);
            self.field[i] += 1.0;
        }
        // 单位换成 0..1：一格挤两人就已经「满」了
        for v in &mut self.field {
            *v = (*v * 0.5).min(1.0);
        }
        // 一次 3x3 盒模糊（可分离：先横后纵），让梯度不至于只有一格宽
        blur_axis(&self.field, &mut self.scratch, w, h, true);
        blur_axis(&self.scratch, &mut self.field, w, h, false);
    }

    /// 压力梯度（指向「更挤」的方向）。调用方要往反方向推。
    pub fn gradient_at(&self, x: f32, y: f32) -> (f32, f32) {
        if self.field.is_empty() {
            return (0.0, 0.0);
        }
        let cx = x.round() as i32;
        let cy = y.round() as i32;
        let sample = |dx: i32, dy: i32| -> f32 {
            let sx = cx + dx;
            let sy = cy + dy;
            if sx < 0 || sy < 0 || sx as usize >= self.w || sy as usize >= self.h {
                return 0.0;
            }
            self.field[(sy as usize) * self.w + (sx as usize)]
        };
        let gx = sample(1, 0) - sample(-1, 0);
        let gy = sample(0, 1) - sample(0, -1);
        (gx * 0.5, gy * 0.5)
    }
}

fn blur_axis(src: &[f32], dst: &mut [f32], w: usize, h: usize, horizontal: bool) {
    for y in 0..h {
        for x in 0..w {
            let mut sum = 0.0;
            let mut n = 0.0;
            for k in -1i32..=1 {
                let (sx, sy) = if horizontal {
                    (x as i32 + k, y as i32)
                } else {
                    (x as i32, y as i32 + k)
                };
                if sx < 0 || sy < 0 || sx as usize >= w || sy as usize >= h {
                    continue;
                }
                sum += src[(sy as usize) * w + (sx as usize)];
                n += 1.0;
            }
            dst[(y) * w + x] = sum / n;
        }
    }
}

fn norm(x: f32, y: f32) -> (f32, f32) {
    let len = (x * x + y * y).sqrt();
    if len <= 1e-6 {
        (0.0, 0.0)
    } else {
        (x / len, y / len)
    }
}

/// 单个 agent 这一 tick 的速度。
///
/// * `desired_dir` —— 长程方向（流场/路径给的单位向量）
/// * `goal` —— 它要去的那个格（用来减速与判定到位）
pub fn step_agent(
    i: usize,
    agents: &[Agent],
    neighbors: &[usize],
    desired_dir: (f32, f32),
    goal: (f32, f32),
    pressure: &Pressure,
    params: &Params,
    _dt: f32,
) -> (f32, f32) {
    step_agent_preferred(i,agents,neighbors,desired_dir,goal,pressure,params,_dt,None)
}

/// Exclusive parking supplies path-based braking; legacy callers retain their old curve.
pub fn step_agent_preferred(i:usize,agents:&[Agent],neighbors:&[usize],desired_dir:(f32,f32),goal:(f32,f32),
    pressure:&Pressure,params:&Params,_dt:f32,preferred_speed:Option<f32>)->(f32,f32) {
    let me = agents[i];

    let to_goal = (goal.0 - me.x, goal.1 - me.y);
    let goal_dist = (to_goal.0 * to_goal.0 + to_goal.1 * to_goal.1).sqrt();
    if goal_dist <= params.arrive_radius {
        return (0.0, 0.0);
    }

    // ---- 长程期望速度 --------------------------------------------------------
    let preferred=preferred_speed.unwrap_or(me.speed).max(0.0);
    let mut vx = desired_dir.0 * preferred;
    let mut vy = desired_dir.1 * preferred;
    // 快到了就减速，避免冲过终点再掉头（这是「抽搐」的常见来源之一）
    let brake = if preferred_speed.is_some(){1.0}else{(goal_dist / 1.5).clamp(0.25, 1.0)};
    vx *= brake;
    vy *= brake;

    // ---- 互斥力（先算：站定的单位也要能被挤开，否则会冻结在重叠里）-------------
    let mut sx = 0.0;
    let mut sy = 0.0;
    let mut touching = false;  // 有没有被人挤到（挤到时不让加速度限幅拖慢反应）
    let r = me.radius.max(0.05);
    for &j in neighbors {
        let other=&agents[j];
        if j == i {
            continue;
        }
        let dx = me.x - other.x;
        let dy = me.y - other.y;
        let d2 = dx * dx + dy * dy;
        if d2 > params.neighbor_radius * params.neighbor_radius || d2 < 1e-9 {
            continue;
        }
        let d = d2.sqrt();
        let (ux, uy) = (dx / d, dy / d);

        // (1) 硬分离：挤到一起就互相推开（软约束，不禁止重叠，只是不愿意）
        //
        // 衰减形状试过 GPU-Crowd-Godot 的 `strength * f² / d`（f = 1 - d/r）：
        // 数值上更漂亮，但在我们的 dt（1/64）下会和位置修正层组成极限环 ——
        // 实测 24 车收敛后仍以 0.54 格/30tick 缓缓蠕动。线性衰减没有这个问题。
        // 结论：形状不是越“物理”越好，要看和积分步长、投影层的配合。
        let min_d = r * params.overlap_scale * 2.0;
        if d < min_d {
            touching = true;
            let push = (min_d - d) / min_d;
            sx += ux * push * params.sep_weight;
            sy += uy * push * params.sep_weight;
        }

        // 站定的单位：不追目标，但仍然要被人群推开（否则重叠会被永久冻结）
        if me.settled {
            let push = (min_d - d).max(0.0) / min_d;
            sx += ux * push * params.sep_weight;
            sy += uy * push * params.sep_weight;
            continue;
        }

        // (2) 路权：谁该让。priority 越大越优先。
        //     我让：主动横向偏转（垂直于相对速度，方向选「远离对方」的那侧）
        //     他让：我只做很小的一点偏转，别为了躲他跑偏
        // Exclusive teammates use their local parking paths around stationary
        // owners. Artificial side yield against a parked row (or parallel traffic)
        // would push followers out of their still-free arrival columns.
        if me.group.is_some()&&me.group==other.group
            &&(other.settled||me.vx*other.vx+me.vy*other.vy>0.0) {continue;}
        let approach = vx * (-ux) + vy * (-uy); // >0 表示我在朝他走
        if approach > 0.0 {
            let right_of_way = me.priority >= other.priority;
            let strength = if right_of_way {
                0.15 * params.yield_strength
            } else {
                params.yield_strength
            };
            let urgency = (approach / me.speed.max(0.1)).clamp(0.0, 1.0)
                * (1.0 - (d / params.neighbor_radius).clamp(0.0, 1.0));
            // 垂直方向；用叉积定侧，保证两边选相反的那侧（对头时自然错开）
            let cross = (-ux) * vy - (-uy) * vx;
            let side = if cross >= 0.0 { 1.0 } else { -1.0 };
            let (px, py) = (-uy * side, ux * side);
            sx += px * strength * urgency;
            sy += py * strength * urgency;
        }
    }

    // 站定的单位：不追目标，但被挤到仍然要往回挤（否则重叠被永久冻结）
    if me.settled {
        let mag = (sx * sx + sy * sy).sqrt();
        let cap0 = me.speed * 0.5;
        let k = if mag > cap0 { cap0 / mag } else { 1.0 };
        return (sx * k, sy * k);
    }

    // ---- 压力场：往不挤的方向推 ---------------------------------------------
    let (gx, gy) = pressure.gradient_at(me.x, me.y);
    sx -= gx * params.press_weight;
    sy -= gy * params.press_weight;

    vx += sx * me.speed;
    vy += sy * me.speed;

    // ---- 拥挤时收油门 -------------------------------------------------------
    // 不加这条会和分离力「拔河」：一直往人堆里顶，最终稳态距离就落在半重叠上。
    // 这里按邻域里挤进来的人数压低期望速度，人少了自动恢复。
    let mut crowding = 0.0f32;
    for &j in neighbors {
        let other=&agents[j];
        if j == i || other.settled && me.settled {
            continue;
        }
        let d = ((me.x - other.x).powi(2) + (me.y - other.y).powi(2)).sqrt();
        if d < 1.2 {
            crowding += 1.0 - d / 1.2;
        }
    }
    let brake_crowd = (1.0 - 0.55 * crowding).clamp(0.15, 1.0);
    vx *= brake_crowd;
    vy *= brake_crowd;

    // ---- 限速 ---------------------------------------------------------------
    // ---- 限速（Boids 的两段限幅：先加速度，再速度）-------------------------
    // 只截断速度会让「想加速」变成瞬移，没有加减速手感；先按 max_accel*dt 截断
    // 速度的变化量，再截断速度本身。
    let clamp_speed = |x: f32, y: f32, cap: f32| -> (f32, f32) {
        let m = (x * x + y * y).sqrt();
        if m > cap && m > 1e-6 {
            (x / m * cap, y / m * cap)
        } else {
            (x, y)
        }
    };
    let dvx = vx - me.vx;
    let dvy = vy - me.vy;
    let dv = (dvx * dvx + dvy * dvy).sqrt();
    let dv_cap = me.max_accel * _dt;
    // 挤在一起时不限加速度：这时需要的是立刻反应，不是「物理上优雅」
    if !touching && dv > dv_cap && dv > 1e-6 {
        vx = me.vx + dvx / dv * dv_cap;
        vy = me.vy + dvy / dv * dv_cap;
    }
    let cap = me.speed.min(params.speed_cap);
    clamp_speed(vx, vy, cap)
}

/// Compatibility entry for the legacy steering tests. Production also checks old->new
/// pair trajectories with collision::finish_motion before any position is committed.
#[cfg(test)]
pub fn resolve_overlaps(agents: &mut [Agent], grid: &CostGrid, iterations: u32) {
    crate::collision::project_contacts(agents,grid,iterations);
}

/// Legacy steering helper using the same disc sweep as production.
#[cfg(test)]
pub fn resolve_position(
    grid: &CostGrid,
    from: (f32, f32),
    to: (f32, f32),
) -> (f32, f32) {
    let (p,_)=crate::collision::move_disc(grid,bevy::math::Vec2::new(from.0,from.1),
        bevy::math::Vec2::new(to.0,to.1),0.45);
    (p.x,p.y)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parallel_formation_members_do_not_receive_crossing_side_yield() {
        let agents=vec![Agent{group:Some(1),x:4.0,y:4.0,vx:6.0,..Agent::default()},
            Agent{group:Some(1),x:5.1,y:4.0,vx:6.0,..Agent::default()}];
        let velocity=step_agent(0,&agents,&[0,1],(1.0,0.0),(25.0,4.0),&Pressure::default(),&Params::default(),1.0/60.0);
        assert!(velocity.0>0.0);assert!(velocity.1.abs()<1e-6);
    }

    #[test]
    fn a_parked_teammate_beyond_the_exclusive_goal_does_not_deflect_its_follower() {
        let agents=vec![Agent{group:Some(1),x:4.0,y:4.0,vx:4.0,..Agent::default()},
            Agent{group:Some(1),x:5.65,y:4.0,settled:true,..Agent::default()}];
        let velocity=step_agent(0,&agents,&[0,1],(1.0,0.0),(4.55,4.0),&Pressure::default(),&Params::default(),1.0/60.0);
        assert!(velocity.0>0.0);assert!(velocity.1.abs()<1e-6);
    }

    fn flat_grid(w: i32, h: i32) -> CostGrid {
        let mut g = CostGrid::default();
        g.w = w;
        g.h = h;
        g.cost = vec![1u8; (w as usize) * (h as usize)];
        g
    }

    /// 切向逃逸的旋转必须保持长度、且两个方向相反。
    #[test]
    fn rotate90_keeps_length_and_flips_side() {
        let (x, y) = rotate90((1.0, 0.0), 1.0);
        assert!((x - 0.0).abs() < 1e-6 && (y - 1.0).abs() < 1e-6, "逆时针应得 (0,1)，实得 ({x},{y})");
        let (x2, y2) = rotate90((1.0, 0.0), -1.0);
        assert!((x2 - 0.0).abs() < 1e-6 && (y2 + 1.0).abs() < 1e-6, "顺时针应得 (0,-1)，实得 ({x2},{y2})");
        // 长度守恒
        for a in 0..16 {
            let ang = a as f32 * 0.4;
            let d = (ang.cos(), ang.sin());
            let r = rotate90(d, 1.0);
            let m = (r.0 * r.0 + r.1 * r.1).sqrt();
            assert!((m - 1.0).abs() < 1e-5);
        }
    }

    /// 一群人在空地上朝同一个点收拢：不应该有人被永久挤死（速度归零且离终点很远）。
    #[test]
    fn convergence_does_not_deadlock() {
        let grid = flat_grid(64, 64);
        let params = Params::default();
        let goal = (32.0f32, 32.0f32);

        // 一圈人，半径 12，绕中心铺开
        let n = 24usize;
        let mut agents: Vec<Agent> = (0..n)
            .map(|k| {
                let a = (k as f32) * std::f32::consts::TAU / (n as f32);
                Agent {
                    x: 32.0 + a.cos() * 12.0,
                    y: 32.0 + a.sin() * 12.0,
                    speed: 6.0,
                    ..Agent::default()
                }
            })
            .collect();

        let mut pressure = Pressure::default();
        let dt = 1.0 / 64.0;
        let mut last_moved = vec![0usize; n];

        for tick in 0..(64 * 20) {
            pressure.rebuild(&grid, &agents);
            // 路权：离终点越近越优先（每 tick 更新，所以是动态的）
            for a in &mut agents {
                let d = ((goal.0 - a.x).powi(2) + (goal.1 - a.y).powi(2)).sqrt();
                a.priority = -d;
                a.settled = d <= params.arrive_radius;
            }
            let snapshot = agents.clone();
            for i in 0..n {
                let dir = norm(goal.0 - snapshot[i].x, goal.1 - snapshot[i].y);
                let neighbors:Vec<_>=(0..snapshot.len()).collect(); // test-only brute-force reference
                let (vx, vy) = step_agent(i, &snapshot, &neighbors, dir, goal, &pressure, &params, dt);
                let from = (agents[i].x, agents[i].y);
                let to = (from.0 + vx * dt, from.1 + vy * dt);
                let (_nx, _ny) = resolve_position(&grid, from, to);
                agents[i].vx = (_nx - from.0) / dt;
                agents[i].vy = (_ny - from.1) / dt;
                agents[i].x = _nx;
                agents[i].y = _ny;
                if vx.abs() + vy.abs() > 0.01 {
                    last_moved[i] = tick;
                }
            }
            resolve_overlaps(&mut agents, &grid, 2);
            // 全部到位就可以提前结束
            if agents.iter().all(|a| {
                ((goal.0 - a.x).powi(2) + (goal.1 - a.y).powi(2)).sqrt() <= 1.5
            }) {
                break;
            }
        }

        // 断言一：不许有人被甩在半路。
        // 注意下限不是 0 而是「24 个半径 0.45 的圆盘密排后的半径」≈2.3 格 ——
        // 要求所有人都挤进 1.5 格在几何上不可能，那是断言写错了，不是代码错了。
        let mut max_d = 0.0f32;
        for (i, a) in agents.iter().enumerate() {
            let d = ((goal.0 - a.x).powi(2) + (goal.1 - a.y).powi(2)).sqrt();
            max_d = max_d.max(d);
            assert!(d <= 3.6, "agent {i} 停在离终点 {d:.2} 格处（被甩下了）");
        }
        // 24 个半径 0.45 的圆盘六边形密排的理论半径约 2.3 格；软分离会略微更松，
        // 3.6 是「没被甩下」的上界，不是「挤得多紧」的目标。
        assert!(max_d < 3.6, "整团没聚起来：最远的 {max_d:.2} 格");
        println!("[crowd] 收敛完成：最远 {max_d:.2} 格");

        // 断言二：不许抖。所有人最后都该是静止的（这一步没动 = 稳定）。
        let before: Vec<(f32, f32)> = agents.iter().map(|a| (a.x, a.y)).collect();
        for _ in 0..30 {
            pressure.rebuild(&grid, &agents);
            let snapshot = agents.clone();
            for i in 0..n {
                let dir = norm(goal.0 - snapshot[i].x, goal.1 - snapshot[i].y);
                let neighbors:Vec<_>=(0..snapshot.len()).collect(); // test-only brute-force reference
                let (vx, vy) = step_agent(i, &snapshot, &neighbors, dir, goal, &pressure, &params, dt);
                let from = (agents[i].x, agents[i].y);
                let to = (from.0 + vx * dt, from.1 + vy * dt);
                let (nx, ny) = resolve_position(&grid, from, to);
                agents[i].x = nx;
                agents[i].y = ny;
            }
        }
        for (i, a) in agents.iter().enumerate() {
            let drift = ((a.x - before[i].0).powi(2) + (a.y - before[i].1).powi(2)).sqrt();
            assert!(drift < 0.35, "agent {i} 静止后还在抖（位移 {drift:.2} 格）");
        }

        // 断言三：不许真的完全重叠。
        for i in 0..n {
            for j in (i + 1)..n {
                let d = ((agents[i].x - agents[j].x).powi(2)
                    + (agents[i].y - agents[j].y).powi(2))
                .sqrt();
                // 契约：软分离允许约 15% 的视觉重叠（半径 0.45 完全相切是 0.90），
        // 但绝不允许真的叠成一格。0.75 = 相切距离的 83%。
        assert!(d > 0.75, "agent {i} 与 {j} 挤成一团（距离 {d:.2}，半径 0.45）");
            }
        }
    }
}
