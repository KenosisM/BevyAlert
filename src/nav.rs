//! 格网寻路 —— 对等移植。
//!
//! 来源：`I:\bevy\RA2REASM\src\engine\nav.cpp` / `nav.h`。
//!
//! 网格 = 玩法格（rx, ry）；一条路径就是一串格，所以单位始终对齐到格 —— 引擎的地面场正是
//! 按这个假设写的。这里的 CPU 实现是**正确性基准**。
//!
//! 🔴 本文件是**逐行对等搬运**：分支、常数、边界判断、tie-break 顺序一项不少。
//! 唯一无法对等的地方已在注释里标明（C++ `std::priority_queue` 在 f 相等时的弹出顺序本身
//! 就是未定义的，见 `Node::cmp`）。

use std::cmp::Ordering;
use std::collections::{BinaryHeap, VecDeque};

// ============================================================================ 基本类型

/// 玩法格坐标 (rx, ry)。
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default, Hash)]
pub struct Cell {
    pub x: i32,
    pub y: i32,
}

impl Cell {
    pub fn new(x: i32, y: i32) -> Cell {
        Cell { x, y }
    }
}

/// 通行代价网格。`cost` 0 = 不可通行，1 = 畅通，越大越慢。
/// 资产层从地形 land 特性 + 静态障碍填出来。
#[derive(Clone, Debug, Default)]
pub struct CostGrid {
    pub w: i32,
    pub h: i32,
    pub cost: Vec<u8>,
    /// 对角步能不能「从两个被挡格之间穿过去」。网格是解算器看到的东西，所以这条规则跟着
    /// 网格走：`erode_grid` 会从它所用的 footprint 复制过来。车辆填满自己的格子、一格都不许
    /// 切；比格子小的单位（步兵）可以在只有一侧被挡时切过去。
    pub cut_corners: bool,
}

impl CostGrid {
    /// 上游 `CostGrid::in`。`in` 是 Rust 关键字，这里叫 `in_`。
    pub fn in_(&self, x: i32, y: i32) -> bool {
        x >= 0 && y >= 0 && x < self.w && y < self.h
    }
    pub fn at(&self, x: i32, y: i32) -> i32 {
        if self.in_(x, y) {
            self.cost[(y as usize) * (self.w as usize) + (x as usize)] as i32
        } else {
            0
        }
    }
    pub fn walkable(&self, x: i32, y: i32) -> bool {
        self.at(x, y) > 0
    }
    /// `costgrid.rs` 的调用点用的名字，等价于上游 `in`。
    pub fn in_bounds(&self, x: i32, y: i32) -> bool {
        self.in_(x, y)
    }
}

/// 从 (cx,cy) 到 (nx,ny) 这一步能不能走？只有对角步可能被拒。对角两翼都要通，
/// 除非单位小到能切角（那样一侧通就够）；两侧都被挡的对角一律拒 —— 那根本不是缝。
pub fn diagonal_allowed(g: &CostGrid, cx: i32, cy: i32, nx: i32, ny: i32) -> bool {
    if nx == cx || ny == cy {
        return true; // 轴向步没有角可切
    }
    let side_a = g.walkable(nx, cy);
    let side_b = g.walkable(cx, ny);
    if g.cut_corners {
        side_a || side_b
    } else {
        side_a && side_b
    }
}

/// 一个单位要占多大。玩法是格制的，所以再小的单位也要一格；车辆默认 1x1。
/// `ox/oy` 把单位的锚点格放进攻占矩形的内部。
#[derive(Clone, Copy, Debug)]
pub struct Footprint {
    pub w: i32,
    pub h: i32,
    pub ox: i32,
    pub oy: i32,
    /// 能不能切对角（见 `CostGrid::cut_corners`）。车辆不切；步兵比它一格小，可以切。
    pub cut_corners: bool,
}

impl Default for Footprint {
    fn default() -> Self {
        Footprint {
            w: 1,
            h: 1,
            ox: 0,
            oy: 0,
            cut_corners: false,
        }
    }
}

pub struct PathResult {
    /// from -> to，含两端
    pub cells: Vec<Cell>,
    /// 弹出过的节点数
    pub expanded: usize,
    pub found: bool,
}

// ============================================================================ 流场

/// 一张流场：每格到目标的距离 + 每格的最佳步向。多个单位共用一张场，寻路的代价只付一次。
#[derive(Clone, Debug, Default)]
pub struct FlowField {
    pub w: i32,
    pub h: i32,
    /// 到目标的代价；不可达时是巨值
    pub dist: Vec<f32>,
    pub dir_x: Vec<i8>,
    pub dir_y: Vec<i8>,
    /// 被播种的格（重力碗的碗底）。单源场也带同一张图，好让读场的人用同一套判断「我站在
    /// 目的地上」。只有不报种子的后端（GPU 解算器）才会让它空着。
    pub is_source: Vec<u8>,
    pub sources: Vec<Cell>,
}

impl FlowField {
    pub fn in_(&self, x: i32, y: i32) -> bool {
        x >= 0 && y >= 0 && x < self.w && y < self.h
    }
    pub fn dist_at(&self, x: i32, y: i32) -> f32 {
        if self.in_(x, y) {
            self.dist[(y as usize) * (self.w as usize) + (x as usize)]
        } else {
            1e30f32
        }
    }
    /// 上游 `FlowField::dirAt`：越界给 (0,0)。
    pub fn dir_at(&self, x: i32, y: i32) -> (i32, i32) {
        if !self.in_(x, y) {
            return (0, 0);
        }
        let i = (y as usize) * (self.w as usize) + (x as usize);
        (self.dir_x[i] as i32, self.dir_y[i] as i32)
    }
}

// ============================================================================ 内部工具

const K_DIR_X: [i32; 8] = [1, -1, 0, 0, 1, 1, -1, -1];
const K_DIR_Y: [i32; 8] = [0, 0, 1, -1, 1, -1, 1, -1];
const K_DIAG: f32 = 1.41421356;

/// Octile 距离：对 8 向、最小代价为 1 的移动是可采纳的启发。
fn octile(dx: i32, dy: i32) -> f32 {
    let ax = dx.abs();
    let ay = dy.abs();
    let (lo, hi) = if ax < ay { (ax, ay) } else { (ay, ax) };
    (hi as f32) + (K_DIAG - 1.0) * (lo as f32)
}

/// Bresenham supercover：直线只穿过可通行的格，也绝不从被挡的对角缝里溜过去。
fn line_clear(g: &CostGrid, a: Cell, b: Cell) -> bool {
    let mut x = a.x;
    let mut y = a.y;
    let dx = (b.x - x).abs();
    let dy = (b.y - y).abs();
    let sx = if x < b.x { 1 } else { -1 };
    let sy = if y < b.y { 1 } else { -1 };
    let mut err = dx - dy;
    loop {
        if !g.walkable(x, y) {
            return false;
        }
        if x == b.x && y == b.y {
            return true;
        }
        let e2 = 2 * err;
        let mut nx = x;
        let mut ny = y;
        if e2 > -dy {
            err -= dy;
            nx += sx;
        }
        if e2 < dx {
            err += dx;
            ny += sy;
        }
        if !diagonal_allowed(g, x, y, nx, ny) {
            return false;
        }
        x = nx;
        y = ny;
    }
}

/// 小顶堆节点（按 f / d 排）。
///
/// 🔴 上游用的是 `std::priority_queue`，**在 f 相等时弹出顺序本身就是未定义的**；这里用
/// `BinaryHeap` 反转比较得到同样的「按 f 升序」，平局顺序同样不保证 —— 这是唯一无法逐位
/// 对等的地方（路径代价相同，但等代价的路径可能选不同的那条）。**不加索引 tie-break**，
/// 以免用一个上游没有的规则替换上游的不确定性。
#[derive(PartialEq)]
struct Node {
    f: f32,
    i: i32,
}

impl Eq for Node {}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f
            .partial_cmp(&self.f)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// ============================================================================ A*

pub fn find_path(g: &CostGrid, from: Cell, to: Cell) -> PathResult {
    find_path_extra(g, from, to, &|_, _| false, -1)
}

/// 带动态障碍的 A*。`extra_blocked(x,y)` 说的格一律拒；`budget` 限制展开数（-1 = 不限），
/// 让一次局部重算保持便宜。
pub fn find_path_extra(
    g: &CostGrid,
    from: Cell,
    to: Cell,
    extra_blocked: &dyn Fn(i32, i32) -> bool,
    budget: i32,
) -> PathResult {
    // 上游 `PathResult::expanded` 从头到尾没被赋值（恒 0），这里照抄不写。
    let mut out = PathResult {
        cells: Vec::new(),
        expanded: 0,
        found: false,
    };
    if !g.in_(from.x, from.y) || !g.in_(to.x, to.y) {
        return out;
    }
    if !g.walkable(from.x, from.y) || !g.walkable(to.x, to.y) {
        return out;
    }
    if extra_blocked(from.x, from.y) || extra_blocked(to.x, to.y) {
        return out;
    }

    let n = (g.w as usize) * (g.h as usize);
    let idx = |x: i32, y: i32| (y as usize) * (g.w as usize) + (x as usize);

    let mut g_score = vec![1e30f32; n];
    let mut came = vec![-1i32; n];
    let mut closed = vec![0u8; n];

    let mut open: BinaryHeap<Node> = BinaryHeap::new();

    let start = idx(from.x, from.y) as i32;
    let goal = idx(to.x, to.y) as i32;
    g_score[start as usize] = 0.0;
    open.push(Node {
        f: octile(to.x - from.x, to.y - from.y),
        i: start,
    });

    let mut expanded = 0i32;
    while let Some(cur) = open.pop() {
        if closed[cur.i as usize] != 0 {
            continue;
        }
        closed[cur.i as usize] = 1;
        expanded += 1;
        if budget >= 0 && expanded > budget {
            break;
        }
        if cur.i == goal {
            out.found = true;
            let mut i = cur.i;
            while i != -1 {
                out.cells.push(Cell {
                    x: i % g.w,
                    y: i / g.w,
                });
                i = came[i as usize];
            }
            out.cells.reverse();
            return out;
        }
        let cx = cur.i % g.w;
        let cy = cur.i / g.w;
        for d in 0..8 {
            let nx = cx + K_DIR_X[d];
            let ny = cy + K_DIR_Y[d];
            if !g.in_(nx, ny) || !g.walkable(nx, ny) {
                continue;
            }
            if extra_blocked(nx, ny) {
                continue;
            }
            let diag = K_DIR_X[d] != 0 && K_DIR_Y[d] != 0;
            if !diagonal_allowed(g, cx, cy, nx, ny) {
                continue;
            }
            // 对角会蹭到的那个角上若有动态障碍，也算被占 —— 单位贴着队友过去时不能蹭穿它的侧翼。
            if diag && (extra_blocked(nx, cy) || extra_blocked(cx, ny)) {
                continue;
            }
            let ni = idx(nx, ny) as i32;
            if closed[ni as usize] != 0 {
                continue;
            }
            let step = (g.at(nx, ny) as f32) * if diag { K_DIAG } else { 1.0 };
            let tentative = g_score[cur.i as usize] + step;
            if tentative < g_score[ni as usize] {
                g_score[ni as usize] = tentative;
                came[ni as usize] = cur.i;
                open.push(Node {
                    f: tentative + octile(to.x - nx, to.y - ny),
                    i: ni,
                });
            }
        }
    }
    out
}

// ============================================================================ 流场求解

/// 单目标 Dijkstra（8 向、不许切角），再从目标向外写方向场。
pub fn compute_flow_field_cpu(g: &CostGrid, goal: Cell, out: &mut FlowField) -> bool {
    out.w = g.w;
    out.h = g.h;
    let n = (g.w as usize) * (g.h as usize);
    out.dist = vec![1e30f32; n];
    out.dir_x = vec![0i8; n];
    out.dir_y = vec![0i8; n];
    if !g.walkable(goal.x, goal.y) {
        return false;
    }

    let idx = |x: i32, y: i32| (y as usize) * (g.w as usize) + (x as usize);

    let mut pq: BinaryHeap<Node> = BinaryHeap::new();
    let mut done = vec![0u8; n];
    let gi = idx(goal.x, goal.y) as i32;
    out.dist[gi as usize] = 0.0;
    // 单源场就是多源场的单种子特例，所以它带同一张 source 图：读场的人能用同一套判断
    // 「我站在目的地上」。留空正是当年把到达判定悄悄关掉的原因。
    out.is_source = vec![0u8; n];
    out.sources.clear();
    out.is_source[gi as usize] = 1;
    out.sources.push(goal);
    pq.push(Node { f: 0.0, i: gi });

    while let Some(c) = pq.pop() {
        if done[c.i as usize] != 0 {
            continue;
        }
        done[c.i as usize] = 1;
        let cx = c.i % g.w;
        let cy = c.i / g.w;
        for d in 0..8 {
            let nx = cx + K_DIR_X[d];
            let ny = cy + K_DIR_Y[d];
            if !g.in_(nx, ny) || !g.walkable(nx, ny) {
                continue;
            }
            let diag = K_DIR_X[d] != 0 && K_DIR_Y[d] != 0;
            if !diagonal_allowed(g, cx, cy, nx, ny) {
                continue;
            }
            let step = (g.at(nx, ny) as f32) * if diag { K_DIAG } else { 1.0 };
            let nd = c.f + step;
            let ni = idx(nx, ny);
            if nd < out.dist[ni] {
                out.dist[ni] = nd;
                pq.push(Node { f: nd, i: ni as i32 });
            }
        }
    }

    write_directions(g, out);
    true
}

/// 多源版：队列一开始就放入**所有**种子（距离 0）。云从一个整体向外扩，于是每格的距离是
/// 它到**最近**种子的距离、方向也指向那个种子。共享这一张场，正是让一个编队的路线不互相
/// 交叉的原因：两个邻居读同一份箭头，于是跑成平行线，而不是各自解一条恰好撞上的私路。
pub fn compute_flow_field_cpu_multi(g: &CostGrid, sources: &[Cell], out: &mut FlowField) -> bool {
    out.w = g.w;
    out.h = g.h;
    let n = (g.w as usize) * (g.h as usize);
    out.dist = vec![1e30f32; n];
    out.dir_x = vec![0i8; n];
    out.dir_y = vec![0i8; n];
    out.is_source = vec![0u8; n];
    out.sources.clear();

    let idx = |x: i32, y: i32| (y as usize) * (g.w as usize) + (x as usize);

    let mut pq: BinaryHeap<Node> = BinaryHeap::new();
    let mut done = vec![0u8; n];
    let mut seeded = 0;
    for s in sources {
        if !g.in_(s.x, s.y) || !g.walkable(s.x, s.y) {
            continue;
        }
        let si = idx(s.x, s.y);
        if out.dist[si] == 0.0 {
            continue; // 同一个格被列了两次
        }
        out.dist[si] = 0.0;
        out.is_source[si] = 1;
        out.sources.push(*s);
        pq.push(Node { f: 0.0, i: si as i32 });
        seeded += 1;
    }
    if seeded == 0 {
        return false;
    }

    while let Some(c) = pq.pop() {
        if done[c.i as usize] != 0 {
            continue;
        }
        done[c.i as usize] = 1;
        let cx = c.i % g.w;
        let cy = c.i / g.w;
        for d in 0..8 {
            let nx = cx + K_DIR_X[d];
            let ny = cy + K_DIR_Y[d];
            if !g.in_(nx, ny) || !g.walkable(nx, ny) {
                continue;
            }
            let diag = K_DIR_X[d] != 0 && K_DIR_Y[d] != 0;
            if !diagonal_allowed(g, cx, cy, nx, ny) {
                continue;
            }
            let step = (g.at(nx, ny) as f32) * if diag { K_DIAG } else { 1.0 };
            let nd = c.f + step;
            let ni = idx(nx, ny);
            if nd < out.dist[ni] {
                out.dist[ni] = nd;
                pq.push(Node { f: nd, i: ni as i32 });
            }
        }
    }

    write_directions(g, out);
    true
}

/// 方向场：让每格指向「邻居距离 + 进入本格的代价」最小的那个邻居。
/// 用本格自己的进入代价打分，且**允许相等** —— 前驱恰好打平，用严格 `<` 会让每个已收敛的格
/// 都像局部极小。
fn write_directions(g: &CostGrid, out: &mut FlowField) {
    let idx = |x: i32, y: i32| (y as usize) * (g.w as usize) + (x as usize);
    for y in 0..g.h {
        for x in 0..g.w {
            if !g.walkable(x, y) {
                continue;
            }
            let i = idx(x, y);
            if out.dist[i] >= 1e29f32 {
                continue;
            }
            let c_self = g.at(x, y) as f32;
            let mut best = out.dist[i] + 1e-3;
            let mut bx = 0;
            let mut by = 0;
            for d in 0..8 {
                let nx = x + K_DIR_X[d];
                let ny = y + K_DIR_Y[d];
                if !g.in_(nx, ny) || !g.walkable(nx, ny) {
                    continue;
                }
                let diag = K_DIR_X[d] != 0 && K_DIR_Y[d] != 0;
                if !diagonal_allowed(g, x, y, nx, ny) {
                    continue;
                }
                let v = out.dist[idx(nx, ny)] + c_self * if diag { K_DIAG } else { 1.0 };
                if v < best {
                    best = v;
                    bx = K_DIR_X[d];
                    by = K_DIR_Y[d];
                }
            }
            out.dir_x[i] = bx as i8;
            out.dir_y[i] = by as i8;
        }
    }
}

/// 沿共享场从 `from` 一路下行到某个种子格，并报出是哪一个。那个格就是本单位的目的地：
/// 离它最近的种子。因为每个邻居读同一张场，它们的路线保持平行而不是绞在一起。
pub fn path_to_nearest_source(
    g: &CostGrid,
    ff: &FlowField,
    from: Cell,
    reached: &mut Cell,
    out: &mut Vec<Cell>,
) -> bool {
    out.clear();
    if !g.in_(from.x, from.y) || !g.walkable(from.x, from.y) {
        return false;
    }
    if ff.w != g.w || ff.h != g.h {
        return false;
    }
    out.push(from);
    let mut cur = from;
    let guard = (g.w as usize) * (g.h as usize) + 4;
    for _ in 0..guard {
        let ci = (cur.y as usize) * (ff.w as usize) + (cur.x as usize);
        if ci < ff.is_source.len() && ff.is_source[ci] != 0 {
            *reached = cur;
            return true; // 命中种子格：单位到达目的地
        }
        let (dx, dy) = ff.dir_at(cur.x, cur.y);
        if dx == 0 && dy == 0 {
            return false; // 这张场从这里到不了任何种子
        }
        let nx = cur.x + dx;
        let ny = cur.y + dy;
        if !g.in_(nx, ny) || !g.walkable(nx, ny) {
            return false;
        }
        cur = Cell { x: nx, y: ny };
        out.push(cur);
    }
    false
}

/// 沿流场从 `from` 下行到指定的 `goal`；这段下行就是该单位走的格路径。
pub fn path_from_flow_field(
    g: &CostGrid,
    ff: &FlowField,
    from: Cell,
    goal: Cell,
    out: &mut Vec<Cell>,
) -> bool {
    out.clear();
    if !g.walkable(from.x, from.y) || !g.walkable(goal.x, goal.y) {
        return false;
    }
    if ff.w != g.w || ff.h != g.h {
        return false;
    }
    out.push(from);
    let mut cur = from;
    let guard = (g.w as usize) * (g.h as usize) + 4;
    let mut step = 0usize;
    while cur.x != goal.x || cur.y != goal.y {
        if step > guard {
            return false;
        }
        step += 1;
        let (dx, dy) = ff.dir_at(cur.x, cur.y);
        if dx == 0 && dy == 0 {
            return false;
        }
        let nx = cur.x + dx;
        let ny = cur.y + dy;
        if !g.in_(nx, ny) || !g.walkable(nx, ny) {
            return false;
        }
        cur = Cell { x: nx, y: ny };
        out.push(cur);
    }
    !out.is_empty()
}

// ============================================================================ 朝向

/// 一步之后单位朝哪，写成 RA2 的八分位（0 = 东北，顺时针；乘 32 就是游戏角度）。
/// 格步先过等距变换再变成屏幕方向：+x 向右下、+y 向左下。这张表就是整个映射。
pub fn facing_octant(dx: i32, dy: i32) -> i32 {
    const K_OCTANT: [i32; 9] = [
        224, // (-1,-1) 上    = 北
        192, // (-1, 0) 左上  = 西北
        160, // (-1,+1) 左    = 西
        0,   // ( 0,-1) 右上  = 东北
        0,   // ( 0, 0) 不迈步
        128, // ( 0,+1) 左下  = 西南
        32,  // (+1,-1) 右    = 东
        64,  // (+1, 0) 右下  = 东南
        96,  // (+1,+1) 下    = 南
    ];
    let cx = if dx < 0 {
        -1
    } else if dx > 0 {
        1
    } else {
        0
    };
    let cy = if dy < 0 {
        -1
    } else if dy > 0 {
        1
    } else {
        0
    };
    K_OCTANT[((cx + 1) * 3 + (cy + 1)) as usize]
}

/// 同一个朝向，但不取整：游戏角度 0..255，0 在东北。`dx,dy` 可以是小数，所以这也给出
/// 「到任意一点的方位角」（炮塔瞄远处某格就用它）。
pub fn facing_angle_of(dx: f32, dy: f32) -> f32 {
    let sx = dx - dy;
    let sy = dx + dy;
    if sx == 0.0 && sy == 0.0 {
        return 0.0;
    }
    let deg = sy.atan2(sx) * 180.0 / 3.14159265358979;
    let mut a = (32.0 + deg * (256.0 / 360.0)) % 256.0;
    if a < 0.0 {
        a += 256.0;
    }
    a
}

// ============================================================================ 足迹

/// 锚在 (x,y) 的单位是否整块都站在可走格上。
pub fn fits(g: &CostGrid, f: &Footprint, x: i32, y: i32) -> bool {
    for j in 0..f.h {
        for i in 0..f.w {
            if !g.walkable(x + i + f.ox, y + j + f.oy) {
                return false;
            }
        }
    }
    true
}

/// 这个足迹的单位**真正看到的**那张格网：只有整块足迹落上去都清的格才算通，且它继承
/// 覆盖到的最慢的那一格。在侵蚀过的格网上解算，宽车才会拒绝钻比自身窄的缝 ——
/// 在原始格网上解会把每个单位都当成质点。
pub fn erode_grid(g: &CostGrid, f: &Footprint) -> CostGrid {
    let mut out = CostGrid::default();
    out.w = g.w;
    out.h = g.h;
    out.cut_corners = f.cut_corners; // 规则跟着单位解算用的那张网走
    if g.w <= 0 || g.h <= 0 {
        return out;
    }
    out.cost = vec![0u8; (g.w as usize) * (g.h as usize)];
    for y in 0..g.h {
        for x in 0..g.w {
            let mut worst = 1i32;
            let mut ok = true;
            let mut j = 0;
            while j < f.h && ok {
                for i in 0..f.w {
                    let cx = x + i + f.ox;
                    let cy = y + j + f.oy;
                    if !g.walkable(cx, cy) {
                        ok = false;
                        break;
                    }
                    let c = g.at(cx, cy);
                    if c > worst {
                        worst = c; // 单位快不过它最慢的那一格
                    }
                }
                j += 1;
            }
            if ok {
                out.cost[(y as usize) * (g.w as usize) + (x as usize)] =
                    (if worst < 1 { 1 } else { worst }) as u8;
            }
        }
    }
    out
}

// ============================================================================ 编队 / 碗底

/// 上游三个函数里那个同名 lambda：足迹整块在界内、每格都通，且没被 `used` 标记。
fn foot_free(grid: &CostGrid, f: &Footprint, used: &[u8], x: i32, y: i32) -> bool {
    if !grid.in_(x, y) || !grid.walkable(x, y) {
        return false;
    }
    for j in 0..f.h {
        for i in 0..f.w {
            let cx = x + i + f.ox;
            let cy = y + j + f.oy;
            if !grid.in_(cx, cy) {
                return false;
            }
            if used[(cy as usize) * (grid.w as usize) + (cx as usize)] != 0 {
                return false;
            }
        }
    }
    true
}

fn foot_claim(grid: &CostGrid, f: &Footprint, used: &mut [u8], x: i32, y: i32) {
    for j in 0..f.h {
        for i in 0..f.w {
            let cx = x + i + f.ox;
            let cy = y + j + f.oy;
            if grid.in_(cx, cy) {
                used[(cy as usize) * (grid.w as usize) + (cx as usize)] = 1;
            }
        }
    }
}

/// 编队的格，就是碗底按无朝向切出来的同一个紧凑圆盘 —— 出发时的队形与落点处的队形同一个形状。
pub fn formation_cells(g: &CostGrid, f: &Footprint, centre: Cell, n: i32) -> Vec<Cell> {
    well_floor_cells(g, f, centre, n, 0.0, 0.0)
}

/// 重力碗的碗底：一个编队被点到某处时掉进去的那批格（头注释见 nav.h，这里逐行对照）。
/// 它是一个**圆盘**（按欧氏平方距离逐格取，不是切比雪夫方阵），多出来的从**逃逸侧**削掉。
pub fn well_floor_cells(
    grid: &CostGrid,
    f: &Footprint,
    centre: Cell,
    n: i32,
    dir_x: f32,
    dir_y: f32,
) -> Vec<Cell> {
    let mut out: Vec<Cell> = Vec::new();
    if n <= 0 {
        return out;
    }
    out.reserve(n as usize);

    let mut used = vec![0u8; (grid.w as usize) * (grid.h as usize)];

    #[derive(Clone, Copy)]
    struct Cand {
        d2: i32,
        x: i32,
        y: i32,
    }

    // 搜索半径内的每一个候选，按到点击处的**平方**距离排。整数平方距离让环的归属是精确的，
    // 于是永远没有浮点平局、也没有「差一点就属于这一环」的格。
    let max_r = 96.min(grid.w.max(grid.h));
    let mut cands: Vec<Cand> = Vec::with_capacity(((2 * max_r + 1) * (2 * max_r + 1)) as usize);
    for dy in -max_r..=max_r {
        for dx in -max_r..=max_r {
            cands.push(Cand {
                d2: dx * dx + dy * dy,
                x: centre.x + dx,
                y: centre.y + dy,
            });
        }
    }
    cands.sort_by(|a, b| a.d2.cmp(&b.d2).then(a.y.cmp(&b.y)).then(a.x.cmp(&b.x)));

    // 编队行进的横向轴（把朝向转四分之一圈）。
    let head_len = (dir_x * dir_x + dir_y * dir_y).sqrt();
    let off_x = if head_len > 1e-6 { -dir_y / head_len } else { 0.0 };
    let off_y = if head_len > 1e-6 { dir_x / head_len } else { 0.0 };

    let mut i = 0usize;
    while i < cands.len() && (out.len() as i32) < n {
        let d2 = cands[i].d2;
        let mut j = i;
        while j < cands.len() && cands[j].d2 == d2 {
            j += 1;
        }
        // 同一距离内的平局按「离行进线多远」破：离得近的先取，于是正前方的格先于两侧、
        // 两侧成对取走。没有这一层，平局会落到 `y` 上，半个环只留上半 —— 那就是「取整环」
        // 曾经掩盖掉的上偏。
        cands[i..j].sort_by(|a, b| {
            let la = ((a.x - centre.x) as f32 * off_x + (a.y - centre.y) as f32 * off_y).abs();
            let lb = ((b.x - centre.x) as f32 * off_x + (b.y - centre.y) as f32 * off_y).abs();
            la.partial_cmp(&lb)
                .unwrap_or(Ordering::Equal)
                .then(a.y.cmp(&b.y))
                .then(a.x.cmp(&b.x))
        });
        for k in i..j {
            if (out.len() as i32) >= n {
                break;
            }
            let c = cands[k];
            if !foot_free(grid, f, &used, c.x, c.y) {
                continue;
            }
            foot_claim(grid, f, &mut used, c.x, c.y);
            out.push(Cell::new(c.x, c.y));
        }
        i = j;
    }

    // 🔴 上游这段「把超出部分从逃逸侧削掉」在改成逐格取之后**已经不可达**：内层守卫
    //    （`out.size() < n`）保证 out 永远不超过 n。照抄保留，别删。
    if (out.len() as i32) > n {
        let len = (dir_x * dir_x + dir_y * dir_y).sqrt();
        if len > 1e-6 {
            let ux = dir_x / len;
            let uy = dir_y / len;
            out.sort_by(|a, b| {
                let pa = (a.x - centre.x) as f32 * ux + (a.y - centre.y) as f32 * uy;
                let pb = (b.x - centre.x) as f32 * ux + (b.y - centre.y) as f32 * uy;
                pa.partial_cmp(&pb)
                    .unwrap_or(Ordering::Equal)
                    .then(a.y.cmp(&b.y))
                    .then(a.x.cmp(&b.x))
            });
        } else {
            // 没有可言的朝向：均匀地削，最外圈的先没。
            out.sort_by(|a, b| {
                let ax = a.x - centre.x;
                let ay = a.y - centre.y;
                let bx = b.x - centre.x;
                let by = b.y - centre.y;
                let da = ax * ax + ay * ay;
                let db = bx * bx + by * by;
                da.cmp(&db).then(a.y.cmp(&b.y)).then(a.x.cmp(&b.x))
            });
        }
        out.truncate(n as usize);
    }
    out
}

/// 这个足迹最近的空闲锚点，按环外扩搜。锚点互不相同，两辆车才不会抢同一批格。
fn nearest_free_cell(g: &CostGrid, f: &Footprint, want: Cell, taken: &[u8]) -> Cell {
    if foot_free(g, f, taken, want.x, want.y) {
        return want;
    }
    for r in 1..=96i32 {
        for dy in -r..=r {
            for dx in -r..=r {
                if dx.abs().max(dy.abs()) != r {
                    continue;
                }
                if foot_free(g, f, taken, want.x + dx, want.y + dy) {
                    return Cell::new(want.x + dx, want.y + dy);
                }
            }
        }
    }
    want
}

/// 给每个单位在点击处周围派一格：编队**向点击处收拢**（松散的队形收紧，而不是保持旧间距）。
/// 最近优先认领 + 成对交换 uncrossing（见 nav.h）。
pub fn assign_formation_targets(
    g: &CostGrid,
    f: &Footprint,
    dest: Cell,
    starts: &[Cell],
    targets: &mut Vec<Cell>,
) {
    *targets = vec![dest; starts.len()];
    if starts.is_empty() {
        return;
    }
    let n = starts.len();

    // 目标区：点击处周围能装下整群的最小紧凑圆盘。
    let cells = formation_cells(g, f, dest, n as i32);
    if cells.is_empty() {
        return; // 点击处被封死：所有人就停在点击格
    }

    #[derive(Clone, Copy)]
    struct Pair {
        unit: usize,
        cell: usize,
        d2: i64,
    }
    let mut pairs: Vec<Pair> = Vec::with_capacity(n * cells.len());
    for i in 0..n {
        for c in 0..cells.len() {
            let dx = cells[c].x as i64 - starts[i].x as i64;
            let dy = cells[c].y as i64 - starts[i].y as i64;
            pairs.push(Pair {
                unit: i,
                cell: c,
                d2: dx * dx + dy * dy,
            });
        }
    }
    pairs.sort_by(|a, b| a.d2.cmp(&b.d2));

    let mut taken = vec![0u8; (g.w as usize) * (g.h as usize)];
    let mut placed = vec![0u8; n];
    let mut cell_used = vec![0u8; cells.len()];
    let mut assigned = 0usize;
    for p in &pairs {
        if assigned == n {
            break;
        }
        if placed[p.unit] != 0 || cell_used[p.cell] != 0 {
            continue;
        }
        placed[p.unit] = 1;
        cell_used[p.cell] = 1;
        targets[p.unit] = cells[p.cell];
        foot_claim(g, f, &mut taken, cells[p.cell].x, cells[p.cell].y);
        assigned += 1;
    }

    // 🔴 直线距离，**不是**它的平方：保证 uncrossing 的那条交换规则是针对真实长度的；
    //    平方长度会偏好一条交叉配对（5+5 以平方 50 胜过 1+8 的 65），把交叉留下。
    let dist = |a: Cell, b: Cell| -> f64 {
        let dx = (a.x - b.x) as f64;
        let dy = (a.y - b.y) as f64;
        (dx * dx + dy * dy).sqrt()
    };
    let mut improved = true;
    while improved {
        improved = false;
        for i in 0..n {
            for j in (i + 1)..n {
                if dist(starts[i], targets[j]) + dist(starts[j], targets[i])
                    < dist(starts[i], targets[i]) + dist(starts[j], targets[j])
                {
                    targets.swap(i, j);
                    improved = true;
                }
            }
        }
    }

    // 候选比单位还少（地形把它们挤没了）：剩下的去最近的空闲格，而不是站着不动。
    for i in 0..n {
        if placed[i] != 0 {
            continue;
        }
        let t = nearest_free_cell(g, f, dest, &taken);
        targets[i] = t;
        foot_claim(g, f, &mut taken, t.x, t.y);
    }
}

// ============================================================================ 瓶颈 / 侧偏

/// 可走空间收窄的地方：沿通行方向**窄**，而沿这个方向的两侧**都**敞得开（门洞、桥口、
/// 两栋楼之间的缝）。这正是对穿的两支真要碰头的地方。
#[derive(Clone, Copy, Debug, Default)]
pub struct Bottleneck {
    /// 0 = 通道沿 x，开口在 y 上量
    pub axis: i32,
    pub c_min: i32,
    pub c_max: i32,
    pub t_min: i32,
    pub t_max: i32,
    pub n: i32,
}

pub fn detect_bottlenecks(grid: &CostGrid, w_max: i32, reach: i32, grow: i32) -> Vec<Bottleneck> {
    let gw = grid.w;
    let gh = grid.h;
    let n_cells = (gw as usize) * (gh as usize);
    // 从这一格出发，四个格方向上可走空间能延伸多远。两趟前缀；四个数组合起来就说清了
    // 「横向多宽、纵向多长」。
    let mut up = vec![0i32; n_cells];
    let mut dn = vec![0i32; n_cells];
    let mut lf = vec![0i32; n_cells];
    let mut rt = vec![0i32; n_cells];
    let wk = |x: i32, y: i32| grid.in_(x, y) && grid.walkable(x, y);
    for y in 0..gh {
        for x in 0..gw {
            let i = (y as usize) * (gw as usize) + (x as usize);
            up[i] = if wk(x, y) {
                (if y > 0 {
                    up[((y - 1) as usize) * (gw as usize) + (x as usize)]
                } else {
                    0
                }) + 1
            } else {
                0
            };
            lf[i] = if wk(x, y) {
                (if x > 0 {
                    lf[(y as usize) * (gw as usize) + (x - 1) as usize]
                } else {
                    0
                }) + 1
            } else {
                0
            };
        }
    }
    for y in (0..gh).rev() {
        for x in (0..gw).rev() {
            let i = (y as usize) * (gw as usize) + (x as usize);
            dn[i] = if wk(x, y) {
                (if y < gh - 1 {
                    dn[((y + 1) as usize) * (gw as usize) + (x as usize)]
                } else {
                    0
                }) + 1
            } else {
                0
            };
            rt[i] = if wk(x, y) {
                (if x < gw - 1 {
                    rt[(y as usize) * (gw as usize) + (x + 1) as usize]
                } else {
                    0
                }) + 1
            } else {
                0
            };
        }
    }
    // 一格「收窄」，当它沿通行方向是窄的，且沿这个方向的两侧都敞得开。两个轴里较长的那个
    // 当作「通行方向」，另一个就是开口。
    let mut is_b = vec![0u8; n_cells];
    for y in 0..gh {
        for x in 0..gw {
            if !wk(x, y) {
                continue;
            }
            let i = (y as usize) * (gw as usize) + (x as usize);
            let w_y = up[i] + dn[i] - 1;
            let w_x = lf[i] + rt[i] - 1;
            if w_x >= w_y {
                if w_y > w_max {
                    continue;
                }
                let mut a = 0;
                let mut b = 0;
                for k in 1..=reach {
                    if wk(x - k, y) {
                        let j = (y as usize) * (gw as usize) + (x - k) as usize;
                        let w = up[j] + dn[j] - 1;
                        if w > a {
                            a = w;
                        }
                    }
                    if wk(x + k, y) {
                        let j = (y as usize) * (gw as usize) + (x + k) as usize;
                        let w = up[j] + dn[j] - 1;
                        if w > b {
                            b = w;
                        }
                    }
                }
                if a >= w_y + grow && b >= w_y + grow {
                    is_b[i] = 1;
                }
            } else {
                if w_x > w_max {
                    continue;
                }
                let mut a = 0;
                let mut b = 0;
                for k in 1..=reach {
                    if wk(x, y - k) {
                        let j = ((y - k) as usize) * (gw as usize) + (x as usize);
                        let w = lf[j] + rt[j] - 1;
                        if w > a {
                            a = w;
                        }
                    }
                    if wk(x, y + k) {
                        let j = ((y + k) as usize) * (gw as usize) + (x as usize);
                        let w = lf[j] + rt[j] - 1;
                        if w > b {
                            b = w;
                        }
                    }
                }
                if a >= w_x + grow && b >= w_x + grow {
                    is_b[i] = 1;
                }
            }
        }
    }
    // 把一个收窄点的格聚成一个连通块，再读它的包围盒：长边是通行方向，短边是开口。
    let mut out: Vec<Bottleneck> = Vec::new();
    let mut seen = vec![0u8; n_cells];
    for y in 0..gh {
        for x in 0..gw {
            let i0 = (y as usize) * (gw as usize) + (x as usize);
            if is_b[i0] == 0 || seen[i0] != 0 {
                continue;
            }
            let mut st: Vec<Cell> = Vec::new();
            st.push(Cell::new(x, y));
            seen[i0] = 1;
            let mut head = 0usize;
            while head < st.len() {
                let c = st[head];
                head += 1;
                for d in 0..8 {
                    let nx = c.x + K_DIR_X[d];
                    let ny = c.y + K_DIR_Y[d];
                    if !grid.in_(nx, ny) {
                        continue;
                    }
                    let j = (ny as usize) * (gw as usize) + (nx as usize);
                    if is_b[j] == 0 || seen[j] != 0 {
                        continue;
                    }
                    seen[j] = 1;
                    st.push(Cell::new(nx, ny));
                }
            }
            let mut xmin = st[0].x;
            let mut xmax = st[0].x;
            let mut ymin = st[0].y;
            let mut ymax = st[0].y;
            for c in &st {
                if c.x < xmin {
                    xmin = c.x;
                }
                if c.x > xmax {
                    xmax = c.x;
                }
                if c.y < ymin {
                    ymin = c.y;
                }
                if c.y > ymax {
                    ymax = c.y;
                }
            }
            let mut bp = Bottleneck::default();
            bp.n = st.len() as i32;
            if xmax - xmin < ymax - ymin {
                bp.axis = 0; // 长边是 y ⇒ 车流沿 x 走
                bp.c_min = xmin;
                bp.c_max = xmax;
                bp.t_min = ymin;
                bp.t_max = ymax;
            } else {
                bp.axis = 1;
                bp.c_min = ymin;
                bp.c_max = ymax;
                bp.t_min = xmin;
                bp.t_max = xmax;
            }
            out.push(bp);
        }
    }
    out
}

/// 侧向偏好：让对向车流分道而行，而不是迎面顶上。窄口的每一格按它离「本队想守的那条线」
/// 多远定价（从通道中轴往自己右手侧偏 `gamma` 的距离），于是便宜的车道是偏心的那条。
/// 对向那支走的是反方向、右手也在对面，两支队就自然分进两条车道。只在通道宽不超过 `w_max`
/// 处生效；开阔区一格都不动。代价保持在 1..255 —— 只改价格，永不改可通行。
pub fn side_bias_grid(
    grid: &CostGrid,
    dir_x: f32,
    dir_y: f32,
    lambda: f32,
    w_max: i32,
    gamma: f32,
) -> CostGrid {
    let mut out = grid.clone();
    if lambda <= 0.0 || w_max <= 0 || (dir_x == 0.0 && dir_y == 0.0) {
        return out;
    }
    let gw = grid.w;
    let gh = grid.h;
    let n_cells = (gw as usize) * (gh as usize);
    let mut up = vec![0i32; n_cells];
    let mut dn = vec![0i32; n_cells];
    let mut lf = vec![0i32; n_cells];
    let mut rt = vec![0i32; n_cells];
    let wk = |x: i32, y: i32| grid.in_(x, y) && grid.walkable(x, y);
    // 和 detectBottlenecks 同一对前缀趟；这里给出通道宽度与「本格离中轴多远」。
    for y in 0..gh {
        for x in 0..gw {
            let i = (y as usize) * (gw as usize) + (x as usize);
            up[i] = if wk(x, y) {
                (if y > 0 {
                    up[((y - 1) as usize) * (gw as usize) + (x as usize)]
                } else {
                    0
                }) + 1
            } else {
                0
            };
            lf[i] = if wk(x, y) {
                (if x > 0 {
                    lf[(y as usize) * (gw as usize) + (x - 1) as usize]
                } else {
                    0
                }) + 1
            } else {
                0
            };
        }
    }
    for y in (0..gh).rev() {
        for x in (0..gw).rev() {
            let i = (y as usize) * (gw as usize) + (x as usize);
            dn[i] = if wk(x, y) {
                (if y < gh - 1 {
                    dn[((y + 1) as usize) * (gw as usize) + (x as usize)]
                } else {
                    0
                }) + 1
            } else {
                0
            };
            rt[i] = if wk(x, y) {
                (if x < gw - 1 {
                    rt[(y as usize) * (gw as usize) + (x + 1) as usize]
                } else {
                    0
                }) + 1
            } else {
                0
            };
        }
    }
    // 本队沿两个方向分量里较大的那个走；另一个轴就是横向。行进方向的右手（在 y 向下的格网
    // 上把 (dx,dy) 顺时针转四分之一圈）就是要被推向的那侧。
    let along_x = dir_x.abs() >= dir_y.abs();
    for y in 0..gh {
        for x in 0..gw {
            let i = (y as usize) * (gw as usize) + (x as usize);
            if !wk(x, y) {
                continue;
            }
            // 本格两侧（不含本格）直到墙的空格数。
            let mut d_plus = 0;
            let mut d_minus = 0;
            if along_x {
                if dir_x >= 0.0 {
                    d_plus = dn[i] - 1;
                    d_minus = up[i] - 1;
                } else {
                    d_plus = up[i] - 1;
                    d_minus = dn[i] - 1;
                }
            } else if dir_y <= 0.0 {
                d_plus = rt[i] - 1;
                d_minus = lf[i] - 1;
            } else {
                d_plus = lf[i] - 1;
                d_minus = rt[i] - 1;
            }
            let w_full = d_plus + d_minus + 1; // 通道宽度，含本格
            if w_full > w_max {
                continue; // 开阔地：完全没有偏好
            }
            let half_w = 0.5 * (w_full - 1) as f32;
            // 本格离中轴多远，正方向朝右。贴着右墙的格（dMinus 大、dPlus 小）最靠右，
            // 于是它最便宜：本队守自己的右手车道。
            let lat = 0.5 * (d_minus - d_plus) as f32;
            let pen = lambda * (lat - gamma * half_w).abs();
            let mut v = (grid.at(x, y) as f32 + pen).round() as i32; // 上游 std::lround
            if v < 1 {
                v = 1;
            }
            if v > 255 {
                v = 255;
            }
            out.cost[i] = v as u8;
        }
    }
    out
}

// ============================================================================ 路径平滑 / 分层

/// 把路径拉直：从起点贪心地跳到直线可达的最远格；再从终点同样地倒着做一遍。两者都不是
/// 最小拐弯数路径，但失败的位置不同，取拐弯少的那条 —— 车该尽量少转向。
pub fn smooth_path(g: &CostGrid, path: &mut Vec<Cell>) {
    if path.len() < 3 {
        return;
    }
    let n = path.len();

    let forward = |path: &Vec<Cell>| -> Vec<Cell> {
        let mut out: Vec<Cell> = Vec::new();
        out.push(path[0]);
        let mut i = 0usize;
        while i + 1 < n {
            let mut j = n - 1;
            while j > i + 1 {
                if line_clear(g, path[i], path[j]) {
                    break;
                }
                j -= 1;
            }
            out.push(path[j]);
            i = j;
        }
        out
    };
    let backward = |path: &Vec<Cell>| -> Vec<Cell> {
        let mut out: Vec<Cell> = Vec::new();
        out.push(path[n - 1]);
        let mut k = n - 1;
        while k > 0 {
            let mut j = 0usize;
            while j + 1 < k {
                if line_clear(g, path[j], path[k]) {
                    break;
                }
                j += 1;
            }
            out.push(path[j]);
            k = j;
        }
        out.reverse();
        out
    };

    let f = forward(path);
    let b = backward(path);
    // 拐弯少的赢；打平取格数少的，稍微缩短里程。
    let links = |p: &Vec<Cell>| -> usize {
        let mut t = 0usize;
        let mut pdx = 0i32;
        let mut pdy = 0i32;
        for i in 1..p.len() {
            let dx = p[i].x - p[i - 1].x;
            let dy = p[i].y - p[i - 1].y;
            if i > 1 && (dx != pdx || dy != pdy) {
                t += 1;
            }
            pdx = dx;
            pdy = dy;
        }
        t
    };
    let lf = links(&f);
    let lb = links(&b);
    if lf != lb {
        *path = if lf < lb { f } else { b };
    } else {
        *path = if f.len() <= b.len() { f } else { b };
    }
}

#[derive(Clone, Debug, Default)]
pub struct BlockGraph {
    pub block_size: i32,
    pub bw: i32,
    pub bh: i32,
    pub passable: Vec<u8>,
    pub adj: Vec<Vec<i32>>,
}

/// 把格网切成方块，把「共享一条可通过的缝」的相邻块连起来。
pub fn build_block_graph(g: &CostGrid, block_size: i32, out: &mut BlockGraph) {
    out.block_size = if block_size > 0 { block_size } else { 1 };
    let b = out.block_size;
    out.bw = (g.w + b - 1) / b;
    out.bh = (g.h + b - 1) / b;
    let bw = out.bw;
    let nb = (out.bw as usize) * (out.bh as usize);
    out.passable = vec![0u8; nb];
    out.adj = vec![Vec::new(); nb];
    for y in 0..g.h {
        for x in 0..g.w {
            if g.walkable(x, y) {
                out.passable[((y / b) * bw + (x / b)) as usize] = 1;
            }
        }
    }
    // 两块相接：跨缝正对着的至少一对格两边都能走 —— 也就是一扇门。没有门的话，两块即使
    // 在几何上相邻，也是分开的。
    for by in 0..out.bh {
        for bx in 0..(out.bw - 1) {
            let x = (bx + 1) * b - 1;
            let mut seam = false;
            for k in 0..b {
                let y = by * b + k;
                if y < g.h && x + 1 < g.w && g.walkable(x, y) && g.walkable(x + 1, y) {
                    seam = true;
                    break;
                }
            }
            if seam {
                let a = (by * bw + bx) as usize;
                let c = (by * bw + bx + 1) as usize;
                out.adj[a].push(c as i32);
                out.adj[c].push(a as i32);
            }
        }
    }
    for bx in 0..out.bw {
        for by in 0..(out.bh - 1) {
            let y = (by + 1) * b - 1;
            let mut seam = false;
            for k in 0..b {
                let x = bx * b + k;
                if x < g.w && y + 1 < g.h && g.walkable(x, y) && g.walkable(x, y + 1) {
                    seam = true;
                    break;
                }
            }
            if seam {
                let a = (by * bw + bx) as usize;
                let c = ((by + 1) * bw + bx) as usize;
                out.adj[a].push(c as i32);
                out.adj[c].push(a as i32);
            }
        }
    }
}

/// 只在「块走廊」内搜的一条路线。起点终点同块、或走廊算不出来时，退回一趟普通 A*。
pub fn find_path_hierarchical(
    g: &CostGrid,
    from: Cell,
    to: Cell,
    out: &mut Vec<Cell>,
    block_size: i32,
) -> bool {
    out.clear();
    if !g.walkable(from.x, from.y) || !g.walkable(to.x, to.y) {
        return false;
    }
    if from.x == to.x && from.y == to.y {
        out.push(from);
        return true;
    }
    let mut bg = BlockGraph::default();
    build_block_graph(g, block_size, &mut bg);
    let nb = bg.bw * bg.bh;
    let bid_of = |c: Cell| (c.y / bg.block_size) * bg.bw + (c.x / bg.block_size);
    let sb = bid_of(from);
    let tb = bid_of(to);

    let bfs = |s: i32| -> Vec<i32> {
        let mut dist = vec![-1i32; nb as usize];
        let mut q: VecDeque<i32> = VecDeque::new();
        dist[s as usize] = 0;
        q.push_back(s);
        while let Some(b) = q.pop_front() {
            for n in &bg.adj[b as usize] {
                if dist[*n as usize] < 0 {
                    dist[*n as usize] = dist[b as usize] + 1;
                    q.push_back(*n);
                }
            }
        }
        dist
    };
    let d_from = bfs(sb);
    let d_to = bfs(tb);
    let mut min_sum = i32::MAX;
    for b in 0..nb {
        if d_from[b as usize] >= 0 && d_to[b as usize] >= 0 {
            let s = d_from[b as usize] + d_to[b as usize];
            if s < min_sum {
                min_sum = s;
            }
        }
    }

    // 同块、或两块根本不通：普通 A* 又对又便宜。
    if sb == tb || min_sum == i32::MAX {
        let r = find_path(g, from, to);
        if !r.found {
            return false;
        }
        *out = r.cells;
        return true;
    }

    // 只留走廊：一块能落在最短路上，当且仅当它到起点的距离加到终点的距离不比最好那个和大
    // （加一点余量，免得被逼进一条缝里）。走廊外的一切对 A* 都是墙。
    const SLACK: i32 = 2;
    let outside = |x: i32, y: i32| -> bool {
        let b = (y / bg.block_size) * bg.bw + (x / bg.block_size);
        !(d_from[b as usize] >= 0
            && d_to[b as usize] >= 0
            && d_from[b as usize] + d_to[b as usize] <= min_sum + SLACK)
    };
    let mut r = find_path_extra(g, from, to, &outside, -1);
    if !r.found {
        // 走廊太窄（比如一格宽的门）：退回全量。
        r = find_path(g, from, to);
        if !r.found {
            return false;
        }
    }
    *out = r.cells;
    true
}

// ============================================================================ 单测
// 先在算法层把行为钉死（性质测试），再接端到端与 C++ 对照。

#[cfg(test)]
mod tests {
    use super::*;

    /// `row[y]` 的第 x 个字符：`.` = 可走(1)，其它 = 不可走(0)。
    fn grid_of(rows: &[&str]) -> CostGrid {
        let h = rows.len() as i32;
        let w = rows[0].len() as i32;
        let mut cost = vec![0u8; (w * h) as usize];
        for (y, row) in rows.iter().enumerate() {
            for (x, ch) in row.chars().enumerate() {
                cost[y * w as usize + x] = if ch == '.' { 1 } else { 0 };
            }
        }
        CostGrid {
            w,
            h,
            cost,
            cut_corners: false,
        }
    }

    fn footprint(w: i32, h: i32, cut: bool) -> Footprint {
        Footprint {
            w,
            h,
            ox: 0,
            oy: 0,
            cut_corners: cut,
        }
    }

    #[test]
    fn heading_table() {
        assert_eq!(facing_octant(0, -1), 0); // 东北
        assert_eq!(facing_octant(1, -1), 32); // 东
        assert_eq!(facing_octant(1, 0), 64); // 东南
        assert_eq!(facing_octant(1, 1), 96); // 南
        assert_eq!(facing_octant(0, 1), 128); // 西南
        assert_eq!(facing_octant(-1, 1), 160); // 西
        assert_eq!(facing_octant(-1, 0), 192); // 西北
        assert_eq!(facing_octant(-1, -1), 224); // 北
        assert_eq!(facing_octant(0, 0), 0); // 不迈步

        // facingAngleOf：西北一带，与八分位一致
        let a = facing_angle_of(1.0, 0.0);
        assert!((a - 64.0).abs() < 1e-3, "a={a}");
        let b = facing_angle_of(0.0, -1.0);
        assert!(b < 1e-3 || (256.0 - b) < 1e-3, "b={b}");
    }

    #[test]
    fn a_star_finds_route_around_wall() {
        let g = grid_of(&[".....", "..##.", "..##.", "....."]);
        let r = find_path(&g, Cell::new(0, 0), Cell::new(4, 3));
        assert!(r.found);
        assert_eq!(r.cells.first().copied(), Some(Cell::new(0, 0)));
        assert_eq!(r.cells.last().copied(), Some(Cell::new(4, 3)));
        for c in &r.cells {
            assert!(g.walkable(c.x, c.y), "路径落在可走格上");
        }
        for w in r.cells.windows(2) {
            let dx = (w[1].x - w[0].x).abs();
            let dy = (w[1].y - w[0].y).abs();
            assert!(dx <= 1 && dy <= 1 && (dx + dy) > 0, "每步都是相邻格");
        }
    }

    #[test]
    fn a_star_refuses_corner_cut() {
        // 中心是墙，从 (0,0) 到 (2,2) 不能斜穿那个角
        let g = grid_of(&["...", ".#.", "..."]);
        let r = find_path(&g, Cell::new(0, 0), Cell::new(2, 2));
        assert!(r.found);
        assert!(r.cells.len() >= 3, "不许走 2 格的斜穿");
    }

    #[test]
    fn diagonal_rule_follows_cut_corners() {
        // 从 (1,0) 到 (2,1)：两翼是 (2,0)=通 与 (1,1)=墙
        let mut g = grid_of(&["...", ".#.", "..."]);
        g.cut_corners = false;
        assert!(!diagonal_allowed(&g, 1, 0, 2, 1), "车辆不许切角");
        g.cut_corners = true;
        assert!(diagonal_allowed(&g, 1, 0, 2, 1), "步兵一侧通即可切");
    }

    #[test]
    fn erode_blocks_through_one_cell_gap() {
        let g = grid_of(&[".....", "##.##", "....."]);
        // 2x2 过不去 1 格宽的缝
        let e2 = erode_grid(&g, &footprint(2, 2, false));
        assert!(e2.cost.iter().all(|&c| c == 0), "2x2 不该穿过 1 格缝");
        // 1x1 能过
        let e1 = erode_grid(&g, &Footprint::default());
        assert_eq!(e1.at(2, 1), 1, "1x1 能穿过缝");
    }

    #[test]
    fn erode_inherits_slowest_cell() {
        let mut g = grid_of(&["..", ".."]);
        g.cost[1] = 7; // (1,0) 是慢格
        let e = erode_grid(&g, &footprint(2, 2, false));
        assert_eq!(e.at(0, 0), 7, "整块继承最慢那一格");
    }

    #[test]
    fn multi_source_field_aims_at_nearest() {
        let g = grid_of(&[".........."]);
        let sources = vec![Cell::new(0, 0), Cell::new(9, 0)];
        let mut ff = FlowField::default();
        assert!(compute_flow_field_cpu_multi(&g, &sources, &mut ff));
        assert_eq!(ff.sources.len(), 2);
        // (1,0) 离左端近 ⇒ 指向左； (8,0) 离右端近 ⇒ 指向右
        assert_eq!(ff.dir_at(1, 0).0, -1);
        assert_eq!(ff.dir_at(8, 0).0, 1);
        // 距离 = 到最近那个种子
        assert!((ff.dist_at(4, 0) - 4.0).abs() < 1e-3);
        assert!((ff.dist_at(9, 0) - 0.0).abs() < 1e-3);
        assert_eq!(ff.is_source[0], 1);
        assert_eq!(ff.is_source[9], 1);
    }

    #[test]
    fn flow_field_direction_never_goes_uphill() {
        // 一张绕墙的场：每个可走格的最佳步都必须严格下降（除非它就是种子）
        let g = grid_of(&[".....", "..#..", ".....", "..#..", "....."]);
        let sources = vec![Cell::new(4, 4)];
        let mut ff = FlowField::default();
        assert!(compute_flow_field_cpu_multi(&g, &sources, &mut ff));
        for y in 0..g.h {
            for x in 0..g.w {
                if !g.walkable(x, y) {
                    continue;
                }
                let here = ff.dist_at(x, y);
                if here <= 1e-6 {
                    continue;
                }
                let (dx, dy) = ff.dir_at(x, y);
                assert!(dx != 0 || dy != 0, "非种子格必有方向");
                let nx = x + dx;
                let ny = y + dy;
                assert!(g.walkable(nx, ny), "方向指向可走格");
                assert!(
                    ff.dist_at(nx, ny) < here + 1e-3,
                    "步向不许上坡 ({x},{y})->({nx},{ny})"
                );
            }
        }
    }

    #[test]
    fn path_to_nearest_source_lands_on_seed() {
        let g = grid_of(&["........"]);
        let sources = vec![Cell::new(0, 0)];
        let mut ff = FlowField::default();
        assert!(compute_flow_field_cpu_multi(&g, &sources, &mut ff));
        let mut reached = Cell::new(-1, -1);
        let mut route = Vec::new();
        assert!(path_to_nearest_source(
            &g,
            &ff,
            Cell::new(5, 0),
            &mut reached,
            &mut route
        ));
        assert_eq!(reached, Cell::new(0, 0));
        assert_eq!(route.first().copied(), Some(Cell::new(5, 0)));
        assert_eq!(route.last().copied(), Some(Cell::new(0, 0)));
    }

    #[test]
    fn well_floor_takes_exactly_n_distinct_cells() {
        let g = grid_of(&[".............", ".............", "............."]);
        let f = Footprint::default();
        let fl = well_floor_cells(&g, &f, Cell::new(6, 1), 5, 0.0, 0.0);
        assert_eq!(fl.len(), 5, "碗底恰好 n 格");
        let mut s = fl.clone();
        s.sort_by(|a, b| a.x.cmp(&b.x).then(a.y.cmp(&b.y)));
        s.dedup();
        assert_eq!(s.len(), 5, "不能重复占格");
        for c in &fl {
            assert!(g.walkable(c.x, c.y));
        }
        // 中心格一定在里面（最近优先）
        assert!(fl.contains(&Cell::new(6, 1)));
    }

    #[test]
    fn well_floor_trims_on_the_escape_side() {
        // 一行走道：只给 3 格，朝向 +x（逃逸侧 = 更靠右），取到的是靠左（近侧）的三格
        let g = grid_of(&["........."]);
        let f = Footprint::default();
        let fl = well_floor_cells(&g, &f, Cell::new(4, 0), 3, 1.0, 0.0);
        assert_eq!(fl.len(), 3);
        // 取的是包含中心在内的最近三格
        let mut xs: Vec<i32> = fl.iter().map(|c| c.x).collect();
        xs.sort_unstable();
        assert_eq!(xs, vec![3, 4, 5]);
    }

    #[test]
    fn smooth_path_shortens_straight_run() {
        let g = grid_of(&["....", "....", "....", "...."]);
        let mut p = vec![
            Cell::new(0, 0),
            Cell::new(1, 1),
            Cell::new(2, 2),
            Cell::new(3, 3),
        ];
        smooth_path(&g, &mut p);
        assert_eq!(p.first().copied(), Some(Cell::new(0, 0)));
        assert_eq!(p.last().copied(), Some(Cell::new(3, 3)));
        assert_eq!(p.len(), 2, "直线段收敛成两端");
    }

    #[test]
    fn side_bias_leaves_open_ground_alone() {
        let g = grid_of(&[".....", ".....", ".....", ".....", "....."]);
        let out = side_bias_grid(&g, 1.0, 0.0, 10.0, 3, 0.5);
        assert_eq!(out.cost, g.cost, "开阔区（宽 > wMax）一格都不改");
    }

    #[test]
    fn side_bias_prices_narrow_gap_but_keeps_passability() {
        let g = grid_of(&["#########", ".........", ".........", ".........", "#########"]);
        let out = side_bias_grid(&g, 1.0, 0.0, 10.0, 3, 0.5);
        assert!(
            out.cost.iter().zip(g.cost.iter()).any(|(a, b)| a != b),
            "窄口里至少有格被提价"
        );
        for i in 0..g.cost.len() {
            assert_eq!(out.cost[i] > 0, g.cost[i] > 0, "可通行性绝不被改");
        }
    }

    #[test]
    fn no_bottleneck_in_open_field() {
        let g = grid_of(&[
            ".........",
            ".........",
            ".........",
            ".........",
            ".........",
        ]);
        let bs = detect_bottlenecks(&g, 2, 10, 2);
        assert!(bs.is_empty(), "开阔地里没有收窄点");
    }

    #[test]
    fn bottleneck_found_at_a_narrow_neck() {
        // 哑铃形：上下两片开阔区被中间一条一格的脖子连起来 —— 脖子那几格就是收窄点
        let g = grid_of(&[".....", "..#..", "..#..", "..#..", "....."]);
        let bs = detect_bottlenecks(&g, 2, 10, 2);
        assert!(!bs.is_empty(), "细脖子应被识别");
        assert!(bs.iter().any(|b| b.n >= 2), "至少有一个像样的收窄点");
    }

    #[test]
    fn formation_targets_are_distinct() {
        let g = grid_of(&[
            "..........",
            "..........",
            "..........",
            "..........",
            "..........",
        ]);
        let f = Footprint::default();
        let starts = vec![
            Cell::new(0, 0),
            Cell::new(0, 4),
            Cell::new(9, 0),
            Cell::new(9, 4),
        ];
        let mut targets: Vec<Cell> = Vec::new();
        assign_formation_targets(&g, &f, Cell::new(5, 2), &starts, &mut targets);
        assert_eq!(targets.len(), 4);
        let mut t = targets.clone();
        t.sort_by(|a, b| a.x.cmp(&b.x).then(a.y.cmp(&b.y)));
        t.dedup();
        assert_eq!(t.len(), 4, "每个单位必须拿到不同的目标格");
    }

    #[test]
    fn hierarchical_matches_plain_a_star_on_a_maze() {
        let g = grid_of(&[
            "..........",
            ".########.",
            "..........",
            ".########.",
            "..........",
        ]);
        let mut h = Vec::new();
        assert!(find_path_hierarchical(
            &g,
            Cell::new(0, 0),
            Cell::new(9, 4),
            &mut h,
            2
        ));
        let r = find_path(&g, Cell::new(0, 0), Cell::new(9, 4));
        assert!(r.found);
        assert_eq!(h.first().copied(), Some(Cell::new(0, 0)));
        assert_eq!(h.last().copied(), Some(Cell::new(9, 4)));
    }
}
