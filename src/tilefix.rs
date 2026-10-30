//! LAT / ramp 修正 —— 对等移植。
//!
//! 来源：`I:\bevy\RA2REASM\src\compat\tilefix.h` / `tilefix.cpp`。
//!
//! 这是 `CellClass` 重新算 LAT 的移植，参考实现在地图加载后立刻跑它。**不跑的话**，
//! 一格会画地图里字面存着的那张砖 —— 对每一个过渡砖和坡道砖都是错的，引擎会按邻居改写
//! TileNum。上游拿参考实现的 `--debug-tiles` 量过：跳过它会让 **0.6% ~ 10.8%** 的格子画错砖
//! （城市图最惨，因为用得最多）。
//!
//! 三趟，按此顺序，因为后面的趟要读前面趟的结果：
//!   1. 一块"连接型 LAT"变成它那种地形的普通 LAT 砖
//!   2. 四周同地形的 LAT 砖变成对应的 CLAT 块
//!   3. 斜向挨着平地的坡道砖变成平滑坡道块

/// `[General]` 里 LAT 与 ramp 修正所依据的砖 id。全部默认 -1，
/// 读作"这个 theater 没有这个 set"，从而关掉对应规则。
#[derive(Debug, Clone)]
pub struct TileFixupIds {
    pub rough_tile: i32,
    pub sand_tile: i32,
    pub green_tile: i32,
    pub pave_tile: i32,
    pub blue_mold_tile: i32,
    pub crystal_tile: i32,
    pub swamp_tile: i32,

    pub clear_to_rough_lat: i32,
    pub clear_to_sand_lat: i32,
    pub clear_to_green_lat: i32,
    pub clear_to_pave_lat: i32,
    pub clear_to_blue_mold_lat: i32,
    pub clear_to_crystal_lat: i32,
    pub water_to_swamp_lat: i32,

    pub crystal_cliff: i32,
    pub shore_pieces: i32,
    pub water_bridge: i32,
    pub paved_roads: i32,
    pub medians: i32,
    pub misc_pave_tile: i32,

    pub ramp_base: i32,
    pub ramp_smooth: i32,
}

impl Default for TileFixupIds {
    fn default() -> Self {
        TileFixupIds {
            rough_tile: -1,
            sand_tile: -1,
            green_tile: -1,
            pave_tile: -1,
            blue_mold_tile: -1,
            crystal_tile: -1,
            swamp_tile: -1,
            clear_to_rough_lat: -1,
            clear_to_sand_lat: -1,
            clear_to_green_lat: -1,
            clear_to_pave_lat: -1,
            clear_to_blue_mold_lat: -1,
            clear_to_crystal_lat: -1,
            water_to_swamp_lat: -1,
            crystal_cliff: -1,
            shore_pieces: -1,
            water_bridge: -1,
            paved_roads: -1,
            medians: -1,
            misc_pave_tile: -1,
            ramp_base: -1,
            ramp_smooth: -1,
        }
    }
}

/// `FixTiles` 读写的全部输入。网格是 `(2*width - 1) * height` 格，索引 `dx * height + row`，
/// 且 dx/dy 同奇偶。
pub struct TileFixInput<'a> {
    pub width: i32,
    pub height: i32,
    /// 就地改写
    pub tile_num: &'a mut Vec<i32>,
    /// 每格的砖图 RampType
    pub ramp: &'a [u8],
    pub set_first_tile: &'a [i32],
    pub tile_to_set: &'a [i32],
    pub ids: &'a TileFixupIds,
}

/// 八个方向，名字与参考实现一致。
#[derive(Clone, Copy, PartialEq, Eq)]
enum Dir {
    Top,
    TopLeft,
    TopRight,
    Left,
    Right,
    BottomLeft,
    Bottom,
    BottomRight,
}

struct Fixer<'a> {
    tile_num: &'a mut Vec<i32>,
    ramp: &'a [u8],
    set_first_tile: &'a [i32],
    tile_to_set: &'a [i32],
    ids: &'a TileFixupIds,
    cols: i32,
    rows: i32,
    /// 第 1 趟之后每格属于哪个 set。第 2 趟会改写 TileNum 但**绝不**动它，
    /// 这正是邻居判定保持稳定的原因。
    set: Vec<i32>,
}

impl<'a> Fixer<'a> {
    fn new(inp: TileFixInput<'a>) -> Fixer<'a> {
        Fixer {
            cols: 2 * inp.width - 1,
            rows: inp.height,
            tile_num: inp.tile_num,
            ramp: inp.ramp,
            set_first_tile: inp.set_first_tile,
            tile_to_set: inp.tile_to_set,
            ids: inp.ids,
            set: Vec::new(),
        }
    }

    fn at(&self, x: i32, y: i32) -> usize {
        x as usize * self.rows as usize + y as usize
    }
    fn valid(&self, x: i32, y: i32) -> bool {
        x >= 0 && y >= 0 && x < self.cols && y < self.rows
    }

    fn tile_num_at(&self, x: i32, y: i32) -> i32 {
        self.tile_num[self.at(x, y)]
    }
    fn ramp_at(&self, x: i32, y: i32) -> u8 {
        self.ramp[self.at(x, y)]
    }

    fn set_of(&self, tile_num: i32) -> i32 {
        if tile_num < 0 || tile_num as usize >= self.tile_to_set.len() {
            return -1;
        }
        self.tile_to_set[tile_num as usize]
    }

    /// 某个 set 的第 `index` 个全局砖号；set 不存在时 -1。
    fn tile_of(&self, set: i32, index: i32) -> i32 {
        if set < 0 || set as usize >= self.set_first_tile.len() || index < 0 {
            return -1;
        }
        self.set_first_tile[set as usize] + index
    }

    /// 斜向邻居用参考实现同样的奇偶技巧：水平相邻的一格，dy 随 dx 的奇偶 ±1。
    fn neighbour(&self, x: i32, y: i32, dir: Dir) -> Option<(i32, i32)> {
        let (nx, ny) = match dir {
            Dir::Bottom => (x, y + 1),
            Dir::Top => (x, y - 1),
            Dir::Left => (x - 2, y),
            Dir::Right => (x + 2, y),
            _ => {
                let yy = y + (x & 1);
                match dir {
                    Dir::BottomLeft => (x - 1, yy),
                    Dir::BottomRight => (x + 1, yy),
                    Dir::TopLeft => (x - 1, yy - 1),
                    _ => (x + 1, yy - 1), // TopRight
                }
            }
        };
        if self.valid(nx, ny) {
            Some((nx, ny))
        } else {
            None
        }
    }

    /// 邻居的 set 号，没有邻居时 -1。
    ///
    /// 这里**必须**读第 1 趟记录的 set，而不是从邻居的 TileNum 重新推：第 2 趟会把 TileNum
    /// 改写成 CLAT 块却刻意不动 set，重新推就会给一个"本轮其余部分仍视作地形"的格子报出 CLAT set。
    /// 那个错误会让每一格地形都像是挨着过渡块，并让掩码依赖遍历顺序。
    fn neighbour_set(&self, x: i32, y: i32, dir: Dir) -> i32 {
        match self.neighbour(x, y, dir) {
            Some((nx, ny)) => self.set[self.at(nx, ny)],
            None => -1,
        }
    }

    fn neighbour_tile_num(&self, x: i32, y: i32, dir: Dir) -> i32 {
        match self.neighbour(x, y, dir) {
            Some((nx, ny)) => self.tile_num_at(nx, ny),
            None => -1,
        }
    }

    fn neighbour_flat(&self, x: i32, y: i32, dir: Dir) -> bool {
        // 图外的邻居算作平地，就像游戏里的空白格。
        match self.neighbour(x, y, dir) {
            Some((nx, ny)) => self.ramp_at(nx, ny) == 0,
            None => true,
        }
    }

    // --- 规则，全部是 TileCollection 谓词的直译 ------------------------------------
    fn is_lat(&self, set: i32) -> bool {
        let g = self.ids;
        set != -1
            && (set == g.rough_tile
                || set == g.sand_tile
                || set == g.green_tile
                || set == g.pave_tile
                || set == g.blue_mold_tile
                || set == g.crystal_tile
                || set == g.swamp_tile)
    }

    fn is_clat(&self, set: i32) -> bool {
        let g = self.ids;
        set != -1
            && (set == g.clear_to_rough_lat
                || set == g.clear_to_sand_lat
                || set == g.clear_to_green_lat
                || set == g.clear_to_pave_lat
                || set == g.clear_to_blue_mold_lat
                || set == g.clear_to_crystal_lat
                || set == g.water_to_swamp_lat)
    }

    fn lat_of(&self, clat: i32) -> i32 {
        let g = self.ids;
        if clat == g.clear_to_rough_lat {
            return g.rough_tile;
        }
        if clat == g.clear_to_sand_lat {
            return g.sand_tile;
        }
        if clat == g.clear_to_green_lat {
            return g.green_tile;
        }
        if clat == g.clear_to_pave_lat {
            return g.pave_tile;
        }
        if clat == g.clear_to_blue_mold_lat {
            return g.blue_mold_tile;
        }
        if clat == g.clear_to_crystal_lat {
            return g.crystal_tile;
        }
        if clat == g.water_to_swamp_lat {
            return g.swamp_tile;
        }
        -1
    }

    fn clat_of(&self, lat: i32) -> i32 {
        let g = self.ids;
        if lat == g.rough_tile {
            return g.clear_to_rough_lat;
        }
        if lat == g.sand_tile {
            return g.clear_to_sand_lat;
        }
        if lat == g.green_tile {
            return g.clear_to_green_lat;
        }
        if lat == g.pave_tile {
            return g.clear_to_pave_lat;
        }
        if lat == g.blue_mold_tile {
            return g.clear_to_blue_mold_lat;
        }
        if lat == g.crystal_tile {
            return g.clear_to_crystal_lat;
        }
        if lat == g.swamp_tile {
            return g.water_to_swamp_lat;
        }
        -1
    }

    fn connect_tiles(&self, a: i32, b: i32) -> bool {
        if a == -1 || b == -1 || a == b {
            return false;
        }
        let g = self.ids;
        // 草地不连岸线或水上桥；铺装不连铺装路、隔离带或杂铺装。其余的都通过一个 CLAT set 相接。
        if (a == g.green_tile && b == g.shore_pieces) || (b == g.green_tile && a == g.shore_pieces) {
            return false;
        }
        if (a == g.green_tile && b == g.water_bridge) || (b == g.green_tile && a == g.water_bridge) {
            return false;
        }
        if (a == g.pave_tile && b == g.paved_roads) || (b == g.pave_tile && a == g.paved_roads) {
            return false;
        }
        if (a == g.pave_tile && b == g.medians) || (b == g.pave_tile && a == g.medians) {
            return false;
        }
        if (a == g.pave_tile && b == g.misc_pave_tile)
            || (b == g.pave_tile && a == g.misc_pave_tile)
        {
            return false;
        }
        true
    }

    /// 上游 `Fixer::run`（tilefix.cpp:153-245）。
    fn run(&mut self) {
        let g = self.ids;
        self.set = vec![-1; (self.cols * self.rows) as usize];

        // --- 第 1 趟：一块"连接型 LAT"变成它那种地形的普通 LAT 砖 ----------------------
        for x in 0..self.cols {
            for y in 0..self.rows {
                let tn = self.tile_num_at(x, y);
                let mut s = self.set_of(tn);
                if self.is_clat(s) {
                    s = self.lat_of(s);
                    let repl = self.tile_of(s, 0);
                    if repl >= 0 {
                        let i = self.at(x, y);
                        self.tile_num[i] = repl;
                    }
                }
                let i = self.at(x, y);
                self.set[i] = s;
            }
        }

        // --- 第 2、3 趟，走同一次遍历 ------------------------------------------------
        for x in 0..self.cols {
            for y in 0..self.rows {
                let s = self.set[self.at(x, y)];

                if self.is_lat(s) {
                    let mut transition = 0;
                    if self.connect_tiles(s, self.neighbour_set(x, y, Dir::TopRight)) {
                        transition += 1;
                    }
                    if self.connect_tiles(s, self.neighbour_set(x, y, Dir::BottomRight)) {
                        transition += 2;
                    }
                    if self.connect_tiles(s, self.neighbour_set(x, y, Dir::BottomLeft)) {
                        transition += 4;
                    }
                    if self.connect_tiles(s, self.neighbour_set(x, y, Dir::TopLeft)) {
                        transition += 8;
                    }

                    // 水晶 LAT 改成连四块特定的水晶崖。
                    if g.crystal_tile != -1 && s == g.crystal_tile {
                        const PROBES: [(Dir, i32); 4] = [
                            (Dir::TopRight, 1),
                            (Dir::BottomRight, 4),
                            (Dir::BottomLeft, 0),
                            (Dir::TopLeft, 5),
                        ];
                        for (dir, index) in PROBES {
                            let ns = self.neighbour_set(x, y, dir);
                            if ns == g.crystal_cliff
                                && self.neighbour_tile_num(x, y, dir) == self.tile_of(ns, index)
                            {
                                transition = 0;
                                break;
                            }
                        }
                    }

                    // 沼泽的 LAT set 有 9 块砖而不是 1 块，同时兼作普通 set。
                    if g.swamp_tile != -1 && s == g.swamp_tile && self.tile_num_at(x, y) > self.tile_of(s, 0)
                    {
                        transition = 0;
                    }

                    if transition > 0 {
                        let repl = self.tile_of(self.clat_of(s), transition);
                        // set **刻意不更新**：后面的格子仍须把这一格看作它自己的地形，
                        // 而不是它现在画出来的那个 CLAT 块。
                        if repl >= 0 {
                            let i = self.at(x, y);
                            self.tile_num[i] = repl;
                        }
                    }
                    continue;
                }

                // --- 第 3 趟：斜向挨着平地的坡道变成平滑块 ----------------------------
                if g.ramp_base == -1 && g.ramp_smooth == -1 {
                    continue;
                }
                if s != g.ramp_base && s != g.ramp_smooth {
                    continue;
                }
                let ramp = self.ramp_at(x, y) as i32;
                if ramp < 1 || ramp > 4 {
                    continue;
                }

                let mut fixup = -1;
                match ramp {
                    1 => {
                        // 朝西北
                        if self.neighbour_flat(x, y, Dir::TopLeft) {
                            fixup += 1;
                        }
                        if self.neighbour_flat(x, y, Dir::BottomRight) {
                            fixup += 2;
                        }
                    }
                    2 => {
                        // 朝东北
                        if self.neighbour_flat(x, y, Dir::TopRight) {
                            fixup += 1;
                        }
                        if self.neighbour_flat(x, y, Dir::BottomLeft) {
                            fixup += 2;
                        }
                    }
                    3 => {
                        // 朝东南
                        if self.neighbour_flat(x, y, Dir::BottomRight) {
                            fixup += 1;
                        }
                        if self.neighbour_flat(x, y, Dir::TopLeft) {
                            fixup += 2;
                        }
                    }
                    _ => {
                        // 朝西南
                        if self.neighbour_flat(x, y, Dir::BottomLeft) {
                            fixup += 1;
                        }
                        if self.neighbour_flat(x, y, Dir::TopRight) {
                            fixup += 2;
                        }
                    }
                }

                let repl = if fixup != -1 {
                    self.tile_of(g.ramp_smooth, (ramp - 1) * 3 + fixup)
                } else {
                    self.tile_of(g.ramp_base, ramp - 1)
                };
                if repl >= 0 {
                    let i = self.at(x, y);
                    self.tile_num[i] = repl;
                }
            }
        }
    }
}

/// 上游 `fixTiles`（tilefix.cpp:249-254）。
pub fn fix_tiles(inp: TileFixInput<'_>) {
    if inp.width <= 0 || inp.height <= 0 {
        return;
    }
    let mut f = Fixer::new(inp);
    f.run();
}
