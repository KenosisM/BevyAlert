# ra2engine 索引

最后更新：2026-10-18

改代码前先在这里定位模块，再看它对应的文档。

## 工程约束

- 地形几何只有一份：`terrain::build_mesh`。`[Render] LogicMesh` 只决定贴不贴地形图集，不动几何。
- 取格子只有一份：`game::march_cell`（射线步进）+ `game::resolve`（侧壁滞回），对外是 `pick_cell`。
- 「当前指向格」只有一份状态：`game::CursorCell`。高亮、面板、下指令都读它，不许各处自己算。
- 同一个逻辑不许有两份实现，也不许两处常数手工同步。

## 地形渲染

| 项 | 说明 | 文档 |
|---|---|---|
| 路线 | 真 3D 高度场，保留 RA2 等距观感，不抄 Westwood 渲染工艺 | `渲染-地形3D化设计.md` |
| 几何 | 每格一个菱形柱，四角 = `level + ramp::corner_heights`，侧壁到 y=0 | `ramp-mesh-对照表.md` |
| 上色 | `LogicMesh=1` 顶点色 + `unlit`；`=0` 图集 + `AlphaMode::Mask` + 受光 | — |
| `DEPTH_EPS` | `1e-4`，沿光轴偏置破同层深度平局（俯视下看不见） | — |
| 接缝 | 三份成品都不把砖当几何画；缝来自三判据不一致 | `调研-地形接缝-三份成品实现.md` |
| TMP 结构 | 主图（顶面菱形）+ extra（立面/坡面），不需从砖图切 | `调研-框架模式与地形面拆分.md` |
| 补半格 | 只有 `Cliff` 家族且带 extra 的撑边格；判据是**对角双高**（北+西 / 北+东 / 南+东 / 南+西，两邻居同高）→ 把高台那半边菱形抬到邻居 `level`，再沿对角弦补一道竖直墙（上沿到高台、下沿落本格顶面）| — |

## 寻路

| 项 | 说明 | 文档 |
|---|---|---|
| 现状 | 重力碗：一张共享流场，一群车顺场掉下去被「压实」。**没有到达判定** | `寻路-重力碗移植方案.md` |
| 停摆机理 | 三条闸门（碗底格数=车数 / 平地只准往前 / 动量耗尽放弃）+ 一条不可恢复（全工程无重规划） | `调研-寻路替代-重力碗停摆机理与换法.md` |
| 替代 | A 补丁 / B 格位认领 / C 连续避障 / D 移植，四条的成本与天花板 | 同上 |
| 已知错配 | `play.rs:84` 的 `ap_budget` 按 1200 Hz 写，实际跑 64 Hz → 37.5 秒而非 2 秒 | 同上 §1.3 |

## 模块

| 模块 | 作用 |
|---|---|
| `mix.rs` | MIX 归档索引与读取 |
| `mapfile.rs` | `.map` 解析（IsoMapPack5 / 单位 / 建筑） |
| `tmp.rs` | TMP 砖解析（主菱形 + extra + z 面） |
| `theater.rs` | theater INI / TileSet / 全局砖号 |
| `costgrid.rs` | 砖 → `TerrainCell`（extra / ramp / level） |
| `tilefix.rs` | LAT / ramp 砖修正表 |
| `terrain.rs` | 地形几何 `build_mesh` + 砖图集 |
| `ramp.rs` | `RAMP_CORNERS` / `corner_heights` |
| `game.rs` | 引擎主体、相机、`pick_cell`、材质 |
| `debugview.rs` | 逻辑模式覆盖层：悬停黄框 / 信息面板 / 俯视按钮 |
| `world.rs` | 模拟世界 / `GroundField` |
| `nav.rs` | 共享通行与有向代价 / 最短路 / 流场 / 足迹 |
| `collision.rs` | 带半径连续扫掠、受限接触修正及安全停车诊断；见 [碰撞安全](COLLISION.md) |
| `spatial.rs` | 稳定邻居、动态接触对、缓冲复用与密度诊断；见 [空间查询](SPATIAL.md) |
| `formation.rs` / `local_nav.rs` | 跨集群独占停靠预约、入口配对与带半径近端路径；见 [目标区预约](FORMATIONS.md) |
| `nav_properties.rs` | 共享有向边、独立最短路基准及真实移动回归；见 [导航规则](NAVIGATION.md) |
| `simulation.rs` / `nav_headless.rs` | 窗口与无头共用的固定步长核心、采样事件和可复现输入；见 [模拟入口](SIMULATION.md) |
| `nav_scenario.rs` / `nav_demo.rs` | 四色四群同步演示、场景配置与机器可读基线；见 [四群演示](NAV_DEMO.md) |
| `play.rs` | 指令链 |
| `shp.rs` `hva.rs` `vxl.rs` `vxl_normals.rs` `voxel_mesh.rs` `vxl_gpu.rs` | 单位与建筑素材 → mesh |
| `lighting.rs` | 调色板按高度 / 太阳重算 |
| `viewer.rs` | VXL 查看器 |
| `png_out.rs` | 写 PNG |
| `ini.rs` `rules.rs` `vpl.rs` `mat4.rs` `compress.rs` `blowfish_tables.rs` | 基础设施 |

## 调试覆盖层

`LogicMesh=1` 时才注册（`debugview.rs`）。

- 悬停高亮：指向的格顶面染黄，替身 mesh 按 `(w,h,ramp)` 缓存，只挪 `Transform`
- **侧壁滞回**：射线打在悬崖竖面上时 `resolve` 保留上一格不动，只有真正指到某格顶面才换
  （否则等距投影下墙面占一大片屏幕，鼠标一动就跳格）
- 左上角面板：格坐标 / 层高 / ramp / terrain / TMP 文件名 / 砖集 / cell 在砖内偏移 / extra 矩形（相对 cell + 相对砖）
- 控制台：换格时打一行 `hover -> (rx,ry) L.. TMP xxx.tem cellN set ..`，验证不靠截图
- **验证方式**：查格子数据一律走无头 `costgrid --cell`（字段与 `console_line` 同一口径），
  不要为了看一个数字起窗口；`game` 只在需要看画面时起
- 俯视按钮：切俯视、再点切回；`game::cam_basis` 分两套基
- 中文字体读 `C:/Windows/Fonts/simsun.ttc`；必须开 `bevy/complex_script_segmentation`，否则刷 ICU4X 报错
- 光标落在面板矩形内不拾取、不下指令

## 命令

| 命令 | 作用 |
|---|---|
| `game` | 起游戏本体（参数全在 `rulesmd.ini`） |
| `costgrid` | 地图 → 可寻路网格；`--cell "rx,ry;rx,ry"` 探针打出 cell/extra 全字段（**无头验算入口**） |
| `tmpimg` | 导出 TMP 各 cell 图像 |
| `mapimg` | 俯视高度图 |
| `map` | 打印地图结构与 tile 字段 |
| `dump` / `mixprobe` | 从 MIX 导出 / 探字节 |
| `vxl` / `vgpu` / `view` | 体素相关 |

## 已删除

| 东西 | 理由 |
|---|---|
| `isoview.rs` + `isoshot` | 第二套地形建模 |
| `terrain_bake.rs` + `RA2_DUMP_ATLAS` + `plane_point` | 第三条渲染路径，不在链路 |
| `MeshOpts`（`RampTilt` / `WallToGround`） | 假开关，从没被读过 |
| 贴图模式「半格抬到邻居高度」补丁 | 几何必须严格来自地图 |
| `world::cell_at_world` | 第二套取格逻辑 |
| `RA2_TERRAIN_SOLID` 纯色模式 | 只留贴图一条材质分支 |
| `shaders/sprite_blit.wgsl` | 计划删 |

## 待办

1. `交互-缩放与单位方案.md`、`调研-ccmaps地形绘制.md` 未读，读完补进本索引。
2. `Surface` 设计文档第 5 节：模拟层是否整体重做未拍板。
3. 长时间不碰鼠标时相机偶发自行平移（`update_view` 的 `EDGE=24`），只在合成输入的验证脚本里出现过，待确认。
