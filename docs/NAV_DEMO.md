# 四群演示和可复现场景

`nav-demo` 使用 `nav_scenario::populate` 创建窗口与无头运行共用的状态、
指令和固定步长核心。场景不依赖游戏素材。

```text
cargo run --locked -- nav-demo --config scenarios/crossing.ini
cargo run --locked -- nav-demo --config scenarios/head-on.ini
cargo run --locked -- nav-demo --config scenarios/single.ini --per-group 1024
```

平地为 192×160 格，四群起点依次为 A(32,40)、B(32,120)、C(160,40)、
D(160,120)，A/D 和 B/C 交换位置。同高度对穿与单群场景位于 y=80。
所有群体在 tick 0 收到命令，不添加出发延迟。半径沿用模拟参数的 0.45 格，
速度沿用单位的 6 格/秒；固定 60 Hz，初始间距 1.1 格。

窗口操作：

| 按键 | 行为 |
|---|---|
| Space | 暂停或恢复 |
| N | 暂停并推进一个固定 tick |
| R | 按原配置、种子、命令和位置重置 |
| 1 / 2 / 3 | 每群 64 / 256 / 1024 个单位，并重置 |
| F1 / F2 / F3 | 四群交叉 / 同高度对穿 / 单群，并重置 |
| Tab | 连续旧行为与离散旧行为对照，并重置 |
| S | 保存本次采样与末态；默认写入 `_scratch/nav-demo.csv` |
| Esc | 关闭窗口 |

`--paused` 可从暂停状态启动。颜色 A 红、B 蓝、C 绿、D 金。
窗口显示实际模式、模拟 tick、渲染 FPS、活动指令、移动单位、旧停滞计数和
采样耗时。完整到达判定及等待原因尚未实现，明确显示 `NOT READY`。
`max still` 是旧计数器提供的连续静止时间，不是后续路线进度诊断。

无头基线与重放：

```text
cargo run --locked -- nav-demo --headless --config scenarios/crossing.ini --ticks 3600 --csv _scratch/crossing.csv
cargo run --locked -- nav-demo --headless --scene crossing --per-group 256 --seed 42 --jitter 0.02 --ticks 3600 --csv _scratch/crossing-seed42.csv
cargo run --locked -- nav-demo --headless --config _scratch/crossing-seed42.scene.ini --ticks 3600 --csv _scratch/replay.csv
```

`Jitter=0` 为完全对称布局；`--jitter 0.02` 在两个坐标轴分别添加 ±0.01 格的
固定种子扰动。命令行选项覆盖 INI。CSV 路径由 `--csv` 指定；同时生成同名
`.scene.ini` 复现配置和 `.units.csv` 末态。`--sample-every` 指定采样间隔。
GUI 也支持 `--csv`，可用 `--exit-after-ticks` 在指定模拟 tick 保存并退出，
便于与无头末态比较。

窗口可用 `--capture _scratch/nav-demo.png` 导出自身渲染目标的首个稳定画面，
`--exit-after-frames 120` 便于有界的渲染检查，暂停时也可使用。导出只包含
应用自身画面；运行产物继续保存在本地输出目录。

CSV 的 digest 覆盖稳定单位顺序、位置、速度和旧停滞状态，排除观察计时。
相同构建、配置和 tick 数可比较 digest 及末态；计时列不要求逐字相同。
`overlap_end` 只测量采样时刻的圆盘重叠，`bad_centers` 只测量中心所在格；
这两项本身不代表连续轨迹安全；正式连续模式另输出每步圆盘轨迹和接触诊断，
见 [碰撞安全](COLLISION.md)。到达及等待原因列使用 `not-ready`。

当前场景保留真实旧行为，重叠和停车缺陷见 [初始基线](NAV_BASELINE.md)。
所有命令同时发出，缺陷随观测结果保留，作为后续修复的对照。
性能验收使用后续 Release 全量运行；这里的 Dev 计时只用于调试。
运行产物、录像、日志和游戏素材不进入仓库，`_scratch` 已被忽略。
