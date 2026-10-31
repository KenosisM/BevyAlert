# ORCA数值核心

来源为作者的 [Reciprocal n-body Collision Avoidance](https://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf)
（2011，特别是式6–8），以及原始 [RVO2 v2.0.2 Agent.cpp](https://github.com/snape/RVO2/blob/c96a6ec8328492bae0721e59774026a119a60b4a/src/Agent.cpp)。
固定提交为 `c96a6ec8328492bae0721e59774026a119a60b4a`，没有跟随可变main。

`orca.rs` 的单位约束及增量线性规划改写自该版本，保留
Copyright 2008 University of North Carolina at Chapel Hill。原作者为
Jur van den Berg、Stephen J. Guy、Jamie Snape、Ming C. Lin和Dinesh Manocha。
这些改写部分遵循 [Apache License 2.0](../licenses/RVO2-APACHE-2.0.txt)；完整原版
许可证随模块保留，不更改工程其他部分的许可。修改包括Rust/f64实现、有限性
检查、重合时的确定性方向、非互惠对象、工作缓冲复用及明确的无解诊断。

## 接口与数值约定

`HalfPlane` 表示 `normal · velocity >= offset`，构造时归一化。
`unit_constraint` 从同一时刻的位置、实际速度、各自半径和预测时域生成
约束；两半径相加，不用两倍最大半径。互惠对象承担一半责任，停靠、停止
或其他不主动反应的对象必须使用 `Responsibility::Full`。

相互接触或已经重叠时使用实际步长dt代替远期时域。截止圆向量恰为零时
优先沿相对位置的反方向分开；完全重合使用稳定ID得到相反轴，不读随机数。
初始重叠仍被标注。速度上限不足以在一个dt内分开时会报告无解，不宣称
初始穿透已经解决，也不更改位置或半径。

`Solver::solve(planes, preferred, max_speed, hard_count)` 在速度圆盘与所有
半平面的交集内寻找最接近期望速度的点。前 `hard_count` 个约束供静态边界
使用，软约束无解回退时仍受保护；当前模块不复制导航或地形几何。
内部缓冲保留容量，LP1使用法向上的规范锚点减少大切向坐标造成的消减误差。

平行方向阈值为 `1e-12`；最终可行性检查容差为速度单位的
`1e-8 * (1 + max_speed)`。成功解在求解结束后不旋转或裁剪。
结果区分 `Feasible`、带首个失败约束及硬约束标记的 `Infeasible`、明确输入
错误和数值失败，并返回违反数量、硬约束违反数及最大残差。
无解回退速度只是有界的尽力结果，调用者不能把它当作满足全部约束或无碰撞。

## 独立验证

- 可解析圆盘、角点、切线、平行/近乎平行约束、速度为零及不可行域。
- 对称对向与不承担避让责任的对象，使用独立相对轨迹最近距离检查；不同
  半径、零相对速度、完全重合和零截止向量均覆盖。
- 512个带可行见证的集合，与独立的线交点/圆交点/投影枚举比较最近解，并
  反转输入次序检查一致性。该枚举不使用增量LP算法。
- `tests/data/orca-rvo2-v2.0.2.csv` 保存264个原版C++参考用例的全部输入与
  输出：133个线性规划和131个双单位例。参考程序直接编译固定版本原始
  `Agent.cpp`、`KdTree.cpp`、`Obstacle.cpp`、`RVOSimulator.cpp`。
  浮点参考比较容差为 `2e-4 * (1 + max_speed)`，Rust解仍单独满足更严格的
  f64约束残差检查。

原版在用例135（已经重叠且截止向量恰为零）生成非有限结果。参考文件保留
这个反例；Rust对该例验证有限性、可行性和一步分离，不把NaN当作有效参考。

普通Rust测试不需要C++工具或下载。重新生成原版输出时，设置 `RVO2_SOURCE`
为该固定版本的本地源目录，以已有C++编译器编译驱动和上述四个原文件：

```text
c++ -O2 -I "$RVO2_SOURCE/src" tests/orca_reference.cpp "$RVO2_SOURCE/src/Agent.cpp" "$RVO2_SOURCE/src/KdTree.cpp" "$RVO2_SOURCE/src/Obstacle.cpp" "$RVO2_SOURCE/src/RVOSimulator.cpp" -o _scratch/orca-reference
_scratch/orca-reference < tests/data/orca-rvo2-v2.0.2.csv
```

驱动只解析输入并调用原版函数，不是另一份公式实现；已有期望输出位于行尾，
不会影响重新运行。构建及输出目录应由运行环境配置为本地忽略目录。
本模块尚未接入正式速度层，四群通行仍需避障接入、调度与恢复的实际验收。
