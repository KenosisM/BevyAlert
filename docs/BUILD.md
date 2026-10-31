# Rust / Bevy 构建与依赖恢复

## 固定输入

| 输入 | 固定值 |
|---|---|
| Rust / Cargo | 1.97.1，`rust-toolchain.toml`；包声明 `rust-version = "1.97.1"` |
| 本次验证平台 | Windows x64，`x86_64-pc-windows-msvc` |
| 本次本机链接工具 | Visual Studio 2026 18.10.12201.205，MSVC 14.51.36231，Windows SDK 10.0.26100.0 |
| Bevy | 官方 `bevyengine/bevy`，`6d7f0b30939d7dcc24fdde113716d194997ea8d3`，包版本 `0.20.0-dev` |
| wgpu | `30.0.1`，与 Bevy 的 wgpu 主版本保持一致 |
| 其他包 | 仓库内 `Cargo.lock` 固定版本及 registry 校验和 |
| 中文分词 | 保留 `bevy/complex_script_segmentation` |

Bevy 的来源是[固定提交](https://github.com/bevyengine/bevy/commit/6d7f0b30939d7dcc24fdde113716d194997ea8d3)，其 [Cargo.toml](https://github.com/bevyengine/bevy/blob/6d7f0b30939d7dcc24fdde113716d194997ea8d3/Cargo.toml) 要求 Rust 1.97.1；[渲染依赖](https://github.com/bevyengine/bevy/blob/6d7f0b30939d7dcc24fdde113716d194997ea8d3/crates/bevy_render/Cargo.toml) 使用 wgpu 30。

原工程通过 `bevy = { path = ".." }` 使用父目录源码，锁文件中的 69 个 Bevy 包没有来源或 revision。迁移后的父目录没有 Bevy，无法证明旧机器使用的原始提交，也没有留下可还原的父目录补丁。这里依据现有 `0.20.0-dev` / wgpu 30 锁文件、2026-10-18 的源码调研和工程 API 选择上述同日官方快照作为恢复基线，不把它宣称为已找回的原始提交。

所有 Bevy 子包通过同一个 Git source 和完整 revision 获取，不再依赖父目录内容。不要改回本机 path，也不要对 Cargo 缓存打未保存的补丁。新增补丁应作为仓库内文件保存，注明上游 revision、原因和重放方法，再验证锁定构建。

依赖恢复保留了原锁文件所有 registry 包的版本和校验和；Cargo 仅补入该快照所需的 `bevy_annotations 0.20.0-dev` 及其在 `bevy_internal` 中的依赖边。最终锁文件含 583 个包，其中 70 个 Bevy 包来自同一 revision。

## Windows 环境准备

先获取工程，并切换到需要复现的工程提交；已有工作区时跳过克隆，保留未提交改动。提交应包含本说明、`Cargo.toml`、`Cargo.lock` 和 `rust-toolchain.toml`，不需要原机器的 Bevy 父目录。

```powershell
git clone https://github.com/AdrianLuo-X/BevyAlert.git 'D:\BevyAlert\BevyAlert'
```

需要 Git for Windows（下文使用 Git CLI 获取依赖）、Visual Studio 的“使用 C++ 的桌面开发”组件（MSVC x64/x86 工具）和 Windows SDK。Rust 的 MSVC 链接依赖这些组件；已有可用组件时不必重装。需要联网访问官方 Rust 下载站、GitHub、crates.io 索引及 crate 下载站。首次构建应预留足够的磁盘空间，`target` 和下载缓存均不提交。

本次 16 GB 机器并行优化编译 `bevy_ui` 和 `bevy_sprite_render` 时，可用物理内存降至约 0.8 GiB，完整冷构建建议使用 `--jobs 1`。共享构建入口在没有 `CARGO_BUILD_JOBS` 设置时默认单任务；可以用显式 `--jobs` 参数覆盖。此设置仅影响编译并行度，不改变编译优化等级、功能开关或模拟参数。

下面以源码位于 `D:\BevyAlert\BevyAlert`、工具位于仓库外的 `D:\BevyAlert\.tools` 为例。其他机器可替换这两个根路径。安装只需执行一次；已有 Rust 时也可使用已有 rustup 安装固定版本。安装目录变量必须在**同一个 PowerShell 进程**中设置；参见 [rustup 安装说明](https://rust-lang.github.io/rustup/installation/index.html#choosing-where-to-install)。

```powershell
Set-Location 'D:\BevyAlert\BevyAlert'
$bevyAlertTools = 'D:\BevyAlert\.tools'
New-Item -ItemType Directory -Force -Path "$bevyAlertTools\downloads" | Out-Null
$env:CARGO_HOME = "$bevyAlertTools\cargo"
$env:RUSTUP_HOME = "$bevyAlertTools\rustup"

$bevyAlertInstaller = "$bevyAlertTools\downloads\rustup-init.exe"
Invoke-WebRequest 'https://static.rust-lang.org/rustup/dist/x86_64-pc-windows-msvc/rustup-init.exe' -OutFile $bevyAlertInstaller
Invoke-WebRequest 'https://static.rust-lang.org/rustup/dist/x86_64-pc-windows-msvc/rustup-init.exe.sha256' -OutFile "$bevyAlertInstaller.sha256"
$bevyAlertExpected = (Get-Content "$bevyAlertInstaller.sha256" -Raw).Trim().Split()[0]
if ((Get-FileHash $bevyAlertInstaller -Algorithm SHA256).Hash -ne $bevyAlertExpected) {
    throw 'rustup-init SHA-256 mismatch'
}
& $bevyAlertInstaller -y --no-modify-path --profile minimal --default-host x86_64-pc-windows-msvc --default-toolchain 1.97.1
if ($LASTEXITCODE -ne 0) { throw 'Rust installation failed' }
```

这些设置仅作用于当前进程，不需要修改持久 PATH 或 ACL。已有隔离安装时跳过下载，重新设置下面的进程变量即可。

## 锁定获取、检查和测试

在每个新的 PowerShell 中，先核验身份与工具的真实路径，再使用完整路径执行：

```powershell
Set-Location 'D:\BevyAlert\BevyAlert'
whoami
$bevyAlertTools = 'D:\BevyAlert\.tools'
$env:CARGO_HOME = "$bevyAlertTools\cargo"
$env:RUSTUP_HOME = "$bevyAlertTools\rustup"
$bevyAlertBin = "$bevyAlertTools\rustup\toolchains\1.97.1-x86_64-pc-windows-msvc\bin"
$env:RUSTC = "$bevyAlertBin\rustc.exe"
$env:RUSTDOC = "$bevyAlertBin\rustdoc.exe"
$env:CARGO_NET_GIT_FETCH_WITH_CLI = 'true'
$bevyAlertCargo = "$bevyAlertBin\cargo.exe"

& "$bevyAlertTools\cargo\bin\rustup.exe" which rustc
& $env:RUSTC -vV
& $bevyAlertCargo -Vv
& $bevyAlertCargo fetch --locked
& $bevyAlertCargo check --locked --all-targets --jobs 1
& $bevyAlertCargo test --locked --no-run --jobs 1
& $bevyAlertCargo test --locked --jobs 1 nav::tests:: -- --nocapture
& $bevyAlertCargo test --locked --jobs 1 crowd::tests:: -- --nocapture
```

每条 Cargo 命令退出码应单独检查；前一条失败时先处理原因。`--locked` 拒绝隐式改写依赖解析结果。首次 `fetch` 获取固定 revision 的 Bevy 和锁文件内的 registry 包；不会获取 EA / Westwood 游戏素材。缓存完整后可用 `--frozen` 验证离线构建。不要通过 `cargo update` 或切换到 `stable` / Bevy 最新版绕过锁定失败。

`cargo check` 不执行最终代码生成和链接，所以还必须执行 `test --no-run`；仅 check 成功不代表测试目标能编译。需要运行其他现有测试时使用 `test --locked`。测试基线和限制见文末。

## 构建入口与运行素材

```powershell
& $bevyAlertCargo build --locked --jobs 1
& $bevyAlertCargo build --locked --release --jobs 1
```

`build.cmd`、`编译.bat` 和 `启动.bat` 共用同一个带 `--locked` 的构建入口，没有自动删除或清理产物的命令。入口依次识别 `BEVY_ALERT_CARGO` 指定路径、源码旁 `.tools` 中的隔离 rustup、`CARGO_HOME` 中的 rustup 和用户默认 rustup。使用其他安装时须先核验 Rust 版本；直接调用工具链内 cargo.exe 时还应像上例设置 `RUSTC` / `RUSTDOC`。

本地 `.tools` 目录先规范化为不含 `..` 的完整路径，再通过 `rustup which cargo/rustc/rustdoc` 分别取得该工程锁定工具链的原生可执行文件路径。这样既不依赖持久 PATH，也避免同一 Cargo 缓存被路径别名识别成重复包。

`启动.bat` 从工程目录运行 `target\debug\ra2engine.exe game`，仅在该 exe 不存在时构建；修改源码后先执行构建。它也接受拖入的 `.map` 作为地图参数。

实际游戏需要用户本地提供合法的 EA / Westwood 资产，并在 `rulesmd.ini` 配置 `GameDir` 和 `Map`；这些资产不在 GitHub，也不是无头 nav / crowd 单元测试的输入。当前配置仍为 `TickHz=60`、`Crowd=0`。中文界面还使用 Windows 的 `C:/Windows/Fonts/simsun.ttc`。构建和单元测试通过不等于已验证真实地图或 GPU 窗口运行。

## 验证基线

验证日期：2026-10-30 至 2026-10-31（Asia/Shanghai）。机器为 i5-1235U、16 GB 内存，工具版本见上表。

| 实际命令 | 结果 |
|---|---|
| `cargo check --locked --all-targets --jobs 2` | 通过，冷检查 25 分 38 秒 |
| `cargo test --frozen --no-run --jobs 2` | 通过，冷编译及链接 77 分 10 秒，生成测试 exe |
| `cargo test --frozen --jobs 1 -- --nocapture` | 37 通过、0 失败、0 忽略，测试执行 0.14 秒 |
| `build.cmd --offline`（入口默认单任务） | 通过，等价 `cargo build --locked --offline`；Cargo 报告 127 分 25 秒，生成普通 `target/debug/ra2engine.exe` |
| `target/debug/ra2engine.exe costgrid` | 退出 0，输出用法；无需游戏资产，不打开窗口 |
| `cargo metadata --frozen --no-deps --format-version 1` | 通过；工作区仅本工程，Bevy 来源为固定 Git revision，没有父目录 path 依赖 |

正式测试包含 nav 18 项、crowd 2 项、game 9 项、world 8 项。未改导航源码的独立 Rust 对照中，nav / crowd 的同 20 项测试也全部通过；恢复后的基线没有现有测试失败或新增测试失败。本次不修改 `src`、着色器或 `rulesmd.ini`，不新增或更改寻路、碰撞、停车算法。

所选官方 Bevy 源码可直接编译本工程，不需要额外源码补丁。正常二进制检查仍有 66 条、测试目标有 52 条既有源码警告，主要为 unused / dead_code，未用批量改写或隐藏警告来处理。

这些是当前源码的构建与单元测试基线，不代表四群交叉、千单位通行或真实地图行为已经验收。未执行本地游戏资产与 GPU 窗口验证。
