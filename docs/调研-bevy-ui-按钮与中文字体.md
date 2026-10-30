# 调研：Bevy UI 按钮 + 中文文本（0.20.0-dev 本地源码）

> 用途：以后每次要在 ra2engine 里加 UI（按钮 / 面板 / 文本）都先看这一篇，不要再翻源码重新踩一遍。
> 所有结论都读的是 `I:/bevy/crates/**` 的本地源码（bevy `0.20.0-dev`），不是网上教程。
> 标 ⚠️ 的都是「照旧版本写法会直接编译不过 / 静默画不出来」的坑。

---

## 0. 版本前提

- `I:/bevy` 整个目录就是 bevy 本体（workspace 根），`ra2engine` 以 `bevy = { path = ".." }` 依赖它。
- 默认 features：`["2d", "3d", "ui", "audio"]`，其中 `ui` = `default_app + default_platform + ui_api + ui_bevy_render + scene + picking + bevy_ui_widgets`。
- `DefaultPlugins` 里已经包含：`bevy_ui::UiPlugin`、`bevy_ui_render::UiRenderPlugin`、`bevy_text::TextPlugin`、`bevy_ui_widgets::UiWidgetsPlugins`、以及 `bevy_picking::DefaultPickingPlugins`（`bevy_picking` feature 下）。
- **结论：加 UI 不需要手动 `add_plugins`，`DefaultPlugins` 一条全包。**

---

## 1. 按钮

### 1.1 ⚠️ 旧写法已废弃

`bevy::ui::widget::Button` 与 `bevy::ui::Interaction` 在 0.20 已标记废弃：

- `bevy_ui/src/widget/button.rs`：
  ```rust
  pub(crate) struct DeprecatedButton;          // pub(crate)，外部根本用不了
  pub type Button = DeprecatedButton;          // deprecated since 0.20.0
  ```
- `bevy_ui/src/lib.rs:61-66`：那段 `pub use crate::{widget::Button, Interaction}` 整块被 `#[expect(deprecated)]` 包着，且注明 "Should be removed after 0.20 is released"。
- 全仓 `grep "pub enum Interaction"` **零结果** —— `Interaction` 类型已经不存在了，只剩两个标记组件：`bevy_ui::Pressed`、`bevy_ui::InteractionDisabled`。

### 1.2 现在的写法

用 `bevy_ui_widgets::Button`（`bevy::ui_widgets::Button`，re-export 见 `bevy_internal/src/lib.rs:113-114`）：

```rust
use bevy::prelude::*;
use bevy::ui_widgets::{Activate, Button};

commands.spawn((
    Button,                              // 无字段，标记组件
    Node {
        position_type: PositionType::Absolute,
        left: Val::Px(10.0),
        top: Val::Px(10.0),
        padding: UiRect::axes(Val::Px(10.0), Val::Px(4.0)),
        ..Node::DEFAULT
    },
    BackgroundColor(Color::srgb(0.15, 0.15, 0.18)),
    children![(
        Text::new("俯视图"),
        TextFont { font: font_handle.into(), font_size: 16.0.into(), ..default() },
        TextColor(Color::WHITE),
    )],
));
```

`Button` 的 required components 只有 `AccessibilityNode(Role::Button)` + `Focusable`（`bevy_ui_widgets/src/button.rs:27-31`）——

⚠️ **它不 require `Node`**。旧 `DeprecatedButton` 才 `#[require(Node, FocusPolicy::Block)]`。新 `Button` 上必须自己挂 `Node`，否则不参与布局、也不参与 picking，点了没反应。

### 1.3 点击事件

按钮内部逻辑（`bevy_ui_widgets/src/button.rs`）：
- 收到 `PointerPress` → 插入 `Pressed`
- 收到 `PointerRelease` → 移除 `Pressed`
- 收到 `PointerClick` 且仍然 `Pressed` → `commands.trigger(Activate { entity })`
- 键盘 `Enter` / `Space`（`FocusedInput<KeyboardInput>`）→ 同样 trigger `Activate`

`Activate` 定义在 `bevy_ui_widgets/src/lib.rs:129`，唯一字段 `entity: Entity`。

接的两种写法：

```rust
// A. 全局 observer（本项目用这种，按钮只有一个）
app.add_observer(|activate: On<Activate>, mut view: ResMut<View>| {
    // 注意：Activate 会在**所有** Button 上触发，多按钮时要按 entity 区分
    view.top_down = !view.top_down;
});
```

```rust
// B. 挂在实体上（多按钮时用）
commands.spawn(...).observe(|_: On<Activate>, ...| { ... });
```

⚠️ `On<Activate>` 的 `On` 来自 `bevy_ecs::observer::On`，在 `bevy::prelude` 里。

### 1.4 picking 为什么能打到 UI

- `bevy_ui::UiPlugin` 在 `bevy_picking` feature 下自动 `add_plugins(UiPickingPlugin)`（`bevy_ui/src/lib.rs:190-191`）。
- `DefaultPlugins` 里 `bevy_picking::DefaultPickingPlugins` 也是 `#[cfg(feature = "bevy_picking")]` 加上的（`bevy_internal/src/default_plugins.rs:102-103`）。
- 所以 **只要用了 `DefaultPlugins` 且默认 features 没砍，按钮点得到**，不需要任何手动配置。

### 1.5 按钮悬停/按下的视觉反馈

没有内置样式。要自己写 observer 改 `BackgroundColor`：

```rust
app.add_observer(|over: On<Pointer<Over>>, q: Query<&mut BackgroundColor>, mut c: Commands| { ... });
```

（本项目暂不做，按钮只有一个，先保证功能。）

---

## 2. 中文文本

### 2.1 ⚠️ 文本栈换了

0.20 的 `bevy_text` 已经是 **Parley + fontique**（`bevy_text/src/parley_context.rs`），不再是 cosmic-text。字体注册、family 解析、fallback 全走 Parley。

### 2.2 结构体字段变了

```rust
pub struct TextFont {
    pub font: FontSource,      // ⚠️ 不再是 Handle<Font>
    pub font_size: FontSize,   // ⚠️ 不再是 f32
    pub weight: FontWeight,
    pub width: FontWidth,
    ...
}
```
（`bevy_text/src/text.rs:702-726`）

- `FontSize` 是 enum：`Px(f32) / Vw / Vh / VMin / VMax / Rem`，有 `impl From<f32>`（= `Px`）。
  所以 `font_size: 16.0.into()` 或者 `TextFont::from_font_size(16.0)`。
- `FontSource` 是 enum（`text.rs:284-306`）：`Handle(Handle<Font>) | Family(SmolStr) | Families(SmolStr) | List(Vec<..>) | Generic(GenericFontFamily) | Default`。
  有便捷构造：`FontSource::family("SimSun")`、`FontSource::families("Arial, 'Noto Sans', sans-serif")`、`FontSource::list([..])`。

### 2.3 ⚠️ 不能靠 `FontSource::family("SimSun")` 直接找到系统字体

`bevy_text/Cargo.toml:13`：
```toml
system_font_discovery = ["parley/system"]
```
这是 bevy_text 的可选 feature，**默认没开**（`parley` 依赖是 `default-features = false, features = ["std"]`，见同文件 :44）。
没开时 `FontSource::Generic` 会 `error_once!` 报 "the `system_font_discovery` feature is not enabled"（`text.rs:496-499`），`Family("SimSun")` 也只是在**空**的 collection 里查，查不到 → 文本不渲染。

### 2.4 本项目走的路：读系统 `.ttc` 的字节，当一个 Font 资产加进去

```rust
use bevy::prelude::*;

fn load_cjk_font(mut fonts: ResMut<Assets<Font>>) -> Option<Handle<Font>> {
    let path = "C:/Windows/Fonts/simsun.ttc";
    let bytes = std::fs::read(path).ok()?;
    Some(fonts.add(Font::from_bytes(bytes)))
}
```

原理链（`bevy_text/src/font.rs:54-103`）：
1. `Assets<Font>` 里新增资产
2. `load_font_assets_into_font_collection` 系统把每个 Font 的 `data` 交给 `font_cx.collection.register_fonts(...)`
3. 注册时**注册两份**：一份用字体自带的 family 名，一份用 `alias = "asset_id:..."`（`font.rs:82-102`）
4. `FontSource::Handle(h)` 解析成 `FontFamilyName::Named(alias)`（`text.rs:463-471`）

所以 `FontSource::Handle(handle)` 一定能解析到刚加进去的那个 face，不依赖任何 feature 开关。

`Font::from_bytes` 签名（`font.rs:41`）：`pub fn from_bytes(font_data: Vec<u8>) -> Font`。

### 2.5 ⚠️ `.ttc` 是字体集合

`simsun.ttc` 里至少有 SimSun（宋体）和 NSimSun（新宋体）两个 face。
fontique 的 `register_fonts(data, None)` 会遍历集合里的**每一个** face 并各自按内嵌 family 名注册，所以：
- 用 `Handle` 引用 → 走 alias，指向**整个数据块**，Parley 内部按 face 枚举，能拿到宋体；
- 想精确指定 face，用 `FontSource::family("SimSun")`（前提是该 Font 资产已经 add 过，collection 里已经有这个名字）。

### 2.5b ⚠️ 中文一出现就刷 `ICU4X data error: No segmentation model for complex script: Chinese/Japanese`

**症状**：窗口一显示中文 UI，stderr 每帧刷这行（一句提示都不带别的），几十行/秒。

**根因**（逐层读过）：

| 层 | 事实 |
|---|---|
| `bevy_text/Cargo.toml:15` | 有开关 `complex_script_segmentation = ["parley/complex-scripts"]`，**默认不开** |
| `bevy_text/Cargo.toml:44` | `parley = { version = "0.11", default-features = false, features = ["std"] }` |
| `parley-0.11.1/src/analysis/mod.rs:51-53` | 没开时用 `WordSegmenter::new_for_non_complex_scripts(...)` |
| 同上 `:105-108` | 断行用 `LineSegmenter::new_for_non_complex_scripts(...)` |
| `icu_segmenter-2.3.0/src/complex/mod.rs:188` | 这套 segmenter **不含中日文词典**，遇到 Han 走 `DataError::custom("No segmentation model for complex script")` + `with_display_context("Chinese/Japanese")`，直接打 stderr |

也就是说：**这不是代码 bug，是默认没编中日文分词数据**。只要 UI 上有汉字就必然触发。

**修法**（一行，`ra2engine/Cargo.toml`）：

```toml
bevy = { path = "..", features = ["complex_script_segmentation"] }
```

透传链：`Cargo.toml:650`（bevy）→ `bevy_internal/Cargo.toml:319` → `bevy_text:15` → `parley/complex-scripts`，
parley 内部换成 `WordSegmenter::new_dictionary` / `LineSegmenter::new_dictionary`。
`icu_segmenter` 的 `compiled_data` 本来就开着（`parley/Cargo.toml:101-104`），所以词典是编进二进制的静态数据。

**代价**：中日文词典进二进制（1MB 量级），parley 需重编一次（约 2 分钟）。

**不要用 `RUST_LOG` 屏蔽**：该错误经 `icu_provider` 的 `with_display_context` 输出，
它按 feature 可能是 `log::error!` 也可能是 `eprintln!`，屏蔽不干净；而且屏蔽之后中文本该断行的地方
仍然没有词典可用。

**已实测**：开 feature 后同一次运行 `grep -c ICU4X` = 0。

### 2.6 文本组件三件套

```rust
(
    Text::new("格 12,34  层高 8"),          // bevy_ui::widget::Text(pub String)
    TextFont { font: f.into(), font_size: 14.0.into(), ..default() },
    TextColor(Color::WHITE),                 // bevy_text::TextColor(pub Color)
)
```

- `Text::new(impl Into<String>)`（`bevy_ui/src/widget/text.rs:126-133`）
- `TextColor` 默认白色（`bevy_text/src/text.rs:1436`）
- 多行：字符串里直接带 `\n`。自动换行用 `TextLayout { linebreak: LineBreak::WordBoundary, .. }` + 父节点 `max_width`。
  ⚠️ 中文没有空格，`WordBoundary` 对中文的断点依赖 Parley 的分词，最稳的是**自己按 `\n` 显式分行**。

### 2.7 `bevy::prelude` 里能直接拿到什么

`bevy_text/src/lib.rs:71-78` 的 prelude 导出：
`Font, FontHinting, FontSize, FontSmoothing, FontSource, FontStyle, FontWeight, FontWidth, GenericFontFamily, Justify, LineBreak, Strikethrough, StrikethroughColor, TextColor, TextError, TextFont, TextLayout, TextSpan, Underline, UnderlineColor`

`bevy_ui` 的 prelude 另外给 `Text`（widget）、`Node`、`Val`、`UiRect`、`BackgroundColor`、`BorderColor`、`BorderRadius` 等（`bevy_ui/src/lib.rs:54-81` 的 `ui_node::*` / `geometry::*`）。

---

## 3. 布局速查

`Node` 的构造：没有 `Node::new()`，用 `Node { ..Node::DEFAULT }`（`ui_node.rs:820-821` 有 `pub const DEFAULT`，字段与 `Node::default()` 一致）。

常用字段（都在 `Node` 上，`ui_node.rs:491+`）：

| 字段 | 类型 | 说明 |
|---|---|---|
| `position_type` | `PositionType` | `Absolute` = 脱离流、相对父节点定位（做左上角浮层必须用） |
| `left / top / right / bottom` | `Val` | `Absolute` 时的定位 |
| `width / height` | `Val` | 固定尺寸用 `Val::Px(320.0)` |
| `padding / margin / border` | `UiRect` | `UiRect::axes(Val::Px(10.0), Val::Px(6.0))` |
| `flex_direction` | `FlexDirection` | 默认 `Row` |
| `align_items` / `justify_content` | `AlignItems` / `JustifyContent` | |
| `border_radius` | `BorderRadius` | 也在 `Node` 上（0.20 不再是独立组件） |

- `BackgroundColor(pub Color)`（`ui_node.rs:2582`），默认全透明 `Color::NONE`
- 浮层背景要自己给 `BackgroundColor`，否则文字直接压在地形上

⚠️ 根节点（`spawn` 出来那一个）如果是 `Absolute`，它相对**窗口**定位；子节点再按父节点布局。

---

## 4. 本项目（ra2engine）的既有约定

- 相机是**正交 3D**（`Camera3d` + `OrthographicProjection`），UI 不需要额外相机，`UiPlugin` 会自动拿主相机。
- 颜色：`Tonemapping::Linear` + `Msaa::Off` 已经设好（见 `game.rs` 相机 spawn 处的注释），UI 不受影响。
- 逻辑 mesh 模式开关在 `rulesmd.ini` 的 `[Render] LogicMesh`，debug UI 只在 `=1` 时 spawn。
- 中文字体固定走 `C:/Windows/Fonts/simsun.ttc`，不往工程里塞字体文件（用户口径："引擎必然需要支持中文，直接读取系统宋体就行"）。

---

## 5. 最小可跑模板（复制即用）

```rust
use bevy::prelude::*;
use bevy::ui_widgets::{Activate, Button};

#[derive(Resource, Default)]
struct DemoState { pressed: u32 }

fn setup_ui(mut commands: Commands, mut fonts: ResMut<Assets<Font>>) {
    let font = fonts.add(Font::from_bytes(
        std::fs::read("C:/Windows/Fonts/simsun.ttc").expect("系统宋体"),
    ));

    // 面板
    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(8.0), top: Val::Px(8.0),
            width: Val::Px(320.0),
            padding: UiRect::all(Val::Px(8.0)),
            flex_direction: FlexDirection::Column,
            ..Node::DEFAULT
        },
        BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.65)),
        children![
            (
                Text::new("格 12,34"),
                TextFont { font: font.clone().into(), font_size: 14.0.into(), ..default() },
                TextColor(Color::WHITE),
            ),
            (
                Button,
                Node {
                    margin: UiRect::top(Val::Px(6.0)),
                    padding: UiRect::axes(Val::Px(8.0), Val::Px(4.0)),
                    ..Node::DEFAULT
                },
                BackgroundColor(Color::srgb(0.18, 0.20, 0.26)),
                children![(
                    Text::new("俯视图"),
                    TextFont { font: font.into(), font_size: 14.0.into(), ..default() },
                    TextColor(Color::WHITE),
                )],
            ),
        ],
    ));
}

// app.add_observer(|_: On<Activate>, mut s: ResMut<DemoState>| s.pressed += 1);
```

---

## 6. 本轮实现中已验证 / 待验证

| 结论 | 状态 |
|---|---|
| `Button` / `Interaction` 旧 API 废弃，`Interaction` 类型已删除 | 读码确认 |
| `bevy_ui_widgets::Button` 不 require `Node`，必须手挂 | 读码确认 |
| 点击经 `On<Activate>` 接收 | 读码确认 |
| `DefaultPlugins` 已含 UiWidgetsPlugins + Picking | 读码确认 |
| `TextFont.font` 是 `FontSource`、`font_size` 是 `FontSize` | 读码确认 |
| `system_font_discovery` 默认关闭 → 不能按 family 名字找系统字体 | 读码确认 |
| `Font::from_bytes` + `Handle` 引用能拿到 `.ttc` 里的宋体 | **已实测通过**（18,008,680 字节，中文正常出字形） |
| 宋体中文在 UI 里正常出字形 | **已实测通过**（面板与按钮文字都正常） |
| `bevy_ui_widgets::Button` 点击 → `On<Activate>` 一次 | **已实测通过**（1 次物理点击 = 1 次 `Activate`） |
| 按钮点击不会误触发地形指令 | **已实测通过**（`cursor_on_panel` 提前 return） |
