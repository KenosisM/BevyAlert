# ccmaps-net 地形绘制原理（逐文件通读落档）

来源：`I:\bevy\RA2REASM\ref\ccmaps-net\`
本文只写**从代码里读到的事实**，附原文位置；推测单独放在最后一节并标明"待验证"。

---

## 0. 通读清单

| 文件 | 状态 |
|---|---|
| `CNCMaps.Shared/ModConfig.cs` | 已读（TileWidth/TileHeight） |
| `CNCMaps.Engine/Map/Map.cs`（Draw / DrawTilePass / DrawCellPass / LoadAllObjects） | 已读 |
| `CNCMaps.Engine/Map/MapTile.cs` | 已读（全文） |
| `CNCMaps.Engine/Map/TileLayer.cs` | 已读（全文） |
| `CNCMaps.Engine/Game/TileCollection.cs`（InitTilesets / TileSetEntry / GetTmpFile / PickVariant / DrawTileShadow） | 已读 |
| `CNCMaps.Engine/Rendering/TmpRenderer.cs` | 已读（全文） |
| `CNCMaps.Engine/Drawables/TileDrawable.cs` | 已读（全文） |
| `CNCMaps.FileFormats/TmpFile.cs` | 已读（全文） |
| `CNCMaps.Engine/Rendering/RampHeight.cs` | 已读 |
| `CNCMaps.Engine/Rendering/DrawingSurface.cs` | 已读 |

---

## 1. 三套坐标系与换算

### 1.1 地图文件里的四个坐标（`MapTile.cs:14-18`）

```
Dx, Dy : ushort   —— 等距格坐标（display 坐标）
Rx, Ry : ushort   —— 地图逻辑坐标
Z      : short    —— 高度，单位"级"
TileNum: int      —— 砖号（全局）
SubTile: ushort   —— 子格（cell）索引
```

**四个坐标全部来自地图文件**（`Map.cs:153`）：
```csharp
_tiles[iso.Dx, iso.Dy / 2] = new MapTile(iso.Dx, iso.Dy, iso.Rx, iso.Ry, iso.Z,
                                         iso.TileNum, iso.SubTile, iso.IceGrowth, _tiles);
```

### 1.2 逻辑坐标 ↔ 等距坐标（`TileLayer.cs:14-21` 注释原文）

```
dx = rx - ry + mapwidth - 1
dy = rx + ry - mapwidth - 1

rx = (dx + dy) / 2 + 1
ry = dy - rx + mapwidth + 1
```

**注意 `dy` 可以为负**；数组按 `dy / 2` 索引（`TileLayer.cs:77`、`Map.cs:153`）。

### 1.3 格子网格

`tiles = new MapTile[Width*2-1, Height]`（`TileLayer.cs:38`）
→ `Dx ∈ [0, 2W-2]`，`Dy/2 ∈ [0, H-1]`。

邻居偏移（`TileLayer.cs:127-134`，原文断言）：

| 方向 | Δ(Dx, Dy) |
|---|---|
| Top | (0, −2) |
| TopLeft | (−1, −1) |
| TopRight | (+1, −1) |
| Left | (−2, 0) |
| Right | (+2, 0) |
| BottomLeft | (−1, +1) |
| Bottom | (0, +2) |
| BottomRight | (+1, +1) |

### 1.4 屏幕尺寸常量（`ModConfig.cs:155-161`）

```csharp
TileWidth  = (RA2 || YR) ? 60 : 48;
TileHeight = (RA2 || YR) ? 30 : 24;
```

→ **YR：一格 = 60 × 30 像素。**

### 1.5 等距投影（把上面前提串起来）

砖的菱形是 `BlockWidth × BlockHeight`（= 60×30），**菱形上顶点 = 格角 (0,0)**：

```
格内 (u, v) ∈ [0,1]²，高度 w 级：
  sx = ox + 30 + 30·(u − v)
  sy = oy + 15·(u + v) − 15·w
其中 ox = Dx·30、oy = (Dy − Z)·15        ← TmpRenderer.cs:53
```

四个角：上 `(ox+30, oy)`、右 `(ox+60, oy+15)`、下 `(ox+30, oy+30)`、左 `(ox, oy+15)`。

**`ox, oy` 是"菱形包围盒的左上角"，不是格角。** 格角 (0,0) 在 `ox+30, oy`。

---

## 2. 画布与遍历顺序（`Map.cs:1189-1285`）

### 2.1 画布

```csharp
_drawingSurface = new DrawingSurface(FullSize.Width * TileWidth,    // = W*60
                                     FullSize.Height * TileHeight); // = H*30
```

### 2.2 地形砖 pass（`Map.cs:1194-1208`）

```csharp
for (int y = 0; y < FullSize.Height; y++) {
    for (int x = FullSize.Width*2 - 2; x >= 0; x -= 2)  DrawTilePass(_tiles.GetTile(x, y));  // 偶数列，右→左
    for (int x = FullSize.Width*2 - 3; x >= 0; x -= 2)  DrawTilePass(_tiles.GetTile(x, y));  // 奇数列，右→左
}
```

**行从上到下；每行内先画偶数列（右→左），再画奇数列（右→左）。** 两遍交错是引擎的真实顺序。

`DrawTilePass`（`Map.cs:1290`）= `_theater.Draw(tile, ds)` + 该格上的 smudge。

### 2.3 之后的 pass（供参考）

1. 覆盖物（overlay）：`y` 从 H−1 递减、`x` 递增两趟（`Map.cs:1216-1221`）
2. 砖阴影：`DrawTileShadow`（TS/Firestorm 才有；RA2/YR 进来就 return）
3. 地形对象（tree 等）：`y` 递减、`x` 递增两趟
4. 其它对象（建筑/单位）：`y` 递增、`x` 递减两趟
5. deferred anims / deferred alpha

---

## 3. 单块砖的绘制：`TmpRenderer.Draw`（Rendering/TmpRenderer.cs:39-190）

### 3.1 开头

```csharp
tmp.Initialize();
if (tile.SubTile >= tmp.Images.Count) return;          // :42 越界直接不画
TmpFile.TmpImage img = tmp.Images[tile.SubTile];       // :43 ← cell 就是 SubTile
short zBase   = (short)((tile.Rx + tile.Ry) * tmp.BlockHeight / 2);  // :49
short hBufVal = (short)(tile.Z * _config.TileHeight / 2);            // :50
Point offset  = new Point(tile.Dx * tmp.BlockWidth / 2,              // :53
                          (tile.Dy - tile.Z) * tmp.BlockHeight / 2);
bool useZ = zData != null;                                           // :82
```

**`zBase = (Rx+Ry) * BlockHeight/2 = (Rx+Ry)*15`，主菱形与 extra 共用。**
**`hBufVal = Z * 15`。**

### 3.2 主菱形（:74-130）

起点 `offset` 再偏 `halfCx − 2`（`halfCx = BlockWidth/2 = 30`）：

```csharp
w    = Scan0 + stride*offset.Y + (offset.X + halfCx - 2) * 3;   // :74
zIdx = offset.Y * Width + offset.X + halfCx - 2;                // :77
```

逐行（**每行宽度由几何推出，数据里不存**）：

- 上半 `y < halfCy`：`cx += 4`，从 `offset.X + halfCx - 2 - 2y` 起，画 `4 + 4y` 个
- 下半 `y >= halfCy`：先 `w += 12; zIdx += 4`，然后 `cx -= 4`

像素消费顺序 = 一条 `rIdx` 从 0 递增的**行序**；总字节数 = `BlockWidth * BlockHeight / 2` = **900**。

逐像素：
```csharp
byte paletteValue = img.TileData[rIdx];
short zBufVal = (short)(zBase - (useZ ? zData[rIdx] : 0));
if (paletteValue != 0 && 在画布内 && (!useZ || zBufVal >= zBuffer[zIdx])) {
    写 BGR;
    if (useZ) zBuffer[zIdx] = zBufVal;
    heightBuffer[zIdx] = hBufVal;
}
```

### 3.3 extra（:132-186）

```csharp
if (!img.HasExtraData) return;                       // :132
byte[] xzData = img.ExtraZData;
if (tile.ExtraPalette != null) bgr = tile.ExtraPalette.GetBgrBytes();   // :134（tunnel 专用）

offset.X += img.ExtraX - img.X;                      // :136
offset.Y += img.ExtraY - img.Y;                      // :137
w    = w_low + stride*offset.Y + 3*offset.X;         // :138  ← 注意：没有 halfCx-2！
zIdx = offset.X + offset.Y * Width;                  // :139
```

然后**按矩形** `ExtraHeight × ExtraWidth` 逐像素：

```csharp
byte paletteValue = img.ExtraData[rIdx];
short zBufVal = (short)(zBase - (useXz ? xzData[rIdx] : 0));
if (paletteValue != 0 && 在画布内 && (!useXz || zBufVal >= zBuffer[zIdx])) {
    写 BGR;
    if (useXz) zBuffer[zIdx] = zBufVal;
    heightBuffer[zIdx] = (short)(img.ExtraHeight - y + hBufVal);
}
```

源码注释原文（:161-162）：
> Extra graphics are just a square; their z data extends the diamond's ramp from the same zBase anchor

### 3.4 从 `TmpRenderer` 能直接读出的两条位置事实

- **主菱形起点 = `offset + (30 − 2, 0)`**
- **extra 起点 = `offset + (ExtraX − X, ExtraY − Y)`**（不额外加 `halfCx − 2`）

### 3.5 `GetBounds`（:18-37）

```csharp
int left = Dx*BlockWidth/2, top = (Dy - Z)*BlockHeight/2;
int width = BlockWidth, height = BlockHeight;
if (img.HasExtraData) {
    if (ExtraX < 0) { left += ExtraX; width -= ExtraX; }
    if (ExtraY < 0) { top  += ExtraY; height -= ExtraY; }
    width  = Max(width,  ExtraWidth);
    height = Max(height, ExtraHeight);
}
```
→ extra 会**外扩**该砖的包围盒。

---

## 4. TMP 文件结构（`TmpFile.cs`）

### 4.1 文件头（`Initialize`，:103-123）

```csharp
Width      = ReadInt32();   // 块网格宽（cblocks_x）
Height     = ReadInt32();   // 块网格高（cblocks_y）
BlockWidth = ReadInt32();   // 每块宽（60）
BlockHeight= ReadInt32();   // 每块高（30）
byte[] index = Read(Width*Height*4);        // 每个块的偏移表
Images = new List<TmpImage>(Width*Height);
for (x in 0..Width*Height) { Seek(index[x]); Images.Add(Read一个 TmpImage); }
```

**→ 一个 TMP 文件含 `Width × Height` 个 cell（= `cblocks_x × cblocks_y`）。**

### 4.2 每个 cell 的 52 字节头（`TmpImage.Read`，:48-78）

```
int32 X, Y
int32 extraDataOffset, zDataOffset, extraZDataOffset
int32 ExtraX, ExtraY, ExtraWidth, ExtraHeight
uint32 flags        // 1=ExtraData, 2=ZData, 4=DamagedData
byte Height, TerrainType, RampType
sbyte radar 6 字节
3 字节 padding
```

随后：
```csharp
TileData   = Read(BlockWidth * BlockHeight / 2);        // 900 —— 菱形行序
if (HasZData)  ZData = Read(BlockWidth * BlockHeight / 2);
if (HasExtraData) ExtraData = Read(abs(ExtraWidth * ExtraHeight));
if (HasZData && HasExtraData && 0 < extraZDataOffset < Length)
    ExtraZData = Read(abs(ExtraWidth * ExtraHeight));
```

**`ExtraData` 是矩形、尺寸 `abs(ExtraW × ExtraH)`；`TileData` 是菱形行序的 900 字节。**

---

## 5. 大砖（多 cell 砖）

### 5.1 事实（实测 `cliff*.tem`，用 `ra2engine tmpimg` 导出）

| 文件 | cblocks | cells | 各 cell 的 `xy` | 各 cell 的 extra |
|---|---|---|---|---|
| cliff01.tem | 2×3 | 6 | (0,0) (−30,15) (0,30) (−30,45) + 2 个空 cell | 24×13 @(1,1) / 33×44 @(−3,−15) / 30×29 @(0,15) / 30×29 @(0,30) |
| cliff24.tem | 2×1 | 2 | (0,0) (30,15) | 28×16 @(32,−2) / 29×25 @(60,4) |
| cliff26.tem | 1×1 | 1 | (0,0) | 28×16 @(32,−2) |
| cliff35.tem | 1×2 | 2 | (0,0) (−30,15) | 28×15 @(0,−1) / 29×28 @(−29,1) |
| cliff36.tem | 1×2 | 2 | (0,0) (−30,15) | 28×15 @(0,−1) / 29×17 @(−29,12) |
| cliff38.tem | 1×1 | 1 | (0,0) | 29×14 @(2,−1) |

**`xy` 的值与"格子相邻的屏幕差"一致**：相邻格 `(rx+1, ry)` 差 (+30,+15)、`(rx, ry+1)` 差 (−30,+15)。
例如 cliff01 的 (−30,15) = `(rx, ry+1)`、(0,30) = `(rx+1, ry+1)`、(−30,45) = `(rx+1, ry+2)`。

**`ExtraX` 跨度很大：32 / 60 / 0 / 2 / −29** —— 没有任何"按符号分左右边"的规律。

### 5.2 代码里对 `SubTile` 的两处用法

1. **取 cell**：`img = tmp.Images[tile.SubTile]`（`TmpRenderer.cs:43`），越界即 `return`（:42）
2. **取变体时用"原点格"**（`TileCollection.cs:91-101`）：
```csharp
int x = t.Rx, y = t.Ry;
if (t.SubTile != 0) {
    var tmp = TmpFiles[0]; tmp.Initialize();
    x = (x - t.SubTile % tmp.Width) / tmp.Width;
    y = (y - t.SubTile / tmp.Width) / tmp.Height;
}
```
   注释原文：*Multi-cell tiles index by their origin cell scaled to the tile grid.*

**`X`/`Y`（`TmpImage` 的那两个 int32）在 `TmpRenderer` 里只用于 extra**（`:136-137` 的 `ExtraX − X`），
**画主菱形时没有用它们**（`:53` 只有 `Dx/Dy/Z`）。

---

## 6. 变体与砖文件的加载（`TileCollection.cs:318-359`）

```csharp
for (int j = 1; j <= ts.TilesInSet; j++) {
    var rs = new TileSetEntry(ts, j-1);
    for (var r = (char)('a' - 1); r <= 'z'; r++) {       // '`' 表示"无后缀"，然后 a,b,c…
        string filename = ts.FileName + j.ToString("00");
        if (r >= 'a') filename += r;
        filename += _theaterSettings.Extension;
        var tmpFile = _vfs.Open<TmpFile>(filename);
        if (tmpFile != null) rs.AddTile(tmpFile); else break;
    }
    ...
}
```

- **一块砖（段内序号 `j`，1-based）= 一个 `TileSetEntry`，含若干 `TmpFiles`（变体）**
- 变体从"无后缀"开始，遇到文件不存在就停
- NewUrban 例外：`.ubn` 取不到时回落 `.tem`
- 桥 set 不做 a/b/c 变体（`:339`）

**变体选择**（`PickVariant`，:91-106）：`count <= 1` 直接 0；否则
`count > 4` 用 8×8 格 `(x&7, y&7)`、`count <= 4` 用 4×4 拉丁方 `(x&3, y&3)`，
最后 `v >= count ? v % count : v`（与 RA2REASM 的 `pick_tile_variant` 一致）。

---

## 7. 调色板

- 地形砖/树/overlay 用 `LightingType.Full` → **按高度查 19 级调色板**（`isoLevel[Z]`）
- 建筑/单位/步兵/飞机用 `LightingType.Ambient` → **不套光照**
- `TmpImage` 的像素是**调色板索引**，索引 0 不画
- **`ExtraPalette` 只在 tunnel 砖上被设置**（`Map.cs:685-697` `LightTunnelRoofs`），
  注释原文：*Light the extra art like the highest cardinal neighbour instead, the plateau it belongs to.*
  —— 普通悬崖砖的 extra 用的是**本砖自己的 palette**（`TmpRenderer.cs:46`）

---

## 8. 坡道：`RampHeight.cs`

```
LevelLeptons = 104
HalfLevelsAtCentre[21] = { 0,  1,1,1,1,  0,0,0,0,  2,2,2,2,  2,2,2,2,  1,1,1,1 }
ZPixel(leptons) = int(leptons * sin60° * TileWidth / (256*sqrt2) + fudge + 0.5)
    fudge = (leptons >= 7*104) ? 1 : 0        // TS 用 9*104
PixelLift(ramp, cellHeight) = ZPixel(ground + half*(LevelLeptons/2)) - ZPixel(ground)
    ground = LevelLeptons * cellHeight
```

---

## 9. 阴影（TS 专属，RA2/YR 不走）

`DrawTileShadow`（`TileCollection.cs:531`）开头即
```csharp
if (_config.Engine > EngineType.Firestorm) return;   // RA2/YR 直接返回
```
`CliffCasters` / `SlopeCasters` 是 **C_SHADOW.SHP 的帧表**，与贴图无关。

---

## 10. 与 `ra2engine` 现状的差异对照

| 项 | ccmaps 事实 | ra2engine 现状 | 判断 |
|---|---|---|---|
| 变体数 | `TmpFiles.Count`（文件数） | `variant_cells.len()` = 每个变体 push 一次 → 等于文件数 | 一致 |
| cell 索引 | `Images[SubTile]` | `variant_pixels[variant][cell]`，`cell = sub_tile` | 一致 |
| 变体的 x/y | 多格砖用**原点格** `(Rx − SubTile%W)/W` | `pick_tile_variant(rx, ry, count)` 用本格 | **不一致**（仅影响多变体砖） |
| 主菱形位置 | `(Dx·30 + 30 − 2, (Dy−Z)·15)` | 顶面四角 = 格角 + 高度 | 待验证 |
| extra 位置 | `(Dx·30 + ExtraX − X, (Dy−Z)·15 + ExtraY − Y)` | `base + exX·E_SX + exY·E_SY`，`base = 格角 − 30·E_SX` | 待验证 |
| 画家序 | 按行、偶/奇两遍、右→左 | 3D 深度缓冲（无画家序） | 3D 下等价，但同 z 取舍可能不同 |
| `SubTile` 越界 | `TmpRenderer` 直接 `return`；`GetTmpFile` 里 `Images[Min(SubTile, Count-1)]` | `variant_pixels.get(cell)` → None 即跳过 | 待验证 |
| 调色板 | 按 Z 查 19 级；extra 用本砖 palette | `iso_level[level]` | 一致（除非 tunnel） |
| 砖阴影 | RA2/YR 不画 | 未实现 | 一致 |

---

## 11. 待验证清单（尚未证实，不得当成结论）

1. `TmpRenderer` 主菱形起点带 `halfCx − 2`、extra 起点不带，这 2 像素差在 3D 里是否要复刻。
2. `X`/`Y` 字段（`TmpImage` 的前两个 int32）除了 `ExtraX − X` 之外还有没有别的用途
   —— 目前**只读到这一处**。
3. `TmpImage.Read` 里 `TileData` 长度是 `BlockWidth*BlockHeight/2`，而 ra2engine 的
   `decode_diamond` 把它展开成 `cx*cy` 的矩形布局；**展开后的排列是否与 `cx += 4` 的
   消费顺序严格对应**，需要拿一块已知图案的砖逐像素比。
4. `Map.Draw` 的两遍交错（偶数列先、奇数列后）在两块砖 z 相同时决定谁在上面；
   3D 深度缓冲下这个取舍是否会造成可见差异，未验证。
5. 缝隙（高台抬起后留下的屏幕区域）在 RA2 里"最后由哪块砖写"—— 需要按第 2 节的
   遍历顺序逐格模拟一次才敢下结论。
