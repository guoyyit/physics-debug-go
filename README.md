# physics-debug-go：物理世界调试查询与可视化快照

一个零依赖的 2D 物理世界调试库：固定步长推进、确定性可复现、射线/区域
查询、可视化调试数据、状态快照恢复、查询与推进并发安全。

## 1. 核心特性

- **固定步长推进**：`Step()` 每次推进一个固定 `dt`；`Advance(dt)` 用累加器
  消化真实时间，暂停时不推进。
- **逐帧确定**：相同初始状态 + 相同输入序列，逐帧字节级一致。物体始终按
  标识排序遍历，不依赖 map 迭代顺序，不使用并发执行。
- **射线检测 / 区域查询**：支持碰撞层位掩码、静态/动态过滤、自定义谓词；
  结果按（命中距离，物体标识）稳定排序。
- **三种结果可区分**：无命中（`Hits` 为空、`Filtered == 0`）、命中但被过滤
  （`Hits` 为空、`Filtered > 0`）、参数非法（返回 `*Error`，带错误码与定位
  信息）。
- **可视化调试数据**：每帧输出所有物体的 AABB、接触流形（法向、接触点、
  穿透深度）、查询射线与命中位置。数据与帧严格对应，序列化顺序稳定。
- **运行中增删改**：推进过程中可加入、移除物体或修改变换，查询不遗漏、不
  重复；只保留最新一帧，内存不随推进次数增长。
- **暂停 / 单步 / 快照恢复**：快照为纯数据（无指针、无进程内句柄），可
  JSON 序列化；恢复后继续推进与不中断推进逐帧一致。
- **并发安全**：推进与查询可同时发生。查询拿到的是某一完整帧的不可变快
  照，慢查询不阻塞推进，推进也不阻塞查询。

## 2. 确定性模型

- 所有物体保存在按 `ID` 升序排列的切片中，碰撞检测、快照、调试序列化都按
  该顺序遍历，杜绝 map 迭代顺序抖动。
- 推进在 `World.mu` 保护下整体进行，每次推进克隆上一帧、积分、重新生成接
  触、然后原子发布新帧。
- 输入（增删改、重力、暂停）在锁内串行执行，执行顺序即调用顺序。
- 浮点运算为纯标量运算，无并发共享写。

## 3. 查询语义

### 3.1 射线检测

```go
res, err := world.Raycast(origin, dir, maxDist, physics.RayFilter{
    Mask: physics.LayerMask(1), // 只命中第 0 层；0 表示不过滤层
    // StaticOnly: true,       // 仅静态物体
    // DynamicOnly: true,      // 仅动态物体（二者互斥）
    // Accept: func(b *physics.Body) bool { ... }, // 自定义谓词
})
```

- `dir` 会被归一化；`maxDist` 是沿射线的最大命中距离。
- 每个物体返回最近命中点；`Hits` 按 `(Dist, BodyID)` 升序排序。
- `Filtered` 统计几何上命中但被过滤器拒绝的物体数。

| 情况 | 返回 |
| --- | --- |
| 几何上无命中 | `Hits == nil`，`Filtered == 0`，`err == nil` |
| 命中但全部被过滤 | `Hits == nil`，`Filtered > 0`，`err == nil` |
| `dir` 为零向量 | `err.Code == ErrInvalidParam`，`err.Param == "dir"` |
| `maxDist <= 0` | `err.Code == ErrInvalidParam`，`err.Param == "max_dist"` |
| `StaticOnly` 与 `DynamicOnly` 同时设置 | `err.Code == ErrInvalidParam` |

### 3.2 区域查询

```go
res, err := world.OverlapCircle(center, radius, filter)
res, err := world.OverlapBox(center, halfW, halfH, filter)
```

- 返回与区域相交的所有物体，`Hits` 按 `BodyID` 升序排序。
- 过滤规则与射线检测一致；`Filtered` 统计相交但被拒绝的物体数。
- `radius`、`halfW`、`halfH` 必须为正，否则返回 `ErrInvalidParam`。

### 3.3 错误定位

所有错误均为 `*physics.Error`，包含：

- `Code`：错误码（`ErrInvalidParam` / `ErrDuplicateID` / `ErrNotFound` /
  `ErrInvalidShape` / `ErrEmptyID`）；
- `Object`：相关物体标识；
- `Param`：相关参数名；
- `Value`：相关参数值。

例如：`physics: circle radius must be strictly positive [object="ball-03"] [param=radius] [value=-1]`。

## 4. 调试数据格式

`DebugFrame` 是一帧的完整可视化数据，可通过 `world.DebugFrame()` 或
`world.DebugJSON()` 获取。

```json
{
  "version": 1,
  "frame": 30,
  "fixed_dt": 0.016666666666666666,
  "gravity": { "x": 0, "y": -9.8 },
  "bodies": [
    {
      "id": "ball",
      "type": 1,
      "layer": 2,
      "pos": { "x": 0, "y": 3.73 },
      "rot": 0,
      "vel": { "x": 0, "y": -4.9 },
      "ang_vel": 0,
      "shape": { "kind": 0, "radius": 0.5 },
      "aabb": { "min": { "x": -0.5, "y": 3.23 }, "max": { "x": 0.5, "y": 4.23 } }
    }
  ],
  "contacts": [
    {
      "body_a": "ball",
      "body_b": "ground",
      "normal": { "x": 0, "y": -1 },
      "points": [
        { "pos": { "x": 0, "y": -4 }, "depth": 0.26 }
      ]
    }
  ],
  "ray_queries": [
    {
      "origin": { "x": -10, "y": 0 },
      "dir": { "x": 1, "y": 0 },
      "max_dist": 100,
      "hits": [
        { "body_id": "ball", "layer": 2, "dist": 10, "point": { "x": 0, "y": 0 }, "normal": { "x": -1, "y": 0 } }
      ],
      "filtered": 0
    }
  ],
  "area_queries": [
    {
      "kind": 0,
      "center": { "x": 0, "y": 0 },
      "radius": 100,
      "hits": [ { "body_id": "ball", "layer": 2 } ],
      "filtered": 0
    }
  ]
}
```

字段说明：

- `bodies`：按 `id` 升序；`type`：0=静态，1=动态；`shape.kind`：0=圆，
  1=盒；`aabb` 为世界空间轴对齐包围盒。
- `contacts`：按 `(body_a, body_b)` 升序；`normal` 从 `body_a` 指向
  `body_b`；`points[].depth` 为穿透深度。
- `ray_queries` / `area_queries`：本帧内执行过的查询，按查询内容排序
  （与提交顺序、并发调度无关）；`hits` 即查询结果中的命中，位置为世界
  坐标。

所有切片均为值类型深拷贝，序列化结果与进程状态无关。

## 5. 快照与恢复

```go
snap := world.Snapshot()              // *WorldSnapshot，纯数据
data, _ := json.Marshal(snap)         // 可落盘
// ...
var restored physics.WorldSnapshot
json.Unmarshal(data, &restored)
w2, err := physics.NewWorldFromSnapshot(&restored)
```

快照包含：固定步长、重力、暂停标志、帧号、累加器、全部物体（标识、类型、
层、位姿、速度、形状）。恢复后继续 `Step()` 与不中断推进逐帧一致。快照
不含指针、句柄或任何进程内临时状态；恢复时会校验版本、步长、空/重复标
识与退化形状。

## 6. 并发模型

```
推进 goroutine                 查询 goroutine
     │                              │
     ├─ w.mu.Lock()                 ├─ w.mu.RLock() → 取当前帧指针
     ├─ 克隆帧、积分、检测接触       ├─ w.mu.RUnlock()
     ├─ 发布新帧（指针切换）         ├─ 在不可变帧上计算（不持锁）
     └─ w.mu.Unlock()               └─ 向帧日志追加查询记录（帧内锁）
```

- 帧一经发布即不可变（物体与接触切片不再修改），查询读到的永远是某一完整
  帧，不会读到更新到一半的状态。
- 查询计算期间不持有世界锁，慢查询不会阻塞推进；推进也不会因为查询而停
  顿。
- 旧帧仅被正在使用它的查询引用，查询结束后即被 GC 回收，不保留历史。

## 7. 内存模型

- 任意时刻只保留最新一帧；每次推进克隆物体切片，旧帧无引用后由 GC 回收。
- 帧内查询日志只记录自上一帧以来的查询，随帧切换清空，不跨帧累积。
- 物体移除即从切片删除，无墓碑、无累积。

## 8. API 速查

| 类别 | API |
| --- | --- |
| 创建 | `NewWorld(fixedDt)` |
| 推进 | `Step()`、`Advance(dt)`、`SetPaused(bool)`、`Paused()`、`FrameNumber()` |
| 场景 | `SetGravity(Vec2)`、`AddBody(Body)`、`RemoveBody(id)`、`SetTransform(id, pos, rot)`、`SetVelocity(id, vel, angVel)`、`SetLayer(id, layer)`、`SetShape(id, shape)`、`Bodies()` |
| 查询 | `Raycast(origin, dir, maxDist, RayFilter)`、`OverlapCircle(center, radius, RayFilter)`、`OverlapBox(center, halfW, halfH, RayFilter)` |
| 调试 | `DebugFrame()`、`DebugJSON()` |
| 快照 | `Snapshot()`、`NewWorldFromSnapshot(snap)` |

## 9. 测试

```bash
go test -race ./...
```

覆盖：确定性（含运行中增删改）、过滤与排序、无命中/被过滤/非法参数区分、
运行中增删改无幽灵、暂停/单步、快照恢复逐帧一致、快照 JSON 往返、并发查
询（race 检测）、并发确定性、内存有界。
