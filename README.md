# physics-debug-go

一个确定性的 2D 物理世界，内置**调试查询**与**可视化快照**能力。目标是让物理世界不再是黑盒：碰撞对不对、查询为什么没命中，都可以直接查数据，而不是猜日志。

## 快速开始

```go
w, _ := physics.NewWorld(physics.DefaultConfig()) // 固定步长 1/60s
w.AddBody(physics.BodyDef{ID: 1, Kind: physics.Static, Layer: 1,
    Shapes: []physics.Shape{physics.Box{HalfW: 50, HalfH: 1, Offset: physics.Vec2{0, -1}}}})
w.AddBody(physics.BodyDef{ID: 2, Kind: physics.Dynamic, Pos: physics.Vec2{0, 5}, Layer: 2,
    Density: 1, Friction: 0.4, Shapes: []physics.Shape{physics.Circle{Radius: 0.5}}})

for i := 0; i < 120; i++ { w.Step() }          // 每步严格推进 Dt

res, _ := w.Raycast(physics.RayQuery{           // 射线检测
    Origin: physics.Vec2{0, 10}, Dir: physics.Vec2{0, -1}, MaxDist: 50})
frame := w.DebugFrame()                          // 当前帧可视化数据
data, _ := frame.Encode()                        // 稳定序列化（同帧同字节）
```

## 确定性与固定步长

- `Config.Dt` 是唯一步长，`Step()` 每次严格推进一个 `Dt`。
- 所有遍历（积分、宽相、窄相、求解）都按 **body ID 升序**进行；碰撞对按
  `(ID小, ID大)`、流形点按形状索引顺序生成。插入顺序、map 遍历、并发查询
  都不会影响结果。
- 相同初始状态 + 相同输入序列 ⇒ 逐帧 bit 级一致（见 `TestDeterminism`，
  含不同插入顺序的对照）。

## 查询语义

### 射线检测 `Raycast(RayQuery) (RayResult, error)`

- `Dir` 非零（内部归一化），`MaxDist` 为有限正数；否则返回
  `*physics.Error`，`Code == ErrInvalidParam`，错误信息指明具体参数。
- 返回 `RayResult.Status` 区分三种情况：
  - `QueryHit`：有命中，`Hits` 按 **距离 → BodyID → 形状索引** 稳定排序；
  - `QueryNoHit`：几何上什么都没打到；
  - `QueryFiltered`：几何上有命中，但全部被过滤器排除（便于区分"没东西"
    和"东西被过滤掉了"）。
- `QueryFilter`：`LayerMask`（0 = 全部层）与 `Exclude`（排除的 body ID）。

### 区域查询 `OverlapRegion(RegionQuery) (RegionResult, error)`

- 按**形状 AABB** 与区域盒重叠做宽相式测试（非精确几何），`Bodies` 按 ID 升序。
- 区域盒非法（`Min > Max`、含 NaN）返回 `ErrInvalidParam`；状态三值语义同上。

### 并发模型

- 写操作（`Step`/`AddBody`/`RemoveBody`/`SetTransform`/`RestoreSnapshot` 等）
  内部串行化。
- 读操作（`Raycast`/`OverlapRegion`/`DebugFrame`）**不加写锁**：每步结束后
  世界发布一个不可变帧视图（`atomic.Pointer` 交换），查询只读某个完整帧，
  永远读不到推进到一半的中间状态，慢查询也不会阻塞推进。
- 旧帧视图在没有查询引用后被 GC，内存占用与步数无关。

## 调试数据格式（`DebugFrame`）

`DebugFrame()` 返回**最近一次完成帧**的可视化数据，与该帧状态严格对应：

```json
{
  "Frame": 180, "Time": 3.0,
  "Bodies":  [{ "ID": 2, "Kind": 2, "Pos": {"X":0,"Y":0.5}, "Angle": 0,
               "AABB": {"Min":{...},"Max":{...}},
               "Shapes": [{ "Kind": 1, "Center": {...}, "Radius": 0.5,
                            "Corners": null, "AABB": {...}] }] }],
  "Contacts":[{ "A": 1, "B": 2, "Normal": {...},
               "Points": [{ "Pos": {...}, "Penetration": 0.001,
                            "NormalImpulse": 0.82, "TangentImpulse": 0.03 }] }],
  "Rays":    [{ "Query": {...}, "Status": 0, "Hits": [{ "BodyID": 2,
               "ShapeIndex": 0, "Point": {...}, "Normal": {...}, "Dist": 9.5 }] }]
}
```

- `Bodies`：按 ID 升序；每个 body 含世界 AABB、每个形状的中心/半径（圆）
  或四角顶点（盒，固定顺序 `(-w,-h),(+w,-h),(+w,+h),(-w,+h)` 局部序）。
- `Contacts`：接触流形，按 `(A, B)` 排序；`Normal` 恒由 A 指向 B；
  `Points` 含接触点位置、穿透深度与求解器累积冲量。
- `Rays`：针对**该帧**发起过的射线查询及其命中结果，按查询参数
  `(Origin, Dir, MaxDist)` 排序——并发发起查询也不影响序列化结果。
  查询日志按帧隔离，不会随步数累积。
- `Encode()` 输出稳定 JSON：同一帧无论序列化多少次、在哪个进程，字节一致。

## 运行中修改

`AddBody` / `RemoveBody` / `SetTransform` / `SetVelocity` 可在任意时刻调用，
立即对后续查询可见（发布新帧视图）。重复 ID、未知 ID、零尺寸形状、NaN
参数都会返回带 `ErrorCode` 与具体对象/参数信息的 `*Error`，不会静默忽略。

## 暂停、单步与快照

- `Pause()` 后 `Step()` 为空操作；`StepOnce()` 在暂停时仍推进一步；`Resume()` 恢复。
- `SaveSnapshot()` 生成纯数据快照（无进程内指针），`Encode()`/`DecodeSnapshot`
  可跨进程持久化；`RestoreSnapshot` 先完整校验（重复 ID、退化形状、非法参数
  都会拒绝且**不改动当前世界**），通过后原子替换世界状态。
- 从快照恢复后继续推进，与不间断推进到同一时刻的结果逐帧 bit 级一致
  （见 `TestSnapshotRestore`）。

## 测试

```sh
go test ./...        # 单元测试
go test -race ./...  # 并发查询竞态检测
```

覆盖：确定性（含插入顺序乱序）、过滤与排序、运行中增删改、查询日志有界、
快照恢复一致性、并发查询、非法输入、暂停/单步、调试数据内容与序列化稳定性。
