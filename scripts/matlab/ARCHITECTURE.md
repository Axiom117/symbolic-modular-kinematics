# scripts/matlab — 三层架构设计

> 本文档解释 `+core` / `+ir` / `+viz` 三层之间的职责划分与数据流。

## 架构总览

```
┌──────────────────────────────────────────┐
│  mechanism.m / module.m   (调用层)        │
│  只需要和 EdgeGraph 交互                   │
└────────────────┬─────────────────────────┘
                 │  g.addFixedTransform()
                 │  g.addJoint()
                 │  g.addMate()
                 │  g.propagate()          ← 唯一入口
                 │
┌────────────────▼─────────────────────────┐
│  EdgeGraph  (Builder / 累积层)             │
│  - 管理边的增删                            │
│  - 记录 kind, pending 等元数据              │
│  - 管理 ground nodes                      │
│  - propagate() = 适配器，桥接到底层引擎      │
└────────────────┬─────────────────────────┘
                 │  g.toStruct() 剥除元数据
                 │  构造 seed map
                 │
┌────────────────▼─────────────────────────┐
│  PoseGraph  (Algorithm / 纯计算层)         │
│  - propagatePoses()   纯 FK 迭代          │
│  - jointTransform()   关节变换矩阵          │
│  - framePending()     查询 pending 状态    │
│  全部 static，无状态，纯数学                 │
└──────────────────────────────────────────┘
```

## 各层职责

### 调用层：`+viz/mechanism.m`, `+viz/module.m`

- 解析 YAML DSL → 提取 instances / bodies / frames / fixed_transforms / joints / connections
- 调用 `ir.EdgeGraph` 方法累积边（`addFixedTransform`, `addJoint`, `addMate`）
- 调用 `g.propagate()` 获取全局位姿 map
- 将位姿 map 用于可视化（triad、geometry patch、mate 诊断线）
- **不直接调用 `core.PoseGraph` 的任何方法** — 所有 FK 交互通过 `EdgeGraph` 完成

### 累积层：`+ir/EdgeGraph` (handle class)

- **为什么是 handle class**：MATLAB 的值语义会使多函数调用中的累加器需要反复传入传出。handle class 支持原地修改，调用点干净。
- 封装所有边创建逻辑（固定变换、关节、mate、closed-mate），自动插入双向边
- 管理元数据（`kind`: fixed/joint/mate, `pending`: 旋转是否已解冻）
- 管理 ground nodes（多根支持）
- `propagate()` 方法：
  1. 从 `GroundNodes` 构造 seed map（含 fallback 逻辑：无 ground node 时用第一条边的 from 作为根）
  2. 调用 `toStruct()` 剥离 `kind`/`pending` 字段
  3. 委托 `PoseGraph.propagatePoses()` 执行 FK

### 计算层：`+core/PoseGraph` (全部 static)

- **无状态，纯数学**：输入边 + 种子位姿 → 输出全局位姿 map
- `propagatePoses(edges, seed)`: 迭代 FK 传播（支持多根、支持环路悬空节点）
- `jointTransform(kind, axis, value)`: 构造 revolute / prismatic 关节的 4×4 齐次变换
- `framePending(edges, name)`: 查询某帧在边集中是否标记为 pending（供 `module.m` 使用；`mechanism.m` 改用 `EdgeGraph.isFramePending()`）

## 数据流

### mechanism.m 路径（机构装配）

```
specs/dsl/examples/*.yaml
        │
        ▼  core.readYaml()
   DSL struct (mechanism / instances / connections)
        │
        ▼  遍历 instances，每条 instance 加载 module def YAML
   localExpandInstance()
        │
        ├─► g.addFixedTransform()  ──► EdgeGraph.Edges (bidirectional)
        ├─► g.addJoint()           ──► EdgeGraph.Edges (bidirectional)
        └─► g.addGround()          ──► EdgeGraph.GroundNodes
        │
        ▼  遍历 connections
   g.addMate() / g.addClosedMate()  ──► EdgeGraph.Edges
        │
        ▼
   g.propagate()
        │
        ├─► 构造 seed map（ground nodes → eye(4)）
        ├─► g.toStruct()  剥离 kind / pending → 纯 from/to/T struct 数组
        └─► PoseGraph.propagatePoses(edges, seed)  ──► poses: containers.Map
        │
        ▼
   visualization loop
        │
        ├─► poses(f.node)  →  画 triad / geometry patch
        ├─► g.isFramePending(f.node)  →  品红色 pending 样式
        └─► connInfo  →  画 mate 诊断线段
```

### module.m 路径（单模块可视化）

```
specs/modules/*.yaml
        │
        ▼  core.readYaml()
   Module def struct (bodies / frames / fixed_transforms / joints)
        │
        ├─► g.addFixedTransform()  ──► EdgeGraph.Edges (bidirectional)
        ├─► g.addJoint()           ──► EdgeGraph.Edges (bidirectional)
        └─► g.addGround(rootBody)  ──► EdgeGraph.GroundNodes
        │
        ▼
   g.propagate()  ──► poses: containers.Map  (同 mechanism.m 路径)
        │
        ▼
   visualization loop
        ├─► poses(f.name)  →  画 triad / geometry patch
        └─► g.isFramePending(f.name)  →  品红色 pending 样式
```

`module.m` 路径是 `mechanism.m` 的简化版：无多实例拼接、无 inter-instance mate 边、单根（第一个 body）。

## 关键设计决策

### 1. `toStruct()` 剥离元数据

`EdgeGraph.Edges` 内部包含 `kind` 和 `pending` 字段，但 FK 引擎 (`PoseGraph.propagatePoses`) 只需要 `from` / `to` / `T`。`toStruct()` 负责剥离多余字段，保持 FK 引擎接口简洁。

### 2. Pending 追踪

- 存储：每条 fixed-transform 边在 `EdgeGraph` 中以 `pending` 字段标记（当旋转为占位 align 规则时）
- 查询：通过 `g.isFramePending(nodeName)` 方法，遍历所有边查找目标节点是否被 pending 边指向
- 可视化：pending 帧用品红色虚线 triad + `(pending R)` 标签渲染
- 不通过 `toStruct()` 暴露 `pending` 字段（保持 FK 引擎纯净）

### 3. Mate 约定

参见 `specs/dsl/connection-semantics.md`：
```
T_plug←socket = Rz(roll × 2π / symmetry) × Rx(π),  t = 0
```
- `addMate(socket, plug, roll, sym)`: 插入双向边，用于 FK 传播（spanning tree edge）
- `addClosedMate(socket, plug, roll, sym)`: 仅插入单向边，不参与 FK 传播（chord / 环路切割边），仅用于诊断环路闭合残差

### 4. 多根支持

`EdgeGraph` 支持多次调用 `addGround()` 注册多个 ground frame。在 `propagate()` 中所有 ground frame 以 `eye(4)` 为初始位姿。这支持多分支 / 并联机构。

### 5. 为什么 `PoseGraph` 保持独立

| 维度 | EdgeGraph | PoseGraph |
|------|-----------|-----------|
| 状态 | handle class，有状态 | 全部 static，无状态 |
| 职责 | 累积边、管理元数据 | 纯数学计算 |
| 变更原因 | DSL 格式变化、新边类型 | FK 算法改进、新关节类型 |
| 可测试性 | 需要构造完整对象 | 给定输入即得输出 |

合并会违反单一职责原则，降低可测试性。保持分层使得每层可以独立演进和测试。
