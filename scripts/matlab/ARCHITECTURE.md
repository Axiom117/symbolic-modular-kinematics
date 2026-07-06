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

## 运动学核心概念 —— DSL 可视化运作原理

本节深入解释 `mechanism.m` 中涉及的运动学概念：机构如何从 YAML DSL 描述转化为全局位姿，以及闭环、诊断等机制的工作原理。

### 1. 位姿传播模型：生成树 + 弦边

机构装配（mechanism assembly）的核心计算是一个**正向运动学（FK）**问题：给定各关节变量值，求每个 frame 在世界坐标系下的 4×4 齐次位姿。

当 DSL 描述的机构包含**闭环**（kinematic loop）时，简单 FK 会遇到矛盾——同一 frame 可以通过不同路径被赋予不同位姿。`mechanism.m` 采用标准的**生成树（spanning tree）+ 弦边（chord）**策略来解决：

```
                        完整的连接图（含闭环）
                               │
              ┌────────────────┴────────────────┐
              ▼                                  ▼
     closed: false                         closed: true
     生成树边 (spanning-tree edge)          弦边 (chord edge)
     g.addMate()                           g.addClosedMate()
     ┌─────────────────────┐              ┌──────────────────────┐
     │ 双向插入 (socket↔plug) │              │ 单向插入 (socket→plug) │
     │ 参与 FK 位姿传播       │              │ 不参与 FK 位姿传播      │
     │ 定义机构主干位姿流向    │              │ 仅用于诊断闭环残差       │
     └─────────────────────┘              └──────────────────────┘
```

**为什么弦边不能用于位姿传播？**

- 如果用弦边传播，位姿会穿过闭环，导致同一 body 通过不同路径获得不一致的位姿
- 闭环残差会错误地落在生成树边上，掩盖真正的闭合误差
- 弦边保持「诊断专用」（diagnostic-only），其配合间隙（mate gap）准确反映当前关节配置下的闭环残差

**代码实现**（`mechanism.m` 第 127-137 行）：

```matlab
if ~isClosed
    g.addMate(sk.node, pl.node, roll, sym);       % 生成树边：双向，传播位姿
else
    g.addClosedMate(sk.node, pl.node, roll, sym);  % 弦边：单向，仅记录
end
```

`addMate` 插入双向边（`socket→plug` 和 `plug→socket`），确保图可双向遍历；
`addClosedMate` 只插入 `socket→plug` 单向边，在图遍历中成为死胡同，不会影响位姿计算。

**Mate 变换约定**（参见 `specs/dsl/connection-semantics.md`）：

$$T_{\text{plug} \leftarrow \text{socket}} = R_z\left(\text{roll} \times \frac{2\pi}{\text{symmetry}}\right) \times R_x(\pi), \quad \mathbf{t} = \mathbf{0}$$

**弦边隔离机制 —— `kind` 过滤**

`addClosedMate` 使用 `kind='closed_mate'` 标记边，`toStruct()` 在传给 FK 引擎前将其过滤：

```matlab
% toStruct() — 核心过滤逻辑
keepMask = ~strcmp({obj.Edges.kind}, 'closed_mate');
s = obj.Edges(keepMask);       % 弦边被排除，FK 引擎永远不可见
s = rmfield(s, 'kind');
```

**为什么需要硬过滤而非依赖约定？**

`PoseGraph.propagatePoses` 的迭代传播逻辑为：

```matlab
while changed
    changed = false;
    for k = 1:numel(edges)
        e = edges(k);
        if isKey(poses, e.from) && ~isKey(poses, e.to)
            poses(e.to) = poses(e.from) * e.T;   % from 已知 + to 未知 → 传播
            changed = true;
        end
    end
end
```

FK 引擎遍历**所有边**（无 `kind` 字段，无法区分来源），传播条件仅依赖 `from` 已知且 `to` 未知。在 `kind` 过滤引入之前，`addClosedMate` 使用 `kind='mate'`，`toStruct()` 不区分两类 mate 边。这导致一个边界情况：

```
假设边顺序: [... spanning-tree edges ..., C→A (closedMate), ...]
如果 C 先被 spanning tree 放置，而 A 尚未到达：
  → C→A 满足 isKey(C) && ~isKey(A)，closedMate 抢先传播！
  → A 的位姿来自 T_closedMate（纯旋转变换），而非正确的 spanning tree 路径
  → mate gap 被错误报告为 0，闭环残差被掩盖
```

在简单机构中这很少触发（spanning tree 通常同一轮覆盖闭环两端），但拓扑复杂时存在风险。**`kind='closed_mate'` + `toStruct()` 过滤**从根本上消除了这一隐患——弦边永远不会出现在 FK 引擎的输入中。

**边种类汇总**：

| kind | 插入方式 | 方向 | toStruct() | 用途 |
|------|----------|------|------------|------|
| `fixed` | addFixedTransform | 双向 | 保留 | 固定几何变换 |
| `joint` | addJoint | 双向 | 保留 | 关节（revolute/prismatic） |
| `mate` | addMate | 双向 | 保留 | 生成树配合边，参与 FK |
| `closed_mate` | addClosedMate | 单向 (s→p) | **过滤** | 弦边，仅诊断闭环残差 |

### 2. 传播算法细节

位姿传播的起点是**根节点（ground node）**：

```matlab
if ~g.hasGroundNodes()
    g.addGround(inst(1).bodies{1}.node);   % 回退：第一个实例的第一个 body
end
poses = g.propagate();
```

**根节点来源优先级**：
1. DSL 中标记了 `semantic_tag: ground` 的 frame → 自动注册为 ground node
2. 无 ground node 时 → 回退到第一个实例的第一个 body 作为根
3. 支持多根（多次调用 `addGround()`），适用于多分支 / 并联机构

**`g.propagate()` 执行流程**：

```
g.propagate()
    │
    ├─► 1. 构造 seed map: 每个 ground node → eye(4)（世界原点位姿）
    │      无 ground node 时 → Edges(1).from → eye(4) 作为 fallback
    │
    ├─► 2. g.toStruct() 过滤 closed_mate 边 + 剥离 kind → 纯 from/to/T struct 数组
    │
    └─► 3. PoseGraph.propagatePoses(edges, seed)
            │
            └─► 返回 containers.Map(name → 4×4 homogeneous transform)
```

FK 传播是**迭代**过程：从 seed 帧出发，沿所有出边（`from → to`）计算目标帧位姿，逐步扩展直至所有可达帧都被赋值。不可达帧（断开连接的子图）不会被放置，触发警告。

### 3. 特征尺度自动适配

可视化中所有几何元素（坐标 triad、关节轴线、mate 诊断线）需要与机构实际尺寸匹配。`mechanism.m` 通过**特征尺度 `L`** 自动适配：

```matlab
maxr = 1; ks = keys(poses);
for k = 1:numel(ks); P = poses(ks{k}); maxr = max(maxr, norm(P(1:3, 4))); end
L = max(4, 0.20 * maxr);
```

| 变量 | 含义 | 计算 |
|------|------|------|
| `maxr` | 所有已放置 frame 中距离世界原点的最大径向距离 | $\max \|\mathbf{p}_i\|$ |
| `L` | 特征尺度 | $\max(4,\,0.20 \times \text{maxr})$ |

- 取最大距离的 20% 作为基准尺度，底线为 4 mm
- `L` 用于控制 triad（坐标系指示器）大小、关节轴线长度等
- 确保无论机构是毫米级还是米级，可视化元素都有合适的视觉比例

### 4. Mate 诊断：配合间隙与对齐检查

每个 mate 连接（socket-plug 对）在可视化中绘制为一条诊断线段，并输出两个关键几何指标：

| 指标 | 公式 | 含义 | 理想值 |
|------|------|------|--------|
| **gap** | $\|\mathbf{p}_s - \mathbf{p}_p\|$ | socket 与 plug 原点之间的欧氏距离 | $0$（完全对齐） |
| **Zdot** | $\mathbf{z}_s \cdot \mathbf{z}_p$ | 两端口 +Z 轴的点积 | $-1$（反平行，+Z 方向相反） |

```matlab
gap  = norm(Ps(1:3,4) - Pp(1:3,4));
zdot = dot(Ps(1:3,3), Pp(1:3,3));
```

**诊断解读**：
- **gap > 0**：端口原点未对齐，可能原因包括关节变量未正确配置或模块几何参数不匹配
- **zdot ≠ -1**：端口 +Z 方向不一致，可能是 `roll` 参数配置错误
- **弦边**用橙色粗虚线绘制（`[0.95 0.55 0.10]`），**生成树边**用灰色细虚线绘制（`[0.2 0.2 0.2]`），视觉上便于区分

### 5. 未放置节点检测

FK 传播后，某些 frame 可能不出现在 `poses` map 中——这表示它们属于**不连通分量**（disconnected component），在图结构中无法从任何 ground node 通过边到达：

```matlab
if ~isKey(poses, f.node)
    unplaced{end+1} = f.node;
end
```

`result.unplaced` 列表收集所有未放置节点，并在控制台输出警告：
```
[WARNING] N node(s) not placed (disconnected component?).
```
这有助于在 DSL 编写阶段快速发现连接遗漏或图结构错误。

### 6. 端到端 DSL 可视化流水线总结

```
specs/dsl/examples/*.yaml          ← 机构装配 DSL
        │
        ▼  core.readYaml()
   DSL struct (mechanism / instances / connections / closed flags)
        │
        ▼  遍历 instances，每条加载 module def YAML，注入参数
   localExpandInstance()
        │
        ├─► g.addFixedTransform(from, to, T)     固定变换（双向）
        ├─► g.addJoint(from, to, axis, val, kind) 关节（双向，含关节变量）
        └─► g.addGround(node)                    标记地面帧
        │
        ▼  遍历 connections，匹配 socket↔plug
   ┌─────────────────────────────────────────────┐
   │  closed: false  →  g.addMate()               │  生成树边（双向）
   │  closed: true   →  g.addClosedMate()         │  弦边（单向，诊断用）
   └─────────────────────────────────────────────┘
        │
        ▼
   g.propagate()
        │
        ├─► 构造 seed map (ground → eye(4))
        ├─► g.toStruct()  → 纯 from/to/T 数组
        └─► PoseGraph.propagatePoses()  →  poses: containers.Map
        │
        ▼
   可视化循环
        ├─► poses(f.node)  →  triad + geometry patch
        ├─► mate gap / Zdot →  诊断线段 + 控制台报告
        └─► unplaced 列表   →  警告不连通分量
```

此流水线将声明式的 YAML DSL 描述完整转化为可交互的 3D 可视化，同时提供闭环残差诊断，帮助验证机构设计的几何正确性。
