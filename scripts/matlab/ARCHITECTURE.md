# scripts/matlab — 四层架构设计

> 本文档解释 `+core` / `+ir` / `+viz` 三层（外加调用脚本层）之间的职责划分与数据流。
> 最后更新：A.3.2 完成后（引入 `ir.Expander` 和 `ir.SymbolicFK`）。

## 架构总览

```
┌──────────────────────────────────────────────────┐
│  +viz/mechanism.m  /  +viz/module.m  (可视化层)   │
│  薄封装：委托 ir.Expander 做 DSL→IR 展开，         │
│  只读取 e.Poses / e.Instances / e.ConnectionInfo  │
│  渲染 triad、geometry、mate 诊断线                 │
└────────────────────┬─────────────────────────────┘
                     │  e = ir.Expander(dslYaml, configYaml)
                     │  e = ir.Expander(dslYaml, configYaml, symbolicMode)
                     │
┌────────────────────▼─────────────────────────────┐
│  +ir/Expander  (DSL→IR 编排层 / Orchestrator)      │
│  - A.3.0 从 viz/mechanism.m 中抽出的独立模块       │
│  - 加载 DSL + module library + 参数配置            │
│  - 展开实例（localExpandInstance，private）         │
│  - 处理连接（mate / closed-mate）                  │
│  - 符号模式：创建 sym 关节变量，注册到 SymbolVars    │
│  - 暴露 public 属性供可视化和 SymbolicFK 消费       │
└────────────────────┬─────────────────────────────┘
                     │  内部持有 ir.EdgeGraph 实例
                     │
┌────────────────────▼─────────────────────────────┐
│  +ir/EdgeGraph  (图累积层 / Graph Accumulator)     │
│  - 管理边的增删（addFixedTransform / addJoint /     │
│    addMate / addClosedMate）                      │
│  - 记录 kind 元数据（fixed / joint / mate /        │
│    closed_mate）                                   │
│  - 管理 ground nodes（多根支持）                    │
│  - propagate() → toStruct() 过滤 closed_mate      │
│    → 委托 PoseGraph.propagatePoses()               │
│  - 支持 double 和 sym 混合 T 矩阵（多态）           │
└────────────────────┬─────────────────────────────┘
                     │  g.toStruct() 剥除 kind 元数据
                     │  过滤 closed_mate 弦边
                     │
┌────────────────────▼─────────────────────────────┐
│  +core/PoseGraph  (纯计算层 / Math Engine)         │
│  - propagatePoses()   迭代 FK 传播                 │
│  - jointTransform()   关节变换矩阵 (revolute/      │
│                       prismatic, 支持 sym/double)  │
│  全部 static，无状态，纯数学                         │
└──────────────────────────────────────────────────┘

                     ┌──────────────────────────┐
                     │  +ir/SymbolicFK           │
                     │  (符号 FK 薄封装)          │
                     │  - 读取 EdgeGraph 的      │
                     │    sym 位姿 map           │
                     │  - 抽取 TSym / PosExpr /  │
                     │    RotExpr / JointVars    │
                     │  - eval(vals) 数值代入     │
                     └──────────────────────────┘
```

## 各层职责

### 可视化层：`+viz/mechanism.m`, `+viz/module.m`

- `mechanism.m`：委托 `ir.Expander` 完成全部 DSL→IR 展开，自身**只读** `e.Poses`、`e.Instances`、`e.ConnectionInfo` 等公开属性做渲染
- `module.m`：单模块可视化，直接调用 `ir.EdgeGraph` 方法构建模块内部图（不经过 Expander）
- 渲染 triad（RGB 坐标三轴）、geometry patch（STEP/STL 几何）、mate 诊断线、关节轴高亮
- **不直接调用 `core.PoseGraph` 的任何方法** — FK 交互通过 `EdgeGraph.propagate()` 完成

### 编排层：`+ir/Expander` (handle class，A.3.0 新增)

- **为什么独立为编排层**：原先 DSL 解析、实例展开、参数注入、连接处理全部内嵌在 `+viz/mechanism.m` 的 setup 区段和 local functions 中。A.3.0 将其抽离为独立的 `ir.Expander`，使可视化层变为纯消费者，符号管线（`SymbolicFK`）也可复用同一套展开逻辑。
- **为什么是 handle class**：与 `EdgeGraph` 同理——内部持有 `EdgeGraph_` 和 `DefCache_` 两个可变状态，handle 语义避免在多步展开中反复传入传出。
- `Expander(dslYaml, configYaml, symbolicMode)`：构造函数即运行完整管线：
  1. 路径解析 + DSL 加载与校验
  2. 模块库路径解析 + 参数配置加载（`dimensions.yaml` + `joint_config.yaml`）
  3. 实例展开（`localExpandInstance`，private）：加载模块 YAML → 注入参数 → 展开 bodies/frames/fixed_transforms/joints → 名前缀 → 写入 `EdgeGraph_`
  4. 连接处理：极性校验 → 区分 `addMate`（生成树）和 `addClosedMate`（弦边）
  5. Ground fallback + FK 传播 → `Poses` map
- **符号模式**（`symbolicMode = true`）：跳过 `joint_config.yaml` 的数值注入，改为对每个 joint 调用 `sym('instanceName.varName')`，注册到 `SymbolVars`。EdgeGraph 中的 joint 边获取 `sym` 类型的 T 矩阵，`propagate()` 输出含符号表达式的位姿 map。
- **公开属性**：`MechName`、`Instances`、`ConnectionInfo`、`Poses`、`LibDir`、`SymbolVars`、`EdgeGraph_`

### 图累积层：`+ir/EdgeGraph` (handle class)

- **为什么是 handle class**：MATLAB 的值语义会使多函数调用中的累加器需要反复传入传出。handle class 支持原地修改，调用点干净。
- 封装所有边创建逻辑（固定变换、关节、mate、closed-mate），自动插入双向边
- 管理元数据（`kind`: `fixed` / `joint` / `mate` / `closed_mate`）
- 管理 ground nodes（多根支持）
- **支持混合 `double`/`sym` T 矩阵**：MATLAB 的 `*`、`sin`、`cos` 等运算符对 `sym` 多态，`EdgeGraph` 无需区分数值/符号模式即可承载两种管道。
- `propagate()` 方法：
  1. 从 `GroundNodes` 构造 seed map（含 fallback 逻辑：无 ground node 时用第一条边的 from 作为根）
  2. 调用 `toStruct()` 剥离 `kind` 字段并过滤 `closed_mate`
  3. 委托 `PoseGraph.propagatePoses()` 执行 FK

### 符号 FK 层：`+ir/SymbolicFK` (handle class，A.3.2 新增)

- 薄封装：不重复 FK 传播逻辑，直接读取符号 `EdgeGraph` 的 `propagate()` 输出
- `SymbolicFK(edgeGraph, endFrame)`：从 pose map 中抽取 `endFrame` 的 4×4 `sym` 位姿，拆解为 `TSym` / `PosExpr` / `RotExpr`，通过 `symvar()` 自动提取 `JointVars`
- `eval(vals)`：代入数值 joint 值，返回 4×4 double
- `evalPos(vals)` / `evalRot(vals)`：分别返回位置和旋转分量

### 计算层：`+core/PoseGraph` (全部 static)

- **无状态，纯数学**：输入边 + 种子位姿 → 输出全局位姿 map
- `propagatePoses(edges, seed)`: 迭代 FK 传播（支持多根、支持环路悬空节点，自动兼容 `double`/`sym`）
- `jointTransform(kind, axis, value)`: 构造 revolute / prismatic 关节的 4×4 齐次变换（`value` 为 `double` 时输出 `double`，为 `sym` 时输出 `sym`——Rodrigues 公式中的 `sin`/`cos` 对 `sym` 多态）

## 数据流

### mechanism.m 路径（机构装配 + 可视化）

```
specs/dsl/examples/*.yaml
        │
        ▼  ir.Expander(dslYaml, configYaml)   ← 一行调用，内部完成全部展开
   Expander 公开属性：
        │
        ├─► e.MechName, e.Instances, e.ConnectionInfo
        ├─► e.Poses          (containers.Map: frame → 4×4)
        ├─► e.SymbolVars     (struct, numeric 模式下为空)
        └─► e.EdgeGraph_     (ir.EdgeGraph, 含所有边 + ground nodes)
        │
        ▼  viz/mechanism.m 渲染循环
        │
        ├─► poses(f.node)  →  画 triad / geometry patch
        ├─► connInfo       →  画 mate 诊断线段 (gap / Zdot)
        └─► unplaced 列表   →  警告不连通分量
```

### Expander 内部管线（构造函数内完成）

```
DSL YAML 文件
        │
        ▼  core.readYaml()
   DSL struct (mechanism / instances / connections)
        │
        ▼  加载模块库 + dimensions.yaml + joint_config.yaml
   ┌────────────────────────────────────────────┐
   │  localExpandInstance() × N instances       │
   │    ├─► g.addFixedTransform()   (bidir)     │
   │    ├─► g.addJoint()            (bidir)     │
   │    │     numeric mode: value from config   │
   │    │     symbolic mode: sym('inst.var')    │
   │    ├─► g.addGround()  (auto: semantic_tag) │
   │    └─► 注册到 obj.SymbolVars (sym mode)    │
   └────────────────────────────────────────────┘
        │
        ▼  遍历 connections
   ┌────────────────────────────────────────────┐
   │  polarity check → socket/plug 识别          │
   │  closed: false  →  g.addMate()             │
   │  closed: true   →  g.addClosedMate()       │
   └────────────────────────────────────────────┘
        │
        ▼
   g.propagate()
        │
        ├─► 构造 seed map (ground → eye(4))
        ├─► g.toStruct()  过滤 closed_mate → from/to/T 数组
        └─► PoseGraph.propagatePoses()  →  poses: containers.Map
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
   g.propagate()  ──► poses: containers.Map
        │
        ▼
   visualization loop
        ├─► poses(f.name)  →  画 triad / geometry patch
        └─► g.isFramePending(f.name)  →  品红色 pending 样式
```

`module.m` 路径是 `mechanism.m` 的简化版：不经过 Expander（无多实例拼接、无 inter-instance mate 边），直接操作 EdgeGraph。单根（第一个 body）。

### 符号 FK 路径（A.3.2 新增）

```
DSL YAML 文件
        │
        ▼  ir.Expander(dslYaml, '', true)   ← symbolicMode=true
   e.EdgeGraph_ 含 sym 类型的 joint 边
        │
        ▼  ir.SymbolicFK(e.EdgeGraph_, endFrame)
   fk.TSym     (4×4 sym)
   fk.PosExpr  (3×1 sym)
   fk.RotExpr  (3×3 sym)
   fk.JointVars (sym array, via symvar)
        │
        ▼  数值代入
   fk.eval([q1; q2; ...])  →  4×4 double
   fk.evalPos([q1; q2; ...])  →  3×1 double
   fk.evalRot([q1; q2; ...])  →  3×3 double
```

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

### 5. 为什么 `Expander` 独立于 `+viz`

| 维度 | Expander | +viz/mechanism.m |
|------|----------|------------------|
| 状态 | handle class，有状态（EdgeGraph_、DefCache_） | function，无状态 |
| 职责 | DSL→IR 展开、参数注入、图构建、FK 传播 | 纯渲染：triad、geometry、诊断线 |
| 消费者 | `+viz/mechanism.m`（数值）、`+ir/SymbolicFK`（符号） | 终端用户 |
| 变更原因 | DSL 语法演进、新模块类型、符号变量管线 | 渲染样式、新几何格式 |

A.3.0 之前，DSL 解析和 EdgeGraph 构建逻辑全部内嵌在 `+viz/mechanism.m` 的 setup 区段和 local functions 中。
抽离为独立 `Expander` 后：
- 可视化层退化为纯消费者（只读 public 属性）
- `SymbolicFK` 可复用同一套展开逻辑——只需切换 `symbolicMode` flag
- 测试可以独立构造 Expander，不启动图形窗口

### 6. 为什么 `SymbolicFK` 是薄封装而非独立引擎

`SymbolicFK` 不重复 FK 传播逻辑。它依赖两个已有事实：
1. `EdgeGraph` 的 `propagate()` 对 `sym` T 矩阵透明（MATLAB 多态运算符）
2. `Expander(symbolicMode=true)` 已将 joint 边构建为符号变换

因此 `SymbolicFK` 的唯一增量工作是：从 `propagate()` 生成的 pose map 中抽取 `endFrame` 的符号位姿并拆解。
这种设计避免了符号管道和数值管道的代码分叉——同一套 `EdgeGraph` + `PoseGraph` 服务于两种模式。

### 7. 为什么 `PoseGraph` 保持独立

| 维度 | EdgeGraph | PoseGraph |
|------|-----------|-----------|
| 状态 | handle class，有状态 | 全部 static，无状态 |
| 职责 | 累积边、管理元数据 | 纯数学计算 |
| 变更原因 | DSL 格式变化、新边类型 | FK 算法改进、新关节类型 |
| 可测试性 | 需要构造完整对象 | 给定输入即得输出 |

合并会违反单一职责原则，降低可测试性。保持分层使得每层可以独立演进和测试。

## 运动学核心概念 —— DSL 可视化运作原理

本节深入解释机构如何从 YAML DSL 描述转化为全局位姿，以及闭环、诊断等机制的工作原理。
DSL→IR 展开由 `ir.Expander` 完成（原 A.3.0 前内嵌在 `mechanism.m` 中），可视化层仅消费展开结果。

### 1. 位姿传播模型：生成树 + 弦边

机构装配（mechanism assembly）的核心计算是一个**正向运动学（FK）**问题：给定各关节变量值，求每个 frame 在世界坐标系下的 4×4 齐次位姿。

当 DSL 描述的机构包含**闭环**（kinematic loop）时，简单 FK 会遇到矛盾——同一 frame 可以通过不同路径被赋予不同位姿。`Expander` 在连接处理阶段采用标准的**生成树（spanning tree）+ 弦边（chord）**策略来解决：

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

**代码实现**（`Expander.m` 连接处理区段，L120-140）：

```matlab
if ~isClosed
    obj.EdgeGraph_.addMate(sk.node, pl.node, roll, sym);      % 生成树边：双向，传播位姿
else
    obj.EdgeGraph_.addClosedMate(sk.node, pl.node, roll, sym); % 弦边：单向，仅记录
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

位姿传播的起点是**根节点（ground node）**，由 `Expander` 在构造函数末尾处理：

```matlab
% Expander constructor, after all instances + connections processed:
if ~obj.EdgeGraph_.hasGroundNodes()
    obj.EdgeGraph_.addGround(obj.Instances(1).bodies{1}.node);  % fallback
end
obj.Poses = obj.EdgeGraph_.propagate();
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

此流水线（由 `ir.Expander` 编排）将声明式的 YAML DSL 描述完整转化为可交互的 3D 可视化，同时提供闭环残差诊断，帮助验证机构设计的几何正确性。

符号 FK 路径复用同一套 `Expander → EdgeGraph → PoseGraph` 管线，仅切换 `symbolicMode=true`，由 `ir.SymbolicFK` 在出口处将符号位姿 map 转化为可分解、可代入的符号表达式。
