# scripts/matlab — 四层架构设计

> 本文档解释 `+core` / `+ir` / `+viz` 三层（外加调用脚本层）之间的职责划分与数据流。

## 架构总览

项目采用**四层架构**，自上而下为：可视化层 → 中间表示层（编排 + 图累积 + 符号 FK）→ 核心计算层。`TaskFrame` 作为符号 FK 的侧路入口，直接读取图累积层的符号位姿。

```mermaid
graph TB
    %% ── 样式定义 ──
    classDef viz fill:#e3f2fd,stroke:#1565c0,stroke-width:2px,color:#0d47a1
    classDef ir  fill:#fff3e0,stroke:#ef6c00,stroke-width:2px,color:#bf360c
    classDef core fill:#e8f5e9,stroke:#2e7d32,stroke-width:2px,color:#1b5e20

    %% ── 可视化层 ──
    subgraph viz_layer["+viz/ 可视化层"]
        direction LR
        mech["<b>mechanism.m</b><br/>机构装配 + 渲染<br/>委托 Expander → 读 Poses / Instances<br/>调用 evaluateNumeric() 获取数值位姿<br/>画 triad / geometry / mate 诊断线"]
        mod["<b>module.m</b><br/>单模块可视化<br/>跳过 Expander<br/>直接操作 EdgeGraph"]
    end

    %% ── 中间表示层 ──
    subgraph ir_layer["+ir/ 中间表示层"]
        direction LR
        expander["<b>Expander</b> · Orchestrator<br/>构造函数即完整管线<br/>① 加载 DSL + module library<br/>② localExpandInstance 实例展开<br/>③ mate / closed-mate 连接处理<br/>④ propagate → Poses (sym)<br/>evaluateNumeric() 延迟数值代入"]
        edgegraph["<b>EdgeGraph</b> · Accumulator<br/>addFixedTransform / addJoint<br/>addMate / addClosedMate<br/>kind 元数据 + root nodes<br/>propagate → toStruct → PosePropagator<br/>兼容 double / sym 混合 T 矩阵"]
        taskframe["<b>TaskFrame</b><br/>符号 FK 薄封装<br/>读 sym 位姿 map<br/>拆 TSym / PosExpr / RotExpr<br/>eval(vals) 数值求值"]
    end

    %% ── 核心计算层 ──
    subgraph core_layer["+core/ 核心计算层"]
        poseprop["<b>PosePropagator</b> · Math Engine<br/>全部 static · 无状态 · 纯数学<br/>propagatePoses() 迭代 FK 传播<br/>jointTransform() revolute / prismatic<br/>自动兼容 double / sym"]
    end

    %% ── 调用关系 ──
    mech       -->|"new Expander(dslYaml)"| expander
    mech       -->|"evaluateNumeric(configYaml)"| expander
    expander   -->|"持有 EdgeGraph_ 句柄<br/>addFixedTransform / addJoint / ..."| edgegraph
    edgegraph  -->|"toStruct() 过滤 closed_mate<br/>→ propagatePoses()"| poseprop
    taskframe  -.->|"读取 sym 位姿 map"| edgegraph
    mod        -->|"addFixedTransform / addJoint / addRoot"| edgegraph

    class mech,mod viz
    class expander,edgegraph,taskframe ir
    class poseprop core
```

> **层间调用链**（编号对应上图）：
> 1. `mechanism.m` 构造 `Expander(dslYaml)` → 构造函数内自动完成 DSL→IR 符号展开和 FK 传播
> 2. 渲染前调用 `e.evaluateNumeric(configYaml)` → 将符号位姿代入关节数值，得到 `double` 位姿 map
> 3. `Expander` 内部持有 `EdgeGraph_` 句柄，所有实例展开/连接处理均写入该图
> 4. `EdgeGraph.propagate()` 委托 `PosePropagator.propagatePoses()` 执行纯数学 FK
> 5. `TaskFrame` 是符号 FK 的独立入口：直接读取 `EdgeGraph` 的 sym 位姿，不经过 Expander
> 6. `module.m` 跳过 Expander，直接调用 `EdgeGraph` 方法构建单模块内部图

## 各层职责

### 可视化层：`+viz/mechanism.m`, `+viz/module.m`

- `mechanism.m`：委托 `ir.Expander` 完成 DSL→IR 符号展开，调用 `e.evaluateNumeric(configYaml)` 将符号位姿代入数值后渲染。自身**只读** `e.Poses`（符号）、`e.Instances`、`e.ConnectionInfo` 等公开属性
- `module.m`：单模块可视化，直接调用 `ir.EdgeGraph` 方法构建模块内部图（不经过 Expander），仍使用数值管线
- 渲染 triad（RGB 坐标三轴）、geometry patch（STEP/STL 几何）、mate 诊断线、关节轴高亮
- **不直接调用 `core.PosePropagator` 的任何方法** — FK 交互通过 `EdgeGraph.propagate()` 完成

### 编排层：`+ir/Expander` (handle class，A.3.0 新增，A.4.0 纯符号化)

- **为什么独立为编排层**：原先 DSL 解析、实例展开、参数注入、连接处理全部内嵌在 `+viz/mechanism.m` 的 setup 区段和 local functions 中。A.3.0 将其抽离为独立的 `ir.Expander`，使可视化层变为纯消费者，符号管线（`TaskFrame`）也可复用同一套展开逻辑。
- **为什么是 handle class**：与 `EdgeGraph` 同理——内部持有 `EdgeGraph_` 和 `DefCache_` 两个可变状态，handle 语义避免在多步展开中反复传入传出。
- **A.4.0 纯符号化**：移除了 `symbolicMode` 开关，关节变量**始终**创建为 `sym` 对象，`Poses` 始终为符号位姿 map。数值代入推迟到调用 `evaluateNumeric(configYaml)` 时执行。
- `Expander(dslYaml, configYaml)`：构造函数即运行完整管线：
  1. 路径解析 + DSL 加载与校验
  2. 模块库路径解析 + 几何参数加载（`dimensions.yaml`，keyed by `module_type`）
  3. 保存 `configYaml` 路径到 `DefaultConfigPath_`（供 `evaluateNumeric` 默认使用）
  4. 实例展开（`localExpandInstance`，private）：加载模块 YAML → 注入几何参数 → 展开 bodies/frames/fixed_transforms/joints（joint 变量始终为 `sym`）→ 名前缀 → 写入 `EdgeGraph_`
  5. 连接处理：极性校验 → 区分 `addMate`（生成树）和 `addClosedMate`（弦边）
  6. Root fallback + FK 传播 → `Poses` map（含 `sym` 位姿）
- `evaluateNumeric(configYaml)`：公开方法，将 `Poses` 中的 `sym` 位姿代入数值 joint 值，返回 `containers.Map`（frame → 4×4 double）。未在 config 中列出的关节变量默认取 0。
- **公开属性**：`MechName`、`Instances`、`ConnectionInfo`、`Poses`（sym）、`LibDir`、`JointVarMap`（canonical name → sym handle）、`EdgeGraph_`

### 图累积层：`+ir/EdgeGraph` (handle class)

- **为什么是 handle class**：MATLAB 的值语义会使多函数调用中的累加器需要反复传入传出。handle class 支持原地修改，调用点干净。
- 封装所有边创建逻辑（固定变换、关节、mate、closed-mate），自动插入双向边
- 管理元数据（`kind`: `fixed` / `joint` / `mate` / `closed_mate`）
- 管理 root nodes（多根支持）
- **支持混合 `double`/`sym` T 矩阵**：MATLAB 的 `*`、`sin`、`cos` 等运算符对 `sym` 多态，`EdgeGraph` 无需区分数值/符号模式即可承载两种管道。
- `propagate()` 方法：
  1. 从 `RootNodes` 构造 seed map（含 fallback 逻辑：无 root node 时用第一条边的 from 作为根）
  2. 调用 `toStruct()` 剥离 `kind` 字段并过滤 `closed_mate`
  3. 委托 `PosePropagator.propagatePoses()` 执行 FK

### 符号 FK 层：`+ir/TaskFrame` (handle class，A.3.2 新增)

- 薄封装：不重复 FK 传播逻辑，直接读取符号 `EdgeGraph` 的 `propagate()` 输出
- `TaskFrame(edgeGraph, endFrame)`：从 pose map 中抽取 `endFrame` 的 4×4 `sym` 位姿，拆解为 `TSym` / `PosExpr` / `RotExpr`，通过 `symvar()` 自动提取 `JointVars`
- `eval(vals)`：代入数值 joint 值，返回 4×4 double
- `evalPos(vals)` / `evalRot(vals)`：分别返回位置和旋转分量

### 计算层：`+core/PosePropagator` (全部 static)

- **无状态，纯数学**：输入边 + 种子位姿 → 输出全局位姿 map
- `propagatePoses(edges, seed)`: 迭代 FK 传播（支持多根、支持环路悬空节点，自动兼容 `double`/`sym`）
- `jointTransform(kind, axis, value)`: 构造 revolute / prismatic 关节的 4×4 齐次变换（`value` 为 `double` 时输出 `double`，为 `sym` 时输出 `sym`——Rodrigues 公式中的 `sin`/`cos` 对 `sym` 多态）

## 数据流

### mechanism.m 路径（机构装配 + 可视化，A.4.0 纯符号）

```mermaid
flowchart TD
    A["specs/dsl/examples/*.yaml"] --> B["e = ir.Expander(dslYaml)<br/>← 纯符号展开"]
    B --> C["Expander 公开属性"]
    C --> D1["e.MechName, e.Instances, e.ConnectionInfo"]
    C --> D2["e.Poses<br/>(containers.Map: frame → 4×4 sym)"]
    C --> D3["e.JointVarMap<br/>(containers.Map: 'inst.var' → sym handle)"]
    C --> D4["e.EdgeGraph_<br/>(ir.EdgeGraph, 含所有边 + root nodes)"]
    D1 & D2 & D3 & D4 --> E["posesNum = e.evaluateNumeric(configYaml)"]
    E --> F["subs() 代入 joint_config.yaml 数值"]
    F --> G["containers.Map: frame → 4×4 double"]
    G --> H["viz/mechanism.m 渲染循环"]
    H --> I1["posesNum(f.node) → 画 triad / geometry patch"]
    H --> I2["connInfo → 画 mate 诊断线段 (gap / Zdot)"]
    H --> I3["unplaced 列表 → 警告不连通分量"]
```

### Expander 内部管线（构造函数内完成，A.4.0）

```mermaid
flowchart TD
    A["DSL YAML 文件"] --> B["core.readYaml()"]
    B --> C["DSL struct<br/>(mechanism / instances / connections)"]
    C --> D["加载模块库 + dimensions.yaml<br/>（仅几何参数）"]
    D --> E["localExpandInstance() × N instances"]
    subgraph expand["实例展开 (per instance)"]
        E1["g.addFixedTransform() (bidir)"]
        E2["g.addJoint() (bidir)<br/>始终 sym('inst.var')"]
        E3["g.addRoot()<br/>(auto: semantic_tag: root)"]
        E4["注册到 obj.JointVarMap"]
    end
    E --> E1
    E --> E2
    E --> E3
    E --> E4
    E1 & E2 & E3 & E4 --> F["遍历 connections"]
    subgraph conn["连接处理"]
        F1["polarity check → socket/plug 识别"]
        F2["closed: false → g.addMate()"]
        F3["closed: true → g.addClosedMate()"]
    end
    F --> F1
    F1 --> F2
    F1 --> F3
    F2 & F3 --> G["g.propagate()"]
    G --> H1["构造 seed map (root → eye(4))"]
    G --> H2["g.toStruct() 过滤 closed_mate<br/>→ from/to/T 数组"]
    G --> H3["PosePropagator.propagatePoses()<br/>→ poses: containers.Map (sym)"]
```

数值化（延迟到渲染/求解时）：

```mermaid
flowchart TD
    A["e.evaluateNumeric(configYaml)"] --> B["加载 joint_config.yaml<br/>（keyed by instance name）"]
    A --> C["通过 JointVarMap 匹配<br/>canonical name → sym handle"]
    A --> D["未列出的关节变量默认取 0"]
    B & C & D --> E["subs(T_sym, vars, vals) → double"]
    E --> F["数值 poses map"]
```

### module.m 路径（单模块可视化）

```mermaid
flowchart TD
    A["specs/modules/*.yaml"] --> B["core.readYaml()"]
    B --> C["Module def struct<br/>(bodies / frames / fixed_transforms / joints)"]
    C --> D1["g.addFixedTransform() → EdgeGraph.Edges (bidirectional)"]
    C --> D2["g.addJoint() → EdgeGraph.Edges (bidirectional)"]
    C --> D3["g.addRoot(rootBody) → EdgeGraph.RootNodes"]
    D1 & D2 & D3 --> E["g.propagate() → poses: containers.Map"]
    E --> F["visualization loop"]
    F --> G1["poses(f.name) → 画 triad / geometry patch"]
    F --> G2["g.isFramePending(f.name) → 品红色 pending 样式"]
```

`module.m` 路径是 `mechanism.m` 的简化版：不经过 Expander（无多实例拼接、无 inter-instance mate 边），直接操作 EdgeGraph。单根（第一个 body）。

### 符号 FK 路径（A.3.2 新增，A.4.0 适配）

```mermaid
flowchart TD
    A["DSL YAML 文件"] --> B["e = ir.Expander(dslYaml)<br/>← 纯符号展开"]
    B --> C["e.EdgeGraph_ 含 sym 类型的 joint 边"]
    C --> D["ir.TaskFrame(e.EdgeGraph_, endFrame)"]
    D --> E1["fk.TSym (4×4 sym)"]
    D --> E2["fk.PosExpr (3×1 sym)"]
    D --> E3["fk.RotExpr (3×3 sym)"]
    D --> E4["fk.JointVars (sym array, via symvar)"]
    E1 & E2 & E3 & E4 --> F["fk.eval(vals) 或<br/>e.evaluateNumeric(configYaml)"]
    F --> G["数值代入"]
    G --> H1["fk.eval([q1; q2; ...]) → 4×4 double"]
    G --> H2["fk.evalPos([q1; q2; ...]) → 3×1 double"]
    G --> H3["fk.evalRot([q1; q2; ...]) → 3×3 double"]
```

## 关键设计决策

### 1. `toStruct()` 剥离元数据

`EdgeGraph.Edges` 内部包含 `kind` 和 `pending` 字段，但 FK 引擎 (`PosePropagator.propagatePoses`) 只需要 `from` / `to` / `T`。`toStruct()` 负责剥离多余字段，保持 FK 引擎接口简洁。

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

`EdgeGraph` 支持多次调用 `addRoot()` 注册多个 root frame。在 `propagate()` 中所有 root frame 以 `eye(4)` 为初始位姿。这支持多分支 / 并联机构。

### 5. 为什么 `Expander` 独立于 `+viz`

| 维度 | Expander | +viz/mechanism.m |
|------|----------|------------------|
| 状态 | handle class，有状态（EdgeGraph_、DefCache_） | function，无状态 |
| 职责 | DSL→IR 展开、参数注入、图构建、FK 传播 | 纯渲染：triad、geometry、诊断线 |
| 消费者 | `+viz/mechanism.m`（数值）、`+ir/TaskFrame`（符号） | 终端用户 |
| 变更原因 | DSL 语法演进、新模块类型、符号变量管线 | 渲染样式、新几何格式 |

A.3.0 之前，DSL 解析和 EdgeGraph 构建逻辑全部内嵌在 `+viz/mechanism.m` 的 setup 区段和 local functions 中。
抽离为独立 `Expander` 后：
- 可视化层退化为纯消费者（只读 public 属性）
- `TaskFrame` 可复用同一套展开逻辑——Expander 始终产生符号位姿，无需模式切换
- 测试可以独立构造 Expander，不启动图形窗口

### 6. 为什么 `TaskFrame` 是薄封装而非独立引擎

`TaskFrame` 不重复 FK 传播逻辑。它依赖两个已有事实：
1. `EdgeGraph` 的 `propagate()` 对 `sym` T 矩阵透明（MATLAB 多态运算符）
2. `Expander` 已将 joint 边构建为符号变换（A.4.0 纯符号化）

因此 `TaskFrame` 的唯一增量工作是：从 `propagate()` 生成的 pose map 中抽取 `endFrame` 的符号位姿并拆解。
这种设计避免了符号管道和数值管道的代码分叉——同一套 `EdgeGraph` + `PosePropagator` 服务于两种模式。

### 7. 为什么 `PosePropagator` 保持独立

| 维度 | EdgeGraph | PosePropagator |
|------|-----------|-----------|
| 状态 | handle class，有状态 | 全部 static，无状态 |
| 职责 | 累积边、管理元数据 | 纯数学计算 |
| 变更原因 | DSL 格式变化、新边类型 | FK 算法改进、新关节类型 |
| 可测试性 | 需要构造完整对象 | 给定输入即得输出 |

合并会违反单一职责原则，降低可测试性。保持分层使得每层可以独立演进和测试。

## 运动学核心概念

DSL→IR 展开由 `ir.Expander` 完成，可视化层仅消费展开结果。以下为核心机制的简要概述，详细规范见 IR 文档：

| 概念 | 概述 | 权威文档 |
|------|------|------|
| **生成树 + 弦边** | 闭环机构用生成树边（`addMate`，双向）传播 FK；弦边（`addClosedMate`，单向）仅用于诊断闭环残差 | `edge-types.md` §4–§5, `port-attachment.md` |
| **Mate 变换** | $T = R_z(\text{roll}) \cdot R_x(\pi), t=0$ — socket→plug 方向，双向插入时逆向取 $T^{-1}$ | `connection-semantics.md` §2, `port-attachment.md` §2 |
| **`kind` 过滤** | `toStruct()` 排除 `closed_mate` 边后再传给 FK 引擎，确保弦边不参与位姿传播 | `edge-types.md` §6 |
| **Root node** | `semantic_tag: root` 的 frame 自动注册；无 root 时 fallback 到第一个 body。`ground` 标签不触发 root 注册 | `node-types.md` §3.4 |
| **FK 传播** | `PosePropagator.propagatePoses` — 迭代传播：从 seed 帧沿边 `from→to` 累积 $T$，遍历全部边直至收敛。自动兼容 `double`/`sym` | `edge-types.md` §6–§7 |
| **特征尺度** | 可视化元素尺寸按 $L = \max(4, 0.20 \times \max\|\mathbf{p}_i\|)$ 自动适配 | 本文档（实现特有） |
| **Mate 诊断** | gap = $\|\mathbf{p}_s - \mathbf{p}_p\|$（理想 0）；Zdot = $\mathbf{z}_s \cdot \mathbf{z}_p$（理想 -1） | `port-attachment.md` §7 |

> 完整的「DSL YAML → IR 图 → FK 位姿 → 3D 渲染」端到端流水线图见 `dsl-to-ir-mapping.md` §1。
