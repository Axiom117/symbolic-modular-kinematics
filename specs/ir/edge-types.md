# IR 边类型规范（阶段 A.3.1）

> 本文档定义 IR（中间表示）图中四种边的类型、变换公式、双向插入规则和 FK 传播行为。
> 权威代码来源：`scripts/matlab/+ir/EdgeGraph.m`（所有 `add*` 方法 + `toStruct`）。
>
> **状态**：A.3.1 v0。以已验证的 MATLAB 代码为准反推，非从零设计。

---

## 1. 概述

IR 图是一个**有向图**，边存储在 `EdgeGraph.Edges` struct 数组中。每种边在插入时遵循特定的双向/单向规则，在 FK 传播时通过 `toStruct()` 过滤。

### 1.1 通用边结构

```matlab
% EdgeGraph.m L31 (Edges property) + L205-210 (addEdge)
struct('from', <char>, 'to', <char>, 'T', <4×4 double>, 'kind', <char>)
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `from` | char | 源节点名（实例限定名） |
| `to` | char | 目标节点名（实例限定名） |
| `T` | 4×4 double | 齐次变换矩阵：`pose(to) = pose(from) * T` |
| `kind` | char | 边类型枚举（见 §1.2） |

### 1.2 边类型枚举

| kind | 含义 | 双向 | 参与 FK | toStruct 保留 |
|------|------|------|------|------|
| `'fixed'` | 固定刚体变换 | 是 | 是 | 是 |
| `'joint'` | 关节自由度变换 | 是 | 是 | 是 |
| `'mate'` | 生成树 mate 连接 | 是 | 是 | 是 |
| `'closed_mate'` | 闭环弦边 mate 连接 | 否 | 否 | 否 |

---

## 2. fixed 边

双向插入（`EdgeGraph.addFixedTransform`，L48-53）。变换公式：

$$T = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix}$$

$R$ 由 `rotation` 字段求值（`align`/`rpy`/`axis_angle`），$t$ 由 `translation` 求值（mm）。用于模块内部 body↔frame 刚性连接，参数可为符号表达式（如 `cubeLength/2`）。

## 3. joint 边

双向插入（`EdgeGraph.addJoint`，L58-63）。变换公式：

**revolute**：$T = \begin{bmatrix} R_{\text{axis}}(q) & 0 \\ 0 & 1 \end{bmatrix}$（Rodrigues 公式）

**prismatic**：$T = \begin{bmatrix} I & d \\ 0 & 1 \end{bmatrix}, \quad d = \frac{\text{axis}}{\|\text{axis}\|} \cdot \text{value}$

零位（value=0）时 $T = I_4$。

### 3.3 零位约定

`value = 0` 时 $T = I_4$（恒等变换）。展开参考（如合页从合拢→展开的 180°）烘焙进 fixed 边，不写进 joint。

---

## 4. mate 边

双向插入生成树边（`EdgeGraph.addMate`，L70-82）。

$$T_{\text{plug} \leftarrow \text{socket}} = R_z\!\left(\text{roll} \cdot \tfrac{2\pi}{\text{symmetry}}\right) \cdot R_x(\pi), \qquad t = 0$$

- `Rx(π)`（冻结）：绕 +X 翻转 180°，使两端口 +Z 反平行
- `Rz(...)`：离散滚转（`roll`=0..symmetry-1，缺省 symmetry=4）
- 方向约定：Socket→Plug 为正向（由极性决定，不由 DSL `ports` 书写顺序）

---

## 5. closed_mate 边

**单向插入**（`EdgeGraph.addClosedMate`，L88-98）：仅 `socket→plug`。变换公式同 §4。

- **不参与 FK 传播**：toStruct 过滤（§6），诊断专用（计算闭环残差 gap/Zdot）
- **可视化**：橙色粗虚线

## 6. toStruct 过滤规则

`EdgeGraph.propagate()` 在调用 `PosePropagator.propagatePoses` 前，通过 `toStruct()` 剥离元数据并排除诊断边：

```matlab
% EdgeGraph.m L129-137
function s = toStruct(obj)
    keepMask = ~strcmp({obj.Edges.kind}, 'closed_mate');
    s = obj.Edges(keepMask);
    s = rmfield(s, 'kind');
end
```

**过滤规则**：

1. 排除所有 `kind = 'closed_mate'` 的边（弦边不参与 FK）
2. 剥离 `kind` 字段（FK 引擎只需要 `from` / `to` / `T`）

**输出格式**：`struct` 数组，字段 `{from, to, T}`，直接传给 `PosePropagator.propagatePoses(edges, seed)`。

---

## 7. 双向边与图遍历性

### 7.1 规则

除 `closed_mate` 外，所有边类型在插入时均生成**双向边**（正向 + 逆向逆变换）。这确保图在任意方向可遍历。

### 7.2 逆向变换

```matlab
% EdgeGraph.m L214-217 (localInvT)
function Ti = localInvT(T)
    R = T(1:3,1:3); t = T(1:3,4);
    Ti = eye(4); Ti(1:3,1:3) = R'; Ti(1:3,4) = -R' * t;
end
```

$$T^{-1} = \begin{bmatrix} R^T & -R^T t \\ 0 & 1 \end{bmatrix}$$

### 7.3 各类型边数

| 类型 | 每次调用插入边数 | 总边数（含逆向） |
|------|------|------|
| `addFixedTransform` | 2 | 2 |
| `addJoint` | 2 | 2 |
| `addMate` | 2 | 2 |
| `addClosedMate` | 1 | 1 |

---

## 代码对照表

| 规范条目 | 代码位置 |
|------|------|
| 通用边结构 `{from, to, T, kind}` | `EdgeGraph.m` L31 (`Edges` property) + L205-210 (`addEdge`) |
| `addFixedTransform` 双向插入 | `EdgeGraph.m` L48-53 |
| `addJoint` 双向插入 | `EdgeGraph.m` L58-63 |
| `addMate` 双向插入 + mate 变换 | `EdgeGraph.m` L70-82 |
| `addClosedMate` 单向插入 | `EdgeGraph.m` L88-98 |
| `jointTransform` revolute 公式 | `PosePropagator.m` L12-25 |
| `jointTransform` prismatic 公式 | `PosePropagator.m` L15-19 |
| `toStruct` 过滤规则 | `EdgeGraph.m` L129-137 |
| 逆向变换 `localInvT` | `EdgeGraph.m` L214-217 |
| `Rx(π)` 翻转约定 | `conventions.yaml` → `connection.mate_flip_axis` |
| mate 变换公式 | `conventions.yaml` → `connection.mate_transform` |
