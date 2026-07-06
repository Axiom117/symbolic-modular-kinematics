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

### 2.1 插入规则

双向插入：`from→to`（正向）和 `to→from`（逆向，取 $T^{-1}$）。

```matlab
% EdgeGraph.m L48-53
function addFixedTransform(obj, from, to, T)
    obj.addEdge(from, to, T, 'fixed');
    obj.addEdge(to, from, localInvT(T), 'fixed');
end
```

### 2.2 变换公式

$T$ 由模块定义中的 `translation` + `rotation` 组合而成：

$$T = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix}$$

- $R$：由 `rotation` 字段求值（支持 `align` / `rpy` / `axis_angle`，见 `RigidBodyMath.rot`）
- $t$：由 `translation` 字段求值（3×1 向量，单位 mm）

### 2.3 用途

- 模块内部 body↔frame 之间的刚性连接
- 模块内部 body↔hinge 之间的展开参考变换
- 参数化：`translation` 和 `rotation` 可为符号表达式（如 `cubeLength/2`），在展开时通过参数注入求值为数值

---

## 3. joint 边

### 3.1 插入规则

双向插入，同 `fixed`。

```matlab
% EdgeGraph.m L58-63
function addJoint(obj, from, to, axis, value, kind)
    T = core.PoseGraph.jointTransform(kind, axis, value);
    obj.addEdge(from, to, T, 'joint');
    obj.addEdge(to, from, localInvT(T), 'joint');
end
```

### 3.2 变换公式

**revolute**（转动副）：绕轴 `axis` 旋转角度 `value`（rad）

$$T = \begin{bmatrix} R_{\text{axis}}(\text{value}) & 0 \\ 0 & 1 \end{bmatrix}$$

其中 $R_{\text{axis}}(\theta) = I + \sin\theta \cdot K + (1-\cos\theta) \cdot K^2$（Rodrigues 公式，`RigidBodyMath.axang`）。

**prismatic**（移动副）：沿轴 `axis` 平移距离 `value`（mm）

$$T = \begin{bmatrix} I & d \\ 0 & 1 \end{bmatrix}, \quad d = \frac{\text{axis}}{\|\text{axis}\|} \cdot \text{value}$$

```matlab
% PoseGraph.m L12-25
function T = jointTransform(kind, ax, val)
    switch lower(kind)
        case 'prismatic'
            n = norm(ax);
            if n < eps; d = [0;0;0]; else; d = ax(:)/n * val; end
            T = core.RigidBodyMath.T(eye(3), d);
        otherwise  % revolute
            T = core.RigidBodyMath.T(core.RigidBodyMath.axang(ax, val), [0;0;0]);
    end
end
```

### 3.3 零位约定

`value = 0` 时 $T = I_4$（恒等变换）。展开参考（如合页从合拢→展开的 180°）烘焙进 fixed 边，不写进 joint。

---

## 4. mate 边

### 4.1 插入规则

双向插入（生成树边）：`socket→plug` 和 `plug→socket`（逆向取 $T_m^{-1}$）。

```matlab
% EdgeGraph.m L70-82
function addMate(obj, socket, plug, roll, symmetry)
    rollAngle = roll * 2 * pi / symmetry;
    Tm = core.RigidBodyMath.T( ...
        core.RigidBodyMath.rotz(rollAngle) * core.RigidBodyMath.rotx(pi), ...
        [0; 0; 0]);
    obj.addEdge(socket, plug, Tm, 'mate');
    obj.addEdge(plug, socket, localInvT(Tm), 'mate');
end
```

### 4.2 变换公式（冻结）

$$T_{\text{plug} \leftarrow \text{socket}} = R_z\!\left(\text{roll} \cdot \tfrac{2\pi}{\text{symmetry}}\right) \cdot R_x(\pi), \qquad t = 0$$

- **`Rx(π)`**（冻结）：绕端口 +X 翻转 180°，使两端口 +Z 反平行（面对面对插）
- **`Rz(...)`**：绕公共对插法向的离散滚转
- **平移为零**：两对接面原点相触

### 4.3 参数

| 参数 | 默认值 | 来源 | 说明 |
|------|------|------|------|
| `roll` | 0 | DSL 连接字段 | 离散滚转索引，整数 0..symmetry-1 |
| `symmetry` | 4 | socket frame 的 `symmetry` 字段 | 绕 +Z 旋转对称阶 |

### 4.4 方向约定

Socket→Plug 为正向。父/子由极性决定（socket 为父，plug 为子），不由 DSL `ports` 书写顺序决定。

---

## 5. closed_mate 边

### 5.1 插入规则

**单向插入**（弦边 / chord edge）：仅 `socket→plug`，不插入逆向边。

```matlab
% EdgeGraph.m L88-98
function addClosedMate(obj, socket, plug, roll, symmetry)
    rollAngle = roll * 2 * pi / symmetry;
    Tm = core.RigidBodyMath.T( ...
        core.RigidBodyMath.rotz(rollAngle) * core.RigidBodyMath.rotx(pi), ...
        [0; 0; 0]);
    obj.addEdge(socket, plug, Tm, 'closed_mate');
end
```

### 5.2 变换公式

与 `mate` 完全相同（§4.2）。

### 5.3 语义

- **不参与 FK 传播**：`closed_mate` 是闭环的切口（cut），若用于传播会将位姿穿过闭环，导致不一致。
- **诊断专用**：仅用于计算闭环残差（mate gap / Zdot）。在可视化中以橙色粗虚线标出。
- **toStruct 排除**：传播前被过滤掉（§6）。

---

## 6. toStruct 过滤规则

`EdgeGraph.propagate()` 在调用 `PoseGraph.propagatePoses` 前，通过 `toStruct()` 剥离元数据并排除诊断边：

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

**输出格式**：`struct` 数组，字段 `{from, to, T}`，直接传给 `PoseGraph.propagatePoses(edges, seed)`。

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
| `jointTransform` revolute 公式 | `PoseGraph.m` L12-25 |
| `jointTransform` prismatic 公式 | `PoseGraph.m` L15-19 |
| `toStruct` 过滤规则 | `EdgeGraph.m` L129-137 |
| 逆向变换 `localInvT` | `EdgeGraph.m` L214-217 |
| `Rx(π)` 翻转约定 | `conventions.yaml` → `connection.mate_flip_axis` |
| mate 变换公式 | `conventions.yaml` → `connection.mate_transform` |
