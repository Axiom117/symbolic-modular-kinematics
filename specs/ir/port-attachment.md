# 端口连接在 IR 中的表示（阶段 A.3.1）

> 本文档定义端口连接（mate）在 IR（中间表示）层的精确边表示：标准 mate 变换、
> `addMate` vs `addClosedMate` 两种边类型、极性门控规则。
> 权威代码来源：`scripts/matlab/+ir/EdgeGraph.m`（`addMate`、`addClosedMate`）。
> DSL 层的连接语义定义见 `../dsl/connection-semantics.md`。
>
> **状态**：A.3.1 v0。以已验证的 MATLAB 代码为准反推，非从零设计。

---

## 1. 与 `connection-semantics.md` 的关系

| 层级 | 文档 | 定义内容 |
|------|------|------|
| DSL 层 | `../dsl/connection-semantics.md` | 连接在 YAML 中的语法、标准 mate 变换公式、`roll` 参数语义、`closed` 标记含义 |
| IR 层 | **本文档** | mate 变换在 IR 图中的边表示、`addMate`/`addClosedMate` 两种边类型、FK 传播参与规则 |

两文档通过标准 mate 变换公式桥接：

$$T_{\text{plug} \leftarrow \text{socket}} = R_z\!\left(\text{roll} \cdot \tfrac{2\pi}{\text{symmetry}}\right) \cdot R_x(\pi), \qquad t = 0$$

该公式在 `connection-semantics.md` §2.1 中定义，在 IR 层原样使用（`EdgeGraph.addMate` L76-79）。

---

## 2. 标准 mate 变换（IR 实现）

### 2.1 代码

```matlab
% EdgeGraph.m L76-79 (addMate) / L93-96 (addClosedMate)
rollAngle = roll * 2 * pi / symmetry;
Tm = core.RigidBodyMath.T( ...
    core.RigidBodyMath.rotz(rollAngle) * core.RigidBodyMath.rotx(pi), ...
    [0; 0; 0]);
```

### 2.2 矩阵形式

$$T_m = \begin{bmatrix} R_z(\theta) \cdot R_x(\pi) & 0 \\ 0 & 1 \end{bmatrix}, \quad \theta = \text{roll} \cdot \frac{2\pi}{\text{symmetry}}$$

### 2.3 分量分解

- **`Rx(π)`**：绕端口 +X 翻转 180°
  - `+Z → -Z`（法向反平行，面对面对插）
  - `+Y → -Y`（翻转）
  - `+X → +X`（不变）
- **`Rz(θ)`**：绕公共对插法向的离散滚转
  - θ 由 roll 索引和 symmetry 阶数确定
  - 仅离散取值（连续转动须用 Joint 模块）
- **平移 t = 0**：两对接面原点相触

---

## 3. 极性门控

### 3.1 规则

仅 `socket ↔ plug` 合法。连接两端必须极性互补，否则报错。

### 3.2 代码

```matlab
% Expander.m L121-133
polA = fa.polarity; polB = fb.polarity;
if strcmp(polA, 'socket') && strcmp(polB, 'plug')
    sk = fa; pl = fb;
elseif strcmp(polA, 'plug') && strcmp(polB, 'socket')
    sk = fb; pl = fa;
else
    error('ir:Expander:polarity', ...);
end
```

### 3.3 非法组合

| 组合 | 行为 |
|------|------|
| `socket ↔ socket` | 报错 `ir:Expander:polarity` |
| `plug ↔ plug` | 报错 `ir:Expander:polarity` |
| 含无 polarity 端口 | 报错（polarity 为空字符串，不匹配 socket/plug） |

### 3.4 方向约定

- socket 始终为**父**（`from` 端），plug 始终为**子**（`to` 端）
- 由极性决定，**不由** DSL `ports` 书写顺序决定
- `[frame0.faceXPlus, joint1.linkA]` 与 `[joint1.linkA, frame0.faceXPlus]` 完全等价

---

## 4. addMate：生成树边

### 4.1 语义

`addMate` 表示**生成树边**（spanning-tree edge）——机构主干位姿流向的一部分。

### 4.2 特性

| 属性 | 值 |
|------|------|
| 方向 | **双向**（socket→plug + plug→socket 逆变换） |
| kind | `'mate'` |
| 参与 FK 传播 | **是** |
| toStruct 保留 | **是** |
| DSL 对应 | `closed: false`（默认）的连接 |

### 4.3 代码

```matlab
% EdgeGraph.m L70-82
function addMate(obj, socket, plug, roll, symmetry)
    ...
    obj.addEdge(socket, plug, Tm, 'mate');
    obj.addEdge(plug, socket, localInvT(Tm), 'mate');
end
```

---

## 5. addClosedMate：弦边（诊断专用）

### 5.1 语义

`addClosedMate` 表示**弦边**（chord edge）——闭环的切口（cut），不参与位姿传播。

### 5.2 特性

| 属性 | 值 |
|------|------|
| 方向 | **单向**（仅 socket→plug） |
| kind | `'closed_mate'` |
| 参与 FK 传播 | **否** |
| toStruct 保留 | **否**（被过滤） |
| DSL 对应 | `closed: true` 的连接 |

### 5.3 为什么弦边不能用于传播

- 若用弦边传播，位姿会穿过闭环，导致同一 body 通过不同路径获得不一致的位姿
- 闭环残差会错误地落在生成树边上，掩盖真正的闭合误差
- 弦边保持「诊断专用」，其 mate gap 准确反映当前关节配置下的闭环残差

### 5.4 代码

```matlab
% EdgeGraph.m L88-98
function addClosedMate(obj, socket, plug, roll, symmetry)
    ...
    obj.addEdge(socket, plug, Tm, 'closed_mate');
    % 注意：不插入逆向边
end
```

### 5.5 toStruct 排除

```matlab
% EdgeGraph.m L132-133
keepMask = ~strcmp({obj.Edges.kind}, 'closed_mate');
s = obj.Edges(keepMask);
```

---

## 6. 连接参数

### 6.1 roll（离散滚转）

| 属性 | 值 |
|------|------|
| DSL 字段 | `connections[].roll` |
| 类型 | 整数 |
| 默认值 | `0` |
| 取值范围 | `0 .. symmetry-1` |
| IR 变换 | 实际滚转角 = `roll * 2π / symmetry` |

### 6.2 symmetry（旋转对称阶）

| 属性 | 值 |
|------|------|
| 来源 | socket 端口的 `symmetry` 字段 |
| 类型 | 整数 |
| 默认值 | `4`（方钢片 C4 对称） |
| 含义 | 绕 +Z 旋转 `360°/symmetry` 后端口几何不变 |
| 约束 | 连接 `roll` 必须在 `0 .. symmetry-1` 范围内 |

### 6.3 取值示例（symmetry=4）

| roll | 实际角度 |
|------|------|
| 0 | 0° |
| 1 | 90° |
| 2 | 180° |
| 3 | 270° |

---

## 7. 诊断：mate gap 与 Zdot

在可视化中，每条 mate 连接产生两条诊断值：

### 7.1 mate gap

```matlab
% mechanism.m (visualization section)
gap = norm(Ps(1:3,4) - Pp(1:3,4));
```

socket 原点与 plug 原点之间的欧氏距离（mm）。对齐状态下 gap ≈ 0。

### 7.2 Zdot

```matlab
zdot = dot(Ps(1:3,3), Pp(1:3,3));
```

socket 的 +Z 与 plug 的 +Z 的点积。面对面对插时两法向反平行，期望值 Zdot ≈ -1。

### 7.3 诊断用途

| 边类型 | 诊断含义 |
|------|------|
| `mate`（生成树） | gap ≈ 0, Zdot ≈ -1 表示连接正确对齐 |
| `closed_mate`（弦边） | gap 和 Zdot 反映闭环残差——gap 非零表示回路未闭合 |

---

## 代码对照表

| 规范条目 | 代码位置 |
|------|------|
| 标准 mate 变换公式 | `EdgeGraph.m` L76-79 (`addMate`) |
| `Rx(π)` 翻转 | `EdgeGraph.m` L78 (`rotx(pi)`) |
| `Rz(θ)` 离散滚转 | `EdgeGraph.m` L76 (`rotz(rollAngle)`) |
| 平移为零 | `EdgeGraph.m` L79 (`[0;0;0]`) |
| addMate 双向插入 | `EdgeGraph.m` L80-82 |
| addClosedMate 单向插入 | `EdgeGraph.m` L95-97 |
| 极性门控 | `Expander.m` L121-133 |
| 方向约定（极性决定父/子） | `Expander.m` L123-128 |
| roll 默认值 | `Expander.m` L136 (`field(cn, 'roll', 0)`) |
| symmetry 默认值（来自 socket） | `Expander.m` L137 (`field(sk, 'symmetry', 4)`) |
| closed 判断 | `Expander.m` L138 |
| toStruct 排除 closed_mate | `EdgeGraph.m` L132-133 |
| mate gap 计算 | `mechanism.m` (mate diagnostics section) |
| Zdot 计算 | `mechanism.m` (mate diagnostics section) |
| mate 变换公式权威定义 | `connection-semantics.md` §2.1 |
| 极性约定 | `conventions.yaml` → `connection.polarity_rule` |
| symmetry 默认值 | `conventions.yaml` → `port.default_symmetry` |
