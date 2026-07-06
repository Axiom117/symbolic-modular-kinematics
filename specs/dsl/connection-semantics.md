# 连接语义（DSL 层）（阶段 A.2.1）

> 本文档定义端口连接在 DSL 层的精确语义：标准 mate 变换在 YAML 中如何表达、`roll` 参数、
> `closed` 标记的解释器行为，以及连接不能承载的语义。
> 权威几何约定见 `../modeling-conventions.md` §9（端口）、§10（连接）；机器可读常量见 `../conventions.yaml`。
> 语法规则见 `grammar.md`。
>
> **状态**：A.2 v0。mate 变换公式为 A.0 冻结项，本文档不重新定义，仅说明其 DSL 层表达。

---

## 1. 连接的唯一含义

一条 DSL 连接的含义**唯一**：把两个端口**面对面对插贴合**。连接本身**不携带任意空间变换、
不引入运行时自由度**。所有连接产生的几何都由下面的标准 mate 变换给出——用户在 DSL 中只声明
「哪两个端口相连」，几何由解释器统一施加。

---

## 2. 标准 mate 变换（冻结）

### 2.1 公式

设连接的 `socket` 端为父、`plug` 端为子，二者局部 `+Z` 均朝模块外。子端口坐标系相对父端口坐标系的位姿：

$$T_{\text{plug} \leftarrow \text{socket}} = R_z\!\left(\text{roll} \cdot \tfrac{360^\circ}{\text{symmetry}}\right) \cdot R_x(\pi), \qquad t = 0$$

- **`Rx(π)`（冻结）**：绕端口 `+X` 翻转 180°。把 `+Z` 翻成反平行（两法向面对面）、`+Y` 翻转、`+X` 不变。
  这实现「面对面对插」而非「坐标系数值相等」。
- **`Rz(...)`**：绕公共对插法向的离散滚转（§3）。
- **平移为零**：两对接面在原点相触；任何法向间隙属于模块几何，不属于连接。

### 2.2 DSL 层如何触发

DSL 连接**顺序无关**（`grammar.md` §5.3）：

```yaml
connections:
  - ports: [frame0.faceXPlus, joint1.linkA]
```

解释器处理流程：

1. 读取两端口的 `polarity`（来自各自模块定义）。
2. 校验极性互补：仅 `socket↔plug` 合法（§4）。
3. **自动定向**：把 `socket` 端（如 `frame0.faceXPlus`）取为父，`plug` 端（如 `joint1.linkA`）取为子。
4. 施加 §2.1 的 mate 变换，在 IR 中插入一条桥接边，把两端口 frame 面对面重合。

因此 `[frame0.faceXPlus, joint1.linkA]` 与 `[joint1.linkA, frame0.faceXPlus]` 完全等价——父/子由极性决定，
不由书写顺序决定。

### 2.3 socket 提供者

当前模块库中，`socket` 端口仅由两类模块提供（`modeling-conventions.md` §9.5）：

| socket 提供者 | 端口 |
|------|------|
| `Frame` | 六面 `faceX±`、`faceY±`、`faceZ±` |
| `Manipulator` | `dock` |

其余一切机械对接面（`Pin.sideA/B`、`Joint.linkA/B`、`Adaptor.attachment_point/pin_connector`、
`ToolPipette.connector_side`）均为 `plug`。故合法连接必有一端来自上表。

---

## 3. roll 参数

### 3.1 语义

- `roll` 是**连接级**整数字段（缺省 0），归属连接而非端口。
- 实际滚转角 = `roll × 360 / symmetry` 度，绕公共对插法向（`+Z`）。
- `symmetry` 为端口的旋转对称阶（缺省 4，方钢片 C4）；合法 `roll` 取值 `0 .. symmetry-1`。
- `symmetry = 4` 时，`roll ∈ {0, 1, 2, 3}` 对应 `0° / 90° / 180° / 270°`。

### 3.2 DSL 表达

```yaml
connections:
  - ports: [frame1.faceXPlus, joint2.linkA]
    roll: 1        # 装配时绕对插法向额外转 90°（symmetry=4）
```

### 3.3 roll 的可观测性

- 对**结构面**（`Frame`/`Pin`）：`roll` 在对称群内无差别，纯装配记号，不改变可求解几何。
- 对 **`Joint` 端口**：端口背后藏有打破对称的关节轴，同一个合法 `roll` 会**同步转动关节轴**——
  这正是无需「扭 90° 垫片模块」即可改变关节轴朝向的机理。

### 3.4 roll 只离散

`roll` 永远落在对称群内，机械上必然装得上。**真正需要运行时连续绕法向转动**的场合，
不用 `roll`，而是插入一个 revolute `Joint` 模块（轴取公共法向）——那是真自由度（§5）。

---

## 4. 极性门控

- 连接仅允许 `socket↔plug`。`socket↔socket`、`plug↔plug`、以及任一端无极性（任务系如
  `ToolPipette.tip_origin`、接地系如 `Manipulator.ground`）均由解释器直接判非法。
- 极性只是**逻辑门控**，决定连接是否被允许，不产生几何；连接几何仍由 §2 的 mate 变换给出。
- 每个端口**至多被占用一次**；重复占用为非法。

---

## 5. 连接不能承载的语义

连接不携带任意变换或自由度。以下需求**必须插入模块**，不得赋予连接新语义：

### 5.1 坐标适配 → 插入 `Adaptor`

若两端口面对插后朝向/位置不满足需求，需要额外的固定平移或旋转，则在两端口间插入 `Adaptor` 模块，
由其内部 `fixedTransform` 承载所需重定向，两侧各连接一次：

```yaml
# 错误：试图让连接自带一个 45° 旋转 —— DSL 无此语法，也不允许
# 正确：插入 Adaptor
connections:
  - ports: [manip.dock, adaptor.attachment_point]
  - ports: [adaptor.pin_connector, frameBase.faceZPlus]
```

### 5.2 连续绕法向旋转 → 插入 revolute `Joint`

若需运行时绕对插法向连续转动（真自由度），插入 revolute `Joint` 模块，关节轴取公共法向 `+Z`。
该自由度由 `Joint` 元件显式描述，与 §3 的离散 `roll`（装配朝向）语义分离。

---

## 6. closed 标记的解释器行为

### 6.1 语义

- `closed: true` 声明该连接是**闭环补边（chord）**：它在 L2 机构图中闭合了一个回路。
- 缺省 `false`（树边）。

### 6.2 解释器如何处理

被 `closed: true` 标记的连接，在运动学传播中**照常参与**——它和普通连接一样施加 §2 的 mate 变换。
区别在于 **A.4 闭环阶段**：解释器识别该连接为回路补边，在此处**切开**回路，生成一个 `Constraint`。

#### 6.2.1 残差公式

$$T_{\text{residual}} = T_{\text{far}}^{-1} \cdot T_{\text{near}}$$

物理场景：`closed: true` 连接的切口两侧有两个坐标系 `F_near` 和 `F_far`。物理上若环路闭合，
二者应为空间中同一坐标系；但解释器沿两条不同支路分别从共同祖先（或 `world`）正向传播到这两个
坐标系，得到两个位姿：

| 符号 | 含义 |
|------|------|
| $T_{\text{near}}$ | `F_near` 在 `world` 下的位姿，沿**支路 A**（树边侧）传播 |
| $T_{\text{far}}$ | `F_far` 在 `world` 下的位姿，沿**支路 B**（补边侧）传播 |
| $T_{\text{residual}}$ | `F_near` 在 `F_far` 坐标系下的**相对位姿**（$F_{\text{far}} \to \text{world} \to F_{\text{near}}$） |

环路物理闭合时 $T_{\text{residual}} = I$（单位矩阵），即平移分量 $(t_x, t_y, t_z) = (0,0,0)$、
旋转分量为零。实际上因为关节变量未必满足闭环条件，$T_{\text{residual}} \neq I$，求解器的目标
就是调节关节变量将其压到零。

从 $T_{\text{residual}}$ 中提取 6 个候选残差分量 $[t_x, t_y, t_z, r_x, r_y, r_z]$
（平移 mm，姿态 Z-Y-X 欧拉角 rad）。**不是所有 6 个都要归零**——具体约束哪些分量子集
由**机构 DOF 分析与 L3 执行配置的 `constrained_components`** 决定，**不在 DSL 中声明**
（`grammar.md` §1.2、§5.5）。

#### 6.2.2 `closed` 标签在下游流水线中的意义

`closed` 是一个**纯标记位**，本身不携带几何或求解参数，但在后续每一层都有精确分工：

| 流水线阶段 | `closed` 的作用 |
|------|------|
| **A.3 IR 展开** | 与普通连接一样施加 mate 变换，IR 图完整含环，不做切开 |
| **A.3.2 IR 校验** | 验证 `closed: true` 连接确实闭合 L2 回路（不悬空）、独立回路数与标记数一致（不欠标/多标） |
| **A.3.3 开环 FK** | 作为「不可处理」信号：含 `closed: true` 的机构不能走纯开环 FK 管线，须路由到 A.4 |
| **A.4 回路识别** | 显式指定切口位置，区分树边与补边（chord），避免解释器自行猜测切口 |
| **A.4 约束构造** | 切口两侧 frame 对 $(F_{\text{near}}, F_{\text{far}})$ 直接取自补边连接的两端；每个 `closed: true` → 一个独立回路 → 一个 `Constraint` |
| **A.5 求解器** | 不直接读取 `closed`；它接收 A.4 输出的符号残差表达式与变量分区，用 `fmincon` SQP 数值求解 |

`closed` **不携带**的信息（全部归 L3 execution-config）：
- 切口哪些分量需要归零（`constrained_components`）
- 哪些关节是驱动/未知量（`actuated_joints` / `external_drivers`）
- 收敛阈值、初值等求解配置

#### 6.2.3 mate 变换与关节角度的关系

**mate 变换始终正确施加，不受关节角度影响。** mate 变换 $R_z(\text{roll}) \cdot R_x(\pi)$
是纯几何操作，只依赖连接级 `roll` 和端口 `symmetry`，与关节变量完全无关。它唯一的作用是把
两个端口坐标系面对面贴合（`+Z` 反平行、原点重合）。

错位的真正来源不在 mate 变换，而在**全局 FK 传播**：mate 变换保证了局部对接正确，但若关节
变量不满足闭环约束，沿两条支路分别传播到切口两侧 frame 的全局位姿就会不一致——两个 frame
的原点和朝向在空间中不重合。这不是 mate 变换的失败，而是「关节值不闭合」的几何后果。

#### 6.2.4 DSL 可视化中的闭环行为

A.2.5 可视化不做闭环求解，只做装配渲染：

- **所有连接（含 `closed: true`）都施加 mate 变换**，机构被完整装配成一张全局图。
- 关节变量的具体数值通过 `mechanism_viz_config.yaml` 注入；若未提供，默认取零位。
- **开环机构**：零位默认值即正确装配。
- **L2 闭环机构**（如四杆环）：零位是特意设计为满足闭环的参考构型（A.4.4 过关标准要求
  「残差表达式经手工代入零位构型验证恒为零」）。若用户提供非零位的关节值且不满足闭环约束，
  `closed: true` 连接处会出现肉眼可见错位——此时可视化忠实地反映了「这组关节值不闭合」的
  事实，并非渲染错误。
- **L3 世界系闭环机构**（如 M-REx 主构型）：DSL 中**没有 `closed: true`**，机构本体是
  挂在两个 `Manipulator` 之间的开环链。两个 `Manipulator.ground` frame 在 L3 绑定到同一
  `world` 原点——可视化中它们重合在原点处即表示「闭合」。详见 §6.4。

### 6.3 多闭环

- 每个 `closed: true` 连接对应**一个独立回路切口**。
- N 个独立回路 → N 个 `closed: true` 连接 → N 个 `Constraint`。
- 对「M 条支链连接同两个刚体」的并联结构，独立回路数 = `M − 1`，故其中 `M − 1` 条支链的闭合连接
  标 `closed: true`（选哪条支链作补边不影响物理等价性，仅影响切口位置）。

### 6.4 L2 闭环 vs L3 世界系闭环

`closed` **只标记在 L2 机构图内部即闭合的回路**（如四杆环、并联平台的支链环）。
若回路仅在 L3 绑定 `world` 与外部驱动后才闭合（M-REx 主构型：世界原点 → 驱动 #1 → 机构 → 驱动 #2 → 世界原点），
则**不写 `closed`**——该闭合由 L3 execution-config 的 `closure_cuts` 声明（`grammar.md` §5.7）。

#### 6.4.1 M-REx 主构型：闭合发生在世界原点

M-REx 主构型的实际拓扑是：

```
world 原点 ──[ground]── Manipulator1 ──[dock]── Adaptor ── 机构本体 ── Adaptor ──[dock]── Manipulator2 ──[ground]── world 原点
```

关键特征：

- 两个 `Manipulator` 各有一个 `ground` frame（`semantic_tag: ground`，无极性）。
- 在 L3 execution-config 中，两个 `ground` frame 都绑定到同一个 `world` 原点。
- 机构本体（两个 `Adaptor` 之间的模块链）是**开环的**——DSL 中没有任何 `closed: true`。
- 「回路」的形成是因为两条独立树（`world → Manipulator1 → 机构` 和 `world → Manipulator2 → 机构`）共享同一个 `world` 根和同一段机构本体——闭合判据不是端口对插，而是两条支路到达机构本体同一点时位姿应一致。

#### 6.4.2 两种闭合的对比

| | L2 内部闭环（四杆环） | L3 世界系闭环（M-REx） |
|---|---|---|
| 闭合位置 | 两个模块端口之间 | 两个 `ground` frame 绑到同一 `world` 原点 |
| DSL 中是否写 `closed` | **是**，在连接级标记 | **否**，DSL 中机构本体是开环链 |
| 切口声明位置 | DSL `connections` 的 `closed: true` | L3 execution-config 的 `closure_cuts` |
| 约束生成 | A.4 在补边连接的端口 frame 对处切开 | A.4 在 L3 指定的切口位置切开（如机构本体两端） |
| 可视化中的闭合表现 | 一条 mate 边连接两端口，含 `closed` 标记 | 两个 `Manipulator` 的 `ground` frame 重合在 `world` 原点；机构本体是开环链挂在两者之间 |
| 可视化错位表现 | `closed: true` 连接处端口不重合 | 机构本体两端的位姿不一致（从 Manipulator1 传播和从 Manipulator2 传播得到的位姿不相等） |

#### 6.4.3 M-REx 构型在可视化中的表现

A.2.5 可视化读取 DSL 机构文件 + L3 world_binding 后：

1. 两个 `Manipulator.ground` frame 都放在 `world` 原点（重合）。
2. 分别沿 `Manipulator1` 和 `Manipulator2` 的内部链做 FK 传播。
3. 机构本体挂在其中一侧（或两侧分别传播后取其一侧渲染）。
4. 没有 `closed: true` 边需要渲染——两张 `ground` frame 在原点重合就是「回路闭合」的几何表现。

如果 Manipulator 的关节值（`dx, dy, dz`）使得机构本体两端位姿不一致，可视化不会像 L2 闭环
那样显示一条错位的 mate 边，而是表现为机构本体两端不连续——这同样是「关节值不闭合」的
忠实反映。

#### 6.4.4 M-REx 构型的约束生成（A.4）

虽然 DSL 中没有 `closed: true`，A.4 仍然需要为此构型生成闭环约束。流程是：

1. L3 execution-config 声明 `closure_cuts`：指定切口位置（如机构本体两端的两个 frame）。
2. A.4 沿两条支路分别传播到切口 frame 对：
   - 支路 A：`world → Manipulator1 → … → F_near`
   - 支路 B：`world → Manipulator2 → … → F_far`
3. 构造 $T_{\text{residual}} = T_{\text{far}}^{-1} \cdot T_{\text{near}}$，与 §6.2.1 同构。
4. 由 `constrained_components` 指定须归零的分量子集；`external_drivers` 指定 Manipulator
   的 `dx/dy/dz` 为未知量。

因此从 A.4/A.5 求解器的视角看，L2 闭环和 L3 闭环的**残差公式完全一致**——区别仅在于
切口位置是谁指定的（DSL `closed` vs L3 `closure_cuts`），以及未知量是谁（L2 内部关节
vs L3 外部驱动关节）。

---

## 7. 与手工推导对照（验证用）

对任一连接 `[socket_port, plug_port]`，解释器施加的桥接变换应满足：

- 平移分量为 0（两对接面原点相触）。
- 子端口 `+Z` 与父端口 `+Z` 反平行（法向面对面）。
- `roll = 0` 时子端口 `+X` 与父端口 `+X` 对齐；`roll = k` 时子端口绕公共法向再转 `k·360/symmetry` 度。

A.2.5 可视化脚本据此检查：对插处两端口坐标系原点重合、法向相反；A.4 据此验证零位构型下
`closed` 切口残差恒为零。
