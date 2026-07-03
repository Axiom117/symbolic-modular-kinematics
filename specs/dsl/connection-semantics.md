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
  - ports: [frame0.faceX+, joint1.linkA]
```

解释器处理流程：

1. 读取两端口的 `polarity`（来自各自模块定义）。
2. 校验极性互补：仅 `socket↔plug` 合法（§4）。
3. **自动定向**：把 `socket` 端（如 `frame0.faceX+`）取为父，`plug` 端（如 `joint1.linkA`）取为子。
4. 施加 §2.1 的 mate 变换，在 IR 中插入一条桥接边，把两端口 frame 面对面重合。

因此 `[frame0.faceX+, joint1.linkA]` 与 `[joint1.linkA, frame0.faceX+]` 完全等价——父/子由极性决定，
不由书写顺序决定。

### 2.3 socket 提供者

当前模块库中，`socket` 端口仅由两类模块提供（`modeling-conventions.md` §9.5）：

| socket 提供者 | 端口 |
|------|------|
| `Frame` | 六面 `faceX±`、`faceY±`、`faceZ±` |
| `Manipulator` | `dock` |

其余一切机械对接面（`Pin.sideA/B`、`Joint.linkA/B`、`Adaptor.attachment_point/pin_connector`、
`Pipette_body.connector_side`）均为 `plug`。故合法连接必有一端来自上表。

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
  - ports: [frame1.faceX+, joint2.linkA]
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
  `Pipette_body.tip_origin`、接地系如 `Manipulator.ground`）均由解释器直接判非法。
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
  - ports: [adaptor.pin_connector, frameBase.faceZ+]
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
区别在于 **A.4 闭环阶段**：解释器识别该连接为回路补边，在此处**切开**回路，生成一个 `Constraint`：

$$T_{\text{residual}} = T_{\text{far}}^{-1} \cdot T_{\text{near}}$$

其中 `(F_near, F_far)` 为切口两侧坐标系。残差须归零的具体分量子集（取自 `{tx, ty, tz, rx, ry, rz}`）
由**机构 DOF 分析与 L3 执行配置**决定，**不在 DSL 中声明**（`grammar.md` §1.2、§5.5）。

### 6.3 多闭环

- 每个 `closed: true` 连接对应**一个独立回路切口**。
- N 个独立回路 → N 个 `closed: true` 连接 → N 个 `Constraint`。
- 对「M 条支链连接同两个刚体」的并联结构，独立回路数 = `M − 1`，故其中 `M − 1` 条支链的闭合连接
  标 `closed: true`（选哪条支链作补边不影响物理等价性，仅影响切口位置）。

### 6.4 L2 闭环 vs L3 世界系闭环

`closed` **只标记在 L2 机构图内部即闭合的回路**（如四杆环、并联平台的支链环）。
若回路仅在 L3 绑定 `world` 与外部驱动后才闭合（M-REx 主构型：世界原点 → 驱动 #1 → 机构 → 驱动 #2 → 世界原点），
则**不写 `closed`**——该闭合由 L3 execution-config 的 `closure_cuts` 声明（`grammar.md` §5.7）。

---

## 7. 与手工推导对照（验证用）

对任一连接 `[socket_port, plug_port]`，解释器施加的桥接变换应满足：

- 平移分量为 0（两对接面原点相触）。
- 子端口 `+Z` 与父端口 `+Z` 反平行（法向面对面）。
- `roll = 0` 时子端口 `+X` 与父端口 `+X` 对齐；`roll = k` 时子端口绕公共法向再转 `k·360/symmetry` 度。

A.2.5 可视化脚本据此检查：对插处两端口坐标系原点重合、法向相反；A.4 据此验证零位构型下
`closed` 切口残差恒为零。
