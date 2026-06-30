# URDF/Xacro 模块化机构参考报告

> 对象：`docs/reference/urdf_module_reference/` 下的 M-REx 模块库与机构装配。
> 目的：提炼其端口定义、模块封装、实例化与连接、闭环耦合写法，作为本项目
> DSL（A.1/A.2）与解释器（A.3/A.4）的对照参考。
> 关联：`modeling-conventions.md`、`module_library_reference.md`、`project-overview.md`。

---

## 1. 文件总览

| 文件 | 角色 | 端口（连接 link） | 自由度 |
|------|------|------|------|
| `base.urdf.xacro` | 机构根 | `connector_A` | 0（固定到 `basePlate`） |
| `link.urdf.xacro` | 被动连杆 | `connector_{ID}A`、`connector_{ID}B` | 0 |
| `joint_C.urdf.xacro` | 转动副（轴 Z） | `connector_{ID}A`、`connector_{ID}B` | 1 revolute |
| `joint_L.urdf.xacro` | 转动副（轴 X） | `connector_{ID}A`、`connector_{ID}B` | 1 revolute |
| `joint_T.urdf.xacro` | T 型三端转动副 | `connector_{ID}A`、`B`、`C` | 1 revolute |
| `joint_V.urdf.xacro` | 万向 2-DOF | `inputConnector_{ID}`、`outputConnector_{ID}` | 2 revolute（pitch+yaw） |
| `gazebo_config.xacro` | 仿真材质/插件 | — | — |
| `m-rex.urdf.xacro` | 机构装配 | 调用全部宏 | 含闭环 |

---

## 2. 端口定义模式

### 2.1 端口 = 一个空 link

每个模块用**空 link**（无 geometry，仅占位）作对外端口，命名固定为 `connector_{ID}{A|B|C}`：

```xml
<link name="connector_${moduleID}A"> ... </link>
<joint name="connectorA_to_proximalPart_${moduleID}" type="fixed">
    <origin xyz="0 0 0" rpy="${-pi} 0 0" />
    <parent link="connector_${moduleID}A" />
    <child  link="proximalPart_${moduleID}" />
</joint>
```

要点：

- 端口是一等公民，但形式上是 link，端口位姿由其与内部刚体之间的 `fixed joint` 的 `origin` 确定。
- 入口端 `connectorA` 一般带 `rpy=-pi 0 0`，把端口坐标系翻转 180°，使下一模块沿 +Z 串接，与 A.0 §5 “端口含原点+主轴+法向”一致。
- 出口端 `connectorB` 通过 `xyz="0 0 link_length"` 平移到杆末端，即“沿 +Z 生长”。

### 2.2 关节模块内部链

`joint_L/C` 内部统一为：

```
connectorA → (fixed,-pi) → proximalPart → (revolute) → distalPart → (fixed,+L) → connectorB
```

差别仅在 `<axis>`：`joint_C` 轴 `0 0 1`、`joint_L` 轴 `1 0 0`。这正对应 A.0 §4“关节用单位轴+转角”——同一模板换轴即得不同关节。

### 2.3 多端口与多自由度

- `joint_T`：在 `proximalPart` 末加第三端口 `connector_{ID}C`，是闭环并联的分叉点。
- `joint_V`：`proximal →(revolute X)→ virtualLink →(revolute Y)→ distal`，两 DOF 间夹零长 `virtualLink`，等价万向节，端口名为 `inputConnector/outputConnector`。

---

## 3. 模块封装：xacro 宏 = 模块类

每个模块是一个 `xacro:macro`，参数即模块类参数（对应 A.0 §7 模块类作用域）：

```xml
<xacro:macro name="joint_L" params="moduleID angle *mimic"> ... </xacro:macro>
```

- `moduleID`：实例命名空间，靠 `_${moduleID}` 拼名做隔离（无真正命名空间）。
- `angle`：装配 `origin rpy` 的安装角，属实例参数。
- `*mimic`：插入块，注入耦合关系（见 §5）。
- 几何尺寸（`link_length` 等）是全局 property，属机构配置作用域。

---

## 4. 机构装配：实例化 + 连接

`m-rex.urdf.xacro` 两步走：先实例化、再连接。

```xml
<xacro:base />
<xacro:joint_C moduleID="1" angle="0"><empty/></xacro:joint_C>
<xacro:connector moduleA="A" moduleB="1A" />
```

连接宏统一为固定关节，含 180° 翻转使两端口面对面：

```xml
<xacro:macro name="connector" params="moduleA moduleB">
  <joint name="connect_${moduleA}_to_${moduleB}" type="fixed">
    <origin xyz="0 0 0" rpy="${-pi} 0 0" />
    <parent link="connector_${moduleA}" /><child link="connector_${moduleB}" />
  </joint>
</xacro:macro>
```

对应本项目 DSL 的 `实例.端口 -> 实例.端口`，mate 类型即 A.0 §6 `coincident`。

---

## 5. 闭环与耦合（最关键）

虽然底层格式是 URDF 树，但用两招做出闭环并联：

1. **冗余连接成环**：除链式 `7C→8A` 外，额外 `7B→4C`、`11B→2B`，把树补成环。
2. **mimic 耦合**：`<mimic joint=... multiplier=±1>` 让被动副镜像主动副，靠等式约束补偿 URDF 不能直接表达闭环。

```xml
<xacro:joint_T moduleID="4" angle="${pi/2}">
  <mimic joint="proximalPart_to_distalPart_2" multiplier="-1.0" />
</xacro:joint_T>
<xacro:connector moduleA="7B" moduleB="4C" />
```

对应 A.4：解释器需识别补边、生成闭环残差，mimic 即等式约束的显式版。

---

## 6. 对本项目的借鉴与差距

**可借鉴**：模块=参数化宏、端口=带固定变换的命名坐标系、装配=实例化+显式连接、入口 `-pi` 翻转串接、mimic≈等式约束、ID 拼名隔离。

**应改进**：端口是空 link 而非一等 Frame（A.0 已升级为 Frame）；ID 拼名易撞，本项目用真命名空间；闭环靠冗余 fixed+mimic 隐式，本项目要显式 loop 标记+残差候选；单位 m、固定 rpy 欧拉，本项目内部用 mm+轴角。

**结论**：M-REx 验证了“宏=模块、空 link=端口、connector=连接、mimic=耦合”可装配含闭环机构；本项目沿用其分层与端口语义，把端口升为 Frame、闭环显式化、欧拉换轴角，即得更适合符号生成的 DSL 骨架。
