# 术语表（阶段 A.0）

> 后续 DSL、模块 schema 与解释器共享的统一术语。中文释义 + 英文规范名（解释器内部 IR 用名）。
> 约定细节见 `modeling-conventions.md`，机器可读常量见 `schema/conventions.yaml`。

---

## 层级架构（建模层次）

自底向上四层（详见 `modeling-conventions.md` §2）。

| 英文规范名 | 层 | 中文 | 含义 |
|------|------|------|------|
| `Element` | L0 | 元件 | 建模原语（`Body`/`Frame`/`Port`/`FixedTransform`/`Joint`/`Constraint`） |
| `Module` | L1 | 模块 | 封装若干元件的参数化模板（= 类） |
| `Mechanism` | L2 | 机构 | 多个模块实例按端口连接组成的机构本体 |
| `Execution` | L3 | 执行层 | 世界系 + 驱动源，闭合成可求解系统 |
| `ExternalDriver` | L3 | 外部驱动 | 提供多自由度力旋量（wrench）的驱动源，用于闭环构型 |
| `ActuatedJoint` | L3 | 驱动关节 | 在执行层被指派为可驱动（`actuated`）的关节，用于开环串联 |
| `Wrench` | L3 | 力旋量 | 外部驱动提供的多自由度广义力/位移输入 |
| `ClosureCriteria` | L3 | 闭环判据 | 回路切口处的相对位姿误差条件，生成求解残差 |

---

---

## 核心运动学原语

| 英文规范名 | 中文 | 含义 |
|------|------|------|
| `Body` | 刚体 | 不可形变的实体，自带中心 `Frame`，模块内部基本单元 |
| `Frame` | 坐标系 | 一个局部右手参考系；位姿由 `FixedTransform` 给出 |
| `Port` | 端口 | `exposed=true` 的 `Frame`，用于跨模块连接；非独立元件 |
| `Joint` | 关节 | 引入自由度的运动副，本阶段以 1-DOF revolute 为主 |
| `FixedTransform` | 固定变换 | 两坐标系间无自由度的刚性位姿关系 |
| `JointTransform` | 关节变换 | 由关节变量参数化的位姿关系 |
| `Constraint` | 约束 | 闭环相容性的代数条件（残差候选项） |
| `SymbolRegistry` | 符号表 | 尺寸参数、任务变量、关节变量的集中登记 |

## 结构与装配

| 英文规范名 | 中文 | 含义 |
|------|------|------|
| `ModuleType` | 模块类 | 模块的类型定义（字段、端口、内部拓扑）；即 L1 `Module` |
| `Instance` | 实例 | 模块类的一次具体化，带实例名与参数 |
| `Connection` | 连接 | 两端口间的 mate 关系（见连接类型） |
| `World` | 世界系 | 机构图唯一根坐标系 |
| `EndFrame` | 末端系 | FK/IK 输出的任务坐标系 |

## 端口语义标签（可选）

| 名 | 含义 |
|------|------|
| `connector` | 结构连接面 |
| `joint_side` | 关节侧连接面 |
| `tool` | 工具/功能参考点 |
| `dock` | 适配/对接面 |

## 连接类型

| 名 | 含义 |
|------|------|
| `coincident` | 两端口坐标系完全重合 |
| `normal_aligned` | 法向对齐，绕共同法向可自由旋转 |
| `adaptor` | 端口间插入显式适配固定变换 |

## 参数作用域

| 名 | 含义 |
|------|------|
| `module-class` | 属于模块类型定义 |
| `instance` | 属于单个实例 |
| `mechanism-config` | 属于整个机构配置 |
