# IR 节点类型规范（阶段 A.3.1）

> 本文档定义 IR（中间表示）图中的节点类型：`body`、`frame`、`joint` 记录以及 `root` 节点。
> 权威代码来源：`scripts/matlab/+ir/Expander.m`（`localExpandInstance`）和 `+ir/EdgeGraph.m`（`RootNodes`）。
> 元件级定义见 `../modeling-conventions.md` §3；机器可读枚举见 `../conventions.yaml`。
>
> **状态**：A.3.1 v0。以已验证的 MATLAB 代码为准反推，非从零设计。

---

## 1. 概述

IR 图中的**节点**分为两类：

| 类别 | 存储位置 | 说明 |
|------|------|------|
| 显式节点 | `Expander.Instances(i).bodies` / `.frames` / `.joints` | 每个模块实例展开后产出的 body、frame、joint 记录 |
| 隐式节点 | `EdgeGraph.RootNodes` | FK 传播的根节点（seed pose = I₄）列表 |

所有节点名均为**实例限定名**（instance-qualified name），格式为 `instanceName.elementName`（如 `frame0.body`、`joint1.linkA`），由 `localExpandInstance` 在展开时通过名前缀 `pre = [iname '.']` 生成。

---

## 2. Body 节点

### 2.1 来源

Body 是 L0 元件 [`body`](../conventions.yaml) 的 IR 表示。每个模块实例的 body 在 `localExpandInstance` 中展开为 struct：

```matlab
% Expander.m L196-201
bList{k} = struct('node', [pre b.name], 'name', b.name, ...
    'geometry', core.CommonUtils.field(b, 'geometry', ''));
```

### 2.2 字段

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `node` | char | 是 | 实例限定名，IR 图中的唯一标识，如 `frame0.body` |
| `name` | char | 是 | 模块定义中的原始 body 名（无前缀），如 `body` |
| `geometry` | char | 否 | STEP/STL 几何文件相对路径；空字符串表示无几何 |

### 2.3 语义

- Body 是**几何载体**：每个 body 隐含一个中心坐标系（与 body 同名），所有附着在该 body 上的 frame 均通过 `fixedTransform` 相对此中心坐标系定位。
- Body **无自由度**（dof=0）：自身不引入关节变量。
- Body 节点名在 `EdgeGraph.Edges` 中作为 `from` / `to` 出现（fixedTransform 和 joint 边的端点可以是 body 或 frame）。

---

## 3. Frame 节点

### 3.1 来源

Frame 是 L0 元件 [`frame`](../conventions.yaml) 的 IR 表示。每个模块实例的 frame 在 `localExpandInstance` 中展开为 struct：

```matlab
% Expander.m L204-217
fList{k} = struct('node', node, 'name', f.name, ...
    'exposed', isfield(f, 'exposed') && isequal(f.exposed, true), ...
    'polarity', core.CommonUtils.field(f, 'polarity', ''), ...
    'semantic_tag', core.CommonUtils.field(f, 'semantic_tag', ''), ...
    'symmetry', core.CommonUtils.field(f, 'symmetry', 4));
```

### 3.2 字段

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `node` | char | 是 | 实例限定名，如 `frame0.faceXPlus` |
| `name` | char | 是 | 模块定义中的原始 frame 名（无前缀），如 `faceXPlus` |
| `exposed` | logical | 是 | `true` 表示对外暴露的端口（Port），可参与跨模块连接 |
| `polarity` | char | 否 | 端口极性：`'socket'` / `'plug'` / `''`（非端口 frame 为空） |
| `semantic_tag` | char | 否 | 语义标签：`'ground'` / `'joint_side'` / `'connector'` / `'tool'` / `'dock'` / `''` |
| `symmetry` | double | 否 | 绕 +Z 的旋转对称阶；默认 4（方钢片 C4）；约束连接 `roll` 取值 0..symmetry-1 |

### 3.3 Port（暴露帧）

当 `exposed = true` 时，frame 即为**端口（Port）**——模块对外唯一的交互界面。

- **极性约束**：连接仅允许 `socket ↔ plug`。socket 为凹面（接收端），plug 为凸面（插入端）。无 polarity 的 frame（如 `tip_origin` 任务参考系）不参与机械连接。
- **对称性**：`symmetry` 字段约束装配时的离散滚转选择数。
- **法向约定**：所有 port 的 +Z 统一朝模块外（见 `../modeling-conventions.md` §9.1）。

### 3.4 Root 自动注册

当 `semantic_tag = 'root'` 时，该 frame 在展开时自动注册为 root node：

```matlab
% Expander.m L299-305
if strcmp(core.CommonUtils.field(f, 'semantic_tag', ''), 'root')
    obj.EdgeGraph_.addRoot(node);
end
```

Root node 在 FK 传播时以 $T = I_4$（世界原点）为初始位姿。

> **标签语义分离**：`semantic_tag: root` 和 `semantic_tag: ground` 是两个不同的标签：
> - `root`：标记 FK 传播起点，触发 `addRoot()` 自动注册。典型用途：`ToolPipette.tip_origin`
> - `ground`：标记 L3 世界绑定端点（如 `Manipulator.ground`），用于执行层识别哪些 frame 绑定到世界原点。**不**触发自动 root 注册。

> **工具端生长范式（Tool-Rooted Growth）**
> 模块化机构的自然装配方向是从工具端向外生长：先确定工具模块（如 `ToolPipette`）的参考系（`semantic_tag: root`），沿连接链向外逐步定位各模块，最终抵达 `Manipulator` 外部驱动端。
>
> 若所有模块均无 `semantic_tag: root` frame，则 fallback 到第一个实例的第一个 body 作为 root（见 `dsl-to-ir-mapping.md` §7 Root Fallback）。支持多 root（多次调用 `addRoot()`），适用于多分支 / 并联机构。

---

## 4. Joint 记录

### 4.1 来源

Joint 记录来源于模块定义中的 `joints` 列表，在 `localExpandInstance` 中展开：

```matlab
% Expander.m L229-245
jList{k} = struct('node', [pre j.from_frame], ...
    'axis', ax(:) / max(norm(ax), eps), ...
    'var', j.variable, 'val', val, 'kind', kind);
```

### 4.2 字段

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `node` | char | 是 | 关节的 `from_frame` 实例限定名，如 `joint1.hingeA` |
| `axis` | 3×1 double | 是 | 关节轴单位方向向量（已归一化） |
| `var` | char | 是 | 关节变量名，如 `'q'`（revolute）或 `'dx'`/`'dy'`/`'dz'`（prismatic） |
| `val` | double | 是 | 关节变量当前数值（未提供时默认 0） |
| `kind` | char | 是 | 关节类型：`'revolute'` 或 `'prismatic'` |

### 4.3 语义

- **Joint 不是独立节点**：在 IR 图中，关节通过 `EdgeGraph.addJoint` 转化为两条双向 `joint` 边（`from_frame → to_frame` 和 `to_frame → from_frame`）。
- **关节变量**：`var` 为变量名（不含实例前缀），完整限定名为 `instanceName.var`（如 `joint1.q`）。`val` 为当前展开时的数值。
- **零位约定**：`val = 0` 时关节变换为单位阵 $T = I_4$（见 `../modeling-conventions.md` §3.4）。

---

## 5. Root 节点

### 5.1 存储

Root 节点存储在 `EdgeGraph.RootNodes` cell array 中：

```matlab
% EdgeGraph.m L37
RootNodes (:,1) cell = {}
```

每个元素为 frame 实例限定名（char）。

### 5.2 注册方式

1. **自动注册**：展开时 `semantic_tag = 'root'` 的 frame → `addRoot(node)`（Expander.m L299-305）
2. **手动注册**：调用方直接 `g.addRoot(node)`（如 `module.m` 中以第一个 body 为根）

### 5.3 Fallback 规则

若展开后无任何 root node，以第一个实例的第一个 body 为根：

```matlab
% Expander.m L166-168
if ~obj.EdgeGraph_.hasRootNodes()
    obj.EdgeGraph_.addRoot(obj.Instances(1).bodies{1}.node);
end
```

### 5.4 FK 传播行为

Root node 在 `EdgeGraph.propagate()` 中以 $T = I_4$ 为种子位姿。支持多 root node（多分支/并联机构）。

```matlab
% EdgeGraph.m L121-123
if ~isempty(obj.RootNodes)
    for k = 1:numel(obj.RootNodes)
        seed(obj.RootNodes{k}) = eye(4);
    end
```

> **工具端生长范式（Tool-Rooted Growth）**：root node 通常是工具模块（如 `ToolPipette`）的参考 frame，机构从工具端开始沿连接链向外生长，最终抵达 `Manipulator` 驱动端。详见 §3.4。

---

## 6. 命名规则

### 6.1 实例限定名

所有 IR 节点名遵循格式：

```
<instanceName>.<elementName>
```

- `instanceName`：DSL `instances` 中的键名（如 `frame0`、`joint1`）
- `elementName`：模块定义中的 body/frame 名（如 `body`、`faceXPlus`、`linkA`）
- 分隔符：`.`（点号）

### 6.2 前缀生成

```matlab
% Expander.m L192
pre = [iname '.'];
```

### 6.3 端口引用解析

DSL 连接中的 `instance.port` 引用通过点号切分：

```matlab
% Expander.m L254-259
d = strfind(ref, '.');
instName = ref(1:d(1)-1);
portName = ref(d(1)+1:end);
```

---

## 代码对照表

| 规范条目 | 代码位置 |
|------|------|
| Body struct 字段 | `Expander.m` L196-201 (`bList{k} = struct(...)`) |
| Frame struct 字段 | `Expander.m` L204-217 (`fList{k} = struct(...)`) |
| Joint struct 字段 | `Expander.m` L236-245 (`jList{k} = struct(...)`) |
| Root 自动注册 | `Expander.m` L299-301 (`semantic_tag == 'ground'`) |
| Root fallback | `Expander.m` L166-168 (`hasRootNodes()`) |
| RootNodes 存储 | `EdgeGraph.m` L37 (`RootNodes (:,1) cell`) |
| FK 种子位姿 | `EdgeGraph.m` L121-123 (`seed(...) = eye(4)`) |
| 名前缀生成 | `Expander.m` L192 (`pre = [iname '.']`) |
| 端口引用解析 | `Expander.m` L254-259 (`localParsePort`) |
| Body 元件定义 | `conventions.yaml` → `element_types.nodes.body` |
| Frame 元件定义 | `conventions.yaml` → `element_types.nodes.frame` |
| Joint 元件定义 | `conventions.yaml` → `element_types.edges.joint` |
| Port 语义约定 | `modeling-conventions.md` §9 |
