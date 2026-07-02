# specs/modules — 模块库文本定义（阶段 A.1）

> L1 模块库的文本优先定义。每个 `*.yaml` 是一个模块类（`module_type`），
> 由 [../schema/module-definition.schema.yaml](../schema/module-definition.schema.yaml) 校验，
> 遵循 [../modeling-conventions.md](../modeling-conventions.md) 与
> [../schema/conventions.yaml](../schema/conventions.yaml)。

## 内容

| 文件 | 模块 | 类别 | DOF |
|------|------|------|------|
| [frame.yaml](frame.yaml) | `Frame` 立方体结构件 | structural | 0 |
| [pin.yaml](pin.yaml) | `Pin` 销钉连接件 | structural | 0 |
| [joint.yaml](joint.yaml) | `Joint` 铰接关节件 | kinematic | 1 |
| [adaptor.yaml](adaptor.yaml) | `Adaptor` 坐标适配件 | structural | 0 |
| [pipette_body.yaml](pipette_body.yaml) | `Pipette_body` 工具末端件 | structural | 0 |
| [slx-to-text-mapping.md](slx-to-text-mapping.md) | SLX → 文本 一一映射表 | — | — |

## 三层数据结构

每份模块定义按 A.1 要求显式分为三层（见 modeling-conventions.md §2.2）：

1. **模块定义表** —— 顶层字段 `module_type` / `category` / `dof` / `geometry` / `parameters`：模块是什么。
2. **端口框架表** —— `frames`（其中 `exposed: true` 即 port）：模块对外暴露哪些局部坐标系。
3. **内部运动学图** —— `bodies` + `fixed_transforms` + `joints`：端口如何经刚体变换与关节连到内部参考体。

## 冻结状态

`extraction_status` 标注数据是否已从参考冻结：

- `complete` —— 全部数值已冻结（当前仅 `Frame`）。
- `partial` —— 含 `provisional: true` / `rotation: { pending: true }` 项，待源文件
  `mrex_hypercube_modules.slx` 二进制可用后核实。详见 mapping 表第 0 节。

## 校验

```sh
# 任选一个 JSON Schema 校验器，例：check-jsonschema（pip install check-jsonschema）
check-jsonschema --schemafile specs/schema/module-definition.schema.yaml specs/modules/*.yaml
```
