# DSL 校验分界清单（阶段 A.2.1 / A.2.2）

> 本文档逐条列出 DSL 机构文件的校验项，划分为 **Schema 可静态校验** 与 **解释器须校验** 两类。
> Schema = `../schema/mechanism-assembly.schema.yaml`；解释器 = A.3 v0（加载模块库后做跨文件/跨数组校验）。
> 语法规则见 `grammar.md`，连接语义见 `connection-semantics.md`。

---

## 1. Schema 可静态校验（`mechanism-assembly.schema.yaml`）

只需单文件 + 字段类型/正则即可判定，无需加载模块库。

| # | 校验项 | Schema 机制 |
|---|------|------|
| S1 | 顶层含 `dsl_version` / `mechanism` / `instances`，无多余键 | `required` + `additionalProperties: false` |
| S2 | `dsl_version == 0` | `const: 0` |
| S3 | `mechanism` 符合 `^[a-zA-Z][a-zA-Z0-9_]*$` | `$defs.mechanismName.pattern` |
| S4 | `instances` 至少一个，键符合实例名正则 | `minProperties: 1` + `propertyNames` |
| S5 | 每个实例含 `type`，`type` 符合 UpperCamelCase 正则，无多余键 | `instance.required` + `moduleType.pattern` + `additionalProperties: false` |
| S6 | 实例 `parameters` 块不出现在 v0 DSL 中（v0 禁止 variant，参数值由 `specs/modules/config/parameters.yaml` 按 module_type 注入） | `instance.additionalProperties: false` + 无 `parameters` |
| S7 | 每条连接含 `ports`，恰 2 元素，符合端口引用正则 | `connection.required` + `ports.minItems/maxItems` + `portRef.pattern` |
| S8 | `roll` 为非负整数 | `integer` + `minimum: 0` |
| S9 | `closed` 为布尔 | `type: boolean` |
| S10 | 连接无多余键 | `connection.additionalProperties: false` |

---

## 2. 解释器须校验（跨文件 / 跨数组，Schema 无法表达）

需加载模块库（`../modules/*.yaml`）或跨连接数组分析。

| # | 校验项 | 依据 | 失败示例 |
|---|------|------|------|
| I1 | 每个实例 `type` 在模块库中存在 | grammar §4.3 | `type: Frmae`（拼写错误） |
| I2 | 实例名不与 L0 关键字冲突（`body`/`frame`/`fixedTransform`/`joint`/`constraint`/`port`） | grammar §4.2 | 实例名写成 `joint` |
| I3 | 每个端口引用的实例已在 `instances` 中声明 | grammar §5.2 | `ports: [ghost.faceX+, ...]` |
| I4 | 每个端口在其模块定义中存在且 `exposed: true` | grammar §5.2 | `frame0.faceW+`（不存在的面） |
| I5 | 连接两端**极性互补**（仅 `socket↔plug`） | conn-sem §4 | `[joint1.linkA, joint2.linkA]`（plug↔plug） |
| I6 | 无极性端口（任务系/接地系）不得作连接端 | conn-sem §4 | `[pipette.tip_origin, frame0.faceX+]` |
| I7 | 每个端口至多被占用一次 | grammar §6 | 同一 `frame0.faceX+` 连两次 |
| I8 | `roll` 落在 `0 .. symmetry-1` | conn-sem §3.1 | `roll: 4`（symmetry=4） |
| I9 | `closed: true` 连接确实闭合一个 L2 回路（不悬空、有环） | conn-sem §6 | 在树边上误标 `closed` |
| I10 | 独立回路数与 `closed` 标记数一致（不欠标/多标） | conn-sem §6.3 | 3 支链并联只标 1 个 closed |

---

## 3. 三示例预期校验结果

| 示例 | Schema (§1) | 解释器 (§2) | closed 数 | 独立回路数 |
|------|------|------|------|------|
| `open-chain-2r.yaml` | 通过 | 通过 | 0 | 0（纯树） |
| `single-closed-loop.yaml` | 通过 | 通过 | 1 | 1（四杆环） |
| `parallel-prototype.yaml` | 通过 | 通过 | 2 | 2（3 支链 → 2 独立环） |

---

## 4. 校验命令（参考）

Schema 静态校验（需 `check-jsonschema`，Python `jsonschema` 生态）：

```bash
check-jsonschema --schema-file specs/schema/mechanism-assembly.schema.yaml \
  specs/dsl/examples/open-chain-2r.yaml \
  specs/dsl/examples/single-closed-loop.yaml \
  specs/dsl/examples/parallel-prototype.yaml
```

§2 的解释器校验项在 A.3 解释器实现后接入；本阶段先靠人工对照模块库端口表核验。
