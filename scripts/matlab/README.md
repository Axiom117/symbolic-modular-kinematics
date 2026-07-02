　# scripts/matlab — 模块定义可视化校验

> 用纯 MATLAB（无外部工具箱依赖）读取 `specs/modules/*.yaml` 模块定义，
> 注入参数后构建内部坐标系图并可视化，用于**验证每个模块的文本定义是否能
> 成功生成对应的模块描述**（端口/本体坐标系 + 简单几何体）。

## 文件

| 文件 | 作用 |
|------|------|
| `read_module_yaml.m` | 极简 YAML 子集解析器（已对照 PyYAML 验证，逐文件一致） |
| `visualize_module.m` | 解析单个模块 → 求解坐标系位姿 → 画三元轴 + body 方块 |
| `visualize_all_modules.m` | 批量跑完 `specs/modules/` 下全部模块 |
| `module_viz_config.yaml` | 参数注入配置（按 `module_type` 提供 `cubeLength`/`tipDistance`/关节角等） |

## 用法

在 MATLAB 中 `cd` 到本目录后：

```matlab
% 单个模块（带参数注入）
visualize_module('../../specs/modules/frame.yaml', 'module_viz_config.yaml');

% 不注入参数：仅当模块平移无符号量时可行，否则会报未解析符号错误
visualize_module('../../specs/modules/pin.yaml');

% 一次性校验全部模块
visualize_all_modules();

% 取回数值结果（每个 frame 的 4x4 全局位姿）做无图检查
r = visualize_module('../../specs/modules/joint.yaml', 'module_viz_config.yaml');
disp(r.frames.linkB);
```

## 显示约定

- **三元坐标轴**：X=红、Y=绿、Z=蓝。轴长按机构特征尺寸自适应。
- **body**：半透明蓝色方块（简单几何占位，非真实 STEP）。Frame 模块用 `cubeLength` 定尺寸，其余用默认尺寸。
- **port**（`exposed: true`）：实线三元轴 + 实心点 + 粗体标签。
- **内部 frame**（`exposed: false`，如 Adaptor 的 `rectify`/`align_axis`）：虚线三元轴 + 细标签。
- **pending 旋转**（待 SLX 提取）：洋红色标注 `(pending R)`，旋转按单位阵占位。
- `world` 坐标系画在原点，根 body 与之重合。

每个模块同时在命令行打印一份**坐标系报告**（名称、全局位置、+Z 朝向、是否 pending），即使不看图也能校验。

## 旋转语义（与 `specs/modeling-conventions.md` 一致）

- `align{a,b}`：规则 `s -> d` 表示子坐标系的 `s` 轴对应父坐标系的 `d` 轴；
  `R = DST * SRC'`，第三轴由右手定则补出（§9.4）。
- `rpy = [Rx,Ry,Rz]`：Z-Y-X 内旋，`R = Rz*Ry*Rx`（§8）。
- `axis_angle`：Rodrigues(omega, q)（§8 权威表示）。
- `pending`：单位阵占位，仅作图，提示该旋转数值未冻结。

## 参数注入

`module_viz_config.yaml` 按 `module_type` 分组，提供：

- 模块类几何参数（`cubeLength`、`tipDistance`），把符号平移（`cubeLength/2`、`-tipDistance`）解析为数值；
- 可选关节变量值（`Joint.q`），用于摆姿态展示自由度，缺省取 0（零位）。

未提供的符号会在求值时报明确错误，提示补进配置——这正是「定义能否成功生成描述」的校验点之一。

## 局限

- 解析器只覆盖本项目模块/配置所用的 YAML 子集，非通用 YAML。
- body 仅用方块占位，不加载 STEP 几何。
- 关节默认按零位摆放（可经 config 指定角度）。
