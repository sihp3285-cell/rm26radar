# 共享场地区域

这里保存场地标注的唯一来源，随 `radar27_bringup` 安装到
`share/radar27_bringup/assets/regions`。导航网格仍在 `assets/generated`。
由 launch 将文件路径传给消费者，不在业务包中复制多边形。
自定义 `assets_dir` 部署也需采用这个目录结构。

## 文件与消费者

| 文件 | 用途 | 当前消费者 |
| --- | --- | --- |
| `home.yaml` | 基地附近盲区，不等于整个己方半场 | Position Prior、RViz、Decision |
| `gully.yaml` | 沟区及其中心对称区域 | Position Prior、PoseNode、RViz、Decision |
| `engineer.yaml` | 工程盲区 | Position Prior、RViz、Decision |
| `engineer_home.yaml` | 工程回家后的取矿锚点，点列表而非多边形 | Position Prior、RViz |
| `other_home.yaml` | 非工程车辆回家后的候选区域 | Position Prior、RViz |
| `middle.yaml` | 中央战术区域 | Decision（区域统计） |
| `left_highland.yaml` | 标注视角的左侧高地 | Decision（区域统计） |
| `right_highland.yaml` | 标注视角的右侧高地 | Decision（区域统计） |
| `flying_slope.yaml` | 飞坡区域 | Decision（区域统计） |

## 格式与兼容

已有五份先验资源保持原始内容与文件名。多边形使用 `blind_zones`，
`engineer_home.yaml` 使用点列表；现有消费者按这些格式读取。
尤其 Position Prior 当前通过文件名 `home.yaml` 识别回家盲区，不要随意改名。

新增战术多边形统一使用：

```yaml
regions:
  - name: middle
    mirror_centrally: false
    polygon:
      - [x, y]
      # 至少三个顶点，沿边界顺序排列
```

新增文件原有的 `same_elevation_as_last` 和
`max_height_delta_from_last_m` 暂时保留，避免丢失标注信息；它们属于
先验候选的高度约束，不自动成为战术判定规则。
不要将 `regions` 文件加入现有 `blind_zone_paths`，现有读取器要求
`blind_zones`。Decision 已复用 home、gully 原文件的几何部分，
不要复制顶点。状态阈值、持续时间等放在 decision/config 中。

## 坐标与标注维护

- 场地平面坐标单位为米，范围约为 x ∈ [0, 28]、y ∈ [0, 15]；
  不直接等同于雷达世界坐标 `(world_x, world_z)`。
- 既有先验区域采用红方 canonical 坐标。新增区域本次保留全部原始顶点、
  顺序和镜像设置；接入前需在底图上确认其与 canonical 坐标一致。
- `mirror_centrally: true` 表示同时生成 `(28-x, 15-y)` 的区域，
  不表示 UI 左右翻转。不要按文件名擅自增加镜像。
- `left/right` 目前是标注名称；己方阵营改变时的战术左右含义，需在
  decision 中明确映射，不能靠文件名推断。
- 飞坡现只有区域边界，尚未提供助跑方向、入口、出口或通过事件规则。
- 修改共享边界会影响所有消费者；若战术范围与盲区确实不同，应新增
  有独立名称和含义的区域，而不是让同名副本逐渐分叉。

全局战况 1.0 已启用区域统计。Decision 排除 engineer_home、other_home，
其余区域统一按固定 field 坐标加载；配置与判定说明见 radar27_decision/README.md。
