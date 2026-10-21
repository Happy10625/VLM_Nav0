# 2026-10-20 用户要求的协同简化

探索点生成与排序的后续调整以 [探索点同步记录](ranger_exploration_20261020.md) 为准；本记录保留原简化变更及验证证据。

C 阶段现已改为双方独立接近、无自动退出，以 [独立 C 同步记录](ranger_independent_c_20261020.md) 为准；下文 Go2 先行和 EXIT 仅记录当时行为。

本记录覆盖 [首次私有 C 接入](ranger_private_c_20261020.md) 中的家具必填、门/篮子许可和人工退出点要求。任务仍是双方完成 A/B 后 READY，各自找 C，Go2 先放入并退出，再由 Ranger 放入并退出。

## 配置

- 源码与当前 `build/ranger-local/acceptance-20261015_161341/config` 的两端 exploration_strategy 改为 generated_candidates_vlm；semantic_yaml 暂为空，不创建虚假家具标注。
- Ranger stop_margin 使用上述原协同验收目录已填写的 **0.20 m**。Go2 原值保持 1.0 m。
- model、endpoint 和 API 配置不改，继续使用已有环境变量；不读取或输出密钥。
- 当前 acceptance 的 B=纸巾、已有验证标志等其他配置均保留；源码目标描述也保留原值。当前 acceptance 补充 share_c_coordinates=false。
- camera_calibrated 保留现场记录值，移除其启动门控。实际 RGB-D、图像时刻 TF、定位、costmap、执行路径、动作终止和真实停稳检查继续执行。其他已有来源/功能确认门槛及 scene.enabled 不自动修改。

## 执行行为

不标注家具时，生成器在本机占据地图空闲区采样，按 footprint 外接圆与 padding 检查完整旋转空间；未知、障碍和图边界仍否决。候选沿用两图/P ID/VLM 选择接口，优先检查附近候选；不向模型宣称存在家具标注。当前 Ranger 配置地图离线生成 **257** 个静态候选，运行时仍需 costmap 和路径过滤。

如果明确填写了标注文件，则仍校验其 source_robot、地图路径和摘要，错误文件不会被静默忽略。空闲区固定间距采样可能漏掉狭小空隙；后续现场覆盖不足时再调整采样密度或提供家具标注。

私有模式不读取门/篮子区域，不发送或接受 GRANT_RESOURCE，不要求 transit_exits/exit_points；消息 schema 和五条定向话题不改，资源字段为空。旧共享模式保留原兼容实现。

C 搜索和接近继续复用 A/B 的本地核心与 ≤1.00 m 严格验收。放入后的 EXIT 以本机缓存 C 为参考，要求中心距至少达到 `1.00 m + footprint外接圆半径 + padding + stop_margin`，再加实际 XY 容差和既有安全余量选择/验证真实路径终点。规划点由本机地图和 Nav2 决定，无需人工区域或退出点。只有当前 EXIT action 成功、动作终止、真实停稳并确认实际净距后才 exited；EXIT parent 仍绑定本机放入命令，之后才放行下一台。

这会取消窄门区域的跨机互斥，通行依赖各自 Nav2 的本地动态避障；两端须同步同一简化语义，不能与旧许可执行器混跑。

## 验证

离线联合回归：**585 passed、13 个原有失败、1 skipped、5 deselected**。失败集合与修改前完全一致。覆盖无标注旋转地图候选、两端公共 Runner、无区域完整任务、退出未到位/运动中/ABORTED 不放行 Ranger，以及退出边界浮点误差。

独立代码审查未发现新的重要问题；一个退出采样浮点边界问题已复现并修复，未放宽实际验收净距。未启动 ROS 节点、机器人、bridge 或模型 API，未替换已安装产物，也未提交 git。

补丁见 [ranger_simple_20261020.patch](ranger_simple_20261020.patch)。测试副本、日志与 XML 位于 `/tmp/ranger-simple-20261020`。

已写回 21 个源码/测试及当前验收配置文件，逐一校验摘要与测试副本一致；原文件备份在 `/tmp/ranger-simple-20261020/source-backup`。写回后在原仓库联合复验结果同上，13 个失败名称与修改前完全一致。
