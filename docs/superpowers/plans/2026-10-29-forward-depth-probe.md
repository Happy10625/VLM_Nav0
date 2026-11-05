# Ranger 前向深度探测实现计划

> 执行方式：在当前已修改工作区内按 executing-plans 内联执行，保留已有3米排除区修改。

**Goal:** 深度无效不转动重观测，只向当前车头方向导航2米一次，到达后原地最多3次无效观察，失败后返回原观察点续扫。
**Architecture:** RangerSingleRobotExecution保存往返起点和无效次数，复用Nav2规划/运动与既有暂停检查点；RangerAdapter保存原方向观察记录并在返程完成后续接下一方向。VLMNavigator只补充返程kind的通用失败处理，通用单机旧策略保持。
**Tech Stack:** Python、ROS2 Humble、Nav2、pytest。
**Spec:** 用户本轮要求及确认“只前进一次，在新位置观察3次”。有效深度沿用现有3帧目标确认与接近；路径无法执行仍安全停止，不绕过规划或运动监护。

## 范围与验证

- 修改 vlm_nav/ranger_single_robot_execution.py 的深度恢复、结果处理、往返终态；cooperative_ws/src/multi_agent_nav/multi_agent_nav/adapters/ranger.py 的扫描错误处理与返回续扫；vlm_nav/vlm_navigator.py 的返程失败kind。
- 配置沿用target_probe_distance=2.0、target_probe_attempt_limit=3（Ranger解释为到达后的无效观察上限），保留通用节点原参数语义并更新注释。
- test/test_ranger_single_robot_parity.py 中旧旋转/目标像素方向行为改为本轮明确授权例外；新增 test/test_ranger_depth_probe.py 验证新状态链。
- 观察点原姿态和扫描索引须保留；Nav2滚动分段未完成不能提前观察或续扫；旧回调、暂停/恢复不得重复推进或丢失起点。
- 导航失败、无有效路径和取消不能伪装返程成功；不重启机器人，不做真机运动。
- Go2参考机上一轮SSH失败（No route to host）；本轮未取得可比较的新实现，不宣称对齐其现状。

- [x] 写失败测试：前向2米不旋转、3次无效返程、有效深度接近、原方向续扫、往返失败与暂停。
- [x] 实现最小Ranger覆盖及调用方衔接，更新旧行为测试。
- [x] 运行定向回归，核实同一安装加载源码，更新交接。

## 执行记录

已完成。本轮新增专项18项通过，相关回归317通过/1项Go2专用脚本跳过；未执行硬件运动。独立审查发现部分确认后的投影失败漏计数、sensor恢复丢失原点两项，均经失败测试复现后修复。保留旧非探测sensor恢复策略；探测期间按本轮明确的“只前进一次并返回原点”要求续接原状态。
