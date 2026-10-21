# 2026-10-20 Ranger 私有 C 协同接入

以下为首次接入记录。用户后续已明确简化家具标注和门/篮子设计，当前行为以 [协同简化记录](ranger_simple_20261020.md) 为准。

本次按 `docs/ranger_handoff.md` 末尾覆盖节和 `docs/协同VLM_Nav框架.md` 实施。补丁：[ranger_private_c_20261020.patch](ranger_private_c_20261020.patch)。不提交、覆盖或回退工作区原有改动。

已将 21 个审查过的源码/测试文件写回本地，并逐一校验 SHA256 与测试副本一致。原文件备份在 `/tmp/ranger-cooperative-20261020/source-backup`；未提交 git，未重建或替换现有部署 overlay。

## 已实现

- `scene.share_c_coordinates=false`：控制组合是 `action=SYNC, kind=SYNC_LOCAL_C`，沿用 schema v1 和原五条话题；位置字段无效且为零，common_frame_id/alignment_id 为空。拒绝私有模式里的坐标命令及旧同步模式。
- Go2 coordinator/RulePlanner 只发任务意图；两端各自找 A/B，严格完成后 READY；双方 READY 后进入本机 C 搜索/缓存检查；双方各有本机 C 引用才进入放入阶段。Go2 放入、整机退出并释放后才安排 Ranger。
- AgentMachine 保存本机有效目标坐标，网络 Observation 的坐标统一清零；只有有效本机 C 发 CACHED。切换任务清空缓存、序号和旧命令状态。接近 C 拒绝引用 A/B 的缓存。
- 公共 `explore_steps` 由 Runner 每 tick 推进；原 `explore` 同步推进同一核心。单机保持原 0.81 m 契约；协同要求当前接近导航 SUCCEEDED、真实停稳、动作终止、base_link 到目标 ≤1.00 m。
- 协同从实际 Nav2 参数服务异步读取目标检查器 XY 容差和规划器容差；检查真实规划路径终点，并预留 XY 容差与安全余量。不会用请求 goal 替代路径终点，也不修改 Nav2 容差。
- 候选继续用各机器人本机地图/家具标注生成，并由本机 VLM 选择 P ID。全局地图必须覆盖完整 footprint/旋转空间；局部窗口不足不单独否决远端候选，但局部已覆盖部分的碰撞/未知以及缺图、陈旧、缺 TF 仍失败。
- 私有 live runtime 将八个顺时针 45° 扫描角度传入真实 adapter；单机配置不变。扫描等待停稳、新鲜 RGB-D、图像时刻 TF，ABC 同图，发现自身目标可提前结束。
- 门许可覆盖过门与整机退出，释放后恢复原主任务；篮子许可在 C 接近前取得并贯穿放入与 EXIT。EXIT 保留放入命令的 parent/token；held token 不允许被其他 grant 覆盖。整机未退出不释放。
- Ranger live 分支通过现有 RangerAdapter/factory 接入公共 Runner；新增 `ranger_cooperative.launch.py`，只启动本机 agent。健康检查包括本机地图参数/摘要的周期确认、执行路径、停稳和本机运动占用；过期、失联或 adapter 推进异常进入停车。
- 保留原消息文件、桥接 YAML、旧通信入口和 Ranger 硬件/相机/Nav2 参数。修复原生 action 结束清空执行路径时，Runner 尚未消费 terminal 结果的时序问题。

## 构建与入口

本轮仅在 `/tmp/ranger-cooperative-20261020` 隔离构建，未覆盖已安装 overlay。写回源码后，按本机既有流程重新构建 `multi_agent_nav` 并 source 对应 overlay；现有接口包可继续使用，协议字段未改变。运行 Python 必须与本机 ROS CPython ABI 一致。

Ranger 端在原底盘、相机、TF、定位和 Nav2 已启动的情况下附着；关闭旧自主探索入口：

```bash
ros2 launch multi_agent_nav ranger_cooperative.launch.py config_dir:=/实际/Ranger配置目录
```

本机为 Domain0。Go2 端部署相同公共任务语义并运行唯一 coordinator；两端仍通过各自定向 bridge 接 Domain40。启动 agent、coordinator 与开始任务是不同操作，launch 不自动下发任务。

## 当前仍关闭的现场门槛

仓库默认 `scene.enabled=false`，保留未验证标志；不能直接据此启动真实协同任务。

- `robot_ranger.exploration_strategy` 在协同配置中选 `generated_candidates_vlm`，填写本机实际 `generated_candidates.semantic_yaml`。本机目前没有家具标注文件，不能伪造；其 `source_robot` 必须为 ranger，源地图路径和摘要必须一致。
- 实测填写本机 stop_margin，确认 footprint、camera_calibrated、costmap_source_verified、path_source_verified、vision_validated、peer_obstacle_verified。
- 在本机地图配置 basket/门的 resources、门 transit_exits 和 Ranger exit_points。两端相同资源 ID 必须指同一实物，不交换多边形或复制对方坐标。
- C 位置与篮子保护区必须一致；车身从未进入保护区时，不会伪造 ENTERED/释放。退出路线需先离开篮子再进入下一保护资源；不支持同时持有两个许可。
- 保留各端实际地图身份/摘要和模型环境配置。只在补齐并确认现场参数后设置 scene.enabled；私有模式不要求跨机标定。
- Ranger 网卡与 Domain0/40 隔离按现场真实接口配置；沿用五条 Reliable/Volatile 定向任务路由，不桥接 TF、图像、costmap、速度或 Nav2 action。

## 验证记录

新增测试位于 `tests/test_private_c.py`、`test_local_adapter_contract.py`、`test_ranger_task_entry.py`；覆盖完整双机 fake 顺序、门释放、持有许可失联冻结、网络坐标边界、真实路径终点预算、异步暂停、旧身份隔离、Ranger live 接线和真实 adapter 扫描配置。

修改前后用同一离线选择比较公共测试，已有失败集合完全一致，没有新增失败。没有运行 ROS 节点集成/真实通信/真机/API 调用；已存在的 ROS 节点测试也未作为本轮通过证据。

最终结果：新增 37 项测试全部通过；公共包 382 passed、13 个原有失败、1 skipped、5 deselected；单机 Python 191 passed。`multi_agent_nav` 在隔离目录构建通过（复用本机已安装的同 schema 消息包）。

写回后直接在原仓库联合复验：573 passed、13 failed、1 skipped、5 deselected。13 个失败名称与修改前完全一致。测试 XML、日志及构建产物保存在 `/tmp/ranger-cooperative-20261020`；这些临时证据不是部署目录。

公共包原有的 13 项失败为：

- `test_dynamic_costmap_config.py::test_effective_cooperative_costmaps_keep_dynamic_layers_and_other_parameters`
- `test_dynamic_costmap_config.py::test_actual_navigation_launch_applies_optional_overlay_last`
- `test_dynamic_costmap_config.py::test_isolated_acceptance_launch_has_only_costmaps_and_lifecycle_manager`
- `test_dynamic_costmap_config.py::test_no_verification_flags_are_set_by_static_costmap_profile`
- `test_dynamic_costmap_config.py::test_static_collection_dry_run_contains_only_read_commands`
- `test_field_preparation.py::test_empty_resources_allow_independent_exploration_but_not_cooperative_live`
- `test_field_preparation.py::test_global_goal_age_config_must_be_positive_finite` 的六个参数化用例
- `test_field_preparation.py::test_offline_single_robot_config_does_not_require_cooperative_certification`

前三/第五项依赖本机未包含的 Go2 工具、launch 和脚本；其余涉及 Go2 专用地图路径、既有验证标志及大小写断言。本轮保留这些实际限制，没有修改配置来制造通过。
