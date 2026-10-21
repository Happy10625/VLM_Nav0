# 2026-10-20 探索点选择同步

后续 C 阶段行为已按用户确认改为双方独立接近、无退出，见 [独立 C 同步记录](ranger_independent_c_20261020.md)。本记录中的“保留原退出顺序”是当时范围说明，已被后续变更覆盖；探索点算法保持本记录实现。

用户在阅读远端探索点选择分析后要求据此修改本地代码。本次是现有公共探索核心内的局部行为同步，依据 `pppppst/VLM_Nav0` / `go2-cooperative-task-layer` 提交 `4fc673156c5c985c4d9a9149cfe0b08b1582d90b`，没有整体覆盖远端分支。

## 改动

- `generated_candidates.py`：始终生成已知自由区网格点，叠加有效家具边缘点；完整旋转圆避开障碍、未知、边界及家具多边形；空地点与家具候选保持半步间距。网格间距为 `ceil(max(1.0, 2R) / resolution) × resolution`，R 包含 padding，从半步格子中心采样并应用地图原点旋转。空地点 ID 为 `free/row/col`，家具字段为空；家具点保留原 ID 和朝向。
- 家具标注未配置、文件不存在或合法空家具均允许；已存在的非法文件、错误身份/路径/摘要仍报错。地图及标注资产出现、删除或修改使旧选择失效。
- `go2_exploration.py`：有无家具均每轮按距当前机器人位置的直线距离、稳定 ID 排序；保留现有 costmap、Nav2 路径、旋转空间检查，最多 12 个合格点交给 VLM。没有改成按路径长度排序。
- `adapters/vision.py`：明确空地候选同样可选，无需围绕家具探索；保留双图、目标/提示、最近 12 条历史和仅返回本轮 ID/理由的约束。实拍 RGB-D 不参与这一轮地图选点。
- `go2_check.py` 与几何证据：无标注路径使用 JSON null，不输出字符串 `None`。
- 测试更新家具断言与完整扫描点数，覆盖混合候选；修正测试替身停车时将 adapter 时钟提前到模拟时钟未来一秒的问题，并将该测试辅助函数的总预算提高到 600 秒以覆盖新增候选。真实任务时限未改。

本地 Ranger adapter、地图、footprint、0.20 m stop_margin、环境变量/API、原单机 0.81 m / 协同 1.00 m 验收、扫描与失败冷却、C 缓存和原任务退出顺序均保留。未同步远端独立 C 完成等其他行为。源配置和当前 `build/ranger-local/acceptance-20261015_161341/config` 不变，二者 Ranger 地图均得到 257 个静态候选，旋转半径约 0.4683 m；运行时仍需 costmap 和路径过滤。

## 验证

- 修改前相关集合：172 passed、1 skipped。
- 新增 15 个用例先运行：13 failed、2 passed，失败对应旧实现缺少的行为；修改后全部通过。
- 修改后相关集合：187 passed、1 skipped，包含两端公共 Runner、Ranger 本地探索、旧 semantic_furniture 策略和原简化任务。
- 扩展离线基线：585 passed、13 failed、1 skipped、5 deselected。测试副本：600 passed、13 failed、1 skipped、5 deselected；失败名称集合完全相同，无新增失败。
- 写回原仓库后按同一扩展范围复验：600 passed、13 failed、1 skipped、5 deselected；三次 XML 的失败名称集合一致。
- 扩展检查排除 `test_obstacle_cloud_filter_runtime.py`、`test_ros_smoke.py` 及 native costmap / observer 节点检查；没有初始化现场 ROS 节点或进行真实模型/机器人测试。

13 个既有失败名称如下，保留作为基线，不在本次修改范围内：

```text
test_dynamic_costmap_config.py::test_effective_cooperative_costmaps_keep_dynamic_layers_and_other_parameters
test_dynamic_costmap_config.py::test_actual_navigation_launch_applies_optional_overlay_last
test_dynamic_costmap_config.py::test_isolated_acceptance_launch_has_only_costmaps_and_lifecycle_manager
test_dynamic_costmap_config.py::test_no_verification_flags_are_set_by_static_costmap_profile
test_dynamic_costmap_config.py::test_static_collection_dry_run_contains_only_read_commands
test_field_preparation.py::test_empty_resources_allow_independent_exploration_but_not_cooperative_live
test_field_preparation.py::test_global_goal_age_config_must_be_positive_finite[0.0]
test_field_preparation.py::test_global_goal_age_config_must_be_positive_finite[-1.0]
test_field_preparation.py::test_global_goal_age_config_must_be_positive_finite[nan]
test_field_preparation.py::test_global_goal_age_config_must_be_positive_finite[inf]
test_field_preparation.py::test_global_goal_age_config_must_be_positive_finite[True]
test_field_preparation.py::test_global_goal_age_config_must_be_positive_finite[None]
test_field_preparation.py::test_offline_single_robot_config_does_not_require_cooperative_certification
```

这些失败涉及本机缺少的 Go2 文件/地图及旧断言，没有将其报告为通过。XML 和原文件摘要保存在 `/tmp/ranger-exploration-20261020`；补丁为 [ranger_exploration_20261020.patch](ranger_exploration_20261020.patch)。

## 写回与生效边界

已通过文件系统权限检查写回 4 个生产模块和 2 个测试文件，逐一确认与已测试副本摘要一致；包内其他已有文件（包括配置和 launch）与改前摘要一致。原文件备份在 `/tmp/ranger-exploration-20261020/source-backup`。未替换已安装产物、提交或启动机器人。默认运行入口若仍加载旧安装，需要后续构建后才会使用新源码。

原仓库复验命令（仅离线测试，不保持运行）：

```bash
cd /home/isee601/ws/VLM_Nav
source /opt/ros/humble/setup.bash
source /home/isee601/ws/VLM_Nav/build/ranger-local/install/setup.bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
ROS_LOG_DIR=/tmp/ranger-exploration-20261020-ros-final \
PYTHONPATH=/home/isee601/ws/VLM_Nav:/home/isee601/ws/VLM_Nav/cooperative_ws/src/multi_agent_nav:$PYTHONPATH \
python3 -m pytest -q -p no:cacheprovider test cooperative_ws/src/multi_agent_nav/tests \
  --ignore=test/test_obstacle_cloud_filter_runtime.py \
  --ignore=cooperative_ws/src/multi_agent_nav/tests/test_ros_smoke.py \
  -k 'not native_costmap_marking and not observation_only_node and not readonly_observer_has and not aligned_depth_observer' \
  --tb=no --junitxml=/tmp/ranger-exploration-20261020/source-wide.xml
```

当前该命令退出码为 1，原因是上述 13 个既有失败；评估本次回归时应核对失败集合与基线一致，不能将退出码报告为通过。

固定网格、附近优先和最多 12 点的方式不保证视觉覆盖完整；它没有 frontier、遮挡/视域或信息增益模型，已观察记录只属于本次探索执行。这些行为沿用当前设计，本轮未增加覆盖规划或跨任务持久化。
