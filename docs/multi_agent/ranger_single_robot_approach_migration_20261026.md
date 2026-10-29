# Ranger SingleRobotApproach 最小迁移交付

> 2026-10-27 更新：按用户要求，移除共享接近执行器 `safety_tick` 中的 `max_travel_radius + 0.50` 半径超限停止；Ranger 新单机与协同接近均生效。超时、到达距离、取消及停稳检查保留，探索参数和旧独立 `VLMNavigator` 不变。下文为迁移时的历史记录；类 AST 一致性测试现排除 `safety_tick`，该方法由行为测试验证。

## 仓库与范围

- 接收 remote：`git@github.com:Happy10625/VLM_Nav0.git`；分支 `ranger-reuse-go2-framework`；HEAD `ab922276108c21f44612b7805e26fc0560300528`。
- 来源：[pppppst/VLM_Nav0 固定提交](https://github.com/pppppst/VLM_Nav0/tree/1acae1d56e4e43dac29514f2244d439b760520bf)。读取时 `go2-cooperative-task-layer` 分支头与固定 SHA 一致；实际取文件均指定固定 SHA。
- 开始时工作区已有修改。基线指 **HEAD 加开始时的工作区修改**，不是干净的审查 SHA。原修改备份在 `/tmp/ranger-migration-before`，本次没有 reset、checkout、提交或覆盖已有工作。
- 检查父目录及仓库（含隐藏目录），未找到适用的 AGENTS.md。
- 未改任何运行配置、控制器、速度、平滑/输出链、地图、Keepout、点云过滤、动态障碍确认、相机、定位或通信消息/协议。原先已有的这些文件修改仍保留。

## 实现与来源函数对应

`P = cooperative_ws/src/multi_agent_nav/multi_agent_nav/`。来源路径除注明外都位于相同的 `P`。

| 来源函数/分支 | Ranger 接收实现 | 处理 |
|---|---|---|
| `single_robot_approach.SingleRobotApproach` 全类 | `P/single_robot_approach.py` 同名类 | 逐字复制类主体；AST 完全一致，包含状态、条件顺序、比较符、阈值使用和 token 清理 |
| `navigation_tick`, `plan_then_navigate`, `request_next_plan`, `on_plan_response`, `on_plan_result`, `reject_current_plan` | 同名方法 | 规划到固定目标参考点；接受成功且非空的接近路径；不将请求点改成路径终点；不增加 endpoint budget 或第二次规划 |
| `safety_tick`, `approach_stop_radius`, `begin_approach_stop`, `advance_approach_stop` | 同名方法 | 距离停止优先，随后停稳状态，再行程/超时；保留来源的 `max_travel_radius + 0.50` 看门狗，无新增运动阈值 |
| `request_approach_cancel`, `on_approach_cancel_response`, `on_arrival_odom`, `reset_approach_stop`, `finish_approach_success` | 同名方法 | 取消回执与动作终态分离；平面线速度和 yaw 角速度；移动样本清零；一次距离重试 |
| `send_navigation_goal`, `on_goal_response`, `on_navigation_result`, `cancel_motion`, `close` | 同名方法 + `RangerAdapter.start_approach_navigation` | 接上接收端现有 `_motion(on_accept, on_result)`，不注册第二个 native result future 消费者 |
| `runner.AgentRunner.local_approach` | `RangerAdapter.approach_steps` | 放到 adapter 作为两入口共享的生成器；记录成功对象与 task/command 身份 |
| `go2_exploration.explore_steps.approach_target` | 接收端 `_explore_steps.approach_target` | 已确认坐标传给同一个执行器，保留初始方向观察、三帧确认、候选和扫描流程 |
| 来源 `agent.complete_local_execution` 与 runner 的 deferred completion | `AgentRunner._complete_local`、`local_completion` | 使用接近成功证据替代原生 Nav2 SUCCEEDED；等外层健康、身份、距离、/odom 停稳和动作终态后才发布 ready/deposited |
| 来源 `agent._monitor_path` 与 exploration health_check | 接收端 `health_check` + runner 保持 `LOCAL_EXECUTING` | 仅接近期间用当前位置的 footprint/envelope 检查；候选行驶、探测与扫描的旧路径检查保留 |
| 来源 `adapters/go2.on_odometry` | `RangerSingleRobotExecution.on_arrival_odom` | 复用 Ranger 已有 `/fastlio/odom` 订阅转发，不创建 Go2 `/odometry` 订阅 |
| 来源 `load_parameters` | `load_parameters(files, robot_id='ranger')` | Go2 冻结校验保留；Ranger 读取原配置并校验类型/有限值/合法正值，不将 Go2 参数覆盖到 Ranger |

类 AST SHA256：`fbd8dc96f5222955f91e512757dde268612b05d96c61ae6d7b027e3c44da960d`。参数加载在类外，是唯一对来源模块逻辑的适配；类 AST 校验作为回归测试保存。

## 参数与接口差异

运行时从 Ranger `arrival_config_files` 指向的原 `config/robot.yaml` 读取，未改其数值：

| 参数 | 原 Ranger 值 |
|---|---:|
| 首次距离取消 / 成功距离 | 0.89 / 0.81 m，均使用 `<=` |
| 距离重试上限 | 1；重试停止半径为 0.81 m |
| 停稳线速度 / yaw 角速度 | 0.03 / 0.05 |
| 连续停稳样本 / 到达里程计最大接收年龄 | 3 / 0.5 s |
| 取消终态超时 / 停稳超时 | 3 / 5 s，均保持来源 `>` |
| max_travel_radius / goal_timeout | 3 m / 120 s |
| navigation_tick_period / blocked_goal_seconds | 0.5 / 15 s |

接近样本来自 `/fastlio/odom`，不使用底盘 `/odom` 样本冒充；协作外层原有 `/odom` 0.5 s 停稳持续时间仍保留。执行前核对读取参数中的到达话题与 observer 已订阅的话题一致。未添加传感器配置或里程计发布器。

接收端 `_motion` 已提供 `on_accept(handle, goal_id)` 和 `on_result(status, goal_id)`；Ranger 将它们包装成来源方法需要的 Future。注册回调的动作不再进入 `adapter.results`。本次补充 finish 幂等保护，避免回调异常或重复终态重复消费。传输拒绝/异常沿用接收端 ABORTED/fault 路径，来源算法负责后续失败/取消。

外部取消会关闭执行器、使 plan/goal token 失效；迟到接受仍被取消，动作未终止前 `motions` 不会伪装为空。接近中的 NAV_READY 暂停走停止/取消清理，不将旧动作作为 checkpoint 重发；观察/扫描原有暂停恢复保留。规划回调发动作前重新执行既有健康检查；协作发送前还复查当前 task/command 身份。

旧 endpoint budget、`planner_tolerance + xy_goal_tolerance` 叠加否决、接近二次 compute_path 和接近 `/plan` 消息超时不再参与新执行器。总任务 deadline、costmap/定位/NAV_READY、其他动作 owner、footprint/当前停止包络与协作心跳继续有效，未全局关闭检查。来源内部成功与外层 READY 等待分离，等待期间不再发起距离重试。

## 修改文件

本次相对于开始时的工作区修改：

- 新增 `P/single_robot_approach.py`。
- 修改 `P/adapters/ranger.py`：共享接近生成器、动作回调、完成证据与清理。
- 修改 `P/adapters/nav2.py`：动作 finish 幂等保护（仅两行）。
- 修改 `P/go2_exploration.py`：替换 Ranger 接近分支，限定路径检查适配范围。
- 修改 `P/runner.py`：发送身份复核、完成等待和取消清理。
- 修改 `vlm_nav/ranger_single_robot_execution.py`：到达里程计转发，移除旧接近 tick 的第二个执行者；观察/确认/扫描保留。
- 新增 `scripts/start_ranger_single_robot.sh`。
- 新增 `cooperative_ws/src/multi_agent_nav/tests/test_ranger_approach_parity.py`。
- 新增 `cooperative_ws/src/multi_agent_nav/tests/fixtures/go2_frozen/` 下三个文件：真实方法原文切片及两个仅供测试的参数投影；它们不属于运行配置。
- 新增 `test/test_ranger_approach_transport.py`，适配既有 `test/test_ranger_single_robot_parity.py` 的接近测试入口与外层屏障断言。
- 修改 `CMakeLists.txt`：注册新增 Python 测试；cooperative 测试目录已有整体注册，不需新增安装规则。
- 新增本报告与 `docs/superpowers/plans/2026-10-26-ranger-single-robot-approach.md`。

`agent.py`、`ranger_check.py`、消息定义与运行配置没有本次增量修改；它们已有的工作区修改未回退。

## 单机入口

在 Ranger 原硬件、定位和导航已经启动的前提下，由现场人员执行：

```bash
bash /home/isee601/ws/VLM_Nav/scripts/start_ranger_single_robot.sh --allow-motion --target-id B
```

可追加原入口支持的 `--target '目标描述'`、`--config-dir /实际配置目录`、`--timeout 秒`。默认采用 `RangerVLM_env.bash` 当前验收配置，没有当前验收配置则采用仓库 Ranger 配置目录；默认任务时间读取所选 `scene.yaml` 的 `task_timeout`。不覆盖 `robot.yaml` 的任何单机运动参数。

脚本复用原环境，通过源代码 PYTHONPATH 调用 `multi_agent_nav.ranger_check --action explore --approach-target`；没有 bridge/coordinator 或硬件启动/重启命令。`--allow-motion` 是原入口已有要求，没有新增许可条件。`--help` 不加载 ROS 环境，也不启动节点。此次仅验证帮助输出、语法和伪进程参数转发，未运行运动命令。

协作入口仍是现有 agent -> `AgentRunner` -> `explore_steps` -> `RangerAdapter.approach_steps`。单机为 ranger_check -> explore -> 同一 explore_steps 与 approach_steps。后续已按用户授权完成本机构建部署，结果见文末“构建部署记录”；既有运行进程不会因此自动重载。

## 离线验证

参考源提交自带的差分测试，从其指定的冻结单机版本 `32eab6244ec4b483807f8ed68e8667f5495874bb` 直接读取 `VLMNavigator` 方法，再以 AST 提取并执行真实方法；没有用测试摘要重写参考算法。fixture 保存原文方法切片及完整原文件 SHA256。差分覆盖规划结果、0.89/0.81 边界、取消与终态顺序、移动样本清零、陈旧样本、一次重试、看门狗优先级与超时边界。

新增缺失模块测试先失败；身份替换回调和独立健康复查分别先复现“旧回调仍发导航”，随后修复并通过。另覆盖双入口 B/C、取消未终止、外部取消、迟到 plan/goal 回调、已完成 future、结果重复消费、协作外层等待/身份/心跳/owner/终态屏障。

运行环境及完整回归命令：

```bash
source /opt/ros/humble/setup.bash
source build/ranger-local/install/local_setup.bash
PYTHONPATH="$PWD/cooperative_ws/src/multi_agent_nav:$PWD:$PYTHONPATH" python3 -m pytest -q \
  cooperative_ws/src/multi_agent_nav/tests test \
  --ignore=cooperative_ws/src/multi_agent_nav/tests/test_ros_smoke.py \
  --ignore=test/test_obstacle_cloud_filter_runtime.py
```

修改前：**34 failed / 939 passed / 13 skipped**。修改后：**34 failed / 998 passed / 13 skipped**（18.53 s）。失败名称集合完全一致，**新增失败 0，已消除基线失败 0**。重点差分、传输、原观察/扫描、私有任务回归 **168 passed**（2.22 s）。`git diff --check`、新增脚本 `bash -n`、变更 Python AST 语法检查均通过；原有脏文件全部仍在，未出现声明范围外的既有文件改动。开始时缺少 Python 导入环境的收集错误单独记录，未拿收集失败与修复后的执行结果混比。测试使用现有 overlay 作为消息/依赖来源，源码路径优先。

原基线失败包括缺失 Go2 文件/模块、环境和现有断言问题；未扩展到修复这些任务外问题。完整基线失败名称如下：

```text
cooperative_ws/src/multi_agent_nav/tests/test_dynamic_costmap_config.py::test_effective_cooperative_costmaps_keep_dynamic_layers_and_other_parameters
cooperative_ws/src/multi_agent_nav/tests/test_dynamic_costmap_config.py::test_actual_navigation_launch_applies_optional_overlay_last
cooperative_ws/src/multi_agent_nav/tests/test_dynamic_costmap_config.py::test_isolated_acceptance_launch_has_only_costmaps_and_lifecycle_manager
cooperative_ws/src/multi_agent_nav/tests/test_dynamic_costmap_config.py::test_no_verification_flags_are_set_by_static_costmap_profile
cooperative_ws/src/multi_agent_nav/tests/test_dynamic_costmap_config.py::test_static_collection_dry_run_contains_only_read_commands
cooperative_ws/src/multi_agent_nav/tests/test_dynamic_costmap_config.py::test_native_costmap_marking_and_clearing_with_synthetic_inputs_only[sigint]
cooperative_ws/src/multi_agent_nav/tests/test_dynamic_costmap_config.py::test_native_costmap_marking_and_clearing_with_synthetic_inputs_only[lifecycle_shutdown]
cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py::test_empty_resources_allow_independent_exploration_but_not_cooperative_live
cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py::test_global_goal_age_config_must_be_positive_finite[0.0]
cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py::test_global_goal_age_config_must_be_positive_finite[-1.0]
cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py::test_global_goal_age_config_must_be_positive_finite[nan]
cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py::test_global_goal_age_config_must_be_positive_finite[inf]
cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py::test_global_goal_age_config_must_be_positive_finite[True]
cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py::test_global_goal_age_config_must_be_positive_finite[None]
cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py::test_readonly_observer_has_no_velocity_publisher_and_adapter_rejects_motion
cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py::test_offline_single_robot_config_does_not_require_cooperative_certification
cooperative_ws/src/multi_agent_nav/tests/test_go2_adapter.py::test_observation_only_node_cannot_enable_old_autonomy
cooperative_ws/src/multi_agent_nav/tests/test_go2_adapter.py::test_cancelled_http_requests_release_capacity_across_viewpoints
cooperative_ws/src/multi_agent_nav/tests/test_go2_executing_path.py::test_execution_path_timeout_logs_receipt_age_once[go2]
cooperative_ws/src/multi_agent_nav/tests/test_go2_executing_path.py::test_execution_path_timeout_logs_receipt_age_once[ranger]
cooperative_ws/src/multi_agent_nav/tests/test_go2_health_diagnostics.py::test_localization_loss_logs_failure_time_evidence_once_per_loss[stale]
cooperative_ws/src/multi_agent_nav/tests/test_go2_health_diagnostics.py::test_localization_loss_logs_failure_time_evidence_once_per_loss[tf_error]
cooperative_ws/src/multi_agent_nav/tests/test_ranger_task_entry.py::test_ranger_launch_contains_only_local_agent
cooperative_ws/src/multi_agent_nav/tests/test_robot_adapters.py::test_ranger_observer_keeps_own_name_under_agent_global_remap
cooperative_ws/src/multi_agent_nav/tests/test_robot_adapters.py::test_aligned_depth_observer_processes_disabled_input_without_autonomy[False]
cooperative_ws/src/multi_agent_nav/tests/test_robot_adapters.py::test_aligned_depth_observer_processes_disabled_input_without_autonomy[True]
test/test_keepout.py::test_keepout_launch_configures_both_costmaps_without_changing_navigation
test/test_keepout.py::test_humble_keepout_rejects_rotated_mask_before_launching_nodes
test/test_ranger_check.py::test_hardware_check_receives_large_camera_images_reliably
test/test_ranger_command_guard.py::test_ranger_launch_has_one_acceleration_owner_and_guarded_behavior_output
test/test_ranger_local_launch.py::test_legacy_navigation_default_remains_enabled_but_can_be_excluded
test/test_ranger_local_launch.py::test_system_forwards_legacy_switch_and_nav2_profile
test/test_ranger_local_launch.py::test_keepout_launch_forwards_map_and_mask_across_layers
test/test_ranger_recovery.py::test_navigation_launch_loads_recovery_plugins_and_tree
```

## 未验证与限制

- 未进行真机运动、制动距离、真实 odom 采样/延迟、Nav2 取消到终态时序、B/C 抵达及双机 READY 联合验收；离线通过不能替代这些证据。
- 迁移阶段未构建部署；后续构建部署记录见文末。全程未重启或启动现场组件，未执行 ROS smoke、obstacle cloud runtime 或 C++/控制器测试。
- 单机成功继续采用原 Ranger 的到达样本和动作终态证据；协作额外等待已有 `/odom` 外层 stopped 屏障。未把新的外层停稳条件加到单机算法。
- 来源算法的取消/停稳时序及硬件制动表现必须现场核验。已有基线失败仍存在，不能宣称整个仓库测试通过。


## 构建部署记录（2026-10-26）

用户随后明确授权“构建部署，使得可以进行实机检验”。已沿用原现场流程部署到 `build/ranger-local/install`，未改动环境入口、验收配置或硬件链。

```bash
source RangerVLM_env.bash --build
colcon --log-base "$PWD/build/ranger-local/log" build \
  --base-paths "$PWD" "$PWD/cooperative_ws/src" \
  --build-base "$PWD/build/ranger-local/build" \
  --install-base "$PWD/build/ranger-local/install" \
  --packages-up-to multi_agent_nav --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF -DCOOPERATIVE_RUNTIME_PYTHON=/usr/bin/python3
```

结果：`multi_agent_task_msgs`、`vlm_nav`、`multi_agent_nav` **3 packages finished，退出 0，30.9 s**。唯一 stderr 是原有 CMP0074 / PCL_ROOT CMake 开发警告。

| 部署后核对 | 结果 |
|---|---|
| 仓库外 `/tmp` 导入，不额外加入源码 PYTHONPATH | 三包 prefix 均指向 `build/ranger-local/install`，15 个关键模块内容与源码一致 |
| 消息支持库 | `AgentState` 的 native type support 导入成功 |
| 安装版本接近/传输/观察/私有任务回归 | **168 passed，1.86 s**；31 个运行模块全部由安装路径加载 |
| 原单机入口 `--offline --require-ready` | 退出 0，`single_robot_experiment_gates=[]`，`motion_sent=false` |
| `cooperative.sh ranger preflight` | 退出 0，`live_gates=[]`，API 环境完整，安装核对全部匹配，`motion_sent=false` |
| 受保护文件哈希 | 95 个既有配置、地图、原生程序等文件全部不变 |

测试仓库的 `test/conftest.py` 会强制将 vlm_nav 切回源码。首次检测到该行为后，针对安装验证使用 `--noconftest` 重新运行相同 168 项测试，并在结束后检查所有运行模块路径；没有修改测试或源码来掩盖安装问题。

当前配置仍为 `build/ranger-local/acceptance-20261015_161341/config`，实际地图仍为 `shared_maps/20261018161152/lab.yaml`，候选数 57。到达里程计 `/fastlio/odom`、外层停稳 `/odom` 和原到达参数均保持不变。

### 实机检验入口

在原硬件、定位、相机、Nav2 已由现场原流程准备好的情况下，新开任务终端运行：

```bash
bash /home/isee601/ws/VLM_Nav/scripts/start_ranger_single_robot.sh --allow-motion --target-id B
```

协作 Ranger 端继续使用原入口（只在原 agent 正常退出后手动启动一份，不叠加单机任务）：

```bash
bash /home/isee601/ws/VLM_Nav/scripts/cooperative.sh ranger agent
```

部署只更新安装产物，不会让已有进程自动加载新 Python 模块；本次没有自动启动/重启 agent、导航或硬件，也没有发送导航/速度/任务开始命令。真实 NAV_READY、动态里程计、取消制动和抵达结果仍需现场检验。前文 34 项基线失败未作为本次部署范围修复；未声称全仓测试通过。

证据目录：`build/ranger-local/approach-deploy-20261026-hr8winhp`。

- `build.log`：三包构建结果。
- `installed-verification.json`：安装路径、关键模块 SHA256、实际参数与消息支持库核对。
- `installed-tests-no-source.log`：安装版本的 168 项测试与运行模块路径检查。
- `single-offline.json`、`cooperative-preflight.json`：原入口离线报告。
- `protected-before.json`、`protected-after.json`：受保护资产前后哈希核对。
- `install-before.tar.gz`：构建前安装目录布局备份（保留符号链接，不是独立源码历史备份）。


### 启动入口环境参数修复

现场反馈 `用法：source RangerVLM_env.bash [--build|--legacy]`。原因是新入口 source 环境文件时没有显式传参，Bash 将 `--allow-motion` / `--config-dir` 继承给环境文件。现改为 `source "$ranger_repo/RangerVLM_env.bash" local`，并同步安装目录中的脚本副本。增强后的两个参数转发回归先复现失败，再修复通过；真实环境通过 `--allow-motion --target-id B --help` 加载验证（argparse 帮助退出，不初始化 ROS 或发送运动）。
# 在线检查启动异常修复（2026-10-26）

`inspect` 在第一条底盘里程计到达前将 `odom_age_s=None` 与浮点数比较，触发 `TypeError`，未生成健康报告。共享检查入口现直接复用 adapter 的 `checks.odom_fresh`；未收到数据仍为未就绪，所有阈值和硬件配置保持原样。安装 overlay 为源码符号链接，修改已生效。

增加持续无里程计、里程计延迟到达两项回归：修复前均失败，修复后入口 26 项通过。扩大检查为 100 通过、11 失败，11 项均存在于迁移验证基线，无新增失败。日志：`/tmp/ranger-inspect-red.log`、`/tmp/ranger-inspect-focused.log`、`/tmp/ranger-inspect-green.log`。

复跑 `scripts/check_ranger.sh` 已取得健康报告：13 项通过、17 项失败，当前检查进程未发现相机、雷达、里程计及 Nav2 服务，CAN 接收正常。证据目录：`build/ranger-local/acceptance-20261015_161341/check-20261026_221500-qn2ua8kf`。未启动运动、未重启组件；这次结果不代表真机运动验收通过。

# 单机启动后立即退出排查（2026-10-26 22:17）

现场记录 `build/ranger-local/experiments/experiment.20261026_221751.131478a9` 显示启动健康检查通过，进入初始观察后 `/odom` 接收时间龄为 0.504467 秒，超过原有 0.5 秒阈值，随后退出并确认停稳。没有导航动作 UUID；报告中的 `motion_sent=true` 是入口已进入执行阶段的标记，不能据此认定已有导航运动。

检查发现独立入口同步执行候选生成、地图摘要校验、证据目录分配和选点图渲染，这些工作会阻塞 ROS 回调。现将 Ranger 独立入口接入已有协作后台准备流程；不改变候选算法、初始观察顺序、扫描、健康门限或硬件配置。新增单机后台准备测试在修复前三项失败（候选生成、渲染、摘要），修复后通过；现有安装通过源码符号链接生效。详细测试输出：`/tmp/ranger-start-preparation-tests.log`。

该修复消除了已确认的同步阻塞路径；尚未通过再次运动验证它是否解释现场全部接收延迟。终端长输出为现有完整 JSON 报告，原始数据仍保存在上述实验目录。没有自动重跑运动、重启组件或修改通信配置。

