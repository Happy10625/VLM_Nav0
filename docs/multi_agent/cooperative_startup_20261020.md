# 人工双机启动工具制作记录（2026-10-20）

用户要求人工逐步执行、随时停止排障。本轮没有启动 ROS 节点、bridge、硬件、SSH、模型请求或运动；真实双机结果待现场按 [启动流程](../双机协同启动与验收.md) 记录。

## 改动

- 新增 `Cooperative_env.bash`、`scripts/cooperative.sh`、18项入口/环境/协议测试；CMake 注册测试。
- Ranger 入站统一兼容远端 `action=SYNC_LOCAL_C, kind=WAIT` 与已有 SYNC/kind 编码，不更改 ROS 消息字段；共享坐标模式仍拒绝私有同步。
- 审查使用远端 `pppppst/VLM_Nav0` 的 `go2-cooperative-task-layer@4fc673156c5c985c4d9a9149cfe0b08b1582d90b` 实际 Coordinator 生成报文复核接收。独立复核未发现未解决新问题。
- 保留现有配置、API环境、硬件入口和未提交修改。写入前核对六文件摘要，并备份原有三个文件到 `/tmp/cooperative-startup-20261020/source-backup`。

## 离线验证

Shell语法、无环境help、环境继承、Go2最后加载新overlay、重复agent拒绝、coordinator与已有observer共存、Go2控制入口限制、显式start、预检拒绝start、暂停独立于live gates、桥接Domain/XML、三种同步报文及共享模式拒绝：18项通过。

同修改前范围的离线回归：**627 passed、13 failed、1 skipped、5 deselected**；13失败身份逐项与 `/tmp/ranger-independent-c-20261020/source-wide.xml` 一致，无新增失败。不将此结果描述为全量通过。未运行依赖真实ROS节点/硬件的runtime与smoke检查。XML：`/tmp/cooperative-startup-20261020/source-wide.xml`。

从 `/tmp` 执行实际安装预检：`bash /home/isee601/ws/VLM_Nav/scripts/cooperative.sh ranger preflight`。退出1符合当前现场标记状态；安装导入六模块与源码逐字节一致，257静态候选、两种SYNC编码接收通过、API环境齐全、motion_sent=false。未验证远端Go2当前安装或实时联通。

当前未完成项：

- `ranger vision_validated field verification required`
- `scene disabled: field coordinates missing`
- `ranger path_source_verified field verification required`
- `ranger peer_obstacle_verified field verification required`

外参确认继续作为逐步实测记录，不纳入启动门控。

## 原有13项失败

- `cooperative_ws.src.multi_agent_nav.tests.test_dynamic_costmap_config::test_effective_cooperative_costmaps_keep_dynamic_layers_and_other_parameters`
- `cooperative_ws.src.multi_agent_nav.tests.test_dynamic_costmap_config::test_actual_navigation_launch_applies_optional_overlay_last`
- `cooperative_ws.src.multi_agent_nav.tests.test_dynamic_costmap_config::test_isolated_acceptance_launch_has_only_costmaps_and_lifecycle_manager`
- `cooperative_ws.src.multi_agent_nav.tests.test_dynamic_costmap_config::test_no_verification_flags_are_set_by_static_costmap_profile`
- `cooperative_ws.src.multi_agent_nav.tests.test_dynamic_costmap_config::test_static_collection_dry_run_contains_only_read_commands`
- `cooperative_ws.src.multi_agent_nav.tests.test_field_preparation::test_empty_resources_allow_independent_exploration_but_not_cooperative_live`
- `cooperative_ws.src.multi_agent_nav.tests.test_field_preparation::test_global_goal_age_config_must_be_positive_finite[0.0]`
- `cooperative_ws.src.multi_agent_nav.tests.test_field_preparation::test_global_goal_age_config_must_be_positive_finite[-1.0]`
- `cooperative_ws.src.multi_agent_nav.tests.test_field_preparation::test_global_goal_age_config_must_be_positive_finite[nan]`
- `cooperative_ws.src.multi_agent_nav.tests.test_field_preparation::test_global_goal_age_config_must_be_positive_finite[inf]`
- `cooperative_ws.src.multi_agent_nav.tests.test_field_preparation::test_global_goal_age_config_must_be_positive_finite[True]`
- `cooperative_ws.src.multi_agent_nav.tests.test_field_preparation::test_global_goal_age_config_must_be_positive_finite[None]`
- `cooperative_ws.src.multi_agent_nav.tests.test_field_preparation::test_offline_single_robot_config_does_not_require_cooperative_certification`

## 写入文件摘要

补丁：[cooperative_startup_20261020.patch](cooperative_startup_20261020.patch)。以下为写回后 SHA-256；此表不含文档改动。

- `Cooperative_env.bash`: `4195cb26f4fbc15444572d5521383fef0f544652c636c79a8ffacd8460932f32`
- `scripts/cooperative.sh`: `f883d650af20798d0d226c360e8a66e473f37562d4f961c3c816e25ae31205fa`
- `test/test_cooperative_startup.py`: `c3061cd62ef62a3e7048db1544bb33acc2553f21b8dddfd9445332211aa212cd`
- `cooperative_ws/src/multi_agent_nav/multi_agent_nav/models.py`: `b35db8b07e99c7db3072f48f80df75ec6fce7c5ce89ef07612fef90fdce13713`
- `cooperative_ws/src/multi_agent_nav/multi_agent_nav/agent.py`: `2e6b5802b6141c5f28f221f8f0beb4d879bf3128be5206de60f7f2a31954accf`
- `CMakeLists.txt`: `ab709b3db54168ea4a3ef67cd9ba417ada0e2c8341db38da35b433e89ece9045`
