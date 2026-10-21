# 2026-10-20 双方独立探索和接近 C

用户明确要求取消 C 先后顺序，并确认“采用远端，无退出”。实现依据为 `pppppst/VLM_Nav0` 的 `go2-cooperative-task-layer` 提交 `4fc673156c5c985c4d9a9149cfe0b08b1582d90b`。本次在现有 Ranger 公共任务代码内同步此行为，不覆盖远端整包，不改变原单机或旧共享模式。

## 实现与状态

1. 保留 A/B 双方 READY 屏障；进入 C 后不等待对方缓存。Coordinator 为每个机器人保存本任务自己的 C 引用，内部 Snapshot 以带默认值的 local_c 映射传给 Planner；新任务清空缓存，合法的 CACHED 可更新同引用的先前未定位记录。
2. 任一方有 C 即可进入 DEPOSIT 并独立接近，另一方继续搜索；允许两端接近命令同时执行。阶段切换不取消 SEARCH_C 中已经开始的搜索，Agent 允许在 SEARCH_C、DEPOSIT 搜索 C。
3. 本机缓存补齐当前任务/机器人/地图身份后保存，接近时再次核对，避免错任务或另一台机器人的引用被使用。跨机消息保留 CACHED 引用，全部坐标继续无效且清零。
4. 私有模式删除自动 EXIT 执行路径及专用状态，拒绝外部 EXIT 和旧 GRANT_RESOURCE。当前接近导航 SUCCEEDED、动作终止、真实停稳、实际距离 ≤1.00 m 后标记 deposited；保持原地停稳，不设置 exited。
5. DONE 要求双方 deposited/stopped 且当前任务命令按身份确认结束、无未完成命令；取消、旧结果、未停稳或单方完成不能推进 DONE。失联和异常仍执行原停车/暂停逻辑。

ROS schema、话题、坐标隔离方式不变。改动为 5 个生产模块与 3 个测试文件；旧共享模式继续保留原顺序/退出兼容链。源码和 acceptance 的地图、footprint、stop_margin、API 环境变量、场景开关与现场验证标志均不改。最近同步的混合候选、附近排序、最多 12 点 VLM 选择保持不变。

deposited 仍表示接近验收/模拟放入，不表示机械投放。两端可同时靠近 C，先完成者停在接近位置；没有任务层 C 互斥或自动退让，运动避障依赖各自 Nav2。双方须同步相同行为，旧的私有顺序/EXIT 执行器不可混跑。

## 测试证据

- 修改前扩展离线基线：600 passed、13 failed、1 skipped、5 deselected。
- 新增 14 项独立 C 场景：在原代码上 9 failed、5 passed；已实现副本的扩展回归包含这些场景并全部通过。覆盖两种先发现顺序、并行接近、在途搜索跨阶段、READY 屏障、缓存恢复/重置、旧身份拒绝、EXIT 拒绝、DONE 身份/停稳和接近失败条件。
- 移除 5 项已取消的私有 EXIT/先后顺序专用用例，由上述独立 C 完成及严格接近验收场景替代；更新原私有完整任务和失联停车测试，保留共享模式原顺序测试。
- 副本及写回原仓库后的扩展回归均为：609 passed、13 failed、1 skipped、5 deselected；三份 XML 的失败名称与基线一致，无新增失败。
- 独立只读审查核查了 5 个生产模块、3 个测试文件及探索/ROS 接收/Mock 调用链，未发现需要修复的问题；另行运行相关单元测试 55 项通过。

既有失败涉及本机缺少的 Go2 工具/地图、旧配置和大小写断言；逐项名称见 [前轮基线失败清单](ranger_exploration_20261020.md#验证)，本轮 baseline-wide.xml 与 staged-wide.xml 的失败身份集合已逐项核对一致。没有将扩展回归报告为全绿。

证据目录 `/tmp/ranger-independent-c-20261020` 保存改前摘要、测试 XML、变更清单和审查结果；补丁为 [ranger_independent_c_20261020.patch](ranger_independent_c_20261020.patch)。已通过文件系统权限检查将 8 个文件写回原仓库，摘要逐项与测试副本一致；包内其余已有文件保持改前摘要，包括配置和 launch。原文件备份在该目录的 `source-backup`。没有提交或部署，没有启动 ROS 节点、桥接、模型 API 或真机。

## 无节点复验

目录为原仓库，测试结束后退出，不保持运行。排除实际节点/原生 costmap/observer 检查：

```bash
cd /home/isee601/ws/VLM_Nav
source /opt/ros/humble/setup.bash
source /home/isee601/ws/VLM_Nav/build/ranger-local/install/setup.bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
ROS_LOG_DIR=/tmp/ranger-independent-c-ros-source \
PYTHONPATH=/home/isee601/ws/VLM_Nav:/home/isee601/ws/VLM_Nav/cooperative_ws/src/multi_agent_nav:$PYTHONPATH \
python3 -m pytest -q -p no:cacheprovider test cooperative_ws/src/multi_agent_nav/tests \
  --ignore=test/test_obstacle_cloud_filter_runtime.py \
  --ignore=cooperative_ws/src/multi_agent_nav/tests/test_ros_smoke.py \
  -k 'not native_costmap_marking and not observation_only_node and not readonly_observer_has and not aligned_depth_observer' \
  --tb=no --junitxml=/tmp/ranger-independent-c-20261020/source-wide.xml
```

由于上述 13 个既有失败，预期退出码仍为 1；应核对失败集合没有增加。未重建或替换安装目录，若启动入口加载旧安装，应在后续构建部署时同步更新两端。
