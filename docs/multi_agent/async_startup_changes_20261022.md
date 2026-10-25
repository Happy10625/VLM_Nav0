# 双机异步启动修改说明（2026-10-22）

本轮按用户五阶段方案修改共享任务源码和 Go2 入口的等待判据；没有更新安装、启动 ROS、连接机器人或开始新任务。分支 `go2-cooperative-task-layer`，HEAD `4fc673156c5c985c4d9a9149cfe0b08b1582d90b`，保留原有未提交改动与地图删除。

## 已实现的状态语义

适用于当前 `share_c_coordinates=false` 私有坐标模式；旧共享坐标模式保持原启动语义。

| 阶段 | 行为和推进条件 |
|---|---|
| PREPARING | 本机真实定位/Nav2、停稳或原生动作终态未就绪，持续发布真实状态 |
| WAITING_PEER | 本机已就绪且停稳，无需对端存在；对端晚启动、暂时未就绪或重启不会锁存 PAUSED |
| STANDBY | 对端也健康、停稳，携带新鲜 WAITING_PEER/STANDBY 且无旧任务/命令/goal/验收标志 |
| SYNCING | 显式 StartTask 创建任务；agent 接受 SYNC_LOCAL_C 后立即发布本轮 task_id，不等待对端先接受 |
| SYNCED | 收到当前任务新鲜、健康的干净同步状态；coordinator 等双方 SYNCED 才派工 |
| 执行 | 沿用 PLANNING/NAVIGATING 等状态；失联、健康异常、身份变化仍停止并锁存 PAUSED |

双方 WAITING_PEER 可以相互确认，无循环依赖。Go2 启动采样接受 WAITING_PEER/STANDBY，仍要求各至少三条新鲜推进状态、真实健康停稳、地图身份、唯一正确发布者和 QoS。保留原相机、底层、RViz、Sport、bridge、agent、coordinator 顺序；ARM 提示只要求 Go2 本机准备，不再要求 Ranger 先启动。

准备采样仍无限等待并每5秒输出原因，可 Ctrl+C 退出。任务同步受既有600秒总任务上限约束，不使用执行期3秒失联门槛中断正常初始化；进入执行后3秒通信门槛不变。执行后锁存 PAUSED，恢复通信不自动恢复运动；显式 PAUSE/CANCEL 仍有效。

## 关键修改

- `models.py`：复用 run_state 定义准备与同步状态，严格区分本轮同步确认和 A/B 到达 `ready=true`；无消息字段或桥接话题变更。
- `agent.py`、`agent_node.py`、`mock.py`：共享接收入口分别记录有效心跳到达和当前任务消息到达；在线但 nav_ready=false 不再冒充心跳丢失。旧任务/命令/goal/READY 不能建立握手；本轮完成干净握手后允许对方先收到 EXECUTE，避免不对称派工死锁。
- `coordinator.py`：显式 start 后先 SYNCING，只重发同步；双方当前任务 SYNCED 才 EXECUTE。健康失效、缺少消息和心跳超时分别报告，保留首个暂停原因。
- `go2_exploration.py`：候选几何构造、完整地图摘要、选择图渲染复用现有单 worker，等待时 yield，保留健康、任务/选择 epoch、取消及导航前复核。模型调用继续使用原异步路径。迟到结果不能推进已取消运动。
- `scripts/start_go2_cooperative_task.py`：准备采样和提示改用新待命状态。完整入口仍只自动发起一次任务；单独启动 agent/coordinator 不会创建任务。旧单机/传感器/Nav2 配置、真实验证标志及网卡部署不变。

## 已执行验证

纯离线 pytest：基线集合269通过/5失败；扩大后的最终相关集合465通过/5失败，逐项比较 XML，无新增失败。新增启动/握手35项、真实 worker 受控阻塞8项均通过，涵盖另一端晚启动一分钟、重建单端 agent 后重入当前握手、准备中健康变化、几何/摘要/图像/模型耗时期间20次心跳推进和取消。另有3项验证最后一次地图后台等待结束后重新检查停稳/动作终态/目标footprint，不满足时零导航。另覆盖两种首条 EXECUTE 到达顺序、单端首条丢包及重复命令，原到达验收/独立C/执行失联保护回归保留。

同5项基线失败均来自当前 gates 已打开而旧测试仍断言存在阻塞项，未修改真实配置来满足测试：`test_protocol::test_broadcast_only_sync_and_live_templates_block`、`test_independent_c::test_no_gate_requires_resource_geometry_or_stopping_envelope_margin`、`test_local_tasks::test_live_local_gates_require_local_assets_but_not_remote_alignment` 和两项 `test_malformed_local_geometry_is_reported_as_gate_failure`。

证据在 `build/async-startup-20261022/`：`baseline.xml`、`final.xml`、`startup-red.log`、`worker-red.log`、`review-red.log`、`asymmetric-red.log`、`review-fixed.log`；`source-before/` 保存本轮前源码，便于与用户已有改动区分。build 证据不受 Git 保护，不清理。

## 待确认清单与安装边界

1. Ranger 现场真实 adapter 不在本仓库的可用 live 分支，本轮没有访问其代码。须确认使用同一 AgentMachine/Coordinator 状态语义，移植时保留 Ranger 自己的硬件、地图、运动终态和停稳判据；不得直接覆盖其自定义 adapter。
2. Ranger 自定义状态映射必须保留 PREPARING、WAITING_PEER、STANDBY、SYNCING、SYNCED，并在收到新任务 SYNC 后立即发布身份；不能把 SYNCED 映射成验收 READY，也不能沿用“先等对方 IDLE”门槛。
3. 双方实际安装必须包含公共 `models.py`、`agent.py`、接收回调及 runner 配置；Go2 coordinator 与入口也须更新。本机此前 `build/go2-dual-acceptance-20261022/install` 未更新，新源码与旧安装不一致时预检拒绝是正确行为。完整构建步骤只在 [runbook](go2_runbook.md) 维护，需使用新的隔离安装目录。
4. schema v1 相同不代表启动语义兼容。旧版 IDLE/健康状态不能充当新握手确认，两端更新前禁止把旧链路通过当作此版本通过。
5. 未验证真实 ROS/DDS 调度、跨机重启、Ranger adapter 或物理运动。离线 worker 测试验证生成器可让出执行权，不代表现场心跳时延已有测量；上述四类场景仍须在两端更新后按现场授权验收。

这些事项未用猜测性配置、假坐标、远端部署或自动重启处理；等待用户审核与后续现场授权。
