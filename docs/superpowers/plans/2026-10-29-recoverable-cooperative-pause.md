# Recoverable Cooperative Pause Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans inline. Preserve the existing dirty worktree; do not commit unrelated changes.

**Goal:** 按用户要求，将导航就绪短暂失效和双方心跳超时设为可恢复暂停，避免对端 PAUSED 导致恢复死锁。
**Architecture:** 扩展现有 private local C 模式的暂停检查点和双方停稳握手；沿用 NAV_READY_WAIT 线协议标记作为可恢复等待，兼容既有 Ranger；不调整健康阈值，不自动恢复人工/身份/定位等独立故障。
**Tech Stack:** Python、现有 AgentMachine/Coordinator/AgentRunner、pytest。
**Spec:** 本轮用户原话及 docs/multi_agent/session_handoff.md 的实际故障证据。用户已明确要求执行修改。

## Global Constraints
- 保留已有工作树和 Ranger 单机行为；不新建脱离现有变更的工作树，不启动现场运动。
- 恢复要求当前 task 身份、新鲜 peer/coordinator 心跳、localization/nav 健康、停稳和本地原生动作终止。
- 不改 ROS 消息结构和硬件配置；不把未知暂停或人工取消归为可恢复。
- 任务总超时仍生效；共享坐标旧模式不扩大行为范围。

## Review Focus
- 单向断链：两边暂停，不能收到单侧心跳就提前恢复。
- 同时心跳超时：恢复不能相互等待对方退出 PAUSED。
- 心跳恢复但 nav/localization 未恢复：禁止运动。
- PAUSE/STOP 到达交错、人工停止插入：不被自动恢复覆盖。
- 任务命令及扫描检查点保持，已成功命令不重放。

### Task 1: Expand recoverable pause end to end
**Files:** models.py、agent.py、coordinator.py、runner.py、go2_exploration.py（位于 cooperative_ws/src/multi_agent_nav/multi_agent_nav）；tests/test_nav_ready_resume.py、tests/test_heartbeat_diagnostics.py、tests/test_cooperative_preparation.py；test/test_nav_ready_checkpoint.py。
**Interfaces:** 通用可恢复原因判别供 agent/coordinator/runner 共用；现有 NAV_READY_WAIT 控制标记保留。
- [x] 写失败测试：peer/coordinator/both 心跳失联后双方暂停及同 task 继续、导航恢复需新鲜心跳、未知/人工/定位故障保持暂停、检查点保持。
- [x] 用实际 ranger-local install PYTHONPATH 运行并确认失败。
- [x] 扩展原因判别，修正 agent 恢复门限及 coordinator 断链等待，runner 共用相同分类。
- [x] 运行定向状态机、runner 和单机检查点测试；根据接口影响扩展到 multi_agent_nav 测试。
- [x] 独立代码审查本轮增量，修复发现的问题并验证。
- [x] 保存测试结果、信箱实现边界、交接记录；保持未提交便于用户审阅。

Pre-flight: 单个端到端任务，模型分类同时被 agent、coordinator、runner 消费，必须一致更新。既有测试中 heartbeat 永久锁存断言按用户新语义调整。

执行记录：先10项预期失败→45通过；恢复ACK竞态RED→GREEN；独立审查发现定位被heartbeat elif遮蔽，已RED→GREEN；新EXECUTE绕过恢复ACK、执行内部首次nav故障、active peer尚未停稳均RED→GREEN。定向375通过；广测既有失败由改前快照确认，详见交接与诊断目录。没有提交、部署或现场运动。
