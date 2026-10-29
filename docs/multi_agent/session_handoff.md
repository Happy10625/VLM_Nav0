# 当前交接：可恢复导航/心跳暂停（2026-10-29）

分支 `ranger-reuse-go2-framework`，HEAD `ab922276108c21f44612b7805e26fc0560300528`，本轮保持未提交工作树，不覆盖此前改动。

## 用户要求与跨机消息

用户原话：
> 将此类的navigation invalid设为可恢复,遇到后不直接锁存pause而是可以在恢复正常后继续.以及当heartbeat timeout,则两边都进入暂停状态;恢复正常heartbeat则可继续任务.不因为对端的pause而锁存pause, 避免恢复后死锁.据此进行修改,并将此修改意见原话传递到codex信箱

已按原文发送至信箱任务 `peer-health-774a-20261029`，消息 `151d42ff92a949fb8a4d9b6830e0f7e1` 投递成功，Go2 回信确认收到。Go2随后取得其本机用户授权并完成对应源码修改；最新确认 `d638abf7a83745628b02faf1ccdd6634` 明确逐项核对12项可恢复原因及 ` | nav_ready: ` 后缀兼容。

## 本轮已实现

- `models.py`：将导航专用判别改为 `is_recoverable_pause`，共用12项明确原因白名单；新增内部 `RecoverablePause` 异常。人工停止、定位失效、身份改变、未知原因及总任务超时不自动恢复。
- `agent.py`：peer/coordinator heartbeat timeout可恢复；恢复要求同task、双方loc/nav健康、双方停稳、本地原生动作终态、两个心跳均新鲜和活动阶段。定位硬故障优先于heartbeat检查，失联期间发生定位故障也锁存。
- `agent.py`：先退出PAUSED并发布活动停稳状态，暂存 `LOCAL_EXECUTE/LOCAL_RESUME`，收到对端活动态再真正恢复运动；对端可恢复PAUSED不会造成锁存或循环。已完成旧命令后的新EXECUTE也要等待此确认。新的硬故障清除待发恢复effect。
- `coordinator.py`：heartbeat stale与nav_ready false进入可恢复等待；等待期间持续SYNC与 `PAUSE kind=NAV_READY_WAIT`，不把再次断链升级为硬锁存。双方fresh/healthy/stopped/PAUSED后恢复原phase，原task/command保持。
- `runner.py`：心跳和导航暂停共用已有检查点；扫描/普通导航保留generator，独立approach取消后重启同一未完成command；成功命令不重放。动作前检查或探索内部首次采到可恢复故障不会被通用异常变为硬锁存。
- `go2_exploration.py`：协同执行内部纯nav/costmap失败转明确可恢复异常；owner冲突、定位/pose无效、文件变化等仍硬停止；standalone原行为保留。
- 本次扩展只针对当前private local C协同模式；不调整Nav2新鲜度阈值、硬件适配、消息schema或原600秒任务上限。

## 实际比较的Go2代码

实际读取信箱消息 `3cf194c75f58499b912e2d55910a7de8` 中的工作树片段：HEAD `f6c75f27b20a17a2acc49e2ab3c56e1aa5bbf8f1`、dirty、未部署。
路径均位于 `cooperative_ws/src/multi_agent_nav/multi_agent_nav/`：
- `runner.py` 的执行生成器异常捕获和 `motion_ready`（当时SHA256 `336e8aecde034274c79cf4ae78202d245131bc64558b96ee29354a66b2482836`）。
- `go2_exploration.py` 的 `NavigationNotReady` / 内部 `health_check`（当时SHA256 `b48845e3636814b245da1fcad6a95eb233dd0c7ca998ea29f04dfd84af751026`）。
Go2采用取消后重新规划未完成命令；Ranger保留已有检查点。跨机协议一致，不复制其硬件/网络参数。Go2最终32项新增测试及广测1007通过/14既有失败为对端报告，非本机验证。

## 本轮验证与审查

- 离线定向：同一 `build/ranger-local/install/setup.bash` 环境，375 passed（状态机、协议、恢复、runner、探索、准备门限、Ranger原生检查点和接近传输）。
- 扩展：806 passed、27 failed、1 skipped。27项与本轮开始快照复跑失败集合完全一致；没有宣称全量通过。既有问题包括缺少Go2专用文件/地图、配置断言差异、RealSense库/ROS上下文和sandbox日志权限、日志捕获及HTTP测试超时。完整失败名称保存在下述 `existing-failures.txt`。
- 独立只读审查发现1项P1：heartbeat分支遮蔽定位硬故障；已补失败测试并修复。额外覆盖了双方各自单向peer回调或SYNC丢失、恢复ACK竞态、新命令绕过ACK、执行内部首次nav故障、active peer尚未停稳。所有新增行为均先观察失败再修复。
- `git diff --check`通过；实际导入agent/models/coordinator/runner/go2_exploration都解析到当前源码。安装为symlink且未新增模块，不需要为这次纯Python修改重新生成安装链接。
- 真机验证：无；没有重启现场agent/coordinator、恢复现场任务、发运动指令或改系统校时。

证据目录：`build/ranger-local/diagnostics/recoverable-pause-20261029/`。包含定向/广测/改前基线日志、`existing-failures.txt`、本轮相对原dirty工作树的 `complete-change.patch`、源码哈希、Go2实际代码与协议回信。执行计划在 `docs/superpowers/plans/2026-10-29-recoverable-cooperative-pause.md`。最终完整生产增量及375/806验证结果已通过信箱消息 `2ab6b0d7611b465c8c2dda04fc18554e` 成功投递。

## 待验证与下一步

现有现场进程仍加载启动时的旧代码。Go2明确未构建/切换其 `go2-local-scan-20261029` 安装；本机虽然symlink已指向新源码，旧进程内存不会自动更新。后续须双方加载新实现并在明确真机授权后验收：短暂costmap过期、单向/双向peer失联、SYNC失联、恢复后先双方停稳再续同一任务，以及人工停止不自动恢复。不能将本轮源码/离线完成写成双机部署或现场通过。

---

以下为上一轮只读根因排障的历史记录（其“本轮”指上一轮，不代表当前修改状态）：

# Ranger 会话交接：774a 定位/导航健康联合排障

更新时间：2026-10-29；分支 `ranger-reuse-go2-framework`；HEAD `ab922276108c21f44612b7805e26fc0560300528`。
本轮开始已有大量未提交代码、测试和文档修改，均保留。本轮没有修改运行代码、参数、网络或系统校时设置，没有重启、恢复任务或发送运动命令。

## 联合任务与结论

- 实验 task：`774a19a84fa945b1809715bc734700d6`。
- 信箱 task：`peer-health-774a-20261029`；本机 ranger，对端 go2；doctor 成功。
- 已确认本轮直接触发源：Ranger 系统校时前跳，使局部 costmap 的 ROS 时间戳年龄短暂超过 1 秒，`nav_ready=false`；当时 `localization_valid=true`。
- 当前健康恢复不等于任务恢复：首次 false 到首次 true 的发布记录间隔约 0.135402 秒，随后仍为 PAUSED。
- Go2 已报告同秒收到对端健康无效而停止；其取消之后的 Nav2 halt 错误不是最初触发源。对端报告不代替本机原始证据。

## 本机原始证据

启动记录根目录：`build/ranger-local/acceptance-20261015_161341/cooperative-startup-20261029_193955-3M5Cl7/`。

| 文件/行或来源 | 时间（本地 CST，epoch） | 证据 |
| --- | --- | --- |
| systemd-resolved journal | 19:52:06.523532，1793274726.523532 | Clock change detected. Flushing caches. |
| agent.log:128 | 19:52:06.524569，1793274726.524568999 | failed_checks 仅 local_costmap，reason=header_stale；定位、RGB-D、odom、地图一致性及动作服务均有效 |
| state.log:111343 | 19:52:06.524804 | 首个 nav_ready=false，localization_valid=true，PAUSED，local health invalid: nav_ready |
| systemd-timesyncd journal | 19:52:06.526952 | Initial synchronization to 91.189.91.157:123 (ntp.ubuntu.com) |
| state.log:111541 | 19:52:06.620261 | reason 改为 peer paused during task |
| state.log:111574 | 19:52:06.660207 | nav_ready=true，localization_valid=true，但仍 PAUSED |

健康诊断：costmap header_age=1.085712194 s，receipt_age=0.199324912 s，接收时 header_age=0.001171827 s。
三者相减得到推算墙钟前跳约 0.885215455 s；这是由年龄差推算的幅度，journal 直接确认发生校时，未直接记录调整量。
pose_age=0.993634462 s，小于配置 pose_max_age=1.5 s，因此本轮没有触发定位无效。

取证副本在 `build/ranger-local/diagnostics/peer-health-774a-20261029/`：
- `clock-journal.jsonl`：系统 journal 原始 realtime/monotonic 时间及消息。
- `health-failure.json`：agent.log:128 的健康对象。
- `state-window.json`：19:52:04—19:52:08 状态记录及原始行号；reason 受原始 ros2 echo 截断，完整健康原因以 agent.log 为准。

## 调用链、实际加载与暂停保留

- 宿主 agent PID 74003，19:42:14 启动；ROS_DOMAIN_ID=0，rmw_cyclonedds_cpp。
- 进程 PYTHONPATH 首选 `build/ranger-local/install/multi_agent_nav/local/lib/python3.10/dist-packages`；agent/models/nav2 文件链接到当前源码，相关源码修改时间均早于进程启动。
- `AgentNode` → `node_common.settings` → `load_config` 读取命令行显式 config_dir：`build/ranger-local/acceptance-20261015_161341/config`。
- 实际 `robot_ranger.yaml`：costmap_max_age=1.0，pose_max_age=1.5；未按源码默认值猜测运行参数。
- `AgentNode.tick` → `AgentRunner.tick` → `RangerAdapter.get_health` → `Nav2Adapter.get_health/costmap_transform`：costmap_transform 同时检查单调时钟接收年龄和 ROS 头时间年龄。
- `AgentMachine.tick` 在 nav_ready=false 后调用 stop 并发布状态。
- `models.is_nav_ready_pause` 不识别 `peer localization/navigation invalid`；`AgentMachine._peer_fault` 将该原因的对端 PAUSED 判为 `peer paused during task`，`stop` 撤销原本 nav_ready 暂停的自动恢复资格。这与本机 state.log 的变化一致。
- 已实际读取 Go2 通过信箱消息 `6d29b963abc84af5ae0c5cd87d18a0d2` 返回的代码片段：基于 HEAD `f6c75f27b20a17a2acc49e2ab3c56e1aa5bbf8f1`、分支 `go2-cooperative-task-layer` 的有未提交修改工作树；`cooperative_ws/src/multi_agent_nav/multi_agent_nav/agent.py` 的 `receive_peer`（67）、`stop`（405）、`check_health`（438附近）。不能把片段视作纯 HEAD 内容。对端报告运行 overlay 为 `go2-local-scan-20261029`，安装文件与源码字节一致，agent.py SHA256 `6c4a88a12f49b0c76e6fbae18843bf77a6df062cb673ce8cf32eb057b2f30d52`。
- 所读 Go2 `receive_peer` 先检查 localization/nav false 再检查 PAUSED；`stop` 锁存暂停，没有 `is_nav_ready_pause`。因此并非只改双方错误文案就能实现任务自动恢复。本轮不迁移该行为。
- Go2 回报其 Sport 日志 1481 条 wall/steady 偏移极差约 14.782 微秒，没有观察到本轮 Ranger 同量级的时钟跳变；该统计为对端报告，本机未读取完整 Sport 文件。对端 journal 无可见记录因权限不足，不能据此断言无系统校时。
- 对端回信副本 `build/ranger-local/diagnostics/peer-health-774a-20261029/go2-replies.json`；本机读取文件哈希 `source-sha256.txt`；发送给 Go2 的完整证据 `peer-evidence.txt`。双方已确认故障链条一致。

## 验证范围与下一步

- 已实现：本轮取证与联合消息交换、交接记录；无运行逻辑修复。
- 离线已验证：原始健康对象断言只有 local_costmap 失败；状态窗口定位始终有效、健康恢复后仍暂停；使用同一 install 路径导入 models/agent，断言旧对端原因不属于可自动恢复类型，全部通过。未进行无关全量构建/测试。
- 真机已验证：只读读取当前进程、日志和校时服务；timedatectl 在取证时显示 synchronized=yes、NTP active。未验证修复后的运动。
- 下一步优先在实验启动前确认两机校时完成且稳定；不要为这次校时事件直接放宽 costmap 新鲜度门限或关闭保护。
- 若需要修复跨版本暂停恢复，基于本轮已取得的双方实际代码另行确认任务恢复语义，再独立实现/验证；不能把可能也包含定位故障的混合旧文案一律当作可自动恢复。
- 当前任务仍暂停；任务恢复/重开及真机运动需用户明确授权。
- 另观察到 controller_server 原始日志存在持续的 sensor origin z 超出 local costmap 垂直范围警告；它早于本次事件，尚未证明是本轮触发源，不据此调整硬件/TF参数。
