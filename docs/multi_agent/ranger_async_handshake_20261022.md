# Ranger / Go2 异步启动与任务握手约定（2026-10-22）

> 此文为上一版历史记录。最终状态、时限、部署与接口以 [ans_async_startup.md](ans_async_startup.md) 为准；旧 WAITING_LOCAL/IDLE、3秒握手取消及旧 ACK 规则已被覆盖。

本说明供两侧实现对照。Ranger 分支 `ranger-reuse-go2-framework`，基线 HEAD `942c778d61c6300ab8562be360cb9c9efd1528cd`；本次修改未提交。Go2 侧独立修改时须对齐以下语义，不能仅对照旧 HEAD 或消息摘要。

## 不变的接口

- schema_version=1，AgentState / Command / Observation 的消息字段和类型不变。
- Ranger 本地 Domain0/lo ↔ Wi-Fi Domain40/wlo1，继续使用现有五条任务话题及 QoS。
- share_c_coordinates=false；common_frame_id、alignment_id 为空；pose_valid=false，common_pose 全零。
- 不共享 C 坐标，不复制对方地图、footprint、导航参数；不新增 topic、服务或 PING 消息。
- ready=true 仍只代表 A/B 当前接近导航成功、动作终态、停稳、距离≤1.00m 的严格验收，不能用于启动握手。

## AgentState：无任务时的等待

两侧相同，task_id / command_id / goal_id 为空，sequence=0，reply 为空，ready/deposited=false：

| run_state | 含义 |
|---|---|
| WAITING_LOCAL | 本机 localization_valid、nav_ready、stopped、原生动作终态尚未全部满足 |
| WAITING_PEER | 本机就绪，等待身份匹配且新鲜、健康、停稳的对端状态 |
| IDLE | 双方就绪，可以由操作者显式请求新任务 |

两端先后启动均可。WAITING_PEER 本身可被另一端接受，不要求对方先为 IDLE。无任务时的健康或通信暂失只回到等待，不锁存任务取消；身份、模式不匹配仍阻塞，不自动采纳未知身份。

合法对端消息即使 nav_ready=false，也要更新其本地单调时钟接收时间；是否在线与是否健康分别记录。stamp 仅用于追踪，不跨机相减判定超时。旧任务消息不能刷新当前任务心跳。

## start 与本轮握手

1. Go2 操作者显式 start。coordinator 检查双方新鲜、健康、停稳、空闲，允许 IDLE、WAITING_PEER 或已经停稳的 PAUSED，创建全新 task_id=T，phase=SYNCING；此时禁止发 EXECUTE。WAITING_PEER 允许单端重启后加入新任务，不要求另一端也重启；双向链路由后续任务握手确认。
2. coordinator 持续向两个机器人发送现有同步命令：

   ```text
   schema_version: 1
   task_id: T
   recipient_id: ranger 或 go2
   action: SYNC
   kind: SYNC_LOCAL_C
   phase: SYNCING
   command_id: ''
   sequence: 0
   common_frame_id: ''
   alignment_id: ''
   simulation: false
   ```

   Ranger 也接受 `action=SYNC_LOCAL_C, kind=WAIT`、空 kind 或 kind=SYNC_LOCAL_C，经规范化后语义相同。
3. **建立新 task_id 只接受 phase=SYNCING。** 新启动的 agent 不采纳旧任务 SEARCH_AB/DEPOSIT/PAUSED/DONE 阶段的 SYNC，避免重启后重新加入已取消的旧任务。接受有效同步后立即发布 task_id=T，不等待对方先确认。run_state=SYNCING；没有满足以下条件前 reply 为空：本机健康、停稳、动作终态；最近 coordinator SYNC 新鲜；已收到身份正确、同 task_id=T、健康停稳的对端状态。
4. 条件满足后发布 `task_id=T, run_state=SYNCING, reply=SYNCED`。command_id、goal_id 仍为空，sequence=0，ready/deposited=false。重复 SYNC 不重置已建立的本轮身份或丢失已收到的对端状态。也接受对端处于 SYNCING 且尚无 SYNCED，从而避免互相等待回执。
5. coordinator 必须收到双方**本轮**、新鲜的 SYNCED，且健康停稳、无命令或 goal，才能进入 SEARCH_AB，先同步该阶段再发送 EXECUTE。Ranger 在握手未完成、仍为 SYNCING 时拒绝 EXECUTE。agent 记录本轮已发出过有效 SYNCED；收到 coordinator 的 SEARCH_AB 同步后提交握手。此后对端可以先收到命令并移动，不再要求对端一直 stopped=true，但自身发新目标前仍须停稳/终态，双方健康与心跳检查继续生效。对端移动状态可能先于本机 SEARCH_AB 同步到达，不能丢失此前的有效 ACK 记录。
6. 本仓库 coordinator 接受 run_state=SYNCING 或 IDLE 的 SYNCED，标准发送值为 SYNCING。握手期限复用 scene.lost_timeout（当前 3.0s）；超时报 `task SYNC handshake timeout: <未确认机器人>` 并暂停本轮，无运动。start 返回 accepted 仅表示受理，不代表已经执行。

## 执行期故障与恢复

- 当前任务的有效消息先更新接收时间，再判断健康；消息持续到达但 nav_ready=false 应报告健康失败，不能误报 heartbeat timeout。
- 区分 `peer heartbeat timeout`、`coordinator heartbeat timeout`、`peer health invalid: <字段>`、`local health invalid: <字段>`、`peer restarted during task`、`task SYNC handshake timeout`。
- 执行期间对端 task_id 退为空代表重启：停车并锁存 PAUSED；旧 task_id 非空结果被隔离，不能推进或确认当前任务。
- 健康或消息恢复不自动恢复旧命令。确认双方停稳、原生动作终态后，操作者可以显式建立**新 task_id**重新握手；严禁自动重发 start。
- 心跳按固定频率发送，首次接收或有意义状态变化时及时回报；不能对每条心跳无条件再回复心跳，以免产生应答风暴。
- 两侧需要把耗时地图候选生成、选点图渲染与模型等待移出 ROS 心跳回调。本仓库复用 AsyncVision future，等待期间继续处理 peer / SYNC / PAUSE，取消时丢弃晚到结果。

## 当前需人工对照的身份与摘要

| 机器人 | local_map_id | map_version |
|---|---|---|
| Ranger | lab_ranger_20261018161152 | sha256:8b1a3f0587afd56e20431668647cf2624d83897a587090ef808424eb6049fc30 |
| Go2 | lab_rescan_20261018_155656_0pa6gm | sha256:6ae95d62d745c3bfb6eaf1e247cac27b32542251cbd1a4ccbc55f5d1febdd279 |

消息字段摘要使用 `sha256(json.dumps(Message.get_fields_and_field_types(), sort_keys=True).encode())`：

| 消息 | SHA256 |
|---|---|
| AgentState | 6248792879c5dcaf4907758ab76e758ca153646eb0c468086bf5a3d166c8a819 |
| Observation | 8f91230304d6ba3ad61cb548fa4ac9cad17faa4f7cb802176b99204e4947fd53 |
| Command | b87b90982f53677e3b316cbd4008ad06ccec0c3fe8a145aa85d16378ea0cc030 |

双方还须对照 action/kind 编码、SYNCING phase、SYNCED reply 的语义，以及实际导入文件；消息摘要相同不证明实现语义相同。Go2 侧不要用旧的“仅 task_id 相同就执行”规则。

## 启动和验收

两端先暂停旧任务并确认停稳，再从原终端正常退出旧 agents/coordinator；健康硬件和已核对的唯一任务桥可保留。两侧加载本次修改后重启各自 agent，可以任意先后；等待双方 IDLE，再启动唯一 Go2 coordinator 并显式 start，观察 SYNCING→双方 SYNCED→SEARCH_AB。

Ranger 入口仍为 `bash scripts/start_ranger_cooperative.sh`，或手动 `bash scripts/cooperative.sh ranger agent`；脚本已说明等待状态，不要求 Go2 先看到 Ranger IDLE 再启动自己。state/peer/watch 仅显示消息，可先后运行。

离线验收覆盖：任一端晚启动 60s、等待期间健康恢复、双方同任务 ACK、旧 ACK 隔离、握手超时、执行期失联/重启仍暂停、候选生成/渲染等待期间让出回调及取消。真实双机静止通信和运动须由现场另行验收，不能从离线通过直接判定已经完成。
