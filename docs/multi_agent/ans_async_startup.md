# 双机异步启动最终对齐说明与答复

日期：2026-10-22。适用当前 Ranger / Go2 私有坐标协同模式，`share_c_coordinates=false`。

**本文件是本轮唯一的接口和参数对齐依据。** 已依据用户指定的优先级逐项定稿：Go2 已确定的内容采用 Go2；Go2 未明确的报文细节采用 Ranger 已实现的约定。下文都是确定结论，不保留需要双方再次询问的选项。

依据材料：Go2 的 [async_startup_changes_20261022.md](async_startup_changes_20261022.md)、Ranger 源码及历史 [ranger_async_handshake_20261022.md](ranger_async_handshake_20261022.md)。历史说明中冲突的状态名、3 秒握手期限、IDLE 握手兼容规则均由本文件覆盖。

注意：**接口定稿不等于两端代码、安装或现场验收已经完成。** Ranger 已按本文件实施状态机、预算、采样和后台计算修正；Go2 部署及双方静止通信仍须各自完成。第 9 节记录落地状态，协议内容不再变更。未自动重启进程。

## 1. 最终决策

| 项目 | 最终采用值或规则 | 依据 |
|---|---|---|
| 准备状态 | PREPARING、WAITING_PEER、STANDBY | Go2 已确定 |
| 任务同步状态 | SYNCING、SYNCED，使用 AgentState.run_state | Go2 已确定 |
| 同步确认回执 | SYNCED 状态同时设置 AgentState.reply=SYNCED | Go2 未明确 reply 字面值，采用 Ranger 已实现回执 |
| coordinator 同步阶段 | Command.phase=SYNCING；完成后先同步 SEARCH_AB，再派工 | Ranger 已实现，Go2 说明无冲突 |
| 任务前准备等待 | 无限等待，每 5 秒输出当前原因，Ctrl+C 可退出 | Go2 已确定 |
| 准备采样 | WAITING_PEER/STANDBY；两端各至少 3 条新鲜、推进的合格状态，发布者和 QoS 正确 | Go2 已确定 |
| 心跳频率 | heartbeat_hz=2.0，即正常每 0.5 秒发布状态 | Ranger 现有实际配置；Go2 未指定其他值 |
| 执行期通信门槛 | lost_timeout=3.0 秒，按接收端单调时钟计时 | Go2 已确定，Ranger 配置一致 |
| 本轮总任务上限 | task_timeout=600.0 秒，从 StartTask 受理开始，包含握手与执行 | Go2 已确定 |
| 握手失败期限 | 受上述 600 秒总上限约束，不设 3 秒握手取消期限 | Go2 已确定，覆盖 Ranger 旧约定 |
| 新任务身份建立 | 只通过 phase=SYNCING 的合法 SYNC_LOCAL_C 语义同步建立新 task_id | Ranger 已实现，补齐 Go2 未明确细节 |
| 启动验收 READY | 不使用 ready=true；该字段继续只表示 A/B 到达严格验收 | 双方一致 |
| 消息和路由 | schema v1，现有三个消息类型和五条桥接话题不变 | 双方一致 |

## 2. 准备状态与无循环等待规则

### 2.1 PREPARING

本机真实定位、Nav2、输入、停稳或原生动作终态尚未全部满足。继续发布真实健康字段；不能因尚未收到对端消息而伪造本机 nav_ready=false。

本机就绪条件仅依赖本机。Ranger 继续使用自己的地图、TF、RGB-D、Nav2、/odom、footprint、真实停稳和动作终态；Go2 继续使用自己的硬件链路及控制许可。

### 2.2 WAITING_PEER

本机就绪且停稳即可进入，无需对端存在。此时持续接收和发送状态，不阻塞 ROS 回调。

对端晚启动一分钟、暂时未就绪或在任务前重启，都保持或返回准备/等待状态，不锁存 PAUSED。对端恢复后允许自动推进准备状态。身份或模式不符仍阻塞并报告原因，不自动采纳未知地图身份。

### 2.3 STANDBY

收到对端新鲜、身份匹配、健康停稳的 WAITING_PEER 或 STANDBY，即可进入 STANDBY；**不能先要求对端已为 STANDBY，更不能要求其先为 IDLE。**

无任务准备状态必须干净：task_id、command_id、goal_id 为空，sequence=0，ready/deposited/exited=false，reply 为空；私有模式资源与令牌字段为空。旧任务、旧命令、旧 goal、旧验收标志不能把准备过程推进为新任务握手完成。

入口可在两端各采到至少 3 条合格的 WAITING_PEER/STANDBY 状态后请求 StartTask，不必额外互相等 STANDBY；后续本轮握手负责证明双向消息链路。状态采样必须来自唯一正确发布者、QoS 匹配，并看到连续新状态及 stamp 推进；stamp 不用于跨机计算消息年龄。

## 3. 明确的报文接口

### 3.1 保持不变的字段

双方都使用 schema_version=1、simulation=false，common_frame_id 和 alignment_id 为空。AgentState.pose_valid=false，common_pose={x:0,y:0,theta:0}。

任务意图命令不携带共享坐标：position_valid=false，common_goal、object_position 全零，viewpoint_id 和私有模式资源字段为空。不共享 C 位置，不要求地图坐标对齐。

### 3.2 SYNC_LOCAL_C 的两种合法编码

接收端必须支持下面两种正式编码，发送端可维持自己现有的其中一种；这是确定的兼容集合，不需要再次统一字符串写法：

| action | kind | 接收语义 |
|---|---|---|
| SYNC | SYNC_LOCAL_C | 私有模式任务同步 |
| SYNC_LOCAL_C | WAIT、空字符串或 SYNC_LOCAL_C | 规范化为上一行相同语义 |

新任务同步的其余关键字段固定为：

```yaml
schema_version: 1
task_id: T                  # 本轮新生成的 UUID；不能复用旧任务 ID
recipient_id: ranger        # 对 Go2 的报文则填 go2
sequence: 0
command_id: ''
action: SYNC
kind: SYNC_LOCAL_C
phase: SYNCING
common_frame_id: ''
alignment_id: ''
position_valid: false
simulation: false
```

未列出的坐标字段保持全零，目标、资源、取消关联等字段保持默认空值。ack_sequence 在握手期间为 0，执行后沿用现有命令回执语义。广播仅适用于既有 SYNC 规则；EXECUTE 必须定向到机器人。

**建立或替换 task_id 只接受 phase=SYNCING。** 新启动的 agent 不采纳旧任务 SEARCH_AB、SEARCH_C、DEPOSIT、PAUSED、DONE 阶段的同步来建立身份；运行中的旧 coordinator 不能把重启端带回已经执行或取消的旧任务。

### 3.3 SYNCED 的唯一规范表示

```yaml
task_id: T
run_state: SYNCED
reply: SYNCED
command_id: ''
sequence: 0
goal_id: ''
ready: false
deposited: false
exited: false
localization_valid: true
nav_ready: true
stopped: true
```

同时要求身份、模式、schema 正确，本机原生动作已终态；动作终态由各自本地执行器核验，不新增或假设消息中存在 motion_terminal 字段。

**接收方以同 task_id、run_state=SYNCED、reply=SYNCED 的干净新鲜状态确认本轮握手。** 不再用旧 IDLE、STANDBY、仅 task_id 相同、仅健康为 true，或 run_state=SYNCING 但带旧回执来代替。Go2 说明未写明 reply 字面值，此处按 Ranger 已有实现确定为 SYNCED；Go2 若尚未同时设置两个字段，直接补齐。

## 4. 本轮握手、派工与乱序

1. 操作者发起本轮任务。coordinator 检查双方新鲜、真实健康、停稳及可建立新任务，生成 T，进入 phase=SYNCING，只重发 SYNC，不发送 EXECUTE。
2. agent 接受合法同步后，**立即发布 task_id=T、run_state=SYNCING**；此时不等待对端先接受，reply 保持空。
3. 本机健康、停稳、动作终态成立，且收到新鲜 coordinator SYNC 和身份匹配、同 T 的干净健康停稳对端同步状态后，发布 SYNCED。对端可以仍为 SYNCING、reply 为空；不能要求先收到对方 SYNCED 才发送自己的 SYNCED。
4. coordinator 收到双方本轮新鲜 SYNCED 后，提交握手，先同步 phase=SEARCH_AB，再下发寻找 A/B 的 EXECUTE。Ranger 找 B，Go2 找 A。
5. agent 保留自己已经发出过有效本轮 ACK 的记录。coordinator 提交后，对端可先收到 EXECUTE 并开始移动；不能因 peer.stopped=false 撤销已完成的握手。跨话题的移动状态可以早于本机 SEARCH_AB 同步到达，亦不能丢失此前的有效 ACK 记录。
6. 自身发新运动目标前仍检查本机停稳、动作终态、任务/命令身份和所有原运动门槛。对端当前健康与任务心跳仍须有效；握手提交后不再要求对端一直停稳。
7. 首条 EXECUTE 未到达或先于阶段同步到达时，保留幂等重发和拒绝过早执行的机制；重复命令不得产生重复导航目标。

同 task_id 的重复 SYNC 不重置已接受身份、对端消息或有效 ACK 记录。尚在 SYNCING 且从未派工时，单端进程重建可重新接收当前 T 的 SYNC，重新发布身份并完成干净握手；不能使用重启前缓存的 ACK 直接派工。

## 5. 在线、健康与时间规则

分别记录以下事实，不能复用一个时间戳覆盖全部含义：

- 有效传输接收时间：来自预期机器人、schema/地图/模式正确的有效消息已到达，即使其 nav_ready=false，也属于在线消息。
- 当前任务接收时间：消息还必须属于当前 T；旧任务消息不能刷新当前任务心跳，不能推进命令或握手。
- 当前健康：定位、导航等字段的真实值；在线但未就绪须报告健康原因，而非伪装为没有消息。

所有年龄用本机 time.monotonic() 接收时间计算。不要跨机相减 ROS stamp；准备采样中的 stamp 推进仅证明观察到了新状态。

| 时段 | 3 秒新鲜性门槛的作用 | 允许等待多久 |
|---|---|---|
| 无任务准备 | 决定对端状态当前是否可用；失去新鲜状态退回等待，不锁存取消 | 无限，每 5 秒输出原因 |
| 本轮 SYNCING | ACK 必须新鲜；缺少/过期消息或初始化未完成时撤回当前可确认状态并继续同步，不用 3 秒取消任务 | 从 StartTask 起计入 600 秒总上限 |
| 握手提交、进入执行阶段后 | 对端当前任务消息或 coordinator SYNC 超过 3 秒，停车并锁存 PAUSED | 不自动续跑 |

600 秒是整轮预算，不是“握手 600 秒再重新计执行 600 秒”；重发 SYNC、健康恢复和握手期单端重启均不能重置该预算。超过总上限报告 task timeout 并停止本轮。只有操作者显式建立新 task_id 才开始新的总预算。

心跳按 2Hz 持续发布，首次有效接收或有意义状态变化可立即回报；不能对每条心跳无条件回复另一条心跳形成应答风暴。耗时几何生成、完整地图摘要、图像渲染与模型等待必须复用单 worker/Future，让出 ROS 回调；等待结束后重新验证健康、停稳、动作终态、选择 epoch 和目标 footprint，才能发导航。取消后迟到结果不能推进运动；旧后台任务未结束时，新任务等待 worker 可用，不能阻塞心跳和订阅回调。

诊断至少区分：未收到有效消息、有效消息超时、本机健康失败、对端健康失败、身份不匹配、同步未完成、任务总超时、执行期对端重启及显式暂停。锁存 PAUSED 后保留首个触发原因，不用后续连锁暂停覆盖它。

## 6. 停止、重启与任务恢复

- 无任务时，对端晚启动、重启或短暂健康未就绪只影响等待状态，不锁存任务取消。
- SYNCING、尚未派工时，允许重启端重新加入当前 T，但必须重新满足同步与新鲜 ACK；总预算不重置。
- 握手提交进入执行阶段后，对端 task_id 退为空、健康丢失、身份变化或通信超时均停止并锁存 PAUSED；恢复通信不恢复旧命令。
- 显式 PAUSE 始终保留。CANCEL 沿用 Ranger 已有命令级规则，必须关联当前 command_id，不能取消其他 task/command。SYNCING 尚无执行命令时，使用 PAUSE 终止本轮等待；不新增空 cancel_command_id 的任务级 CANCEL 语义。两种控制均须通过既有 task_id、recipient、schema 和模式校验。
- 任务失败或暂停后，确认双方真实停稳、原生动作终态，再由操作者显式建立新 task_id 重新握手。已停稳的 PAUSED 可以参与这次新任务建立，但旧命令和旧验收标志必须清除；不要求另一端为了配合同步而重启。
- strict READY/deposited、动作取消终态、本机真实停稳、≤1.00m 验收及 task/command/goal 隔离全部保留，不用启动握手状态代替它们。

## 7. 启动入口的确定用法

本节根据本轮“更新启动流程并修改全流程启动脚本”的要求更新入口编排；线上消息、状态、时限及运动验收规则保持本文前述约定。

1. **两端完整入口路径**：Ranger 运行 `scripts/start_ranger_cooperative.sh`，Go2 沿用本机 `scripts/start_go2_cooperative_task.py` 的既有参数和硬件顺序。两端可先后独立运行。Ranger 仅保留本机定位确认（输入4），随后自动启动本机桥、agent、state/peer；不等待 Go2 步骤、不前台阻塞准备采样、不请求任务。Go2 完整入口负责两端各3条合格准备状态采样，通过后只请求一次任务，不补发手工 start。
2. **人工分组件路径**：Ranger 可用上述本机全流程或逐个启动本机组件；Go2 按既有步骤启动 bridge、agent、唯一 coordinator，在 Go2 执行准备采样，满足后由操作者显式 start 一次。单独启动 agent/coordinator 均不创建任务。

Ranger 全流程退出只表示本机组件已启动，agent继续准备并等待对端；不代表双机验收通过。`cooperative.sh ranger wait` 仍可在任务开始前单独做只读准备验收，不放回完整入口前台，避免 Go2 自动开始后永远等待无任务准备状态。新流程不需要新增消息字段或再次询问对齐。

Go2 的相机、底层、RViz、Sport、bridge、agent、coordinator 既有顺序保留。ARM 提示只依赖 Go2 本机准备，不等待 Ranger 已启动。Ranger 不启动 coordinator，不通过 SSH 控制 Go2，不自动重启对端，不更改硬件或导航参数。

准备采样无限等待，每 5 秒说明原因，Ctrl+C 可退出显示/等待。state、peer、watch 都只是显示消息，可先后运行；退出显示不停止 agent 心跳，也不暂停任务。

## 8. 对 Go2 五项待确认清单的逐项答复

### 8.1 Ranger 真实 adapter 与停稳判据

已确认本仓库有真实 Ranger 实现，并非缺少 live adapter：

- [factory.py](../../cooperative_ws/src/multi_agent_nav/multi_agent_nav/adapters/factory.py) 对 robot_id=ranger 选择 RangerAdapter，并只读取 robot_ranger 配置。
- [ranger.py](../../cooperative_ws/src/multi_agent_nav/multi_agent_nav/adapters/ranger.py) 复用本机 VLMNavigator 观察器和共享 Nav2Adapter。
- [nav2.py](../../cooperative_ws/src/multi_agent_nav/multi_agent_nav/adapters/nav2.py) 使用 map→base_link、真实 /odom、Nav2 action、costmap、RGB-D 和原生动作记录。

因此 Go2 保留公共接口即可，不覆盖 Ranger 自定义 adapter、观测器、地图、footprint、CAN 或硬件参数。Ranger 没有 Go2 专属 Sport/CONTROL_ARMED 门槛，不得添加 Go2 的硬件许可要求。

### 8.2 状态映射

最终采用 Go2 的 PREPARING、WAITING_PEER、STANDBY、SYNCING、SYNCED 五种状态；Ranger 旧 WAITING_LOCAL/准备期 IDLE 和仅 reply=SYNCED 的输出已更新。同步确认同时发送 run_state=SYNCED、reply=SYNCED。旧 IDLE 仍可作为旧共享坐标模式或执行后空闲状态，不能用它确认本版本启动握手。

### 8.3 源码、安装及部署

两侧都须使实际导入的 models、agent、agent_node、runner、go2_exploration 与本机最终源码一致；Go2 的 coordinator、入口与状态采样也须更新。不要仅凭 Git HEAD 判断未提交改动是否已加载。

Ranger 基线为分支 ranger-reuse-go2-framework、HEAD `942c778d61c6300ab8562be360cb9c9efd1528cd`，此前相关 33 个 Python 文件通过符号链接与当前源码一致；运行中的旧进程不会热加载修改。Go2 基线为 go2-cooperative-task-layer、HEAD `4fc673156c5c985c4d9a9149cfe0b08b1582d90b`，其说明明确尚未更新安装。

Go2 按自己机器上的 `docs/multi_agent/go2_runbook.md` 构建新的隔离安装目录，并用本机实际 COOP_SETUP_FILE 选中新安装；不得沿用尚未更新的 `build/go2-dual-acceptance-20261022/install` 或其他旧目录。本机没有该 Go2 runbook/最新源码，本文不臆造构建命令，也不把 Ranger 的绝对路径复制到 Go2。

两端均先暂停旧任务、确认停稳及原生动作终态，再从原终端正常退出旧 agents/coordinator 并加载最终版本；健康硬件和唯一任务桥可复用。预检发现安装与源码不一致应拒绝启动，不绕过。

### 8.4 schema 与兼容性

schema v1、三个消息定义、五条路由和 QoS 保持不变；改变的是状态/回执语义。禁止用旧 IDLE/健康状态或旧任务标志充当新握手。接收实现必须遵守第 3、4 节，无旧状态兼容捷径。

按 `sha256(json.dumps(Message.get_fields_and_field_types(), sort_keys=True).encode())` 计算：

| 消息 | 字段摘要 SHA256 |
|---|---|
| AgentState | 6248792879c5dcaf4907758ab76e758ca153646eb0c468086bf5a3d166c8a819 |
| Observation | 8f91230304d6ba3ad61cb548fa4ac9cad17faa4f7cb802176b99204e4947fd53 |
| Command | b87b90982f53677e3b316cbd4008ad06ccec0c3fe8a145aa85d16378ea0cc030 |

Ranger 仍为本地 Domain0/lo ↔ Wi-Fi Domain40/wlo1；Go2 本地 Domain0/enp4s0 ↔ Wi-Fi Domain40/wlo1。地图身份按同一机器人对照，保留双方各自地图：

| 机器人 | local_map_id | map_version |
|---|---|---|
| Ranger | lab_ranger_20261018161152 | sha256:8b1a3f0587afd56e20431668647cf2624d83897a587090ef808424eb6049fc30 |
| Go2 | lab_rescan_20261018_155656_0pa6gm | sha256:6ae95d62d745c3bfb6eaf1e247cac27b32542251cbd1a4ccbc55f5d1febdd279 |

### 8.5 验证边界

Go2 报告的 465 通过/5 项既有失败以及 Ranger 此前的 238 通过/1 跳过，都只是各自当时源码的离线结果，不能证明本最终契约已在双方安装中落实。

最终版本至少覆盖：单端晚启动一分钟；准备期健康恢复；握手等待超过 3 秒但未超过总上限；600 秒总预算；SYNCING 中单端重启后重新 ACK；旧 IDLE/旧任务/旧命令/旧 goal/旧 READY 不可确认；两种首条派工顺序、首条丢失和重复命令；后台几何/摘要/图像/模型阻塞期间持续心跳与取消；最后一次后台等待后的停稳/终态/footprint 复核；执行期失联和重启仍停车锁存。

完成上述离线测试和安装一致性检查，再做双方更新后的静止通信验收。跨机真实调度、单端重启及物理运动尚须现场验收；本文件不授权自动部署、连接对端或开始运动。

## 9. 实现落地记录（非待确定项）

以下是本最终契约的实施记录。Ranger 行已落实到本机源码；Go2 行是其需要独立完成的确定事项，不需要再次询问协议选择：

| 执行方 | 必须完成的工作 |
|---|---|
| Ranger 已实施 | 准备期 PREPARING/WAITING_PEER/STANDBY；同步双字段 SYNCED；新增 cooperative.sh ranger wait，保留独立只读验收；本机全流程不前台调用，由Go2入口采样并唯一请求任务 |
| Ranger 已实施 | coordinator 删除3秒握手取消；握手与执行共用 StartTask 起600秒预算。agent/runner不按命令重置预算；准备/同步等待可恢复，执行期故障锁存 |
| Ranger 已实施 | coordinator 只接受第3.3节双字段干净新鲜确认；拒绝旧 IDLE、SYNCING+旧回执、旧任务和残留命令/goal/资源/验收字段 |
| Ranger 已实施 | 分开记录有效传输与当前任务到达；watch分别显示在线与本轮在线。几何/摘要/渲染使用已有serial worker，摘要仅在文件身份/元数据不变时复用；最后后台等待后复核运动门槛。保留命令级CANCEL，握手取消使用PAUSE |
| Go2 | 保留其已确定的五种状态、等待/采样、600s 总预算、执行期 3s 门槛、首因保留和后台 worker 行为 |
| Go2 | 对照第 3、4 节补齐两种 SYNC 编码、SYNCING phase、新 task_id 建立门槛、双字段 SYNCED、阶段提交与乱序保护；已有行为符合者无需重写 |
| Go2 | 使用新的隔离安装并更新完整入口的状态判据，防止一次完整启动之外再发第二次 start |
| 双方 | 保留各自硬件/地图配置，完成本契约测试、实际导入一致性检查和静止通信验收后再进入运动验收 |

Ranger 新代码需要从原终端正常退出旧 agent 后重新加载，运行中的旧进程不会热更新。未切分支、提交、连接Go2、启动bridge或发送真机运动。

### 9.1 Ranger 本次验证结果

- 分支 `ranger-reuse-go2-framework`，HEAD `942c778d61c6300ab8562be360cb9c9efd1528cd`；未切分支或提交，原有修改保留。
- 最终相关离线回归：**371 passed，3 deselected**。覆盖晚启动一分钟、健康恢复、严格干净双字段ACK、超过3秒仍同步、600秒握手与执行总预算、任务/传输时钟隔离、重启、乱序/重复派工、旧任务迟到、后台等待/取消和最后运动复核。
- 严格验收探针覆盖 B/C 的0.90m及1.00m成功，>1.00m拒绝，导航失败/停稳未确认/动作未终态拒绝，缺少本地执行器不发目标，旧task/goal不推进。
- 3项原生ROS观察器用例未在最终离线集执行；没有启动实际DDS节点。采样器的晚启动、唯一发布者、QoS、Ctrl+C均通过模拟rclpy测试。
- 最终只读预检退出0：`live_gates=[]`、两种SYNC编码均接受、`motion_sent=false`；安装中的33个Python文件与源码逐字节一致。Ranger桥配置仍为Domain0/lo ↔ Domain40/wlo1。
- 原始日志：`/tmp/ranger-final-contract.log`；JUnit：`/tmp/ranger-final-contract.xml`；完整测试参数：`/tmp/ranger-final-contract-command.txt`；只读预检：`/tmp/ranger-final-contract-preflight.json`。

初次扩大测试收集曾出现22项失败：1项旧夹具仍用旧准备状态，已修正；其余依赖本机缺少的Go2文件/配置、历史Go2断言或原生ROS环境，未混入最终离线通过计数，也未为此改动Go2配置或启动真实节点。该次输出保存在 `/tmp/ranger-final-contract-tests.log`。

**Ranger源码和本机overlay已具备进入静止通信验收的代码条件。** Go2仍须独立部署其最终版本；双方停稳后正常退出旧任务进程、加载新代码，再执行准备采样与静止通信验收。尚未证明跨机通信、Go2实际部署或真机运动通过。

完整地图摘要、几何和渲染复用已有单worker；地图文件元数据不变时可暂用已验证摘要，worker空闲时仍周期复核完整摘要，发新导航前另作完整资产校验并复核最后运动门槛。未强行终止取消中的后台线程，其迟到结果无运动权限。


### 9.2 全流程入口编排更新（同日后续）

Ranger 全流程现可与 Go2 完整入口独立先后运行：仅本机定位确认输入4，其余本机组件自动继续；不再要求步骤3/6.2/6.3/6.4/6.5的远端确认，也不前台等待无任务准备采样。agent持续发送真实状态，Go2负责双方采样和唯一一次start。协议、2Hz/3秒/600秒参数、严格READY/deposited门槛均未改变。

相关离线启动、准备采样和健康检查测试 **89 passed**，其中全流程入口21项；覆盖远端晚启动/自动开始不阻塞本机、失败停止、唯一桥/旧任务检查和不发送start。日志 `/tmp/ranger-startup-workflow-20261022.log`。未执行真实启动或双机联调；Go2仍应加载其最终版本和正确的新overlay。

### 9.3 Ranger 本机场地 Keepout

Ranger新增本机地图对齐的Keepout掩膜、两costmap过滤、候选白区筛选与掩膜到达健康门槛，详见[ranger_keepout_20261022.md](ranger_keepout_20261022.md)。原定位地图身份、schema v1、五条bridge路由、异步握手和READY/deposited标准均不变。Go2如接入Keepout须以自己的原图和本机多边形另制掩膜，不复制Ranger坐标、PGM或footprint。
