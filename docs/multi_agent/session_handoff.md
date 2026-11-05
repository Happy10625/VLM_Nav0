# 当前交接：Go2 IDLE而Ranger未恢复——对端主机关机（2026-10-29）

本机分支 `ranger-reuse-go2-framework`，HEAD `a1c1b4c136b285b98732570a63ef2f1c3548eff3`。保留之前全部未提交修改；本轮只读取证、离线测试及更新交接，无运行代码修改、重启或运动。

## 确定时间线

任务 `4e6165e881454f00a9d9572ddf1c279a`，Ranger输出 `build/ranger-local/acceptance-20261015_161341/cooperative-startup-20261029_212451-TeBRGs/`。
- 21:28:07.497154：Go2同任务IDLE、ready=true、reply=SUCCEEDED（peer.log:7129），自身A接近完成，等待Ranger。
- 21:33:19.695920：最后一条Go2状态消息时间戳，仍IDLE；此后本机peer.log没有任何新消息。
- **21:33:21**：Go2上次boot用户systemd日志Reached target Shutdown / Exit the Session。
- **21:33:22**：Go2 `last -x -F` 明确记录system down。
- **21:33:22.982515**：Ranger agent.log:130报 `coordinator heartbeat timeout`；SYNC接收年龄3.038365秒>3秒，peer接收年龄2.852413秒，peer_receive_count=2092。state.log:71710起PAUSED，reason同上。
- 至21:37:10.129178：Ranger仍PAUSED，同一原因；任务共2579条状态loc/nav均true，暂停456条。Go2旧IDLE没有变成新鲜心跳，恢复条件不满足。
- **22:26:06**：Go2 `last -x -F` 记录重新开机（uptime -s为22:26:05）。本轮信箱先No route to host，随后可用，与这次开机一致；SSH期间也有间歇超时。

## 双端证据与逻辑

本机AgentMachine.tick在 `agent.py:469` 起要求同task、双方健康/停稳、peer和SYNC心跳都新鲜才恢复，IDLE本身不是禁止条件；`models.py`明确把coordinator heartbeat timeout列为可恢复。STOPPING/PAUSED期间reply=CANCELED为状态表示，不能单凭该字段判定检查点已丢失。runner对可恢复暂停保留本地执行器。

直接SSH读取Go2仓库 `/home/isee-pst/unitree_ros2/VLM_Nav`；当前HEAD `bce9de1ee4e9d2dd3c4ac6288703335767bb9ae9`（20:37:14提交），不据此倒推实验加载版本。`build/go2-field/cooperative-task.ry3kyafi/start.log`确认同task accepted。`build/go2-vlm-experiments/4e6165e881454f00a9d9572ddf1c279a_go2_1/events.jsonl`末尾APPROACH_VERIFIED=0.7545656959m、APPROACH_STATE=SUCCEEDED、TARGET_APPROACHED=0.7404643610m，印证IDLE是本地阶段成功。

Go2该次agent/coordinator ROS日志只有启动/接近记录，不能单凭静默认定崩溃；关机由wtmp和上次boot journal独立确认。当前双方宿主只读ps均未发现task agent/coordinator/domain_bridge，本机仅ROS CLI daemon。本轮未关闭它们，也不能推断是谁在何时关闭本机组件。

此前VLM TIMEOUT被重试/跳过，Ranger持续执行至心跳失联；不是本次终止的直接触发。实验agent.log仍出现小角度depth reobservation旧行为，本轮前向2米新修改尚未在该实验运行时加载，不能把它写成真机已验证。

## 验证、信箱与边界

- 同一install环境下 `test_nav_ready_resume.py` 和 `test_heartbeat_diagnostics.py`：**49 passed**；覆盖IDLE活动态、双心跳新鲜门槛和断链恢复，不代替现场恢复验收。
- 信箱TASK_ID `idle-ranger-4e6165-20261029`；初始证据消息 `f5bd9fe9eced493f9d7d466c2711cb4e` 已成功投递。最终关机结论消息 `02db09e1c12c4c46b71c3c787ef7e423` 首投及按原ID重试均因No route to host未确认投递；保留outbox，不重复send。原文保存在证据目录mailbox-report.txt，后续连通后可retry该ID。
- 证据目录 `build/ranger-local/diagnostics/idle-ranger-4e6165-20261029/`：state-summary.json、heartbeat-diagnostic.json、go2-shutdown.txt（摘录此前成功SSH输出，后续直接重新导出超时，非完整journal）、mailbox-report.txt。
- 已确认停止链为Go2主机关机→协调器/peer心跳丢失→Ranger可恢复暂停；未确认关机发起者或电源/系统触发方式。此次没有证据显示IDLE导致互锁。
- 重新开机不等于原task的agent/coordinator/bridge已重建，更不等于同任务心跳恢复。后续现场应按正常准备/启动流程重建组件并核对心跳；本轮不自动续行旧任务。

---

# 当前交接：单次前向2米深度探测与原点续扫（2026-10-29）

分支 `ranger-reuse-go2-framework`，HEAD `a1c1b4c136b285b98732570a63ef2f1c3548eff3`。本轮保留上一轮3米排除区的全部未提交修改，新增本节及本轮实现；未提交、未重启、未发送运动指令。

## 用户确认与实现

用户要求深度无效时不再旋转观察，前进2米继续观察，有效则接近，3次无效则返回原观察点继续。用户随后明确选择：“只前进一次，在新位置观察3次”。

- `vlm_nav/ranger_single_robot_execution.py::recover_target_depth/start_target_probe/build_target_probe_candidates`：稀疏、混合或超量程深度直接保存当前观察位置/朝向，沿车头方向提出唯一2米目标；不做小角度重观测，也不按目标像素偏转或提供1/1.5米回退目标。保持Nav2规划、滚动路径半径与协同路径监护。
- 探测到达后要求新鲜停稳帧，原地最多3次无效观察；深度/投影无效、目标不匹配/未检出以及API错误/超时计数。原点的首次深度失败不计入到达后的3次。有效深度沿用现有3帧确认与接近流程，确认完成前仍保留原点。
- 3次失败后以 `target_probe_return` 导航回原位置和朝向；只有返程成功才由 `RangerAdapter.finish_probe_return` 将原方向主目标记为UNCERTAIN，续接原扫描的下一方向，保留已扫描结果。启动点单帧观察则完成该单帧并交还选点流程。
- `on_navigation_result/send_navigation_goal`：滚动分段未完成时继续同一目的地，不提前观察或续扫；最终段使用原请求终点与朝向。`VLMNavigator`仅补充返程kind的通用失败处理，通用单机原深度恢复策略不变。
- 心跳/导航健康暂停复用已有检查点。RGB-D恢复新增探测专用续接：保留原点、方向、无效次数；运动中断先等原动作终止和停稳，再继续同一段。导航ABORTED、规划失败或返程目标被封锁仍进入安全失败，不能伪记已返回。
- 配置沿用target_probe_distance=2.0、target_probe_attempt_limit=3，注释注明Ranger将后者用于探测点无效观察次数；没有修改传感器/底盘/TF参数。

## 验证与审查

- 新专项测试先看到11项失败（仍旋转、仍按目标像素前进等），实现后通过；扩展覆盖返程初始单帧、部分确认和封锁目标。
- 独立只读审查发现2项需修复：部分确认后的projection_error漏计数；RGB-D恢复清空原点并串用旧方向结果。均先加入失败测试（1+3项），再修复；专项最终 **18 passed**。
- 最终相关13文件回归 **317 passed, 1 skipped**（7.93秒）。唯一跳过为test_exploration_display_records.py:151，Go2专用SSH部署脚本不在Ranger移植范围。日志：`build/ranger-local/diagnostics/forward-depth-probe-20261029/targeted-tests.log`。覆盖深度门限、目标确认/接近、暂停恢复、扫描续扫、3米排除、候选筛选和可视化。
- 使用同一 `build/ranger-local/install/setup.bash`；实际导入ranger_single_robot_execution、vlm_navigator、adapters.ranger均resolve到当前源码。纯Python修改复用symlink安装，无新增运行模块或构建需求。`git diff --check`通过。
- 计划与范围记录：`docs/superpowers/plans/2026-10-29-forward-depth-probe.md`。未跑无关全量测试。
- Go2参考机最近只读SSH访问报No route to host，未获得可用于本轮比较的新commit/实现；未声称已对齐对端，未发送新消息。

## 待现场验证

尚未重新加载现场进程，旧进程内存不会自动刷新。须在已有授权允许的现场重载后验证前向2米、无效三次原点返程续扫、有效深度接近及中途暂停恢复。没有真机通过结论。Nav2拒绝或执行失败仍会暂停，不保证障碍环境中总能到达探测点或返回原点。

---

# 当前交接：扫描有效性不再影响3米排除（2026-10-29）

分支 `ranger-reuse-go2-framework`，HEAD `a1c1b4c136b285b98732570a63ef2f1c3548eff3`，本轮代码未提交；保留前两轮交接修改。
用户要求：“无论这些观察点是否八个方向都有效,都建立3m排除区”。

## 已实现

- Ranger生成候选探索：导航到观察点、通过停稳/旋转空间检查并准备启动扫描时，记录 `scan_started=True`；不再等待八方向有效才建立3米候选排除。启动位置的单帧task_start沿用原规则，不纳入排除；没有到达或未开始扫描的候选不纳入。
- `go2_exploration.py::_explore_steps/update` 保留scan_started标记及原始observed/observation_failed状态，并复用generated_observations保存本任务扫描点；同任务重新进入探索保留，新任务清空。失败扫描不伪记为观察成功。
- `generated_candidates.py::observed_points/uncovered_points` 对已扫描点排除距离<=3米的候选；范围边界、中心坐标沿用原规则。两张VLM候选图、`ranger_visualization.py::record` 的RViz排除圈和 `adapters/vision.py::choose_candidate` 图例同步采用“尝试扫描”的含义。
- Go2不设置scan_started且observed_radius仍为0，不改变其候选策略。尝试SSH只读核对Go2当前参考实现，但192.168.137.128报No route to host；未取得新commit，不能宣称本轮比较了对端最新代码，也未发送新消息。

## 本轮验证

- 测试先行：新增/扩展排除边界、失败扫描地图显示及同任务保留测试，生产修改前9 failed/10 passed（符合预期的未排除近点等失败），修改后19 passed。
- 定向回归：同一 `build/ranger-local/install/setup.bash` 环境下，test_ranger_observed_coverage、test_ranger_visualization、test_ranger_initial_observation、test_generated_candidates、test_exploration_display_records、test_cooperative_preparation、test_robot_local_exploration、test_semantic_search：**157 passed, 1 skipped**（7.19秒）。覆盖完整扫描、API_ERROR、缺方向、扫描future异常、同任务保存/新任务清空、3米边界、Go2策略及启动单帧规则。
- 实际导入go2_exploration/generated_candidates/ranger_visualization/adapters.vision的文件resolve均指向当前仓库源码；沿用现有symlink安装，没有新增模块或构建需求。`git diff --check`通过。
- 未重启现场agent/coordinator、未发送运动指令、未做真机验收；现有进程不会因源码编辑自动刷新内存。需重新加载后验证不完整扫描结束时下一导航观察点落在排除圈外。排除表示本任务不再选邻域候选，不表示无效方向已完成有效识别。

---

# 当前交接：前两个观察点不足3米（2026-10-29）

分支 `ranger-reuse-go2-framework`，HEAD `a1c1b4c136b285b98732570a63ef2f1c3548eff3`。保留上一轮交接修改；此次只读排查并更新本文，没有修改运行代码/参数或启动运动。

同一任务 `5a420091f0e94e9d91ee611bcac117cf` 的两个导航观察点：
- `free/350/350`：目标map坐标 `(-4.575,-6.675)`，选点记录 `selections/2026-10-29/000019`，本轮显示P1。
- `free/370/330`：目标map坐标 `(-5.575,-5.675)`，选点记录 `selections/2026-10-29/000020`，本轮显示P3。
- 两目标直线距离 **1.414213562米**；两个点首帧拍照位置（scans/000072、000083）距离约 **1.307614296米**。不是下发3米目标后被Nav2截短。

原因：当前3米逻辑是“完整观察成功点周围的候选排除半径”，并非任意相邻停靠点的最小间距。`generated_candidates.py::observed_points/uncovered_points` 仅针对候选列表内、status为observed的点排除半径<=3米的候选。首点第7视角确认阶段两次校验失败（scans/000080、000081，validation_success=false、ValueError，响应仅有A）；agent.log记为API_ERROR并跳过该视角。`go2_exploration.py::explore_steps` 的incomplete_abc分支记为observation_failed；下一轮选点请求000020的history直接记录前点P1为observation_failed，因此没有3米排除圈。VLM选择P3的理由为“邻近机器人且未观察，符合优先探索规则。”失败只对该点设置30秒重试等待，不排除其邻域。

如将任务启动位置算作第一个观察点：scans/000071为task_start，拍照位置 `(-4.450666811,-6.285734314)`，到首个导航目标约 **0.408639837米**。task_start仅current_view_only单视角观察，不在生成候选列表中，也明确不写入completed_points，故不产生3米排除圈。首轮选点request000019记录START观察完成，但不表示完成全周覆盖。

证据均位于 `build/ranger-local/experiments/cooperative/ranger/`；Nav2实际目标由 `build/ranger-local/field-logs/bt_navigator_85197_1793278061996.log:9` 和`:11`交叉确认。读取的是当前源码，agent启动预检显示generated_candidates/go2_exploration安装与源码一致。

离线验证：source `build/ranger-local/install/setup.bash` 后运行 `python3 -m pytest -q cooperative_ws/src/multi_agent_nav/tests/test_ranger_observed_coverage.py::test_disk_boundary_failed_scans_and_initial_view`，**1 passed**。该既有用例明确验证observation_failed和task_start均不产生覆盖排除，而observed排除<=3米候选。

当前实现不保证任意相邻观察点都相距3米。若后续要求无论扫描成败都保持间距，需要独立区分“到访点间距”和“观察完成覆盖”，不能将失败扫描直接伪记为observed，否则会跳过未完成观察的区域。本轮未修改此策略，未做真机复测。

---

# 当前交接：最近 target_probe 导航终止排查（2026-10-29）

仓库 `/home/isee601/ws/VLM_Nav`，分支 `ranger-reuse-go2-framework`，HEAD `a1c1b4c136b285b98732570a63ef2f1c3548eff3`（20:36:15提交）；本轮开始 `git status --short` 为空。此次只读取证并更新交接，没有修改运行代码/参数、重启或运动。

## 已确认的终止原因

最近任务 `5a420091f0e94e9d91ee611bcac117cf`，Ranger命令 `/ranger/1`。启动目录 `build/ranger-local/acceptance-20261015_161341/cooperative-startup-20261029_204707-eayLHK/`。

- 目标B“浅棕色盒子”连续4次深度落地失败，`insufficient_valid_depth_samples:0/8`，3次小角度重观测后仍未取得可靠深度；最后目标像素 `(808,30)`。
- 20:52:51.393，发出 `target_probe` 观察子目标：从map `(-5.44,-5.85)` 到 `(-5.47,-7.85)`，约2米。它是深度恢复过程中的探测目标，还不是已确定三维位置的最终目标接近。
- 20:53:20.365598，controller_server首次 `Failed to make progress`。
- 20:53:20.383956，行为树将原因判为 `no_progress`，开始180°恢复旋转；20:53:33.985794旋转成功。
- 20:53:34.014248重新执行FollowPath；20:53:49.965666再次 `Failed to make progress`。
- 20:53:49.993736，Nav2导航失败；20:53:50.000879，Ranger收到 `target_probe goal failed with status 6`。本机action_msgs定义6为 `STATUS_ABORTED`。
- 20:53:50.035076，Ranger STOPPING，reason=`local execution: target_probe goal failed with status 6`；20:53:50.184163，peer.log中的Go2因同一对端原因PAUSED；Ranger20:53:50.533075确认PAUSED。

## 原始证据与调用链

- `agent.log:117`：target_probe失败；同目录 `state.log:58147` 和 `58345`：STOPPING→PAUSED；`peer.log:55936`：Go2随后的暂停。
- `build/ranger-local/field-logs/controller_server_85187_1793278061992.log:825`、`:1143`：两次进展失败。
- `build/ranger-local/field-logs/bt_navigator_85197_1793278061996.log:13`—`:16`：探测目标、恢复与最终失败。
- controller PID85187由20:47:41启动至取证时仍在，命令行实际参数 `/tmp/vlm_nav2_yexpy4p9.yaml`：`RangerProgressChecker`，required_movement_radius=0.1，movement_time_allowance=15.0。
- 当前commit的 `src/ranger_dwb_controller.cpp::RangerProgressChecker::check` 调用SimpleProgressChecker并发布no_progress；实际加载行为树 `build/ranger-local/install/vlm_nav/share/vlm_nav/behavior_trees/ranger_checked_turnaround.xml` 的FollowPath RecoveryNode只重试1次，故第二次失败结束导航。
- `vlm_nav/vlm_navigator.py::start_target_probe/build_target_probe_candidates` 从目标像素方位构造探测点，实际observer_config_file指向 `config/robot.yaml`，target_probe_distance=2.0；`on_navigation_result` 对失败target_probe调用fail_safe。runner将执行失败转STOP，不属于可恢复健康白名单。
- 本次启动agent.log预检证明agent/models/runner等安装内容与源码一致。任务启动在新commit之后，本次不再沿用上一轮“现场未重启”的历史判断。

## 验证与未定事项

- 离线逐条检查该task的3107条Ranger状态：localization_valid/nav_ready始终true。终止原因为原生动作ABORTED，不是heartbeat timeout或navigation readiness失效；上轮可恢复暂停修改不覆盖这类原生任务失败。
- 持续警告：传感器原点z约-0.19至-0.23，低于local costmap下界-0.10，提示无法raytrace。它可能影响障碍清除，但不能从警告直接证明此次无进展由它引起。
- 已查看扫描证据 `build/ranger-local/experiments/cooperative/ranger/scans/2026-10-29/000090/result.png` 和metadata/response：VLM置信度0.95、标点位于画面上沿；图像及深度日志不足以确认目标的可靠三维位置。不能据此直接断言误检。
- 没有找到本次cmd_vel/odom/costmap的rosbag；command_guard日志为空。现有证据能确认进展保护触发与终止流程，不能进一步确定底盘不动、控制器输出受限或障碍代价阻挡哪一项是物理原因。下一轮若排查底层无进展，应同步采集cmd_vel_nav、guard输出、底盘odom、TF和局部costmap，再决定是否修正TF/代价地图或恢复策略，不先放宽门限。
- 导航失败后agent于20:56:16被外部重新启动，任务清空；这发生在故障之后，不能倒置成终止原因。本轮未执行该重启。
- 信箱doctor成功，读取时没有待处理对端消息；本轮未发送新消息或启动跨机操作。

证据副本：`build/ranger-local/diagnostics/target-probe-5a420091-20261029/`，包括timeline.log、state-window.json、health-count.json、effective-nav2.yaml。无代码修复、无新增构建测试、无真机运动验证。

---

以下为此前任务的历史交接记录：

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


## 2026-11-05 — RoboRSI＋Codex CLI 可选入口，阶段 A

- 实施依据：`/home/isee601/Downloads/Ranger_RoboRSI_CodexCLI_Implementation_Plan.md`；本轮仅隔离安装、适配与离线验证。没有真实模型请求、ROS节点、硬件搜索、真机信号、Go2修改或信箱通信。
- 仓库/分支/HEAD：`/home/isee601/ws/VLM_Nav`，`ranger-reuse-go2-framework`，`a1c1b4c136b285b98732570a63ef2f1c3548eff3`。没有提交、推送、切分支或worktree。原13个unstaged及2个untracked保留；本文件原内容保留，仅追加本节。
- 原工作树与配置私有备份：`/home/isee601/ws/ranger-roborsi-backups/20261105-092309`（目录0700）；含完整摘要、原diff/dirty文件、源码与acceptance配置及marker。原入口、探索/识别/靠近/停止实现与参数均未修改。
- 新增：`scripts/start_ranger_roborsi.sh`；`tools/ranger_agent/{__init__.py,session.py,runtime.py,config.yaml,skills/find_target/SKILL.md}`；`test/test_ranger_roborsi_{session,runtime}.py`；`docs/multi_agent/ranger_roborsi_runbook.md`。
- 新宿主固定目标原文/ID/配置，持有原完整单机脚本的Popen；搜索与CLI互为兄弟进程。文件锁＋spawn前持久化消费记录保证同会话最多启动一次。失败、取消、模型提前结束、证据缺损均不自动整轮重试；原内部恢复行为保留。
- 取消：只对宿主持有且出生身份/进程组匹配的进程发一次SIGINT；观察退出最多30秒，不TERM/KILL搜索、不删除原运动锁。宿主状态落盘失败仍用内存身份收尾；超时保留STOP_UNCONFIRMED/active_process。业务、停止、证据与framework_error分别记录，模型自称成功不参与验收。
- 框架安装：`/home/isee601/ws/SenseNova-RoboRSI`，干净detached HEAD `0208cd72ec7c885a7219040cd2be459d127e374e`；专用`.venv/bin/python`（3.10.12），base editable＋pytest9.2.0/PyYAML6.0.3，无runtime/test extra。清除ROS Python环境后pip check通过，完整依赖见`build/ranger-roborsi-validation/20261105/framework-freeze.txt`。未升级系统Python/ROS/Codex。
- 固定CLI：`/home/isee601/.codex/packages/standalone/releases/0.166.0-x86_64-unknown-linux-musl/bin/codex`，`codex-cli 0.166.0`，SHA-256 `b8a11fedf24b2dfb59a70c9e52b030f9a508dcd49cd2dd0d41956f4c785156b8`。venv依赖自带0.163.0不作为执行器。配置模型gpt-6-astra/high，未验证登录或可用性。
- 接口调整：采用公开`EmbodiedAgent.start_episode(skills=..., deferred_tools=..., include_finish=True)`，保留静态skills和相同内部_run；固定版本run()无取消句柄。这样信号处理器只置位，由宿主主循环取消planner；未改上游源码。action_timeout_s=30在此SDK不为本地handler提供硬超时，取消观察由宿主单独实现。
- 新增离线测试：清除PYTHONPATH/PYTHONHOME，以专用venv执行两份新测试，最终51 passed。包含真实框架注册/延迟分发＋假Planner、跨进程争用、无害子进程SIGINT、框架清理范围、信号到达与启动竞争、证据/业务分离、外部语义输入及落盘失败收尾。终审3项Important均修复；额外磁盘异常用例亦通过。RED 5 failed/46 passed → GREEN 51 passed；无模型/ROS。
- 原定向回归：source同一`build/ranger-local/install/setup.bash`，源码PYTHONPATH优先，`/usr/bin/python3 -m pytest test/test_ranger_single_robot_parity.py test/test_ranger_depth_probe.py test/test_ranger_approach_transport.py -q`，117 passed。具体环境与命令见runbook。安装11个模块均为源码链接且摘要一致；acceptance仍为普通配置副本，未覆盖。
- 新shell语法/帮助及新增文件空白检查通过；全局git diff --check仍报告原有`AGENTS.md:33`尾随空格，不改旧dirty，不宣称全局通过。日志与版本/源码对照在`build/ranger-roborsi-validation/20261105/`；执行记录在`.superpowers/sdd/Ranger_RoboRSI_CodexCLI_Implementation_Plan/progress.md`。
- **真实后端仍关闭**：`real_backend_enabled=false`、`capability_verification=null`。本地features查询显示shell_tool/code_mode/code_mode_host/multi_agent关闭，但unified_exec仍true；接受disable参数不等于禁用能力，实际工具清单及间接执行路径未经模型验证。
- 后续阶段B须另行授权，在接触不到机器人及真实执行器的环境中验证CLI能力隔离并绑定版本/二进制/有效策略/证据摘要；阶段C须另行授权核实日常附加参数、现场Nav2/map_server和原dirty前向探测行为，再做两入口现场对照。本轮不声称模型或真机验收通过。
- 日常继续使用原入口无需Git回退，也不依赖新框架。未来共享Python源码改动需重启任务；安装项/C++变动按原流程构建/source，配置与相关节点分别重载。没有热更新承诺。

## 2026-11-05 — 最新单机实验接近失败取证（11:03:54 实验）

- 分支 `ranger-reuse-go2-framework`，HEAD `a1c1b4c136b285b98732570a63ef2f1c3548eff3`。本轮仅排障及追加交接，保留所有既有 dirty；未改运行逻辑、参数，未启动/恢复真机。
- 最新实验目录 `build/ranger-local/experiments/experiment.20261105_110354.29578786/`；run_id `b025101d4fac45739a86b0c0fb5b4474`，目标 B「浅棕色盒子」，mode=local_ranger_explore，最终 `APPROACH_FAILED: approach goal failed with status 6`。这是单机探索/接近执行链，没有跨机心跳故障证据，本轮无需信箱或 Go2 迁移比较。
- 直接失败链：11:08:09.427 接近目标 (-2.7784,-3.9071)；11:08:13.005 DWB 报 441 条轨迹全部无效、ObstacleFootprint/Trajectory Hits Obstacle（field-logs/controller_server_15300_1793847802004.log:284—287）；13.018 恢复旋转被 RangerSpin 拒绝，原因 footprint collision in remaining turn（behavior_server_15306_1793847802009.log:54）；13.047 NavigateToPose 中止。events.jsonl:282 对应 UUID `14576d2fe548494e809e57604686e230`、status=ABORTED/native_status=6；283记录距目标仍3.48748米；285进入APPROACH_FAILED，289结束会话。
- 退出而非等待的当前代码链：`single_robot_approach.py::on_navigation_result`（626—643）对 approach 非成功状态调用 `fail_safe`（95），置FAILED；`adapters/ranger.py::approach_steps`（60）返回APPROACH_FAILED；`go2_check.py::run_ros` 的explore结果分支（约401）返回失败，并在finally停止、落盘。距离验收不达标的重新观察分支不适用于此次ABORTED。该行为与用户要求“运行异常可等待恢复”存在缺口，本轮未修复。
- 之前两个候选导航分别发生no_progress（首个恢复旋转30秒超时；第二个旋转完成后再次no_progress），但探索继续，第三候选成功。11:07:16 因目标深度0/8启动2米前向探测，11:07:37导航成功；探测点前两次深度观察无效，后续恢复并确认目标。因此深度不足不是最终退出触发源。14份VLM metadata均validation_success=true；不以此断言识别物体绝对正确。
- 11:08:13.630 events.jsonl:288 的最终STOP_OBSERVATION确认stopped=true、motion_terminal=true，result.stop_confirmed=true。result.health为早期快照（pose仍在起点），不能当作最终位姿或最终健康证明；最终停稳位姿约(0.6252,-4.6433,2.7186)。事件继承字段中navigation_status=SUCCEEDED/native_status=4也可能陈旧，应以MOTION_TERMINAL的UUID/status为准。
- 参数/加载核对：入口 `start_ranger_single_robot.sh` source RangerVLM_env.bash local，再将仓库源码置于PYTHONPATH前端；observer_parameters先读取observer配置，再叠加arrival字段及固定本机覆盖。现有install中的single_robot_approach、adapters/ranger和行为树均resolve到当前源码。当天生成的 `/tmp/vlm_nav2_hmfo9jqy.yaml` 指向ranger_checked_turnaround.xml，failure_tolerance=0，行为树只允许一次checked turnaround；日志与其一致。但故障进程已不在当前沙箱可见进程中，未独立取得历史完整cmdline/config-dir/内存代码，不能宣称精确还原所有运行配置。
- 离线验证：读取原始事件、结果和14份VLM元数据，assert确认动作UUID/ABORTED、最终结果、最终停稳、VLM校验、探测成功、SESSION_END全部通过。未做无关构建或全量测试；无新增真机验证。
- 待查：实验未保存故障瞬间local costmap/footprint/TF的完整回放数据，不能判定碰撞对应实际物体、局部地图残留/误标还是姿态/足迹问题。下一步应针对该瞬间数据取证，并单独修复接近ABORTED后的等待恢复语义；保留碰撞检查，不凭这次日志放宽足迹或碰撞阈值。

## 2026-11-05 — 诊断终端误显示 TARGET_APPROACHED 的原因

- 分支/HEAD仍为 `ranger-reuse-go2-framework` / `a1c1b4c136b285b98732570a63ef2f1c3548eff3`；本轮只读排障并追加交接，未改代码、实验指针、运行参数或启动真机。
- `RangerVLM_env.bash` 从current-acceptance-run.txt设置ACCEPTANCE_RUN；`scripts/ranger_monitor.py::main`（272）默认读取该目录的current-single-experiment.txt。当前文件 `build/ranger-local/acceptance-20261015_161341/current-single-experiment.txt` 仍指向 `single-B-20261017_221059`，目标B为纸巾。
- 用当前ExperimentView.poll/render离线读取该旧目录，精确复现用户三行：结果TARGET_APPROACHED、任务停止确认是、原因-；排障VLM:视角3/DETECTED；提示底盘指令过期。前两行使用真实旧实验文件；第三行用人工构造的超过1秒未更新cmd遥测验证显示条件，不冒充现场遥测。
- 根因：旧文档 `docs/10.20协作Ranger单机验收.md:401` 要求手工写入指针；当前 `start_ranger_single_robot.sh` → `ranger_check.main` → `go2_check.main` 自动在experiment_records_root创建新目录，但代码没有更新current-single-experiment.txt。监控并不按最新目录自动选择，也不会因旧任务完成清除结果，因此把历史结果和当前/cmd_vel、odom、CAN遥测显示在同一屏。
- render的“任务停止确认”仅直接展示所选result.json的stop_confirmed，不等于本轮抵达；“底盘指令过期”仅表示距最近/cmd_vel回调超过1秒，不是该历史成功结果的失败原因，也不表示已经到达。
- 离线assert通过：旧指针精确显示上述三行；同一个ExperimentView切换到 `build/ranger-local/experiments/experiment.20261105_110354.29578786` 后正确显示APPROACH_FAILED/status 6及动作ABORTED，证明此次是目录选择陈旧而非切换后的结果缓存串用。没有真机复测。
- 临时查看本轮可用 `bash scripts/monitor_ranger.sh --experiment-dir /home/isee601/ws/VLM_Nav/build/ranger-local/experiments/experiment.20261105_110354.29578786`，仅启动只读诊断。长期需使实验创建与诊断选取使用一致的当前会话指针，并清晰标注历史结果；本轮按“为什么”请求仅定位原因，未实施该改动。

## 2026-11-05 — 诊断默认显示最新实验（已实现）

- 分支 `ranger-reuse-go2-framework`，HEAD `a1c1b4c136b285b98732570a63ef2f1c3548eff3`。保留既有未提交修改；本轮仅改scripts/ranger_monitor.py、test/test_ranger_monitor.py、使用说明和本交接。
- 默认不再读取current-single-experiment.txt；从RANGER_CONFIG/robot_ranger.yaml的experiment_records_root（未配置时为仓库build/ranger-local/experiments）读取受管理实验，用既有.experiment.json的owner及created_ns选择最新创建目录，每秒更新。复用ExperimentView.poll切换清空机制，新轮尚无事件也不继承旧结果；旧轮晚写日志不影响选择。显式--experiment-dir保留历史排障功能，根目录外的自定义输出须用它指定。
- 新增两个回归：旧成功→新空目录→新失败的自动切换，以及缺目录/损坏或无效元数据/旧指针不会被选中。先2 failed、17 passed（缺选择函数），实现后19 passed。source同一build/ranger-local/install/setup.bash后再验证19 passed；实际配置根目录选择experiment.20261105_110354.29578786，确认APPROACH_FAILED/status ABORTED。只读Python脚本无需构建或重启Nav2。
- 文档已移除旧指针写入步骤，说明默认最新选择及历史外部输出的显式查看方式。相关文件git diff --check通过。
- 未改运动、碰撞检查、实验写入或已有指针；未启动机器人/现场任务，未做真机验收。旧诊断进程不会自动热加载，需要仅关闭旧R5并重新运行bash scripts/monitor_ranger.sh；默认随后跟随最新实验。

## 2026-11-05 — 13:23:38 实验结束距离核对

- 分支 `ranger-reuse-go2-framework`，HEAD `a1c1b4c136b285b98732570a63ef2f1c3548eff3`；本轮只读取证并追加交接，保留现有修改。
- 最新实验为 `build/ranger-local/experiments/experiment.20261105_132338.5efe27a9/`，目标B浅棕色盒子，与上午失败实验不同。
- events.jsonl:153 APPROACH_VERIFIED及:155 TARGET_APPROACHED/:156 SESSION_END一致记录最终距离0.8049017813420389米。结束时间13:24:20.949765，arrival_retry=1；153记录stationary_samples=4、action_terminal=true，result.stop_confirmed=true。
- 本轮成功半径0.81米，记录距离比门限小0.0050982186579611米（约5.1毫米），符合当前程序的成功条件。距离定义见single_robot_approach.py::target_reference_distance：机器人定位参考点到已确认目标参考点的地图XY平面直线距离，不是车身边缘净空或实体尺量距离。
- 已断言最终验收/结束距离相同、动作终止/停稳记录及门限关系。末尾pose字段沿用task_start观察点，不能据其独立重算最终位置距离；本轮报告的是执行器实际保存的measured_distance，而非用陈旧pose伪造复算。未做真机重新测距、无运动或配置修改。

## 2026-11-05 — 13:47:31 实验失败：前向探测无进展，恢复旋转碰撞

- 分支 `ranger-reuse-go2-framework`，HEAD `a1c1b4c136b285b98732570a63ef2f1c3548eff3`；保留既有dirty，本轮仅取证与追加交接，无运行代码、参数修改及真机启动。
- 最新实验 `build/ranger-local/experiments/experiment.20261105_134731.59b99cc4/`，result=RuntimeError、stop_confirmed=true。失败动作是TARGET_PROBE，不是最终接近；UUID `c99f19301c414d44b4d62c6c6d3bb5f6`。
- 13:48:06候选点导航成功；13:48:10扫描旋转成功；13:48:15.812 VLM识别到B，但深度insufficient_valid_depth_samples:0/8（像素437,78），随后启动2米前向探测。两份VLM元数据校验均成功，没有API错误证据。
- 13:48:15.850探测从约(-1.63,-4.75)导航到(-3.46217,-3.95261)；13:48:45.572 controller报Failed to make progress（field-logs/controller_server_29299_1793857581219.log:226）。45.591行为树开始180度恢复旋转；47.992旋转过程中报Ranger spin rejected: footprint collision in remaining turn（behavior_server_29305_1793857581219.log:28），不是启动即拒绝。48.020导航ABORTED（bt_navigator_29307_1793857581220.log:13），events.jsonl:102确认同UUID/status6。
- 上层退出链：RangerSingleRobotExecution.on_navigation_result调用父类VLMNavigator.on_navigation_result，对target_probe ABORTED调用fail_safe，日志python3_29962_1793857651288.log:7明确target_probe goal failed with status 6。RangerAdapter.tick（248）把失败原因装入RuntimeError交给scan future；go2_exploration取future.result后因execution FAILED重抛；go2_check.run_ros（463）仅为ExplorationStopped保留str(error)，普通RuntimeError仅保存类型名。因此诊断结果丢失具体原因，且该分支没有等待恢复。本轮未修复。
- 13:48:48.553826最终STOP_OBSERVATION（events.jsonl:107）stopped=true、motion_terminal=true，位姿(-3.3303587201,-4.3350455048,2.5442232894)。距前向探测终点约0.40450978米；这是探测点距离，不是物体距离，目标尚无有效深度坐标，不能报告到物体的可靠距离。
- 离线assert已核对同UUID/ABORTED、最终停稳、SESSION_END和result一致。最新生成/tmp/vlm_nav2_iot7m696.yaml记录RangerProgressChecker(required_movement_radius=0.1,movement_time_allowance=15.0)、failure_tolerance=0；未取得历史进程完整加载参数，不将该文件独自视为运行参数快照。
- 未保存故障瞬间代价地图完整回放，不能确定无进展/旋转预测碰撞是现场障碍、地图误标还是运动执行偏差。后续需对应时刻costmap/footprint/TF与速度证据；另应修复异常原因透传及可等待恢复语义，不能用放宽碰撞检查替代排障。

## 2026-11-05 — 取消Ranger前进2米探测，恢复每点最多3次小角重观察（已实现）

- 分支 `ranger-reuse-go2-framework`，HEAD `a1c1b4c136b285b98732570a63ef2f1c3548eff3`。按用户明确要求撤销本工作树中前向探测/原点返程恢复行为；不是整体回退，其他dirty改动保留。修改前相关文件副本 `/tmp/ranger-rotation-before/`。
- RangerSingleRobotExecution删除前向/返程目标、分段导航、探测返程及其传感器恢复分支；深度不足、混合、超范围均只发送depth_reobserve Spin。沿用10°小角上限和朝目标方向/接近中线时交替转向；无效焦距不能求bearing时用已有交替小角。通用VLMNavigator原有前向探测方法未修改，Ranger恢复不再调用它们。
- 计数保存在同一scan字典，最多min(3,depth_reobserve_attempt_limit)次恢复旋转。每次发起即消耗额度；API失败、格式无效、超时、未检出、投影/深度失败不会获得额外重试额度。三次耗尽后RangerAdapter.finish_depth_reobservation把当前目标记UNCERTAIN、position_valid=false，推进原扫描索引；初始单帧直接结束该观察。后续方向不重置额度，新观察点start_scan有新额度。
- 有效深度仍按原有多帧确认进入接近；部分确认后失败继续使用已有额度。暂停/传感器恢复保留扫描位置和预算。正常导航/接近的执行路径监护、目标固定、距离验收和旋转碰撞检查均保留；原地Spin没有导航路径要求。没有增加运动门槛。
- 已实际读取本仓库a1c1b4c的ranger_single_robot_execution.py历史调用链及当前vlm_navigator.py::recover_target_depth/target_relative_bearing，复用已有小角策略；本轮未读取新Go2参考实现、未声称与Go2对齐、未跨机迁移。
- 修改范围：vlm_nav/ranger_single_robot_execution.py，adapters/ranger.py，config/robot.yaml注释，两份Ranger恢复/parity测试及使用说明。test_ranger_depth_probe.py沿用文件名但内容改为原地恢复专项。此前针对前进导航路径丢失的专项改为验证原地旋转不需要导航路径，正常导航/接近监护测试仍保留。
- 离线验证：新专项首次13 failed（仍发NavigateToPose等）；实现后通过。只读审查发现深度NaN与无效焦距同时出现会除零，新增首次/活动恢复失败测试并修复；投影异常路径亦有失败后通过记录。最终相关8文件 **241 passed**（2.70秒），独立终审专项 **19 passed**，无剩余重要审查问题。证据 `build/ranger-local/diagnostics/depth-rotation-20261105/`。未做无关全量测试。
- 使用同一build/ranger-local/install/setup.bash；实际导入RangerSingleRobotExecution和adapters.ranger均resolve到源码。acceptance的observer_config_file与arrival_config_files指向仓库config/robot.yaml，observer_parameters实际输出angle=10.0、limit=3；未覆盖acceptance副本/硬件参数。Python symlink安装复用，无需构建。相关文件git diff --check通过。
- 真机验证：无。未重启任务或发送运动指令；已有进程不会热更新，需下一次重新启动任务加载。待验证实体旋转幅度、停稳与现场三次后续扫效果。旋转被已有碰撞检查拒绝时仍遵循原动作失败处理，本轮未扩大到全局异常等待恢复修复。

## 2026-11-05 — 14:11:05实验结果核对：距离未达标后重新观察被停稳检查中断

- 分支 `ranger-reuse-go2-framework`，HEAD `a1c1b4c136b285b98732570a63ef2f1c3548eff3`；只读取证及追加交接，无运行逻辑修改/运动。
- 最近完成实验 `build/ranger-local/experiments/experiment.20261105_141105.7408647b/`，14:11:05创建、14:15:56.728929结束，目标浅棕色盒子。result=`initial observation requires confirmed stop`，stop_confirmed=true；证据写入未失败、VLM证据无挂起。
- 接近阶段：第一次在约0.8828m处发取消并停稳后不达标，自动重试一次；第二次取消时记录约0.80785m，但最终接近验收(events.jsonl:412)为0.8182511343m，超过0.81m约8.25mm，arrival_retry=1、stationary_samples=4、action_terminal=true。不是Nav2 ABORTED；两次接近MOTION_TERMINAL均为主动取消CANCELED。
- :415 APPROACH_REOBSERVE后，RangerAdapter.approach_steps调用reset_task并返回RESUME_SEARCH；go2_exploration.explore_steps的initial_point检查（590）遇health.stopped或motion_terminal不满足即抛异常，未等待恢复，:416停止。:418记录motion_terminal=true、stopped=false，:419于14:15:56.727045变为stopped=true。因此本轮接近的停稳采样验收与adapter的持续停稳判据未在重新观察入口同步，导致任务退出；不能把结束停稳反推为入口检查已满足。
- 当前配置arrival_stationary_samples=3；acceptance robot_ranger配置stationary_linear_speed=.03、stationary_angular_speed=.05、stationary_duration=.5。以上是当前磁盘配置核对，不替代历史进程参数快照。收尾后出现rgbd_ready.no_frame警告晚于退出，不是本轮首先触发源。
- 最后实际STOP_OBSERVATION位姿(-7.3881768256,-4.2206952680)，固定目标(-7.5961191311,-3.4255461070)，独立XY计算最终停稳距离0.8218894029m，超过门限约11.89mm。末尾继承的distance=.818251仍是先前验收值，勿混为最新复算距离。
- 新恢复行为现场证据：python3_35548_1793859065161.log:6/8/10记录同一观察点恢复旋转2.8°、-10°、+10°；三次后:12恢复SCANNING，随后继续其他观察点。本轮所有事件没有TARGET_PROBE动作。只确认本轮这条成功执行路径，不据此宣称API异常等全部现场验收通过。
- 离线断言已确认最终结果/停稳、APPROACH_REOBSERVE及无前向探测，复算最终目标距离。待修：重新观察入口应等待既有停稳条件满足，而不是因短暂未满足直接退出；保留0.81m验收及现有停稳条件，不凭这次毫米差调参。本轮按结果检查请求未修代码或重启。


## 2026-11-05 — Ranger单机运行异常改为等待恢复（已实现，待真机验证）

- 分支 `ranger-reuse-go2-framework`，HEAD `a1c1b4c136b285b98732570a63ef2f1c3548eff3`。保留原dirty；无提交、推送、参数放宽、真机启动或运动。实施计划 `docs/superpowers/plans/2026-11-05-recoverable-ranger-experiment.md`；修改前副本及验证日志位于 `build/ranger-local/diagnostics/recoverable-experiment-20261105/`。
- 用户要求运行异常等待恢复，尤其重新观察前未停稳不能终止。`go2_exploration.py::explore_steps` 为Ranger standalone维持同一个task/command，捕获运行异常及非成功返回；`explore` 将ROS spin回调异常注入恢复流程。观察入口等待原有stopped/motion_terminal条件。原默认总体timeout不再终止standalone等待，局部请求/动作超时仍用于触发停止与恢复。持续回调故障每轮让出控制，不忙循环；人工取消或task身份改变不恢复旧任务。
- `RangerAdapter.wait_for_experiment_recovery` 取消旧动作并发布停止，等待原定位/Nav2/local+global costmap/RGB-D/原生动作终态/底盘停稳/无其他owner条件；沿用blocked_goal_seconds冷却及sensor_recovery_frames。动态启动准备 `go2_check.py::wait_ranger_startup` 同样等待。静态错误配置、地图身份不一致和人工取消仍保留拒绝/退出语义。
- 恢复保留固定目标、扫描index/initial/headings、扫描点ID、已观察覆盖、接近重试次数和blocked目标缓存。原地深度重观察预算按原点ID保存，重建scan不重置最多3次额度。短暂RGB-D恢复保留原扫描方向；中断的SPIN/SPIN_WAIT恢复后重新完成该转向，不直接以旧方向拍照。
- 独立审查发现Nav2.tick内部stop_motion会误把定位/证据故障当人工结束；已增加stop_failed_scan分派，Ranger standalone保存扫描检查点后进入等待，其余模式沿用原停止。正常Spin ABORTED的失败scan也保留检查点。
- `Nav2Adapter._set_motion_fault` 记录action发送/响应/结果通信失败导致的未知原生终态，即使此前已有cancel_timeout也不能被掩盖。只有cancel_timeout/cancel_request_error且真实旧动作终态已收到、无剩余动作并停稳时清除对应锁存。未知动作终态或证据写入故障不能自动证明修复，会持续等待；没有用空motions或零速冒充终态，没有自动清除证据故障。
- 诊断 `scripts/ranger_monitor.py` 显示WAITING_RECOVERY等待原因和RECOVERED继续状态，清理旧失败摘要；记录仍落在当前实验。使用说明已更新。原read-only/协同start_scan调用仍可在未begin_local_execution时使用。
- 离线验证：恢复专项23 passed（独立只读终审同样23 passed）；最终相关16文件 **453 passed、2 failed**，日志 `regression-final.log`。两项失败均为 `test_go2_executing_path.py::test_execution_path_timeout_logs_receipt_age_once[go2/ranger]`：日志实际写stderr但caplog未捕获（0条），不是路径行为断言失败。在进程内用HEAD版nav2.py替换当前模块复核，同两项失败、54 passed（`path-baseline.log`），不是本轮新引入；未修改无关日志测试。各新增故障先失败再修复日志已保存，内部扫描故障追加回归60 passed。相关修改git diff --check通过。
- 加载验证：source `build/ranger-local/install/setup.bash`，`/usr/bin/python3` 实际导入go2_check/go2_exploration/adapters.ranger/adapters.nav2/single_robot_approach/ranger_single_robot_execution均resolve到当前仓库源码；Python链接安装无需另建overlay。现有进程不会热更新，下一次重新启动任务才加载。
- Go2参考限制：已读本机信箱说明及角色配置并尝试doctor；只读SSH访问192.168.137.128返回No route to host，无法读取本轮Go2对应commit/函数；未声称已比较或迁移Go2修复，未发送跨机消息。
- 真机验证：无。待现场验证停稳后再次观察、定位/代价地图/RGB-D短暂中断恢复、路径/旋转/接近失败恢复，以及长期等待时人工Ctrl+C取消。不可把离线通过当现场通过。


## 2026-11-05 — Go2/Ranger等待恢复兼容同步（离线闭环，未真机验收）

- TASK_ID `go2-resilient-experiments-20261105`。最初只读；随后用户分别明确批准“允许代码同步和离线测试”及“允许发送这些源码快照”（七个纯状态机文件同步前后）。无提交、推送、机器人/桥接/ARM/运动启动；保留原dirty。仓库分支 `ranger-reuse-go2-framework`，HEAD `a1c1b4c136b285b98732570a63ef2f1c3548eff3`。
- 信箱原192.168.137.128超时，用户临时改址192.168.1.118。使用 `/tmp/ranger-mailbox-go2-resilient-20261105` 独立客户端，角色ranger/peer go2；SSH HostKeyAlias严格复用原Go2已信任密钥，doctor ok。未改原SSH/信箱配置。首次源码外发被自动审批拒绝，获得用户单独明确授权后才发出。邮件回复确认delivered后才ack。
- 已实际读取Go2 `/home/isee-pst/unitree_ros2/VLM_Nav`，基线 `323d5c52737477962a874ae012db117be5a93f34`，其实现为未提交工作树，以保存的source SHA为准。读取 `models.py::TaskClock/is_recoverable_pause/Health`、`agent.py::task_clock/check_health/stop/_resume`、`coordinator.py::task_clock/_fault/_resume/tick`、`mock.py::make_machine`、`runner.py`任务钟接线/错误分类/派发，以及`go2_exploration.py::is_recoverable_execution_error`、`go2_check.py`证据收尾。Go2无Ranger的adapters/nav2.py与LOCAL_RESUME检查点，未整文件覆盖或迁移Sport/网卡/传感器参数；wire消息无改动。
- 原不兼容已实际证明：Ranger缺wait_recovery、三条新增明确软原因、有效任务时钟；STOPPING/PAUSED/恢复握手也消耗600秒，定位瞬断锁为hard。收到新peer reason也变peer paused during task。旧版实际混合回放v2为5/11通过、六个恢复场景不兼容（不是API适配失败）。
- 当前已实现：models增加默认false的AgentContext.wait_recovery、内部Health.hard_fault和TaskClock；加入三条精确软原因local localization temporarily unavailable、peer localization temporarily unavailable、local execution temporarily unavailable。agent/coordinator冻结SYNCING、STOPPING、PAUSED、awaiting-peer恢复握手的任务预算；runner/exploration额外传任务钟，健康/心跳/路径/单次动作/取消仍用真实monotonic。只迁移明确临时错误，status5/未知结果/身份/证据/人工停止不自动清锁；新定位软原因不能覆盖已收到hard。
- 恢复保持同task、成功command/READY与已确认目标；Go2没有的Ranger扫描checkpoint与每点最多三次小角深度预算保留。同task重入不重置预算/接近retry，仅同task+command恢复固定目标。Nav2恢复采用成功后才移除checkpoint的顺序；guard或callback再失败会重暂停已发动作并保留未执行项。只补偿确认/扫描工作预算，不平移旧取消/停稳安全时刻；确实重发且身份匹配的新原生action才记录新开始时间。
- 取消、暂停、健康读取等异常不能逃逸停车链：runner保护故障分类读取，取消/清理异常仍尝试stop_motion；Nav2/Ranger以finally保证零速发布尝试，先detach旧scan避免视觉取消失败后继续执行。人工stop令execution_active=False，未知原生终态/证据故障保持hard。Go2独立发现首EXECUTE待效果遇hard仍留LOCAL_EXECUTE，本机新增三项先失败回归后修复：agent.stop清空旧效果，仅排STOP；soft升级hard也换成新hard STOP并保留原goal_id。独立审查确认完成标记与待STOP不丢失。
- 必要证据与展示隔离：go2_check事件/SESSION_END/result写失败返回非零、标记EVIDENCE_WRITE_FAILED；VLM证据pending标记EVIDENCE_INCOMPLETE；motion_result单独保留。展示/marker/末尾stdout故障不遮盖取消/停车；StopUnconfirmed优先。不是所有日志错误都新增运动门槛，也不能用后续可写冒充历史证据完整。
- 配置已接线：仓库模板 `cooperative_ws/src/multi_agent_nav/config/{scene,robot_ranger}.yaml` 与实际入口 `build/ranger-local/acceptance-20261015_161341/config/{scene,robot_ranger}.yaml` 设置wait_recovery=true，仅修改该字段；修改前文件在临时目录config-before。使用实际load_config→make_machine(ranger,False)/make_coordinator验证两开关True且private=True。Go2亦通过信箱确认其真实构造开关。未改本机robot_go2硬件配置。
- 加载/构建：source `build/ranger-local/install/setup.bash`，multi_agent_nav/vlm_nav prefix都为该install；任务模块导入和包share的scene/robot_ranger YAML均resolve当前源码。纯Python/配置链接安装，无需新增overlay或构建；已有进程不会热加载，新启动任务才生效。
- 最终定向扩大回归：27文件 **707 passed、8 failed、1 deselected**。8失败均test_field_preparation既有Go2地图路径/报错大小写/离线就绪断言，用HEAD的config/go2_exploration/go2_check在独立进程替换复现同8失败；不修改无关断言。初轮误包含的旧read-only observer用例尝试ROS初始化并因缺realsense/沙箱网络失败，随后明确剔除；没有硬件或运动验收。新恢复安全20项、必要证据20项、runner停车优先10项、兼容性21项均先复现失败再修复。相关diff --check与py_compile通过。
- 双方实际版本互验：本机审读Go2发来的v2驱动（SHA ce311257c651878b2cb0fed894d069927799f1b10cca661f5a3a4eb22a1b7a18），使用实际Go2纯模块及其arrival.py（SHA a7acb0df12139c0deca548c469e524c6de6001f65d709ee7add989ec1440f8cb），严格比较wire dataclass字段，无字段过滤；final2 **11/11 PASS**，运行前后源码hash不变、未导入rclpy。六类长暂停逐1300tick检查不出运动效果；旧混合5/11。Go2信箱 `182ecdc8b923447a984874d33fd7481c` 独立确认其本地v3 **17/17 PASS**（含对称Ranger故障）以及首EXECUTE三hard反例 **3/3 PASS**。v3源码未获得对端外发审批，本机未读取/运行v3；该17+3是对端验证声明，非本机执行。
- 最终七文件原文JSON SHA `3c9eefcc29ee5b61f3258f1e7bbe38631f3fdbc56ae0abfc73243224f5e658ae`；agent SHA `caaf2ac403335e2be6ac4ec4dcb1cb080f9973c03d33bc50ccbd3117256b2ff7`。最终快照通过信箱回复 `57aa269b6f5247aab589e67dbe852038` 投递成功。全部本机验证/源码清单/混合回放结果保存于 `build/ranger-local/diagnostics/go2-wait-compatibility-20261105/validation-manifest.json`及同目录；实施计划 `docs/superpowers/plans/2026-11-05-ranger-go2-wait-compatibility.md`。
- 边界：双机离线同步通过；没有真机、实体停稳或现场网络验收。wire没有motion_terminal字段，对端终态依赖双方只在停稳+真实终态后发布PAUSED的约定。异步消息尚未传播的远端hard仍存在传播窗口，不承诺全局原子恢复/零延迟急停；v2末两native场景仅验证Go2状态机，Ranger原生adapter由本机受控Future专项验证，不扩大结论。后续真机实验须另有明确授权。
- 最终信箱闭环：Ranger回复 `2ae972e901194063a8dbeed43e7f650a` 与 `34a4904223904034bbd9157c5f65992f` 均delivered=true，随后对Go2终验消息 `182ecdc8b923447a984874d33fd7481c` / 接收消息 `b38076e73ccc4b30bead9a7c0d9cde4d` ack=true。双方无待实施现场动作；收到新授权再进行真机实验。
