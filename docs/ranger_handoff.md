# Ranger PC handoff (no Ranger connection performed)

当前私有 C 模式以本文末尾 **2026-10-20 独立 C 完成（最新覆盖）** 为准；前文中的 Go2 先行、双缓存屏障和 EXIT 要求仅保留为历史接入参考。

Transfer the commits on `go2-cooperative-task-layer` through the final commit
listed in the delivery/test record. Keep `ranger_go2_codex_plan.md` and the
implementation plan beside the code. Use identical `multi_agent_task_msgs` v1
and shared `multi_agent_nav`; keep old `multi_agent_msgs` and
`/multi_agent/agents/{go2,ranger}/state` consumers unchanged.

## Public protocol

Five reliable, volatile topics:

| Topic | Type | depth |
|---|---|---|
| /multi_agent/go2/state, /multi_agent/ranger/state | multi_agent_task_msgs/msg/AgentState | 10 |
| /multi_agent/go2/observation, /multi_agent/ranger/observation | multi_agent_task_msgs/msg/Observation | 50 |
| /multi_agent/go2/command | multi_agent_task_msgs/msg/Command | 10 |

Full fields are in generated interfaces and `models.py`; all poses on these
topics are common lab_ref, with local_map_id/map_version/alignment_id explicitly
identifying the source. Image/stamp fields are trace-only; heartbeat expiry
uses receiver monotonic time. `simulation=false` is mandatory for live peers.

AgentState persists READY/deposited, current command identity and terminal reply,
resource request/occupancy/exit. Coordinator SYNC contains ack_sequence and phase.
Command actions: EXECUTE, SYNC, PAUSE, CANCEL, GRANT_RESOURCE. EXECUTE sequence
is per robot/task and idempotent. CANCEL targets cancel_command_id. Resource
GRANT binds parent_command_id; kind=RELEASE acknowledges a validated exit.
EXIT retains the **deposit command** parent identity and token, although it is
a new EXECUTE command. Resource grant must never replace the active main task.

Observation emits A/B/C individually per view, with DETECTED, NOT_DETECTED,
UNCERTAIN or API_ERROR; invalid depth retains DETECTED but position_valid=false.
Object common positions are not transferable parking points: select a local
approach using Ranger map/costmap and ComputePath before navigating.

## Ranger-specific work

Implement `adapters/ranger.py` against `adapters/base.py` and wire only the Ranger
live branch of AgentNode on that PC. This round supplies no guessed Ranger SDK,
camera topics, footprint, map path or hardware adapter.

Adapter operations: get_health; asynchronous compute_path and select_approach;
start_navigation returning goal_id; cancel_navigation; start_scan; stop_motion.
Use the same Future/result-queue contract as Go2/Fake and AgentRunner. Results
must bind task_id/command_id/goal_id/goal_kind. Health reports local map pose,
fresh localization/path/footprint, stationary confirmation and action terminal.
Scan checks all ABC, serializes Spin and navigation, and keeps inference off
the ROS heartbeat thread. Do not copy Go2 sensor-height/footprint assumptions.

Deploy Ranger agent on Ranger **Domain0**; no coordinator on Ranger. Build/source
Ranger's existing ROS overlay, generated interfaces and the common task package
using a locally verified CPython ABI/runtime. Go2's absolute venv path is not a
Ranger deployment requirement; configure COOPERATIVE_RUNTIME_PYTHON locally.
The common package's vlm_nav dependency presently carries Go2's vision utilities;
keep the existing shared vlm_nav package available, without starting its Go2 nodes.

Both PCs use separate occupancy maps. Fit common_from_ranger from ≥3 separated
non-collinear fixed references plus held-out references; set actual map versions
and the same alignment_id. Rebuilding a map invalidates the previous alignment.
No task-time calibration changes; missing/invalid configuration blocks live start.
Each robot configuration now includes `map_digest`: SHA256 of concatenated YAML
and referenced image SHA256 hex strings. Use `map_file_info` or the offline Go2
check entry to calculate it; verify each PC's own actual files and active map locally.
The Go2 PC does not need a copied Ranger map file. Remote map identity and digest
must still be filled consistently; unknown values remain unset.

## Pending communication setup

Use `config/bridge_ranger_pending.yaml` on Ranger and
`config/bridge_go2_pending.yaml` on Go2. Ranger exports only its state/observation
0→40 and imports Go2 state/observation/command 40→0. Go2 does the complementary
half. Both local domains are 0; robot identity comes from PC/topic source.
Keep Go2 bridge XML Domain0=enp4s0, Domain40=wlo1; supply Ranger's actual local NIC
isolation separately. Never enable bidirectional task routes or bridge Domain31.

Before activation, install/source custom interfaces in both bridge environments.
No connection, SSH, packets, bridge startup or Ranger code changes were performed
from the Go2 implementation session.

## Subsequent field acceptance

Validate Ranger local projection, approach/cancel/scan and executing-path monitor;
verify map alignment in both rooms/door; verify new task message routes/QoS,
Domain0 isolation and no recirculation; verify peer-body marking/clearing and
stopping; test door FIFO and whole-body exit, then the same-C full ordered mission.
These are separate from synthetic tests and prior single-robot acceptance.

## 2026-10-20 本轮最小接入说明（覆盖旧共享 C 要求）

本节是当前实施范围；上文的共同 lab_ref、跨地图标定、同一个 C 绑定只适用于旧共享模式。
本轮 `share_c_coordinates=false`，不共享 C 或 A/B 坐标，不需要任务坐标对齐。
**这里只完成 Go2 和公共任务层、fake Ranger；Ranger live adapter 仍不存在，未连接/部署/验收 Ranger。**
本轮没有自动提交；不得按上文“transfer commits”整体覆盖 Ranger 分支，应逐项合并公共改动并保留其硬件实现。

需同步的公共代码均在 `cooperative_ws/src/multi_agent_nav/multi_agent_nav/`：
`go2_exploration.py`、`generated_candidates.py`（公共逐步核心和本机候选）；
`models.py`、`planner.py`、`coordinator.py`、`agent.py`、`runner.py`、`frame_adapter.py`、`config.py`
（目标任务、私有坐标边界、严格验收、许可链）；`adapters/base.py`/`fake.py`、`mock.py` 及相关测试。
`agent_node.py` 的 Runner/身份/状态接线也需合并到 Ranger 自己的节点。
Go2 的 `adapters/go2.py` 只供参考接口和 ABC 请求，不替代 Ranger adapter。
`multi_agent_task_msgs` 字段、schema v1、action/kind 字符串、五话题和 bridge YAML **没有增加**；
字段指纹相同不证明下面的新命令语义已同步。旧 SYNC 模式不能与 SYNC_LOCAL_C 混跑。

### Ranger 本地执行接口

在 Ranger 控制电脑实现/接入自己的 adapter，交给
`AgentRunner(machine, adapter, descriptions, on_observation, config=config, success_radius=1.0)`。
原单机 `explore` 同步推进公共 `explore_steps`，仍读取0.81 m；协同逐 tick 推进并强制当前接近导航成功。
不要调用 Go2 启动脚本或复制完整探索循环。机器人参数读 `robot_ranger`，候选来源校验 `source_robot=ranger`。

- `get_health`：本机 map 下 base_link 位置、真实停稳、motion_terminal、定位/Nav2健康、当前执行路径及完整 footprint。
- `compute_path`、`select_approach(local_object, excluded, *, standoff_distance, path_accept)`、
  `read_approach_parameters` 返回 Future；后者返回当前 `xy_goal_tolerance` 和 `planner_tolerance`。
  Ranger 直接请求目标坐标，`select_approach` 不再枚举圆周候选；原生容差沿用0.76 m规划＋0.05 m位置到达。
  必须检查**实际路径终点**预算，不能用请求 goal 替代。预算和最后停稳验收均用1.00 m；保留原安全余量。
- `costmap_transform(global_goal=False)`、`costmap_candidate_free(...,global_goal=False)`、
  `observation_candidate_free`、`approach_candidate_free` 使用 Ranger 自己的地图、TF和几何。
  探索候选须有全局完整 footprint/旋转空间/路径；Ranger目标接近只检查实际路径终点的footprint，不要求完整旋转空间。不得仅因 local 窗口不足否决。
  local 已覆盖的碰撞/未知和缺图/陈旧/缺TF仍失败；运动继续本机局部避障/RegionGuard。
- `client.choose_candidate(snapshot)` 与 `vision.submit(callable)`（后台 Future）供本地 VLM 选本轮 P ID；
  `start_scan(descriptions, scan_id, *, stop_on_target)` 返回含 ABC 的 Observation 列表 Future。
  保留最多8次顺时针45°、停稳后新鲜RGB-D、图像时刻TF、ABC同图以及自身目标提前结束扫描。
- `start_navigation` 返回独立 goal_id，`results` 队列结果绑定 task/主command/goal/kind；
  `cancel_navigation`、`stop_motion` 和可选 `tick` 沿同样异步契约，不阻塞心跳。
  遵守 Runner 注入的 `motion_guard`；提供 `simulation`、`client`、`vision`，
  Runner 会设置 `selection_evidence_dir`、`scan_evidence_dir` 保存本机证据。
  原有 `selection_epoch` 若用于取消，需随取消变化并丢弃迟到结果。

Ranger 保留自己的相机、adapter、地图、TF、footprint、Nav2和配置路径。
各端沿用现有 `scene.resources`、`transit_exits`、`exit_points`，**在各自本机地图中**配置；
相同资源 ID 必须现场确认对应同一实物。无需交换多边形，也不将 Go2 配置复制到 Ranger。
本地地图身份/摘要、候选资产、定位、停止和资源门槛仍有效；本轮只取消跨机变换依赖。

### 命令和状态语义

| 既有消息组合 | 本轮含义 |
|---|---|
| SYNC_LOCAL_C | 建立私有坐标模式、task身份、阶段、心跳和ack；common_frame_id/alignment_id为空 |
| EXECUTE APPROACH_PICKUP，Go2=A / Ranger=B | 本机生成候选、VLM选ID、导航扫描并接近；验收后READY |
| EXECUTE SEARCH_VIEWPOINT，target_id=C | 双方READY后，本机继续找有效C；只完成定位，不执行放入 |
| EXECUTE APPROACH_DEPOSIT，target_id=C，observation_id=本机引用 | 复用本机C，接近/放入仍经过资源许可 |
| EXECUTE EXIT | 使用本机退出点；保留C接近命令的parent_command_id/resource_id/token |
| PAUSE / CANCEL / GRANT_RESOURCE | 继续原取消身份、许可、整机退出与RELEASE规则 |

私有 EXECUTE 不接受位置：position_valid=false、common_goal/object_position均为零；
coordinator 不设置 viewpoint_id，不选探索点。禁止收到坐标式pickup后退回旧验收分支。
AgentState 不发共同位置（pose_valid=false，common_pose零），但必须如实回报 localization_valid、nav_ready、
stopped、task/command/sequence/reply、ready/deposited/exited 及资源申请/占用/退出字段。
Observation 所有网络位置均无效且置零；仅本机可用 C 回报 `status=CACHED` 和自己的 observation_id，
原始目标坐标仅保留在本机缓存。task切换清空缓存，旧scan/goal/取消迟到结果不推进状态。
地图身份版本仍非空并固定在任务内；变化暂停。接收计时用本机 monotonic。

Go2、Ranger 完成各自A/B严格验收才READY；双方READY才进入C阶段。
CACHED可提前回报，但不能提前接近C。Go2 C放入并整机退出释放后才放行Ranger。
C许可贯穿接近/放入/EXIT；EXIT不能换token或parent。搜索和A/B接近不申请basket，避免READY屏障死锁；
门通行仍允许申请并在整机退出后释放。导航成功而未停稳、超过1m或身份过期均不得完成。

### 桥接和后续验收

沿用原五条 Reliable/Volatile 定向路由，无新增路由：

| 话题 | Go2 bridge | Ranger bridge | depth |
|---|---|---|---|
| /multi_agent/go2/state | 0→40 | 40→0 | 10 |
| /multi_agent/go2/observation | 0→40 | 40→0 | 50 |
| /multi_agent/ranger/state | 40→0 | 0→40 | 10 |
| /multi_agent/ranger/observation | 40→0 | 0→40 | 50 |
| /multi_agent/go2/command | 0→40 | 40→0 | 10 |

两端本地Domain0、共享Domain40；Go2 Domain0=enp4s0、Domain40=wlo1及原 bridge XML不变，
Ranger按自身真实网卡隔离。不得桥接TF/costmap/图像/action/速度或启用双向回流。
先在 Ranger 本机接入公共接口并完成本地验证，再按新增现场授权测试真实跨机任务、握手失联停车、
资源ID一致、门整机退出及Go2→Ranger顺序；本轮fake通过不替代这些检查。
构建和无节点复现统一见 [runbook](go2_runbook.md#2026-10-20-公共本地执行核心本轮入口差异与离线复现)。

## 2026-10-20 用户简化要求（最新覆盖）

本节覆盖前文私有模式中的家具必填、门/篮子许可与人工退出点要求；任务顺序不变。

- 两端 exploration_strategy=generated_candidates_vlm。家具暂不标注，semantic_yaml 为空或文件不存在时从本机地图空闲区生成满足完整旋转空间的候选，并继续用 VLM 选择本轮 P ID。已存在但非法、身份或地图摘要不匹配的标注文件仍报错。
- Ranger stop_margin 沿用此前真实验收目录中的 0.20 m；模型和 API 继续用环境变量，model/endpoint 不填。
- 私有模式不读门/篮子多边形、不申请 GRANT_RESOURCE、不需要 transit_exits/exit_points，资源字段为空。C 的搜索和接近复用 A/B 的本地核心。
- 双方 READY 后各自寻找/复用本机 C；先 Go2 接近/模拟放入并自动退出，再 Ranger。EXIT parent_command_id 绑定自己的放入命令；本机根据 C、footprint、padding、stop_margin 和实际 Nav2 容差选安全退离点。当前 EXIT 必须 SUCCEEDED、动作终止、真实停稳且完整车身退离，不能因导航成功而省略几何确认。
- camera_calibrated 保留实际现场校验记录，不作为启动门控；不伪造 true。实际 RGB-D、TF、地图/定位、costmap、原生动作与停稳检查继续生效。
- share_c_coordinates=false / SYNC_LOCAL_C、网络坐标清零、五条定向话题、schema v1 和任务身份隔离均保留。两端需要同步本节语义，不与旧私有许可执行器混跑。

## 2026-10-20 探索点选择同步

按 `pppppst/VLM_Nav0` 的 `go2-cooperative-task-layer` 分支提交 `4fc673156c5c985c4d9a9149cfe0b08b1582d90b` 同步候选生成和排序：空地网格为基础，家具边缘点为补充；每轮按当前位置的直线距离、稳定 ID 排序，通过实时 costmap、完整旋转空间和路径检查后取最多 12 点交给 VLM。空地间距下限 1 m，并至少覆盖旋转直径，向上取整到地图分辨率；家具 observation_distance 不控制空地间距。

本次只同步探索点选择及相关证据输出，保留本地 Ranger adapter、真实导航/停稳检查、≤1.00 m 协同验收和上节任务顺序。无家具时两张选点地图相同，不创建标注。源码与当前 acceptance 的 Ranger 地图均离线生成 257 个静态候选；数量不代表实时可达性或完整视觉覆盖。详细变更、测试和生效边界见 [探索点同步记录](multi_agent/ranger_exploration_20261020.md)。

## 2026-10-20 独立 C 完成（最新覆盖）

用户明确取消 C 先后顺序，并确认采用远端无退出行为。本节覆盖前文的“双 C 缓存后才接近”“Go2 先行”和“自动 EXIT”要求，其他配置与探索点算法沿用。

- 保留 A/B 双方 READY 屏障。进入 C 后各端独立推进，有本机有效 C 缓存即可接近；另一方继续搜索，无需双方都有缓存。DEPOSIT 可同时存在两条接近命令，也允许继续 C 搜索。
- 阶段切换不打断原有 C 搜索。只使用本机当前任务、当前机器人及地图身份匹配的 C 定位；网络只发送 CACHED 引用，坐标仍无效且清零。
- 当前接近导航 SUCCEEDED、动作终止、真实停稳、base_link 到 C 的实际平面距离 ≤1.00 m 后标记 deposited。无 EXIT，无 exited 完成依赖，不自动离开 C；先完成的一方停稳等待整体结束。
- 双方 deposited/stopped 且当前任务命令按身份结束、无未完成命令才 DONE。失联、取消、定位或导航异常仍停车/暂停。
- 当前私有模式拒绝 EXIT 和旧 GRANT_RESOURCE。schema v1、五条定向话题和字段定义不变；旧共享坐标模式保留兼容实现。两端应同步此行为，不能用旧的顺序/退出执行器混跑。

最小代码同步涉及 planner、coordinator、models、agent、runner；保留 Ranger 自己的 adapter 和 Nav2 栈，不复制远端 Go2 硬件实现。记录、补丁与无节点复验见 [独立 C 同步记录](multi_agent/ranger_independent_c_20261020.md)。
