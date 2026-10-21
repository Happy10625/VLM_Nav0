当前采用的是：Go2 集中管理任务阶段，两台机器人各自完成本地探索与导航。按 2026-10-20 用户最新确认，私有坐标模式不使用门/篮子区域许可，也不需要退出点；双方完成 A/B 并 READY 后，各自探索和接近 C，不等待对方找到 C，不限制先后，也不执行自动退出。Ranger 已接入自己的适配器和公共任务层。

最新人工启动、暂停与逐步排障命令见 [双机协同启动与验收](双机协同启动与验收.md)。

┌────────────────────────────────────┐
│ Go2：coordinator + RulePlanner     │
│                                    ├◄─命令、状态───────────────────┐
│                                    ├◄─定向桥接：命令、状态─────────╪─┐
└────────────────────────────────────┘                               │ │
                                                                     │ │
                                                                     │ │
┌────────────────────────────────────┐                               │ │
│ Go2：AgentMachine + AgentRunner    │                               │ │
│                                    ├◄──────────────────────────────┘ │
│                                    ├─────────────────────────────────╪─┐
└────────────────────────────────────┘                                 │ │
                                                                       │ │
                                                                       │ │
┌────────────────────────────────────┐                                 │ │
│ Ranger：AgentMachine + AgentRunner │                                 │ │
│                                    ├◄────────────────────────────────┘ │
│                                    ├───────────────────────────────────╪─┐
└────────────────────────────────────┘                                   │ │
                                                                         │ │
                                                                         │ │
┌────────────────────────────────────┐                                   │ │
│ 公共 explore_steps 核心            │                                   │ │
│                                    ├◄──────────────────────────────────┘ │
│                                    ├─────────────────────────────────────╪─┐
└────────────────────────────────────┘                                     │ │
                                                                           │ │
                                                                           │ │
┌────────────────────────────────────┐                                     │ │
│ 同一公共 explore_steps 核心        │                                     │ │
│                                    ├◄────────────────────────────────────┘ │
│                                    ├───────────────────────────────────────╪─┐
└────────────────────────────────────┘                                       │ │
                                                                             │ │
                                                                             │ │
┌────────────────────────────────────┐                                       │ │
│ Go2 adapter → 本机感知 / Nav2      │                                       │ │
│                                    ├◄──────────────────────────────────────┘ │
└────────────────────────────────────┘                                         │
                                                                               │
                                                                               │
┌────────────────────────────────────┐                                         │
│ Ranger adapter → 本机感知 / Nav2   │                                         │
│                                    ├◄────────────────────────────────────────┘
└────────────────────────────────────┘

各模块的职责如下：

| 模块 | 部署位置 | 职责 |
|---|---|---|
| RulePlanner | Go2 | 分配 Go2 找 A、Ranger 找 B；各端根据自己的 C 引用独立搜索或接近 |
| coordinator | Go2 | 管理阶段、READY 屏障、命令重发、心跳、超时和双方完成条件 |
| AgentMachine | 两端各一份 | 检查任务身份、处理状态转换、导航结果及本机 C 观测身份 |
| AgentRunner | 两端各一份 | 逐 tick 推进执行核心，连接状态机与 adapter；等待 VLM 时继续处理通信 |
| explore_steps | 两端复用 | 本地生成候选、VLM 选 ID、导航、扫描、定位、接近和距离验收 |
| 机器人 adapter | 各自实现 | 对接自身相机、TF、costmap、Nav2、停止确认和异步结果 |

RulePlanner 和 coordinator 都不选择本地探索点。虽然公共核心文件仍叫 go2_exploration.py，现在已按 robot_id 使用对应机器人的配置。

整个任务按以下顺序推进：

| 阶段 | 行为与完成条件 |
|---|---|
| SEARCH_AB | Go2 本地找 A，Ranger 本地找 B；各自接近验收成功后 READY，先完成的一方停稳等待 |
| SEARCH_C | 双方 READY 后，各自检查本机 C 缓存；任一方有有效引用即可进入 DEPOSIT |
| DEPOSIT | 有本机 C 引用的一方独立接近并验收；另一方可继续搜索，找到后接近；允许同时执行 |
| DONE | 双方均完成 C 接近验收并停稳，当前任务命令按身份结束且无未完成命令 |

场内虽然只有一个 C，软件仍只使用各自识别的位置，不共享 C 坐标，也不做共同 C 绑定或跨地图转换。

机器人内部的执行流程为：

本机地图＋可选家具标注（当前不标注，从空闲区生成候选）
→ 在已知自由区域生成网格点，并叠加可选家具边缘观察点
→ 每轮按距当前位置的直线距离、稳定 ID 排序
→ 检查全局 costmap、完整 footprint、旋转空间和规划路径
→ 将最近优先筛出的最多 12 个可达点绘图，以本轮 P 编号交给本地 VLM
→ 导航到选中点并停稳
→ 最多 8 次顺时针 45°，每次停稳后使用新鲜 RGB-D 同图识别 ABC
→ 发现自身目标后提前结束扫描
→ 本机定位目标、规划接近、导航并验收

空地网格间距为 `ceil(max(1.0 m, 2 × 完整旋转半径) / 地图分辨率) × 地图分辨率`，从半步网格采样；完整旋转半径为 footprint 外接圆半径加 padding。observation_distance 仅控制家具边缘观察点的偏移，不影响空地网格。家具候选与空地点合并，空地点避开家具占据区，并与家具候选保持至少半个网格间距。未配置、缺失或合法空家具标注均可探索；已存在但非法、身份或地图摘要不匹配的标注仍拒绝。资产出现、删除或变化使当前选择失效。选点阶段向模型提供地图、目标、提示和简要历史；实拍 RGB-D 用于到点后的识别。该方法不计算 frontier、信息增益或相机可见覆盖率。同步依据及验证见 [探索点同步记录](multi_agent/ranger_exploration_20261020.md)。

远端候选不会仅因超出局部 costmap 而被否决；运动期间仍使用本机局部避障和执行路径监护。协同接近必须同时满足：当前任务身份有效、当前目标接近导航 SUCCEEDED、真实停稳、动作终止，以及 base_link 到已定位目标的平面距离 ≤1.00 m。原单机入口保持 0.81 m。

Ranger 使用本机 adapter 接入公共 Runner：

AgentRunner(
    machine,
    ranger_adapter,
    descriptions,
    on_observation,
    config=config,
    success_radius=1.0,
)

接口应遵守 RobotAdapter (cooperative_ws/src/multi_agent_nav/multi_agent_nav/adapters/base.py) 的契约：

| 接口组 | Ranger 需要提供的内容 |
|---|---|
| 健康状态 | get_health()：本机位置、定位/Nav2 健康、真实停稳、动作是否终止、执行路径和完整 footprint |
| 路径与接近 | compute_path() → Future[PathResult]；select_approach() → Future[ApproachResult] |
| 距离参数 | read_approach_parameters()：读取实际 XY 和 planner 容差，不修改它们 |
| 地图检查 | costmap_transform、costmap_candidate_free、observation_candidate_free、approach_candidate_free |
| 运动 | start_navigation() 返回独立 goal ID；结果进入 results 队列，并携带 task/command/goal/kind 身份 |
| 扫描与 VLM | start_scan() 返回观测列表 Future；提供 client.choose_candidate() 和后台 vision.submit() |
| 取消与停车 | cancel_navigation()、stop_motion()；遵守 Runner 注入的 motion_guard |

这里有一个容易接错的边界：adapter 内部的有效目标观测需要保留本机坐标。现有观测字段仍叫 common_position，但私有模式下内部承载本机位置；由 AgentMachine 在发送网络消息前统一清零，不能在 adapter 产生观测时就清掉位置。

通信继续使用既有五条任务话题：

| 话题 | 跨机方向 |
|---|---|
| /multi_agent/go2/state | Go2 → Ranger |
| /multi_agent/go2/observation | Go2 → Ranger |
| /multi_agent/ranger/state | Ranger → Go2 |
| /multi_agent/ranger/observation | Ranger → Go2 |
| /multi_agent/go2/command | Go2 → Ranger，按 recipient_id 分发 |

两端本地均为 Domain0，经各自定向 bridge 接共享 Domain40；任务话题采用 Reliable/Volatile。TF、图像、costmap、速度和 Nav2 action 留在各自本机。

本轮使用 share_c_coordinates=false、SYNC_LOCAL_C。网络命令只表达“找 B”“找 C”“接近本机 C”等意图；位置字段无效且置零。仍检查任务、命令序号、机器人身份、地图版本和 simulation 标志，不能只看消息字段指纹判断两端兼容。

私有模式不申请门或篮子许可。C 搜索、定位和接近与 A/B 复用同一个本地执行核心。SEARCH_C 切换到 DEPOSIT 不取消另一方正在执行的 C 搜索；没有本机 C 引用的一方也可在 DEPOSIT 中继续搜索。

C 接近必须完成当前导航成功、动作终止、真实停稳和 ≤1.00 m 实际距离验收；随后标记 deposited，保持停稳，不执行 EXIT，也不设置 exited。deposited 表示接近验收/模拟放入，不代表机械臂实物投放。先完成的一方留在接近位置；两机互相避障继续依赖各自 Nav2，任务层不再安排 C 互斥或退让。实现和验证见 [独立 C 同步记录](multi_agent/ranger_independent_c_20261020.md)。

两端探索策略为 generated_candidates_vlm，家具标注可暂为空；已有有效标注时保留家具边缘候选。Ranger stop_margin 沿用原协同验收配置的 0.20 m。模型/API 继续从环境变量读取。camera_calibrated 仅记录现场逐步校验状态，不作为启动门槛；实际 RGB-D、图像时刻 TF、定位和停稳检查照常执行。

Ranger 搭建时建议按依赖顺序完成：

1. 合并公共任务代码和消息语义，保留 Ranger 自己的底层、单机栈和参数。
2. 实现 Ranger adapter，并接入 AgentNode 的 Ranger live 分支；替换 Go2 专属 observer 和运动占用检查。
3. 核对本机地图身份/摘要、footprint、costmap；暂不需要家具标注或资源多边形/退出点。以后提供家具标注时，source_robot 必须与本机一致。
4. 先验证本地接口与 fake 流程，再核对定向桥接，最后进行真实双机验收。当前模式无需数值地图对齐。

完整同步清单和字段约定见 Ranger 最小接入说明 (`docs/ranger_handoff.md` 末尾最新覆盖节)。公共包仍依赖 vlm_nav 的公共 Python 工具，应按需合并兼容代码，避免整体覆盖 Ranger 分支。

Ranger adapter 已完成代码接入；已进行离线/fake 验证。未进行本轮 ROS 通信、模型 API 或真机运动验收。两端需同步相同的简化任务语义，旧许可模式不可混跑。
