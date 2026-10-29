# Ranger 指定提交行为迁移记录

唯一行为来源：`251f9fb39c25849dac00e81aec97fa5c45d98c1d`（10.6）。
执行方案：`/home/isee601/Downloads/2026-10-25-ranger-single-robot-parity-plan.md`。
修改前 HEAD：`ab922276108c21f44612b7805e26fc0560300528`，origin 为
`git@github.com:Happy10625/VLM_Nav0.git`，无未提交修改。
实现先在工具创建的 detached worktree 完成。随后按用户明确要求，于 2026-10-25
整合到 `/home/isee601/ws/VLM_Nav` 的 `ranger-reuse-go2-framework` 分支并完成构建。
整合前现场工作树干净且 HEAD 与上述修改前提交一致；没有切换、重置、提交或推送分支。

## Task 1：旧方法与执行链路

独立：`multi_agent_ranger_check → go2_check.run_ros → explore → explore_steps`。
协作：`multi_agent_agent_node → AgentRunner._start_local → explore_steps`。
修改前两者均经 `RangerAdapter(Nav2Adapter)` 执行动作；`make_observer` 创建的
`VLMNavigator(observation_only=True)` 只启动 RGB-D/TF 处理，不启动旧导航定时器。
两个 CMake 包均使用 `ament_python_install_package` 安装整个 Python 包。

|旧来源（均为固定 SHA）|当前实际位置|迁移位置|保留例外|验证|
|---|---|---|---|---|
|geometry.py TargetTracker；navigator.confirm_target、handle_worker_result|Nav2Adapter.tick 单次 DETECTED|共享本机执行器，Ranger 扫描回调|锁定确认坐标；缓存 C 不增加重观察|三帧、空间重置、重复帧、置信度与匹配|
|geometry.depth_at_pixel_with_reason|geometry 同名函数，六样本平均|原函数|保留原始深度 SDK 接口|旧函数同矩阵对照|
|recover_target_depth、target_relative_bearing、retry_depth_reobservation、build_target_probe_candidates、start_target_probe、clear_target_depth_recovery|Nav2Adapter.tick 简化三次重拍|共享执行器及 Nav2 回调|当前 costmap/keepout 执行|旋转方向、次数、探测点、旧结果|
|navigation_tick 接近分支；plan_then_navigate/request_next_plan/on_plan_response/on_plan_result|explore_steps.approach_target/approach_attempt；RangerAdapter.select_approach|共享执行器|当前探索筛选与协作路径监护|目标 XY/yaw、非空路径、取消、冷却|
|target_reference_distance、approach_stop_radius、reset/request/begin/advance/finish_approach_*；on_navigation_result|explore_steps.navigate/stopped；Runner._advance_local|共享执行器|外部取消、任务身份独立|0.89/0.81、一次重试、提前成功、终态、过期 odom|
|on_arrival_odom|observer 同名方法尚保留；adapter 另有三轴/持续时间判定|接近专用验收|命令切换仍用当前 adapter 停稳条件|终态前样本有效、vx/vy/wz、接收单调时钟|
|enter_sensor_wait/recover_from_sensor_wait/safety_tick/resolve_pending_rgbd|健康失败直接中止共享执行|共享执行器与 Ranger 健康路由|只恢复 RGB-D/图像 TF；外部停止优先|三帧恢复、30 秒、重新扫描、任务隔离|

旧方法 AST 初查：确认、深度恢复、规划、导航结果、sensor wait 与 odom 方法
在当前 navigator 中仍与旧 SHA 相同，但未接入当前实际动作链。
共享执行器直接采用旧 `target_reference_distance`、`approach_stop_radius`、
`advance_approach_stop`，没有依赖当前 arrival helper 的替代判断。
所有方案列明数值与旧 robot.yaml 一致；当前 robot.yaml 数值也相同。
planner tolerance=0.76、xy_goal_tolerance=0.05 已相同，不改配置。

## 实施结果与源码依据

新增 `vlm_nav/ranger_single_robot_execution.py` 是原观察器的子类。它仍以
`observation_only=True` 构造，不启动旧自主导航定时器。两入口创建同一种观察器，
`explore_steps` 为每个 task/command 启动其本机生命周期，主目标和 C 使用同一个实例。
`Nav2Adapter._motion` 为本机动作保留 task/command/goal/native UUID，并将终态仅交给
本机回调，不再同时放进 Runner 结果队列。

下列行号均指固定旧提交的 `vlm_nav/vlm_navigator.py`：

|范围|旧方法及行号|实现方式与验证|
|---|---|---|
|确认|handle_worker_result:1402；confirm_target:1694；geometry.TargetTracker|复制旧结果判断；继承原三帧追踪器，冻结首次确认坐标。真实适配器扫描用例验证重复帧不计数、五类匹配/置信度拒绝、20 秒确认超时。|
|深度恢复|retry_depth_reobservation:1807；target_relative_bearing:1827；build_target_probe_candidates:1843；clear_target_depth_recovery:1875；start_target_probe:1881；recover_target_depth:1925|继承与旧 SHA AST 相同的方法。像素方向/候选点/小角度旋转对同输入直接调用旧方法比对；实际回调验证稀疏深度旋转、超量程路径截断与重新取帧。|
|规划|navigation_tick 接近分支:2138；plan_then_navigate:2378；request_next_plan:2426；on_plan_response:2453；on_plan_result:2468|只移入接近调度分支；其余规划方法直接继承旧实现。目标参考 XY 与规划时保存的 yaw 沿用至 NavigateToPose；非空成功路径不经当前终点预算/253车体/旋转空间否决。|
|停车|target_reference_distance:2243；approach_stop_radius:2251；reset_approach_stop:2256；request_approach_cancel:2266；on_approach_cancel_response:2276；begin_approach_stop:2285；advance_approach_stop:2316；finish_approach_success:2364|0.89 触发，0.81 验收，一次重试；实际动作回调覆盖终态前样本、提前 SUCCEEDED、ABORTED、取消未终态、已有近目标。距离/重试/陈旧 odom 事件与固定旧方法对照。|
|里程计与终态|on_arrival_odom:1079；on_navigation_result:2779|原方法保持；`hypot(vx,vy)`、`abs(wz)`、接收单调时钟、连续三样本不变。`observer_parameters` 恢复原 arrival 话题，命令切换仍使用适配器 `/odom` 条件。|
|传感器|enter_sensor_wait:910；recover_from_sensor_wait:926；safety_tick:959；resolve_pending_rgbd 中恢复计数|继承取消/轮次隔离/清目标/重扫方法，接入旧 RGB-D/图像时刻 TF 超时与三帧恢复。定位失效和协作失联仍外部停止。恢复扫描无目标后回到现有候选探索。|

上述 20 个继承方法与旧 SHA 的 AST 逐一一致；复制的四个方法
`handle_worker_result`、`target_reference_distance`、`approach_stop_radius`、
`advance_approach_stop` 也一致。测试不以这些字符串比较代替行为断言：
`test_ranger_single_robot_parity.py` 直接从旧 SHA 加载方法，对相同输入/事件比对。

### 源码冲突与授权边界

- 原 `observer_parameters` 用 `robot_ranger.yaml:odom_topic=/odom` 覆盖了
  旧 arrival 的 `/fastlio/odom`。取消该覆盖，只合并原 arrival 配置；不改适配器的命令切换里程计。
- 旧 `navigation_tick` 保存规划请求时的机器人 yaw，`on_plan_result` 发送同一
  `requested_pose`；不会在规划完成后重新读取 yaw。方案“发送目标时当前朝向”按这两处
  源码的实际顺序实施，没有增加规划后重定向。
- 原共享扫描整体超时不属于旧本机算法。启用本机执行器时不再叠加该期限，使用旧确认、
  目标动作、传感器超时；Go2 和未启用本机执行器的只读/工具扫描保持原逻辑。
- 保留明确例外：首次仅当前方向；正常接近锁定坐标且不运行持续 VLM/视觉丢失超时；
  缓存 C 原输入不增加再确认；候选点/地图/keepout/动态层/控制器/行为树/协作协议及心跳不改。
- 旧 `safety_tick` 中 frontier、easy-case、视觉丢失和活动半径预算未引入当前执行链；
  本次只接入所列本机调度，继续使用当前执行路径监护，不增加距离预算或监护阈值。
- 普通命令切换仍用适配器停稳条件。真实 Ranger 的 READY/deposited 和独立返回只消费
  本机已验收结果，不再叠加 1.0 米或持续 0.5 秒的成功判定。
  FakeAdapter 任务模拟仍只用于协议回归，真实本机验收另由实际 Ranger 适配器的模拟事件测试覆盖。

### 文件差异

- `vlm_nav/geometry.py`：恢复旧深度函数；`config/robot.yaml` 仅修正失实注释，数值不变。
- `vlm_nav/ranger_single_robot_execution.py`：共享旧本机流程及必要动作/任务包装。
- `adapters/ranger.py`：真实观察器入口、旧参数加载、确认/恢复调度、终点否决移除。
- `adapters/nav2.py`：可选单一终态消费者、Ranger 视觉结果回调；Go2 默认流程不变。
- `go2_exploration.py`：两入口共同启动本机生命周期；恢复后重新探索；按当前 goal 隔离路径接收状态。
- `runner.py`、`agent_node.py`、`go2_check.py`：消费本机结果、移除真实 Ranger 的外层验收覆盖。
- 两组新增测试、geometry/adapter 既有测试更新；删除与指定旧源码冲突的“三次原地重拍且禁止旋转”测试，
  替换为真实适配器的旋转/探测/取消/健康/超时用例；CMake 注册新增测试。

## 验证记录

- 未 source ROS 的试运行：35 passed、13 failed，均为 ROS 消息模块缺失等环境问题。
- source `/opt/ros/humble/setup.bash` 后基线：geometry、navigation_gate、robot_adapters、
  ranger_initial_observation：102 passed、3 deselected；排除实际创建观察节点的测试。
- 不运行硬件、实际 VLM 或现场导航。未提交、推送、部署。

新增回归的失败证据保留在 `/tmp/ranger-parity-*-red.log`：深度混合拒绝、传输终态消费者、
arrival 话题覆盖、恢复前取消未终态、额外扫描期限、重试路径初始化、深度探测路径丢失。
后两项修复只按当前 goal 保留原路径监护状态，不改变现有超时或阈值。

只读整体审查发现一项 Important：传感器三帧恢复可能早于取消终态。已用
`test_sensor_frames_can_recover_before_canceled_navigation_is_terminal` 复现失败，再修复为
保留恢复意图、等待原动作终态后创建扫描。未引入第二动作拥有者。
审查未覆盖现场构建/硬件，未遗留 Minor 建议。

离线广回归命令（两次均 source ROS 与现有消息安装，Python 优先当前源码）：

```bash
source /opt/ros/humble/setup.bash
source /home/isee601/ws/VLM_Nav/build/ranger-local/install/setup.bash
ROS_LOG_DIR=/tmp/ranger-parity-ros-log \
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH="$PWD/cooperative_ws/src/multi_agent_nav:$PWD:$PYTHONPATH" \
python3 -m pytest -q test cooperative_ws/src/multi_agent_nav/tests \
  --ignore=test/test_obstacle_cloud_filter_runtime.py \
  --ignore=cooperative_ws/src/multi_agent_nav/tests/test_ros_smoke.py \
  -k 'not native_costmap_marking and not observation_only_node and not readonly_observer_has and not aligned_depth_observer and not observer_keeps_own_name' \
  --tb=short
```

修改前 HEAD 用 `git archive HEAD` 导出到 `/tmp/ranger-parity-head` 后以相同环境运行。
该副本仅作当前版本的回归基线，行为来源始终只有指定旧 SHA。

最终结果：

|集合|结果|证据|
|---|---|---|
|geometry + navigation_gate + 新本机/适配器合同|137 passed|`/tmp/ranger-parity-contract-final.log`|
|private_c + independent_c + 初始观察 + 本地探索|84 passed|`/tmp/ranger-parity-c.log`|
|修改前 HEAD 同环境广回归|838 passed、18 failed、13 skipped、7 deselected|`/tmp/ranger-parity-head-final.log`|
|修改后同环境广回归|874 passed、18 failed、13 skipped、7 deselected|`/tmp/ranger-parity-final.log`、`/tmp/ranger-parity-final.xml`|
|差异/语法|`git diff --check` 与修改 Python 模块 compileall 通过|本次执行输出|

失败名称集合程序比较相等，新增失败集合为空。不能把广回归退出码 1 报成全绿。
18 项由隔离树无 build overlay、缺 Go2 文件/现场地图、已有配置断言及日志捕获断言组成，
均在修改前副本复现；没有为使其通过修改场地图、配置或监护规则。完整名称见下方。

代码差异交付为 `/tmp/ranger-single-robot-parity-20261025.patch`，包含新增文件；
使用 `git apply --reverse --check` 对当前工作树验证补丁可反向匹配，未实际应用/回滚。

## 后续用户授权：RViz 候选点与目标标记（2026-10-25）

- 蓝色球点/P 编号：`/vlm_nav/ranger_exploration/vlm_candidates`。只发布共享执行链的
  `CANDIDATE_REQUEST.candidates`，对应本次发送给 VLM 的筛选后候选集合，不把全部生成点混入显示。
- 绿色球点/A、B、C 标签：`/vlm_nav/ranger_exploration/targets`。只显示 DETECTED 且坐标有效的
  本机观察结果，在原有结果检查之后发布，并将 common_position 转回本机 map 平面 XY。
  缓存 C 进入接近流程时同样显示；未定位/超时结果不生成目标位置。
- 独立与协作入口均由 `create_runtime` 连接 `RangerVisualization`，只读检查不创建显示发布器。
  两个独立 MarkerArray 话题使用 Reliable/Transient Local、深度 1，完整消息包含清理旧标记和当前集合。
- 每个新命令清理旧候选集合；同一任务保留最近识别目标，新任务清空目标。
- 修改 `config/vlm_nav.rviz`，并更新启动脚本实际使用的
  `/home/isee601/松灵小车2/固定地图轻量验证.rviz`，增加上述两个启用的显示项。
  外部 RViz 原配置备份 `/tmp/ranger-rviz-before.rviz`，其他显示设置保持原样。
- 没有修改候选点筛选、VLM 提示词、目标确认或导航规则；原 8 秒/两次尝试设置保持。
- 针对性测试 83 passed（`/tmp/ranger-rviz-targeted.log`）：候选集合一致性、蓝/绿颜色、坐标转换、
  非法目标过滤、任务清理、缓存行为与两入口接线。
- 现场 `ranger-local` 构建 3 包成功（`/tmp/ranger-rviz-build.log`），现场相关回归 297 passed
  （`/tmp/ranger-rviz-field-tests.log`）；安装模块、仓库文件与实际 RViz 配置已逐项检查一致。
- 完整回归 906 passed、18 failed、1 skipped、7 deselected；18 项失败与改动前完全一致，
  没有新增失败（`/tmp/ranger-rviz-regression-final.log`）。旧工厂测试的模拟节点补齐发布器接口。
- 本轮增量补丁 `/tmp/ranger-rviz-markers.patch`；现场源码备份 `/tmp/ranger-rviz-before.tar.gz`。
  尚未验证真实 RViz 渲染、DDS 发布接收与实机导航，未自动重启运行进程。
- 正在运行的任务进程需正常退出再重启以发布新话题；RViz 需重新载入该配置。
  本轮不自动重启任务或启动运动，也不调用真实 VLM。

## 最新用户授权：VLM 返回门限 8 秒（2026-10-25）

**当前生效口径：每次 8 秒，每视角总共最多 2 次尝试。** 下节 4 秒记录是上一轮历史行为。
本次用户明确要求放宽返回门限；不更改定位、心跳、外部取消及其他安全阈值。

- `RangerSingleRobotExecution.vlm_request_timeout = 8.0` 为当前 Ranger 链路的统一值。
- `RangerAdapter` 将该值传给每次 HTTP 请求，并用同一值检查异步请求超时和图像结果年龄。
- 本机 `handle_worker_result` 同步采用 8 秒结果有效期，避免已经返回的 4～8 秒结果被旧检查拒绝。
- 每次重试仍在同一视角重新取帧；两次均失败才跳过视角，最多约 16 秒请求等待，取帧与旋转时间另计。
- 起点只看当前方向、三个不同新帧确认、取消旧 HTTP/隔离迟到结果等行为保持。
- 新增测试在修改前失败，修改后约 7.5 秒的三个独立结果均能完成确认；7.99 秒保持等待，8.01 秒超时重试。
- 八方向全部失败仍为 16 次尝试；静止、无执行路径超过 17 秒不会仅因此 PAUSED。
- 针对性验证：80 passed（`/tmp/ranger-vlm-eight-seconds-targeted.log`）。
- 补齐上轮 `test_api_response_probe.py` 的 ament 登记；原登记检查失败已修复，该检查和脚本两项测试共 3 passed。
- 现场三个包重建成功，安装模块的 `vlm_request_timeout` 核对为 8.0。
  构建日志 `/tmp/ranger-vlm-eight-seconds-build.log`；仅有原 CMake CMP0074/PCL_ROOT 开发警告。
- 构建后现场针对性验证 286 passed，日志 `/tmp/ranger-vlm-eight-seconds-field-tests.log`。
- 最终隔离树广回归 900 passed、18 failed、1 skipped、7 deselected；失败名称与原基线完全相同。
  日志 `/tmp/ranger-vlm-eight-seconds-regression-final.log`，完整失败清单见文末。
- 增量补丁 `/tmp/ranger-vlm-eight-seconds.patch`；整合前备份 `/tmp/ranger-vlm-eight-seconds-before.tar.gz`。
- 本轮没有额外调用真实 VLM 或启动运动；正在运行的旧任务进程需正常退出后重启以加载门限。


## 后续用户授权：VLM 单视角两次尝试（2026-10-25）

本节是用户在现场排查后明确追加的行为变更，覆盖前文对应的旧版请求失败处理，
不是从旧 SHA 推导的新规则。最终口径为**总共两次尝试（首次 + 一次重试）**。

|来源|当前行为|对应位置|
|---|---|---|
|用户要求请求超过 4 秒不继续等待|每次最多等待 4 秒，由 ROS tick 触发取消；下一次请求不等待旧 HTTP 返回|RangerAdapter.tick、CancellableCompletion|
|用户明确“两次尝试拍摄同一视角”|保持视角，重取新帧并沿用相同目标/提示词；每次证据单独保存|Nav2Adapter.tick、RangerAdapter.consume_local_observation|
|用户要求全部失败才跳过视角|超时、API/响应校验失败计入两次预算；耗尽后记 TIMEOUT/API_ERROR，转下一角度|RangerAdapter.consume_local_observation|
|用户要求等待时不因无移动/路径不动 PAUSED|保持 LOCAL_EXECUTING；原路径监护仅在 NAVIGATING 生效，无需放宽监护阈值|AgentRunner._record_local、AgentMachine._monitor_region、explore_steps.health_check|
|原方案保留项|起点仍只观察当前方向；目标确认仍需三个不同新帧；定位、心跳、外部取消、传感器安全检查保留|原执行链及回归测试|

异步 HTTP 使用现有 SDK 的 AsyncOpenAI，在工作线程的私有事件循环中运行；
超时/取消会取消其 HTTP await，并释放后台线程。每次请求独立持有客户端、图像和证据，
迟到结果不能替换当前结果。工作线程池为两个槽位，Go2 原单任务执行语义保持不变。
局部执行器已经 FAILED/API_ERROR 时，外层不再吞掉该终态后继续切换候选点。

正常扫描的一个方向最多约 8 秒请求等待，另加取帧和调度时间；旋转/停稳时间另计。
检测成功的三帧确认仍是三个不同图像序列，不能把同一结果累计三次。
八方向全部超时的离线测试确认：同一候选点共 16 次尝试，输出八个 TIMEOUT 视角记录。
日志会写出 `VLM view N attempt 1/2 failed ... retrying same view` 或
`attempt 2/2 failed ... skipping this view`。

验证：

- 修改前新增超时/错误/迟到/后台容量用例失败，修改后针对性集合 110 passed。
- 真实线程池配合假的异步 VLM，验证连续取消后下一视角请求确实开始，覆盖线程被旧请求占满的问题。
- 持续静止、无执行路径 13 秒时，协作状态仍为 LOCAL_EXECUTING，没有 PAUSED。
- 针对性日志：`/tmp/ranger-vlm-retry-targeted-final.log`。
- 最终广回归：897 passed、18 failed、1 skipped、7 deselected；失败名称集合与前述基线完全相同。
  日志 `/tmp/ranger-vlm-retry-regression-final.log`，失败完整名称仍见文末清单。
- 现场已同步本轮 8 个文件，三包重建成功（退出码 0），沿用 `build/ranger-local`。
  构建日志 `/tmp/ranger-vlm-two-attempts-build.log`。
- 现场构建后行为/入口/协作回归：282 passed；日志 `/tmp/ranger-vlm-two-attempts-field-tests.log`。
- 从 `/tmp` 导入安装模块，vision/nav2/ranger/go2_exploration 全部与现场源码一致；
  独立脚本 `--offline` 成功，configuration_only、motion_sent=false。
- 增量补丁 `/tmp/ranger-vlm-two-attempts.patch`；整合前备份
  `/tmp/ranger-vlm-two-attempts-before.tar.gz`，文件哈希清单同名 `.json`。
- 独立审查确认无剩余阻塞项；真实 HTTP 取消由假异步服务验证，未调用实际服务。
  没有重启正在运行的任务，旧进程需正常退出后由既有入口重新启动。
- asyncio 跨线程唤醒需要本机套接字权限；沙箱禁用该操作时测试无法唤醒循环。
  已在允许本机套接字的环境通过测试，VLM 为替身，未请求网络服务。

## 现场整合与构建（2026-10-25）

用户明确要求整合到现场仓库后，补丁检查通过并应用了全部 16 个文件。
整合前受影响文件已备份到 `/tmp/ranger-parity-field-before-20261025.tar.gz`，
对应 SHA-256/新增文件清单在 `/tmp/ranger-parity-field-before-20261025.json`。
未改动现场地图、验收配置目录或启动脚本；脚本已有的 overlay 路径就是本次构建目录。

在 `/home/isee601/ws/VLM_Nav` 执行：

```bash
source RangerVLM_env.bash --build
colcon --log-base "$PWD/build/ranger-local/log" build \
  --base-paths "$PWD" "$PWD/cooperative_ws/src" \
  --build-base "$PWD/build/ranger-local/build" \
  --install-base "$PWD/build/ranger-local/install" \
  --packages-up-to multi_agent_nav --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF \
  -DCOOPERATIVE_RUNTIME_PYTHON=/usr/bin/python3
```

结果：`multi_agent_task_msgs`、`vlm_nav`、`multi_agent_nav` 三包构建成功，退出码 0。
只有 vlm_nav 的 CMake CMP0074/PCL_ROOT 开发警告；未运行完整 colcon test。
构建日志：`/tmp/ranger-parity-field-build-20261025.log`。

安装核对在 `/tmp` 工作目录完成，使用现场环境、没有添加源码 PYTHONPATH：

- `ros2 pkg prefix vlm_nav` 和 `multi_agent_nav` 均指向现场 `build/ranger-local/install`。
- 九个相关安装模块（geometry、新执行器、nav2/ranger adapters、agent_node、go2_check、
  go2_exploration、runner、ranger_check）的内容哈希均与现场源码相同。
- 模拟构造器、不初始化 ROS 节点时，安装后的 `make_observer` 确实创建
  `RangerSingleRobotExecution`；两入口继续共用 `explore_steps`。
- `ros2 pkg executables multi_agent_nav` 包含 `multi_agent_ranger_check` 与 `multi_agent_agent_node`。

|现场构建后验证|结果|日志|
|---|---|---|
|geometry + navigation_gate + 新本机/适配器合同|137 passed|`/tmp/ranger-parity-field-contract-20261025.log`|
|Ranger 入口 + private_c + independent_c + 初始观察 + 本地探索|102 passed|`/tmp/ranger-parity-field-entry-tests-20261025.log`|
|`start_ranger_local.sh --offline --config-dir "$RANGER_CONFIG"`|退出码 0，configuration_only，motion_sent=false|`/tmp/ranger-parity-field-offline-20261025.log`|
|`cooperative.sh ranger preflight`|退出码 0，12 个公共安装模块匹配，live_gates=[]，motion_sent=false|`/tmp/ranger-parity-field-preflight-20261025.log`|

现场配置仍由 `current-acceptance-run.txt` 指向
`build/ranger-local/acceptance-20261015_161341/config`。离线检查使用其实际地图
`shared_maps/20261018161152/lab.yaml`，未替换地图或认证标记。
以上是构建后针对性验证，不是现场全量回归；前述 18 项失败记录属于隔离树测试环境。
其中缺少 overlay 的 shell 入口用例在现场本轮验证已通过。

## 启动流程与入口

独立运行：先通过现有 `scripts/start_ranger_stack.sh` 启动硬件/定位/Nav2；准备完成后，
在新的任务终端运行：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
bash "$REPO/scripts/start_ranger_local.sh" --config-dir "$RANGER_CONFIG" \
  --action explore --allow-motion --target-id B --approach-target --timeout 600
```

实际链路：`start_ranger_local.sh → multi_agent_ranger_check → go2_check → explore_steps
→ RangerAdapter / RangerSingleRobotExecution`。`--action approach` 仅做接近规划，
实际自主探索并接近使用上面的 `explore --approach-target`。

完整协作启动入口仍为：

```bash
bash /home/isee601/ws/VLM_Nav/scripts/start_ranger_cooperative.sh
```

它经过 preflight、硬件/定位确认、健康检查后启动 bridge 与 agent。
实际链路：`cooperative.sh ranger agent → multi_agent_agent_node → AgentRunner
→ explore_steps → RangerAdapter / RangerSingleRobotExecution`。
Ranger 入口只准备并等待对端；任务仍由 Go2 侧原入口发起。
已有运行中的 Python 节点不会因构建自动重载；需先从原入口取消任务并确认停稳，
正常退出旧任务进程后再使用上述入口。此次没有启动、停止或重启现场任务。

## 未验证事项

- 未运行实际 ROS 节点/传感器/导航、原生 costmap 运行测试、硬件停车或真实 VLM。
  Python 离线事件测试不证明实际 DDS 回调时序、制动距离或现场障碍行为通过。
- 后续人工验收：三帧确认、深度旋转/探测、0.89/0.81 停稳与一次重试、短传感器中断、
  取消/断联和 C 双 READY。本次没有自动执行这些动作。
- 未进行 Go2 远端部署；未提交或推送 Git。本轮修改在现场分支中保持未提交状态。

## 同环境基线中已有的 18 项失败

```text
test/test_ranger_check.py::test_shell_entry_does_not_pass_check_options_to_environment_loader
cooperative_ws/src/multi_agent_nav/tests/test_dynamic_costmap_config.py::test_effective_cooperative_costmaps_keep_dynamic_layers_and_other_parameters
cooperative_ws/src/multi_agent_nav/tests/test_dynamic_costmap_config.py::test_actual_navigation_launch_applies_optional_overlay_last
cooperative_ws/src/multi_agent_nav/tests/test_dynamic_costmap_config.py::test_isolated_acceptance_launch_has_only_costmaps_and_lifecycle_manager
cooperative_ws/src/multi_agent_nav/tests/test_dynamic_costmap_config.py::test_no_verification_flags_are_set_by_static_costmap_profile
cooperative_ws/src/multi_agent_nav/tests/test_dynamic_costmap_config.py::test_static_collection_dry_run_contains_only_read_commands
cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py::test_empty_resources_allow_independent_exploration_but_not_cooperative_live
cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py::test_global_goal_age_config_must_be_positive_finite[0.0]
cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py::test_global_goal_age_config_must_be_positive_finite[-1.0]
cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py::test_global_goal_age_config_must_be_positive_finite[nan]
cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py::test_global_goal_age_config_must_be_positive_finite[inf]
cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py::test_global_goal_age_config_must_be_positive_finite[True]
cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py::test_global_goal_age_config_must_be_positive_finite[None]
cooperative_ws/src/multi_agent_nav/tests/test_field_preparation.py::test_offline_single_robot_config_does_not_require_cooperative_certification
cooperative_ws/src/multi_agent_nav/tests/test_go2_executing_path.py::test_execution_path_timeout_logs_receipt_age_once[go2]
cooperative_ws/src/multi_agent_nav/tests/test_go2_executing_path.py::test_execution_path_timeout_logs_receipt_age_once[ranger]
cooperative_ws/src/multi_agent_nav/tests/test_go2_health_diagnostics.py::test_localization_loss_logs_failure_time_evidence_once_per_loss[stale]
cooperative_ws/src/multi_agent_nav/tests/test_go2_health_diagnostics.py::test_localization_loss_logs_failure_time_evidence_once_per_loss[tf_error]
```

## 后续诊断：nav_ready 子项日志（2026-10-25）

- 本次任务在 21:52:23.887 出现 nav_ready=false；localization_valid 始终为真，约 150 ms 后 nav_ready 恢复，但 PAUSED 保留原因。旧日志没有具体失败子项，不能事后断言是哪一项。
- `RangerAdapter.get_health` 在有效到失效的边沿记录 failed_checks、执行状态与已有 health_details；沿用执行期 RGB-D 等待豁免，未改变健康门限、暂停策略和导航行为。
- 回归 83 passed，现场安装后接口回归 4 passed；3 个包构建成功。日志 `/tmp/ranger-nav-ready-evidence-{tests,field-tests,build}.log`；增量 `/tmp/ranger-nav-ready-evidence.patch`，备份 `/tmp/ranger-nav-ready-before.tar.gz`。
- 当前运行进程未自动重启，新诊断日志在下次正常启动任务进程后生效。


## nav_ready 自动恢复并保留观察进度（2026-10-25）

来源：用户新增要求“允许 nav_ready 恢复后解除 pause”，并明确“必须从原候选点、原方向继续”。
这是对现行健康暂停策略的显式变更，不把它归因于历史提交 251f9fb；其余旧版行为仍只依据该提交。

|位置|原行为|本次行为与保留项|
|---|---|---|
|`AgentMachine.stop/tick`|健康暂停锁存，STOP 取消执行|仅本地图任务的纯 nav_ready 暂停可恢复；停车、动作终止、定位和心跳仍需有效；其他故障/手动暂停撤销恢复资格|
|`Coordinator.pause/tick`|联动 PAUSE 后永久等待新任务|记录原阶段，保留任务和命令；双方就绪且确认暂停停车后发送原阶段 SYNC，并等待状态回执；显式暂停和新增故障仍锁存|
|`AgentRunner.dispatch/_advance_local`|STOP 关闭局部生成器|可恢复暂停保留同一个生成器、候选点和扫描 future；恢复推进原生成器；永久停止/换任务仍销毁旧执行|
|`Nav2Adapter._motion/pause_local_execution/resume_local_execution`|原生取消直接成为执行失败|取消原生动作并确认终态；保留逻辑 goal ID/future；恢复同一导航终点或补转至同一绝对朝向；原生真实失败仍报告失败|
|`RangerAdapter.tick`、`RangerSingleRobotExecution.on_plan_response/on_plan_result`|失效扫描被清空，异步规划可推进状态|暂停期间保留扫描索引、目标确认状态并延后规划回调；迟到的中断 VLM 结果不消费，恢复同方向新取帧；2 次尝试/8 秒规则不变|
|`_explore_steps.health_check`|已有路径曾取得后不再等待初始路径|原生导航恢复后重新等待真实执行路径，仍使用原 1 秒门限和静止要求，不用预规划路径替代|

共享坐标/资源占用模式维持原暂停策略。任务总时限不延长；候选排序、到达阈值、定位和健康门限均不变。
已有独立入口仍使用原健康终止规则，本次自动恢复接入发生该报错的本地图 agent/coordinator 链路。

验证：268 项相关测试通过，含完整八方向在第 5 方向暂停后继续、同候选/方向、剩余转角、迟到结果隔离、
取消未确认禁止恢复、手动暂停、定位/心跳/任务超时、C 目标绑定和新任务清理。
新增测试：`test/test_nav_ready_checkpoint.py`、`cooperative_ws/src/multi_agent_nav/tests/test_nav_ready_resume.py`。
日志：`/tmp/nav-resume-targeted.log`；增量补丁：`/tmp/nav-ready-resume.patch`。

广回归：936 passed、18 failed、1 skipped、7 deselected；18 个失败名称与 `/tmp/ranger-parity-final.log` 完全一致，无新增失败。
完整日志与失败名称：`/tmp/nav-resume-final.log`、`/tmp/nav-resume-final.xml`。首次未加载现场 overlay 的扩大测试为 502 passed、62 failed、1 skipped，包含缺失生成接口/模块及受限 ROS 环境；已使用正确 overlay 与既定排除项复验，以上述最终结果为准。
当前未执行真实机器人自动恢复实验，也未自动重启任务或发送运动指令。
协同自动恢复需要 agent 和 coordinator 均加载本次版本；本机构建不代表远端 Go2 已更新。

现场安装验证：3 个包构建成功，安装路径确认后 33 项测试通过；启动脚本继续加载
`build/ranger-local/install`。新增恢复顺序回归覆盖对端尚未停车、旧 PAUSED 回执，以及恢复后新 STOPPING/移动/手动暂停。
代码审查已完成，发现的恢复顺序问题已修正。任务进程未自动重启；agent 和 coordinator 下次按原入口启动加载新版本。


## nav_ready 原因细分（2026-10-25）

来源：用户要求“将 nav_ready 的原因细化为更方便排查的小项”。本次仅增加诊断证据和文本传播，
不修改健康门限、导航算法、VLM 次数/时限、原候选点/方向续跑及永久暂停条件；不新增 ROS 消息字段。

示例状态：

```text
local health invalid: nav_ready | nav_ready: local_costmap.receipt_stale(max_age_s=1.000, receipt_age_s=1.200, topic=/local_costmap/costmap_raw)
```

|子项|具体原因|源码证据|
|---|---|---|
|`navigate_server` / `planner_server` / `spin_server`|`unavailable`，附配置的 action 名|原 `server_is_ready()` 检查|
|`odom_fresh`|`no_valid_message`、`header_stale`、`receipt_stale`；附 topic、时间龄和门限|`on_odometry` 原消息时间戳过滤及 `get_health` 原接收时间检查|
|`local_costmap`|`no_payload`、`rejected_header_stale`、`rejected_invalid_metadata`、`rejected_invalid_origin`、`receipt_stale`、`header_stale`、`tf_unavailable`、`invalid_tf`|原 `on_costmap` / `costmap_transform` 的 last_update、reason、frame、时间龄/门限和异常类型|
|`active_task_map`|`path_unverified` / `path_mismatch`、`digest_unverified`、`verification_stale`，可同时出现|`AgentNode.poll_map` 原路径比较、文件摘要与 3 秒验证时效；附 expected/observed path、验证状态及 MapParameterRead.status|
|`active_task_keepout`|`mask_missing` / `info_missing`、`mask_mismatch` / `info_mismatch`|原禁行区 mask/info 匹配结果，并记录是否已经接收；旧数据无接收证据时用 `*_unverified`|
|`motion_fault_clear`|`cancel_timeout`、`cancel_request_error`、`action_send_error`、`action_response_error`、`action_result_error`、`evidence_write_failed`|在原设置 motion_fault 的位置增加原因标签，锁存首先记录的故障；无具体记录时用 `latched_fault`|
|`depth_units_verified`|`unverified`，附报告路径|原深度单位验证标志|
|`rgbd_ready`|`no_frame`、`frame_stale`、`image_tf_missing` / `image_tf_stale`、`sync_unhealthy`、`extrinsic_missing`、`raw_depth_missing`|原输入就绪证据；Ranger 执行期间仍豁免此项，不计入 nav_ready 失败|
|Go2 外部许可|`nav_ready_gate.false`、`control_armed_gate.false`、`external_gates_fresh.no_messages` / `stale`|原外部许可及其接收时效；附 topic 或各许可时间龄|

实现位置：`Nav2Adapter.nav_ready_reason` 生成 `Health.nav_ready_reason` 和
`last_health_details.nav_ready_failures`；AgentMachine、peer、Coordinator 沿用现有 reason 字段传递。
`is_nav_ready_pause` 只识别分隔符之前的原有允许恢复原因，保证诊断后缀不会扩大恢复范围。
同一次暂停中子项变化会更新健康暂停的原因；手动/定位/心跳等永久暂停原因不被覆盖。
Ranger 在有效转无效或失败子项代码变化时记录 `Navigation health invalid`，纯时间龄变化不重复刷警告。

验证证据：`/tmp/nav-detail-tests.log`、`/tmp/nav-detail-full.log`、`/tmp/nav-detail-full.xml`。
增量补丁：`/tmp/nav-ready-details.patch`；修改前备份：`/tmp/nav-detail-before.tar.gz`。
代码审查发现的“旧阻塞项不刷新”问题已修复，新增测试覆盖本机、对端和 coordinator 的原因更新。
实车触发、远端 Go2 部署未验证；没有自动重启进程。agent/coordinator 应同步加载本次版本，
旧版按完整原因字符串判定恢复，不能把仅更新一端当成兼容性已验证。

最终验证：164 项相关回归通过；3 个现场包构建成功；确认从安装路径导入后 55 项测试通过。
广回归 960 passed、18 failed、1 skipped、7 deselected，失败名称与 nav_ready 恢复版本完全一致，无新增失败。
现场证据目录：`build/ranger-local/nav-ready-details-20261025`，含增量补丁、修改前备份、SHA-256 清单、构建和测试日志。
