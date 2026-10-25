# Ranger 本机复用公共任务层

本轮将 `pppppst/VLM_Nav0` 的 `go2-cooperative-task-layer@eb32b4527c37cd0b278c1790f7c0bdde7bf5ad75` 接到 Ranger。公共消息、AgentMachine、AgentRunner、coordinator、FrameAdapter 和资源模块共用源码。单机入口复用 `go2_check → go2_exploration → Nav2Adapter`，Ranger 默认目标 B；不发布协作消息，不启动 coordinator、bridge、fake 对端或硬件。

这份文档中的软件测试/构建结果不等于现场运动验收。源码默认配置的观察点、停车余量和部分现场验证仍缺失，`--offline --require-ready` 会报告具体缺项；实验配置补齐后应退出0。2026-10-17按用户要求移除额外运动许可门槛，Ranger默认许可，无需新增许可发布者。不要将未测项目改成true来绕过其他检查。

## 环境和构建（终端 A，自动退出）

目录 `/home/isee601/ws/VLM_Nav`，ROS Humble，解释器 `/usr/bin/python3`。外部 Livox、RealSense、Ranger 和 FAST-LIO overlay 沿用 `scripts/common.sh`；新输出固定为 `build/ranger-local`，不修改旧 `install`。

```bash
cd /home/isee601/ws/VLM_Nav
source scripts/common.sh
colcon list --base-paths "$PWD" "$PWD/cooperative_ws/src"
colcon --log-base "$PWD/build/ranger-local/log" build \
  --base-paths "$PWD" "$PWD/cooperative_ws/src" \
  --build-base "$PWD/build/ranger-local/build" \
  --install-base "$PWD/build/ranger-local/install" \
  --packages-up-to multi_agent_nav --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF \
  -DCOOPERATIVE_RUNTIME_PYTHON=/usr/bin/python3
set +u
source build/ranger-local/install/setup.bash
set -u
ros2 pkg executables multi_agent_nav
```

预期只发现 `vlm_nav`、`multi_agent_task_msgs`、`multi_agent_nav` 各一次，构建三包成功；入口列表包含 `multi_agent_go2_check`、`multi_agent_ranger_check`。不要再 source 旧 install。`BUILD_TESTING=OFF` 避免本阶段引入 domain_bridge；bridge parser 和完整 colcon test 未运行。初次构建可有 PCL/CMake 策略及未使用 runtime 变量的警告，不代表测试通过或失败。

## 配置检查（终端 A，自动退出，无 ROS 初始化/API/运动）

```bash
cd /home/isee601/ws/VLM_Nav
bash scripts/start_ranger_local.sh --offline --require-ready
```

脚本加载已有依赖，最后加载新 overlay；offline 分支在任何 `ros2 node list` 之前返回。只有退出 0、`single_robot_experiment_gates=[]`、`motion_sent=false` 后才进入现场阶段。普通 `--offline` 退出 0 仅表示报告生成。

配置目录为 `cooperative_ws/src/multi_agent_nav/config`。可复制该目录为自己的实验配置并用 `--config-dir /绝对目录`；配置必须完整包含四份 YAML，不要编辑 install 目录中的生成文件。

- `robot_ranger.yaml`：本机地图 `shared_maps/lab_ranger_20261006/lab.yaml`，图像 `lab.pgm`；配置记录 YAML/图像组合摘要，防止错图。
- 默认 `exploration_strategy=generated_candidates_vlm`，无需填写 viewpoints.ranger；家具标注暂为空时从本机地图空闲区生成候选。若显式选择 fixed_viewpoints，则仍需测量本机观察点。源码与当前 acceptance 配置均为 A=纸巾、B=灭火器、C=椅子；`--target` 只覆盖所选目标的描述。
- `observer_config_file`：读取根 `config/robot.yaml` 的 RGB/对齐深度/TF 参数；Ranger 强制 `raw_depth_mode=false`，不需要 Go2 深度单位报告或 SDK 路径。
- `odom_topic=/odom`：底盘反馈速度用于停稳（源码有反馈赋值，仍需现场动态对照）；map 位姿只取 map→base_link TF，不用 odom pose 冒充 map。
- Ranger默认具备运动许可；不再配置 `motion_permission_topic`、`motion_permission_timeout`、`motion_permission_verified`，不订阅额外许可话题。公共Nav2输入、动作服务、互斥和停稳检查照常执行。
- `stop_margin`：沿用原协同 acceptance 配置的 0.20 m。camera_calibrated 保留现场校验记录，不作为启动门槛；实际 TF/图像可用性仍必须满足。
- 本机 footprint 为原 0.72×0.50 m 矩形及 padding 0.03。`/vlm_nav/obstacle_cloud` 逐帧用于清除；全局和局部 costmap 仅从 `/vlm_nav/confirmed_obstacle_cloud` 标记动态障碍。底层 cmd 超时/固件 watchdog 尚待现场确认，没有移植 Go2 Sport 机制。

Ranger 的 `config/robot.yaml` 使用 `confirmation_period: 0.8`、`confirmation_cycles: 3`。过滤节点在固定 `odom` 坐标中按 5 cm XY 栅格累计，不区分 Z 层：同一周期中任意数量的命中都只算一个周期，连续三个完整周期均有命中、且距离首次命中至少 2.4 秒，才输出该格当前帧中的点。缺失一个周期、长时间断流、时钟回退或确认坐标变换失败均会打断历史累计；未设置的其他配置默认 `confirmation_cycles: 0`，保持逐帧输出。标记前的高度范围仍为 0.10～1.60 m。

该机制抑制短暂回波和短时经过造成的新增占用，会延迟新障碍进入 Nav2；停留足够久的行人仍可被确认。它不让已有体素自动过期，也不主动清除静态地图或禁行掩膜。清除仍沿用每帧点云的有效射线。诊断新增 `confirmed_points`，可与 `output_points` 对照。更新过滤器二进制及两个点云源配置后需重启 R2 导航栈，不要仅设置参数值来声称运行逻辑已经更新。

单机 `local_only` 在内存中使用 `ranger_local_map`、`alignment_id=local_only` 及本地图恒等变换。事件、观察和结果使用相同本机身份；ROS Nav2/marker 仍使用实际 TF 帧 `map`。磁盘 `alignment.yaml` 保持 `validated=false` 和 Ranger `transform=null`。单机 resources 显式为空。

## 附着或显式启动本机导航

默认入口附着现有硬件/定位/Nav2，不自动启动、ARM、杀进程或重启硬件。脚本报告已有旧 VLM/任务拥有者、重复底盘/TF节点；CLI 在真正运动前再次检查拥有者与健康。

若旧 `vlm_nav` 节点仍存在，先通过其原入口取消并确认真实停稳，再退出旧系统。`enabled=false` 仍会保留旧图像订阅，不能与新 observer 同用。新 launch 的 `start_legacy_navigator=false` 直接排除旧 Node，原入口默认仍为 true。

需要冷启动时仍先按旧说明 `docs/小车的VLM_Nav启动.md` 的硬件顺序运行；不要同时用旧完整固定地图入口启动第二套定位/Nav2。以下命令仅显式启动 map/AMCL、原 TF/过滤器和新 Nav2 profile，**保持运行**，不启动任何自主任务：

```bash
# 终端 B；底盘、相机、Livox、FAST-LIO 已按原顺序运行，当前无旧定位/Nav2/TF实例。
cd /home/isee601/ws/VLM_Nav
bash scripts/start_ranger_local.sh --start-navigation
```

地图默认真实 Ranger map；可在命令末尾用 `map:=/绝对路径` 覆盖，但任务配置的路径/摘要必须同步一致。AMCL 初始定位仍按旧操作在 RViz 设置。

## 只读现场检查（终端 C，自动退出，需现场检查授权）

```bash
cd /home/isee601/ws/VLM_Nav
bash scripts/start_ranger_local.sh --action inspect --timeout 15 --require-ready
```

不发 goal、Spin 或零速度；observer 不建立 cmd_vel publisher。检查实际 map_server 路径、TF、RGB-D、真实底盘速度、原生 action 服务及costmap；健康详情列出失败项，Ranger的 `platform_readiness={}` 表示无额外许可门槛。`/local_costmap/costmap_raw`、`/global_costmap/costmap_raw` 是配置接口；执行路径当前设 `/plan`、接收断流 5 秒，`path_source_verified=false`，仍需现场确认发布者/重规划关联。没有新增 path header 的 1 秒年龄门槛。

## 后续明确获运动授权才执行（终端 C，自动退出）

```bash
cd /home/isee601/ws/VLM_Nav
bash scripts/start_ranger_local.sh --action explore --allow-motion \
  --target-id B --approach-target --strategy fixed_viewpoints --timeout 300 \
  --output-dir "$PWD/build/ranger-local/experiments/ranger-B-$(date +%Y%m%d_%H%M%S)"
```

运动前先通过配置与只读检查。每次 Spin 完成→反馈停稳→新图→识别，目标朝向依次为初始朝向+45°、90°…360°。Ranger单机每个视角检出本轮目标后立即结束剩余旋转，确认停止后按 `--approach-target` 进入接近；无有效深度/图像时刻TF时报告 TARGET_DETECTED_UNLOCATED。其他目标不触发提前结束，未检出时最多检查八方向；有合法目标检出时，之前视角的API_ERROR不抹去这次检出，部分扫描不标记为整点已观察。合作任务的默认完整扫描保持原有调用方式。

Ranger模板HTTP超时为15秒，后台识别另留5秒用于图像和证据处理。`metadata.json`中的 `api_timeout_s`/`adapter_timeout_s` 分别记录两种预算；模型标注与任务采纳状态需结合 `events.jsonl` 的 `VLM_VIEW_RESULT.consumer_status` 判断。超过预算或取消后的旧回复仍不用于导航。

Ranger 单机成功半径仍从根 `config/robot.yaml` 读取，为 0.81 m，最多重试一次。Nav2 沿用旧版原生容差分工：`GridBased.tolerance=0.76 m`、controller goal tolerance `0.05 m`；两项合计对应名义 0.81 m 停靠范围，不能都设置成停靠距离。运行前读取实际 Nav2 参数，并核对实际路径终点到目标的距离预算。`TARGET_APPROACHED` 表示本次原生导航成功、本地距离和停止契约成立，不产生公共 READY。

Ranger 协同和新单机接近统一直接把目标本机 map 坐标交给 Nav2，yaw 使用当前机器人朝向，不再筛选目标周围的圆周停靠候选，也不读取 `standoff_distance`。目标参考点可以位于物体占据区；检查 Nav2 实际路径终点的车身空间，不额外要求完整旋转空间。等待本次导航 `SUCCEEDED`、动作终止和真实停稳后，单机距离须 ≤0.81 m，协同须 ≤1.00 m；超界最多重试一次，仍请求同一目标坐标。已在成功距离内也需完成本次导航，不再按 0.89 m 主动取消。取消参数保留供原版独立 VLMNavigator 使用。任务取消、地图/TF时效、保护区与执行路径检查保留。事件 `approach_method=target_reference` 和 `requested_goal` 可核对实际请求。

完整结果在新输出目录的 `result.json`、`events.jsonl`、`vlm/`（每个视角的输入/原始响应/结果）、`selection/`（语义策略时）。取消后旧响应不成为新 goal 或新识别结果；停止未确认时保留运动锁。不要删除或截断 `/tmp/ranger-coop-acceptance-motion.lock` 来继续，须先核对原生 action 和物理停止。

generated_candidates_vlm 支持不标注家具：semantic_yaml 为空时从本机地图空闲区生成完整旋转空间有效的候选，继续使用双图和严格 P ID 解析；显式提供标注时仍校验 source_robot、源地图与摘要。semantic_furniture 策略仍需要自己的资料。生成策略同图查 ABC，只用所选目标决定本地接近；其他目标或视角的 API 错误不会抹去有效 B 检出，也不会把整点标为已观察。

## 软件回归（自动退出）

```bash
cd /home/isee601/ws/VLM_Nav
bash scripts/test_ranger_local.sh -q \
  cooperative_ws/src/multi_agent_nav/tests/test_robot_adapters.py \
  cooperative_ws/src/multi_agent_nav/tests/test_robot_local_exploration.py \
  cooperative_ws/src/multi_agent_nav/tests/test_go2_adapter.py \
  cooperative_ws/src/multi_agent_nav/tests/test_go2_exploration.py \
  cooperative_ws/src/multi_agent_nav/tests/test_go2_scan_evidence.py \
  cooperative_ws/src/multi_agent_nav/tests/test_go2_executing_path.py \
  cooperative_ws/src/multi_agent_nav/tests/test_generated_candidates.py \
  cooperative_ws/src/multi_agent_nav/tests/test_semantic_search.py \
  cooperative_ws/src/multi_agent_nav/tests/test_exploration_display_records.py \
  test/test_arrival_contract.py test/test_geometry.py test/test_pipeline.py \
  test/test_fixed_map_config.py test/test_vlm_client.py \
  test/test_ranger_local_launch.py test/test_navigation_gate.py
```

脚本使用 Domain31、localhost、`/usr/bin/python3`，不接硬件/对端，不请求 API；部分 Node 测试需要本机 DDS 套接字权限。缺少本次未同步的 Go2 SSH 部署脚本时，对应一条部署测试明确 skip，公共算法测试照常执行。没有把 bridge parser 或全量 colcon test 计为通过。

## 双机接入点与回退

双机后续仍使用公共 AgentMachine、Runner、消息和 coordinator。现场准备好真实 Ranger→lab_ref 标定、门/C区域、两端任务状态与定向 bridge 后，才替换 local_only 并接 AgentNode 的 Ranger live 分支。本轮未启用该分支，不使用 fake READY 或假标定来解锁。

回退：先退出新任务并等待其取消和反馈停稳；若停稳未确认，按现场停止流程处理，不能直接开旧任务。退出本轮新导航终端后，在新终端加载原 `scripts/common.sh`，再按旧成功入口运行：

```bash
cd /home/isee601/ws/VLM_Nav
bash scripts/start_fixed_map_navigation.sh \
  --map shared_maps/lab_ranger_20261006/lab.yaml '<目标描述>'
```

这是旧现场启动流程，会启动硬件/导航；仅在已有相应现场授权时执行。无需 reset 仓库，不删除旧 install、实验记录或 bag。
