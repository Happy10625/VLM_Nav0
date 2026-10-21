# 协作 VLM 导航人工验收：Ranger 单机到双机

> 2026-10-20：最新双机人工启动请使用 [双机协同启动与验收](双机协同启动与验收.md)。下文保留单机与静止通信证据；旧共享坐标、保护区、Go2 先行/EXIT 步骤由新流程覆盖。

本文件供人工执行，顺序固定为：**① Ranger 单机现场闭环 → ② 静止通信 → ③ 双机任务闭环**。每阶段记录通过证据，再进入下一阶段；失败停在当前步骤排查，不连续粘贴后续命令。本文不会自动启动任何验收。

Ranger 仓库：`/home/isee601/ws/VLM_Nav`。框架来源：Go2 `go2-cooperative-task-layer@eb32b4527c37cd0b278c1790f7c0bdde7bf5ad75`。两端本地 Domain0，定向 bridge 接 Domain40；地图各自独立，真实协同时通过实测 SE(2) 对齐到 lab_ref。Go2 找 A、Ranger 找 B，双方 READY 后处理同一个 C。

## 0. 当前软件状态与执行边界

| 项目 | 当前状态 | 对人工验收的影响 |
|---|---|---|
| Ranger 公共 Nav2/视觉/探索链、独立 CLI | 已接入，三包构建通过 | 可准备阶段① |
| 离线软件回归 | 338 passed、1 skipped；跳过 Go2 专用 SSH 部署测试 | 不能替代现场结果 |
| Ranger 地图、对齐深度和底盘速度接口 | 已按本机配置接入 | 现场确认实际发布者、新鲜度与外参 |
| Ranger 观察点 | `scene.yaml: viewpoints.ranger=[]` | 填入人工测得的本机 map 点 |
| 实测 stop_margin、相机及 costmap 验证 | 按本轮配置和现场证据核对 | Ranger默认具备运动许可，不再要求额外许可话题或验证参数 |
| Ranger→lab_ref 对齐、门/C 区域及共享场景 | 未验证/未填写 | 阶段①不要求；阶段③必须补齐 |
| 新任务用 domain_bridge | 2026-10-18 已安装 domain_bridge 0.5.0、rmw_cyclonedds_cpp 1.3.5；PAUSED 状态双向收发及人工停流核对通过 | 阶段②状态传输子项通过；完整静止通信待验，见3.5 |
| Ranger AgentNode 的 live 分支 | 尚未接到 create_runtime；当前代码仍限制 Go2 | 阶段②只允许阻塞状态传输检查；阶段③前必须完成代码接线和回归 |
| live_gates 平台化 | 公共 live_gates 仍有 Go2 本机检查耦合 | 阶段③前修复，不能靠放宽 Go2 标记通过 |

已有 `scripts/start_ranger_agent.sh`、`scripts/start_ranger_multi_agent_bridge.sh` 使用的是旧 `multi_agent_msgs` 里程计状态链，不用于本次新任务验收。新消息是 `multi_agent_task_msgs`，新话题是 `/multi_agent/{go2,ranger}/state`、`observation` 和共享命令 `/multi_agent/go2/command`。

以下命令中的“保持运行”表示保留终端；“自动退出”表示等待结束并检查返回码。终端环境只 source，不启动驱动、ARM 或任务。已有健康硬件/定位/Nav2 时复用，不重复冷启动。不要同时运行旧 VLM 自主节点、独立探索 CLI 和公共 task agent。

## 1. 每个 Ranger 终端的环境(已完成)

Ranger 所有验收终端统一加载 [RangerVLM_env.bash](../RangerVLM_env.bash)。它设置 REPO、RANGER_LOCAL_OUTPUT、Domain0、日志目录，先加载原依赖，再最后加载新 overlay，并切换到仓库目录；保留本机 RMW/网卡和 API 环境，不启动节点、不发 ARM 或任务。

每开一个新 Ranger 终端，命令序列的第一行都是下面的 source。返回非0时停在本步骤，修复环境后再执行该终端的后续命令：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
```

R0 创建本轮目录后，其他终端 source 时自动读取 `build/ranger-local/current-acceptance-run.txt`，导出同一个 ACCEPTANCE_RUN、RANGER_CONFIG 和 RANGER_BRIDGE_DDS_XML。首次尚未创建目录时，这三个变量为空；先完成下面的 R0 初始化，再开其他终端。source 不创建新实验、不覆盖配置，也不自动生成 bridge XML。

新 overlay 缺失时默认报错返回；此时先执行下面的构建命令块（首行带 `--build`），构建成功后再初始化本轮记录。只有构建终端使用 `--build`。回退终端使用 `--legacy`，只加载原依赖/旧安装。不要随后 source 旧 install 覆盖新版本。Ranger 不套用 Go2 的 enp4s0、Sport、raw-depth 报告或 DDS XML；不要复用已经设置为 bridge 的终端执行本地硬件任务。

终端 R0 创建本次人工配置和证据目录（自动退出，只执行一次）：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
export ACCEPTANCE_RUN="$REPO/build/ranger-local/acceptance-$(date +%Y%m%d_%H%M%S)"
mkdir -p "$ACCEPTANCE_RUN"
cp -a "$REPO/cooperative_ws/src/multi_agent_nav/config" "$ACCEPTANCE_RUN/config"
printf '%s\n' "$ACCEPTANCE_RUN" > "$REPO/build/ranger-local/current-acceptance-run.txt"
export RANGER_CONFIG="$ACCEPTANCE_RUN/config"
git rev-parse HEAD > "$ACCEPTANCE_RUN/ranger-head.txt"
git status --short > "$ACCEPTANCE_RUN/ranger-working-tree.txt"
```

其他 Ranger 终端的公共环境已读取同一目录；可以只核对（自动退出）：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
test -n "$RANGER_CONFIG" && test -d "$RANGER_CONFIG"
```

若已存在本轮记录，直接读取，不重复创建覆盖。日志、配置和图像不存密钥。

只有源码改动或缺少新 overlay 时才构建（R0，自动退出；不启动机器人）：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash --build
colcon list --base-paths "$REPO" "$REPO/cooperative_ws/src"
colcon --log-base "$REPO/build/ranger-local/log" build \
  --base-paths "$REPO" "$REPO/cooperative_ws/src" \
  --build-base "$REPO/build/ranger-local/build" \
  --install-base "$REPO/build/ranger-local/install" \
  --packages-up-to multi_agent_nav --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF \
  -DCOOPERATIVE_RUNTIME_PYTHON=/usr/bin/python3
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
ros2 pkg executables multi_agent_nav
```

预期三个包各发现一次、构建退出0，入口包含 `multi_agent_go2_check` 和 `multi_agent_ranger_check`。本构建不运行 bridge parser 或完整 colcon test。

## 2. 阶段①：Ranger 单机现场闭环(10.17完成)

### 执行步骤:

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
bash "$REPO/scripts/start_ranger_stack.sh"

bash "$REPO/scripts/check_ranger.sh"
```

然后直接跳转**2.6**

### 2.0 一条命令启动硬件、Nav2和轻量RViz

完成环境构建和本轮目录初始化后，在有桌面显示的R0终端运行：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
bash "$REPO/scripts/start_ranger_stack.sh"
```

统一入口复用现有启动脚本，依次启动相机→Ranger→MID-360→FAST-LIO，再打开R2定位/Nav2、R3轻量RViz和R5只读诊断终端；新开的R2/R3/R5终端先加载同一公共环境。硬件启动器打开的4个终端及R2/R3/R5均保持运行。已有唯一硬件发布者、完整Nav2/TF节点或RViz/诊断终端时复用，检测到部分Nav2节点、重复关键节点或其他上层任务时停止，不自动kill或叠加实例。仅单次启动器持有启动锁，防止两个统一入口同时执行。

默认RViz为 `/home/isee601/松灵小车2/固定地图轻量验证.rviz`，地图和Nav2 profile沿用现有Ranger本机入口。也可将现有配置路径作为唯一参数：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
bash "$REPO/scripts/start_ranger_stack.sh" "/absolute/path/custom.rviz"
```

本入口的启动编排回归已验证冷启动顺序、完整实例复用、部分/重复节点拒绝、硬件失败不继续启动及无环境帮助输出；项目 `test/` 回归176项通过。实际启动已执行至硬件、R2及R3命令全部提交，未发送任务目标。 随后硬件只读检查10项通过、0项失败，5秒CAN采样实际接收增加4654帧、发送队列0帧、累计丢弃0帧；RViz进程1个。这是本次启动后的新证据，不能沿用旧CAN失败快照认定当前仍未接收。硬件日志目录为 `check-20261017_193855-br4dmqsa/`，定位初始位姿及完整导航验收仍需继续。

脚本在Nav2关键节点出现后打开RViz，不等待地图定位通过后才打开窗口；首次定位可能正需要在窗口中操作2D Pose Estimate。用真实位置/朝向完成初始位姿设置，已有正确定位不重复重置，再执行：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
bash "$REPO/scripts/check_ranger.sh"
```

启动入口完成表示各启动命令已提交，不表示生命周期、TF、真实CAN反馈或导航验收通过。它不运行ARM、探索、VLM请求或任何导航目标。CAN为DOWN时仍由原底盘终端请求sudo密码；真实反馈异常按2.3.5处理。使用本节后，2.1–2.2保留分终端启动参考，2.3记录已遇问题及简短恢复办法，不再粘贴一套重复启动命令。

### 2.1 连接、现有节点与唯一发布者（R0，自动退出）

先人工确认机器人周围可通行、相机/雷达/USB-CAN 上电且实体停止手段可用，再执行：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
ip -details link show can0
lsusb
ros2 node list --no-daemon --spin-time 2
ros2 topic list --no-daemon --spin-time 2
```

`can0` 不存在时先处理 USB-CAN 连接/驱动，不创建虚拟 can0 冒充底盘。已有 `/vlm_nav` 或 `multi_agent_*check_motion`/task agent 时，先通过其原入口取消并确认停稳，再退出其上层进程；不直接 kill 或启动第二个拥有者。

### 2.2 必要时按原顺序冷启动硬件（R1，保持运行）

硬件均已运行时跳过。原启动器按相机→Ranger→Livox→FAST-LIO 顺序启动，使用桌面终端；不启用自主运动：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
bash "$REPO/scripts/start_hardware.sh"
```

它会打开各硬件终端并自动退出；**被打开的4个终端保持运行**。没有桌面时使用4个终端，依次执行下列命令，每步确认对应输入正常后再执行下一步：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
# R1a：相机，保持运行。
bash "$REPO/scripts/01_camera.sh"
```

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
# R1b：底盘，保持运行；can0 DOWN 时原脚本会请求 sudo 配置500000 bit/s。
bash "$REPO/scripts/02_ranger.sh"
```

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
# R1c：MID-360，保持运行。
bash "$REPO/scripts/03_livox.sh"
```

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
# R1d：FAST-LIO，保持运行，初始化时保持机器人静止。
bash "$REPO/scripts/04_fastlio.sh"
```

R0 核对（自动退出）：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
ros2 topic info /camera/color/image_raw
ros2 topic info /camera/aligned_depth_to_color/image_raw
ros2 topic info /camera/color/camera_info
ros2 topic info /odom
ros2 topic info /livox/lidar
ros2 topic info /livox/imu
ros2 topic info /Odometry
ros2 topic info /cloud_registered_body
ros2 param get /ranger_base_node publish_odom_tf
timeout 8 ros2 topic echo /odom nav_msgs/msg/Odometry --once --field twist.twist
```

每个硬件输入应有唯一实际发布者；`publish_odom_tf=false`；底盘 `/odom.twist` 的新鲜度须结合真实CAN接收核验，不能只看驱动重新打出的时间戳。单次静止零值不证明动态速度准确或停车能力。相机为本机 RGB/对齐深度，不引入 Go2 相机配置。

在R0运行硬件检查，看到全部通过后再进入2.3（不要求Nav2或AMCL已启动）：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
bash "$REPO/scripts/check_ranger.sh" --hardware-only
```

### 2.3 定位、Nav2和RViz：已遇问题与解决记录

正常启动统一使用2.0的 `start_ranger_stack.sh`，已有实例时复用。默认地图为 `shared_maps/lab_ranger_20261006/lab.yaml`，Nav2配置为 `config/nav2_ranger_local.yaml`，不启动旧VLM、coordinator、agent或bridge。RViz打开后用 **2D Pose Estimate** 设置实车初始位置和朝向，已有正确定位不重设。

本节替换原来的人工排障流程和逐项ROS核对命令，按“现象—排查结论—简单处理”保留实际记录。以下恢复操作均先结束运动任务并确认实体停稳，退出对应旧进程后再启动，不叠加第二套实例。处理后统一运行中文只读检查，无需逐项读取topic、参数或JSON：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
bash "$REPO/scripts/check_ranger.sh"
```

本节最近一次完整复测为 **30项通过、0项失败**，证据位于本轮目录 `check-20261017_194612-s179377c/`。这是启动和输入健康结果，实际导航闭环仍按2.6验收。静止检查时没有正在执行的 `/plan` 属正常；真实导航时仍需验证执行路径持续更新。

### 2.3.1 RViz卡顿与轻量配置

**现象：** RViz拖动或提交初始位姿时无响应；早期重启RViz后FAST-LIO出现 `Dropped stale internal LiDAR scans`，曾怀疑相机数据积压。

**排查结论：** 后续复发时原始/适配里程计仍约10 Hz，TF持续更新，RViz也没有订阅相机图像，因此未证明相机积压导致UI冻结。丢弃旧扫描警告表示FAST-LIO清理内部过期数据，不能单独定位RViz故障。轻量配置关闭完整点云和图像显示、刷新率降至10 FPS，可减轻显示负担；提交初始位姿后仍冻结的问题见2.3.2。

**简单处理：** 在原R3终端Ctrl+C退出RViz，再打开已有轻量配置；硬件和Nav2正常时保持运行。2.0统一启动入口已经使用该配置。

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
bash "$REPO/scripts/start_rviz.sh" "/home/isee601/松灵小车2/固定地图轻量验证.rviz"
```

图像预览默认关闭不代表相机停流。新observer运行时可显示 `/vlm_nav/camera_preview`；只读检查退出后，该预览不再更新属正常，不另开旧VLM或第二个observer补预览。

### 2.3.2 轻量RViz完成2D Pose Estimate后冻结

**现象：** 使用轻量配置仍在提交初始位姿后卡死，主线程等待futex；同期里程计、扫描和定位TF正常。

**排查结论：** 当时 `tf2`、`tf2_ros` 为0.25.23，上游已报告相同操作触发RViz冻结，0.25.24修复了 `waitForTransform` 与 `testTransformableRequests` 的ABBA死锁。该缺陷与现场表现高度吻合；未取得冻结实例的完整锁栈，隔离对照也未稳定复现，故未将现场锁链判定为完全证实。当前本机两个包均已安装0.25.24，不再需要临时修改 `LD_LIBRARY_PATH`。

**简单处理：** 本机直接使用2.0入口及轻量配置。只有回退到旧系统、包版本仍低于修复版本时，才结束使用旧库的ROS进程，更新这两个包，再重新启动；安装不会替换已运行进程映射的旧库。

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
sudo apt-get install --only-upgrade ros-humble-tf2 ros-humble-tf2-ros
bash "$REPO/scripts/start_ranger_stack.sh"
```

依据：[上游问题992](https://github.com/ros2/geometry2/issues/992)、[0.25.24更新记录](https://github.com/ros2/geometry2/blob/0.25.24/tf2_ros/CHANGELOG.rst)。现场证据：`build/ranger-local/rviz-diagnostics/20261017-live/`。

### 2.3.3 costmap未显示或提示No map received

**现象：** RViz没有local/global costmap显示项；添加后出现 `No map received`。

**排查结论：** 显示项需要手动添加，RViz的Map显示应订阅 `/local_costmap/costmap`、`/global_costmap/costmap`，不使用 `costmap_raw`。当时还存在真实启动故障：controller为inactive、planner为unconfigured，导航管理器停在 `Configuring smoother_server`，其 `change_state` 响应发送超时，导致costmap没有正常激活。服务响应超时的底层原因尚未确定，不能归因为已证实的QoS不兼容。

**简单处理：** 在RViz添加上述两个Map显示，Fixed Frame保持map、Color Scheme设为costmap。若中文检查仍报告Nav2/costmap失败，退出原R2终端后重新启动R2，重新设置初始位姿，再运行本节统一检查；不逐个强制activate，也不叠加第二套Nav2。

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
bash "$REPO/scripts/start_ranger_local.sh" --start-navigation
```

当时通过仅重启导航生命周期管理器并清理未激活节点恢复，未重启硬件/AMCL；恢复后7个导航节点active，两张OccupancyGrid均实际收到。上述退出R2再重启是后续复发时更简单的恢复方式。证据：`build/ranger-local/nav2-startup-diagnostics/`。

### 2.3.4 RGB-D未就绪、相机已有节点或停在Resetting device

**现象：** 相机有节点/发布接口但深度没有帧，检查返回RGB-D未就绪；重启时提示已有相机节点，或停在 `Resetting device...`。

**排查结论：** 一次故障中彩色正常，原始深度、红外和对齐深度均停流，日志包含 `digital_gain:get_xu` 错误；关闭再开启depth无效，设备复位后恢复。“已有节点”来自此前保留的相机恢复进程，不等于发现两个相机实例。后续发现自动 `initial_reset=true` 可能在USB重新枚举后仍停滞，因此当前配置已恢复 `initial_reset=false`。libusb控制传输警告的底层原因仍未完全确定。

**简单处理：** 先结束原相机驱动；深度停流且正常重开无效时，按设备流程做一次设备复位/USB重连，等待重新枚举，再以当前默认配置正常打开。不要反复切换depth开关或把每次启动强制复位设为默认。

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
bash "$REPO/scripts/01_camera.sh"
```

若已经卡在复位阶段，先结束该相机进程，再执行正常启动，避免再次触发复位。当前 [realsense.yaml](../config/realsense.yaml) 保留全局时间戳配置；无需修改分辨率、外参或健康阈值。当时恢复到1280×720、16UC1真实对齐深度，完整只读检查通过。证据：本轮 `repair-camera-reset-inspect.log`、`build/ranger-local/data-diagnostics/` 及 `build/ranger-local/hardware-restart-20261017_181730/camera-reset-stalled.log`。

### 2.3.5 Ranger持续Failed to send CAN frame

**现象：** 探索启动后底盘反复输出 `Failed to send CAN frame`，但 `/odom` 仍可能保持50 Hz。

**排查结论：** `strace`取得实际错误 `ENOBUFS`：CAN发送队列无法排空。当时can0虽为UP、ERROR-ACTIVE、500000 bit/s，RX/TX仍为0，队列积压11帧、累计丢弃148帧。驱动会给SDK缓存状态重新打当前时间戳并发布，因此 `/odom` 频率正常、零速度或软件停稳不能证明CAN链路正常。未锁定最初由供电、线缆、ACK缺失还是适配器状态触发；未证明与相机数据积压有关。

**简单处理：** 退出底盘驱动，确认底盘上电及CAN连接正常，再重置接口；以下命令需要sudo密码。随后重新打开底盘终端，并运行中文硬件检查。

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
bash "$REPO/scripts/02_ranger.sh"
```

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
bash "$REPO/scripts/check_ranger.sh" --hardware-only
```

单纯重启ROS不会清空接口队列；若仍无真实CAN接收，继续处理供电/连接/适配器，不加大发送队列或发速度试探。后续统一启动后的5秒采样实际接收增加4654帧、积压/丢弃均为0，硬件10项通过，说明该次采样通信已恢复；不能据此反推最初的物理故障原因。

若同时重启FAST-LIO，里程计原点会重新初始化，需重启R2并重新设置初始位姿。此前探索另有未取得执行路径而取消的记录，尚未通过真实运动复验。故障证据：`build/ranger-local/hardware-restart-20261017_181730/`；恢复证据：本轮 `check-20261017_193855-br4dmqsa/`。

### 2.3.6 地图路径检查失败且一次显示三项失败

**现象：** 定位、RGB-D、costmap和Nav2均通过，只剩“地图未确认匹配”“在线检查失败”“只读就绪未满足”三项失败。

**排查结论：** 原日志 `active_map_yaml=None`，检查器没有取得地图参数；独立查询发现实际地图与配置相同。旧代码未等待服务发现即只发一次异步请求，缺少响应超时重试；同一地图失败又被两个派生汇总重复计数。

**简单处理：** 代码已修正，无需重启硬件、Nav2或重设定位，直接运行本节统一中文检查。现在先等待服务可用，单次响应超时2秒后清理旧请求，间隔0.25秒重试；空回复/异常回复也重试，始终受本次检查总时限约束。“参数未取得”和“地图确实不一致”分别说明；真正不一致时显示实际及配置路径。

派生汇总不再单独计数；具体失败都未列出而总体仍未就绪或进程异常退出时，仍保留一条失败，避免误报通过。地图匹配检查继续保留，未增加额外运动许可。本次修正后完整检查 **30项通过、0项失败**，相关软件回归251项通过。证据：本轮 `check-20261017_194612-s179377c/`。

### 2.4 填写本轮已测配置（R0，人工编辑，自动退出）

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
${EDITOR:-nano} "$RANGER_CONFIG/scene.yaml" "$RANGER_CONFIG/robot_ranger.yaml"
```

必须逐项有现场依据：

- `viewpoints.ranger`：唯一 `id` 和真实 Ranger map `[x,y,yaw]`（米/弧度），至少一个；不复制 Go2 点、不填示例坐标。可以从 RViz 人工测点；初始 pose 不是目标观察点的自动授权。
- `targets.B`：本轮物体描述；默认水杯。若命令使用 `--target`，只替换本轮 `--target-id B` 的描述。
- `stop_margin`：本机确认的正有限停车余量；当前 null。`/odom` 停稳反馈及速度时效通过现场对照。
- `camera_calibrated`、`costmap_source_verified`：核对本机已有外参和实际来源后再填写结果，不通过改 true 解决未测问题。
- Ranger默认具备运动许可，无需填写 `motion_permission_topic`、`motion_permission_timeout` 或 `motion_permission_verified`，也无需运行额外的许可话题发布者。2026-10-17按用户要求移除此门槛；定位、RGB-D、里程计/costmap时效、Nav2服务、任务互斥与停稳检查仍适用。
- 保留本机 footprint、速度、膨胀及容差；成功半径0.81 m、取消半径0.89 m、planner tolerance0.76 m、controller tolerance0.05 m 分别核对。不要改成同一个数来放行。

单机不填写假 Ranger→lab_ref 变换，不要求 Go2 在线。CLI 在内存使用 `ranger_local_map/local_only` 和空 resources，不修改磁盘真实 alignment。语义标注缺失时先用 fixed_viewpoints。

### 2.5 配置与只读健康检查（R0，自动退出）

统一入口为 `scripts/check_ranger.sh`，像原VLM_Nav检查脚本一样逐项显示中文 `[通过]`/`[失败]`、必要的频率/时间龄和失败建议；默认不在终端打印JSON。默认完整检查依次执行配置校验、5秒真实硬件采样和15秒原有只读inspect。

只检查本轮配置，不访问ROS或硬件：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
bash "$REPO/scripts/check_ranger.sh" --offline
```

硬件启动后、R2尚未启动时，只检查CAN、相机彩色/对齐深度、MID-360点云/IMU、FAST-LIO及底盘ROS里程计的唯一发布者、实际数据与时效：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
bash "$REPO/scripts/check_ranger.sh" --hardware-only
```

R2完成初始位姿设置后，执行完整配置、硬件及定位/Nav2检查：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
bash "$REPO/scripts/check_ranger.sh" --timeout 15
```

三个入口均不发送goal、Spin、零速度或VLM请求，不重启硬件、不重置CAN、不设置初始位姿。`--config-dir`默认使用公共环境中的 `$RANGER_CONFIG`。`--timeout`控制原inspect观察窗口，`--hardware-window`控制真实硬件采样窗口（默认5秒）。退出0表示**本轮选择的检查范围**全部通过，退出1表示有失败项，参数错误返回2，中断返回130；仅配置或仅硬件通过不能代替完整检查及实体现场验收。

完整检查仍使用原配置/inspect判据，没有增加运动许可话题或修改导航参数；另外报告CAN实际接收是否增加，避免把没有CAN反馈时仍有50 Hz的 `/odom` 判为实车通信正常。静止inspect没有执行中的 `/plan` 属正常，不作为额外失败项。软件停稳结论仍需有效CAN反馈及现场确认，检查脚本不认证其他客户端已停止。

完整诊断自动保存在当前 `$ACCEPTANCE_RUN/check-日期_时间-随机串/`，目录不覆盖旧记录。`summary.txt`为中文结果，`configuration.log`/`inspect.log`保留原始报告及进程错误，`hardware.json`保存原始硬件统计，`hardware-ros.log`保存ROS底层日志。终端末行给出该目录；通常直接看中文失败项，无需打开JSON。需要自定目录时可增加 `--output-dir "$ACCEPTANCE_RUN/check-my-run"`，必须使用尚不存在的新目录。

2026-10-17脚本现场复测：`--offline`为6项通过、0项失败；默认完整检查为32项通过、1项失败，唯一失败是CAN采样期间没有真实接收增长。此时相机RGB/对齐深度、MID-360点云/IMU、FAST-LIO、地图/定位/costmap及Nav2检查通过，原inspect稳定窗口约12.87秒；整体仍返回1。该结果不表示底盘CAN已恢复，也没有启动探索。证据目录：`check-20261017_190728-k_i69j_a/`。软件回归：`bash scripts/test_ranger_local.sh -q test`，168项通过。

地图参数读取及重复失败计数的原因、修正和最新复测结果已合并到2.3.6；无需再执行一组人工参数核对命令。

#### 返回1时如何判断（2026-10-15 17:43轻量 RViz 复测）

**以下表格保留2026-10-15旧实现的实测结果。2026-10-17已按用户要求移除额外运动许可门槛，表中的许可失败项不再适用于当前代码。**

返回1表示 `--require-ready` 的就绪条件尚未满足，不能仅凭退出码判断为相机积压、节点崩溃或环境 source 失败。先看新脚本中“配置”“CAN/硬件”及“定位/Nav2”对应的中文失败行；两项检查的返回1可能有不同原因。需要追查时再从日志目录查看原始报告。

本轮证据目录：`build/ranger-local/acceptance-20261015_161341/light-recheck-20261015_174353/`，包含 `offline.log`、`offline.exit`、`inspect.log`、`inspect.exit`、`graph.json`、`timing.json`。

| 检查 | 本轮实际结果 | 含义/后续 |
|---|---|---|
| 离线配置 `--offline --require-ready` | 退出1，`robot_local_gates=[]`，5项 experiment gates 未满足 | 运动许可来源未验、实测正停车余量缺失、costmap来源未验、相机标定未验、Ranger观察点为空；按2.4补现场证据 |
| 旧版在线 `inspect --require-ready` | 退出1，`ranger_motion_permission=false`、`ranger_motion_permission_fresh=false`，`nav_ready=false` | 当时许可话题无发布者；此额外门槛现已移除 |
| 非运动输入 | `non_motion_inputs_stable=true`，约5.97秒首次就绪，随后约8.94秒无输入失效 | TF/RGB-D预热结束后，输入稳定；启动初期TF等待不等于持续相机积压 |
| 定位/地图/costmap/action | `active_map_matches=true`，定位、local costmap、navigate/planner/spin服务均正常 | 可保留当前硬件和导航实例继续只读排查 |
| RGB-D/TF/底盘反馈 | 检查结束时RGB-D约0.036秒、相机TF约0.035秒、底盘里程计约0.015秒 | 本窗口未发现这些输入持续过期 |
| 8秒里程计采样 | `/Odometry`、`/fastlio/odom`均约10.01 Hz，时间龄P95约0.020/0.021秒；`/odom`约50.00 Hz | 当前FAST-LIO与适配输出未降频；`/odom`是底盘反馈 |
| 轻量 RViz 实际订阅 | RViz没有图像订阅；inspect退出后采样时，相机图像话题也无其他订阅者 | 本轮未发现图像在RViz内积压的证据；不能据此认定之前UI无响应的根因已解决 |

在线 `inspect` 会显示配置的 experiment gates，但其返回码由在线就绪条件决定；实测停车余量和观察点等仍必须在运动实验前补齐。`health.reason` 可能同时显示 `execution path missing or receive timeout`：未发送导航目标时，不能仅凭静止时没有 `/plan` 判定导航失败；路径源仍需在实际执行时核验。

该历史窗口 `motion_sent=false`，未重置初始位姿，未发送导航目标或速度命令，观测到的底盘速度为零，当时验收停在2.5。当前按移除许可门槛后的代码重新执行2.5，以本轮报告决定是否进入2.6；不随意填写观察点/停车余量或修改未验证的标记来消除返回1。

### 2.6 一次完整选点→导航→扫描→接近→停止（R4，自动退出，实际运动/API）

现场操作者确认本轮点位和可通行区域、2.5检查通过后执行。R1/R2/R3 保持运行，R0待命。不要同时运行 coordinator、agent、旧VLM或另一个运动CLI。

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
export RANGER_EXPERIMENT_DIR="$ACCEPTANCE_RUN/single-B-$(date +%Y%m%d_%H%M%S)"
printf '%s\n' "$RANGER_EXPERIMENT_DIR" > "$ACCEPTANCE_RUN/current-single-experiment.txt"
printf '实时状态见R5诊断终端；完整日志：%s.console.log\n' "$RANGER_EXPERIMENT_DIR"
bash "$REPO/scripts/start_ranger_local.sh" \
  --config-dir "$RANGER_CONFIG" --action explore --allow-motion \
  --target-id B --approach-target --strategy fixed_viewpoints \
  --timeout 300 --output-dir "$RANGER_EXPERIMENT_DIR" \
  > "${RANGER_EXPERIMENT_DIR}.console.log" 2>&1
experiment_rc=$?
printf 'single experiment exit=%s\n' "$experiment_rc"
```

必要时在命令中增加 `--target '本轮实际描述'`，不改变B的归属。每个选中点经过 PLANNED/NAVIGATING/ARRIVED，再按初始朝向+45°、90°…360°逐段扫描；每次 Spin 完成、反馈停稳后取新图并等待识别。**一旦本视角确认B为DETECTED，就结束后续旋转**，记录 `SCAN_TARGET_DETECTED`，确认停止后进入 TARGET_FOUND→APPROACHING；上面命令的 `--approach-target` 启用接近。A/C检出不提前结束找B；未检出时最多扫描8个方向。B检出但无有效深度/图像时刻TF时结束扫描并报告 TARGET_DETECTED_UNLOCATED，不发送接近目标。

Ranger单机接近沿用原VLM_Nav方法：把目标参考点的本机map坐标交给Nav2，目标yaw取当前机器人朝向，由Nav2按原规划容差寻找路径；不再先筛目标周围的8个停靠候选。距离进入0.89米后取消导航并确认停稳，最终距离≤0.81米才判成功；未达标最多重试一次，重试在0.81米取消并再次验证。已在成功半径内且停稳时直接成功，已在取消半径内时先停稳验证。`standoff_distance`不再参与这条单机接近流程。事件的 `approach_method=target_reference` 与 `requested_goal` 可核对实际请求。

R5只读诊断终端（保持运行）：统一启动入口已自动打开时直接查看；未打开时，在新终端执行：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
bash "$REPO/scripts/monitor_ranger.sh"
```

面板每秒原地刷新，仅显示9行：实验目录名；状态/最近阶段/目标/观察点；当前动作及接受/终态；实测速度 `[vx,vy,wz]`、odom时间龄及CAN接收增量；指令速度数组与接收时间龄；路径点数/暂存或接收状态/时间龄；结果/任务停止确认/原因；排障；异常提示。速度单位依次为m/s、m/s、rad/s。无数据使用 `--`，过期、非法速度和CAN无增长会明确标出，不把缓存零速度作为实体停止证明。路径状态来自任务事件，不另建路径拥有者或改变门控。

“原因”保留具体扫描失败视角及动作终态，后续停止/清理事件不会覆盖。“排障”显示失败动作的原因（有错误码时一并显示）、结果文件中未通过的健康项、最近带reason的业务事件，以及最近一次VLM视角/识别状态或错误类型。Humble的Spin结果没有错误字段，面板按该次失败动作的起止时间匹配 `$ROS_LOG_DIR/behavior_server_*.log` 中的WARN/ERROR；例如 `动作:旋转预测碰撞：Collision Ahead - Exiting Spin`。仅匹配时间窗内的日志，不把旧碰撞归到本轮；日志不存在、已轮转或没有具体原因时显示“未取得具体原因”，不推断为碰撞。日志读取最多取最新8个文件各末尾64KiB，仅在旋转失败后尝试，保持只读低负载。

默认自动跟随 `$ACCEPTANCE_RUN/current-single-experiment.txt`；开始下一轮时自动切换并清空上一轮状态。无需手动export实验目录，也无需再打印完整JSON或整行事件。Ctrl+C只退出诊断显示，**不会暂停或取消R4任务**。

脚本更新后，在旧R5中按Ctrl+C，再执行上面的环境加载及启动命令，即可加载新字段；不需要重启硬件或Nav2。新实验中的扫描失败详情由新启动的R4任务记录，旧实验仍可按原生日志定位旋转原因。

R0或任务结束后只看一屏摘要（约2秒采样后自动退出）：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
bash "$REPO/scripts/monitor_ranger.sh" --once
```

如需固定查看历史实验，增加 `--experiment-dir /absolute/path/to/experiment`；此时速度/CAN仍是当前实车采样，结果和状态来自指定历史目录。诊断终端只订阅 `/odom`、`/cmd_vel`，读取CAN接收计数、已有事件/结果文件及失败旋转对应的原生日志，不订阅相机/点云、不发运动、VLM或停止指令。它是观察面板，退出0不等于验收通过；就绪判据仍以2.5的 `check_ranger.sh` 为准。

诊断回归覆盖写入半行等待、实验切换、旧goal不覆盖当前动作、日志替换、缓存路径接收时间、速度过期/非法值、缺少数据、CAN无增长，以及具体失败原因保留、旋转日志时间窗匹配、无日志时不臆测碰撞、健康失败项和VLM结果摘要。历史 `single-B-20261017_201045` 可直接显示第二次旋转中止及预测碰撞；本次没有启动或重跑运动任务。

`events.jsonl`、`result.json`及 `vlm/` 原始证据继续保留，`vlm/` 保存每视角图像、响应与解析；不额外录bag。R4的完整控制台输出保存为实验目录旁的同名 `.console.log`，需要深入定位时再读取。

阶段①通过条件：本轮实际找到B并完成接近；`result=TARGET_APPROACHED`、进程退出0、`stop_confirmed=true`，原生动作终态和底盘反馈停稳一致；核对实际视角的新图、目标B、本机frame及提前检出事件。未提前检出时才核对完整八方向。`EXHAUSTED`、`WAITING_VIEWPOINTS`、TARGET_FOUND（未完成接近）或单纯 Nav2 SUCCEEDED 均不能记为闭环通过。`TARGET_APPROACHED` 不产生公共 READY。

### 2.6.1 目标已接受却因执行路径为空而取消（2026-10-17修正）

**现象：** 开始探索后出现短暂图像时刻TF等待警告，随后报告 `invalid pose/path/footprint/stopping margin` 并退出1。失败实验为 `single-B-20261017_195101/`，原生导航最终CANCELED，`stop_confirmed=true`。

**排查结论：** 图像时刻TF与RGB-D后来已恢复，地图、定位、costmap、action和实测 `stop_margin=0.20` 均有效。事件记录显示，导航期间 `/plan` 回调比目标接受回调早约2.19毫秒，被旧适配层按“没有已接受导航”丢弃；全过程没有 `PATH_RECEIVED`。请求后约0.26秒底盘开始运动，缺少有效执行路径触发保护取消。只读检查不会发送导航目标，因此此前输入检查通过不能发现这项执行时序问题。

**简单处理：** 代码已修正：当前唯一未取消导航请求期间，先按原有坐标系、时间戳和有限值条件校验路径，并仅暂存最新一条；目标确认接受后才启用，记录 `PATH_BUFFERED`→`MOTION_ACCEPTED`→`PATH_RECEIVED`。缓存沿用最初接收时间，接受回调不会续期；取消、拒绝、无效更新或接收超时均不复用缓存。旧目标/预规划路径过滤、缺少执行路径时停止的保护和原时效阈值继续保留。

结束旧CLI后，直接按2.6使用新实验目录重新执行；当前源码由新入口直接加载，无需重建或重启硬件/Nav2/RViz，也无需重填停车余量。此修正未自动重发导航目标。Ranger和Go2共享路径逻辑的54项回归通过。全量回归为502项通过、1项跳过、19项失败；失败涉及未迁入的Go2文件、旧配置断言、缺失librealsense2及后续ROS context初始化，完整名称与错误见[全量回归日志](../build/ranger-local/path-race-regression.log)。真机闭环仍待人工复验。

`/plan`消息本身没有目标UUID，来源唯一性及执行/重规划关联仍需现场确认；本次修正不自动设置 `path_source_verified=true`。

### 2.6.2 图像标注已检出，任务却报告API错误（2026-10-17修正）

**现象：** `single-B-20261017_205935` 完成8个扫描方向后报 `target scan missing or API/validation error`。`vlm/*/result.png` 和 `metadata.json` 中第1、5个视角（view_index为0、4）已检出纸巾，任务事件却为 TIMEOUT/API_ERROR。

**原因：** 原配置 `api_timeout: 3.0` 同时用于HTTP请求与整个后台识别作业；后台还包含图像编码、响应解析、标注及落盘。第1、5个视角的证据处理总耗时约3.557秒、3.013秒，任务层先按3秒预算判超时，稍后生成的检出标注不能回填已结束的Future。第2、6个视角另有真实APITimeoutError。旧流程等待整圈完成，再因存在API_ERROR拒绝本次扫描；这些检出结果未进入深度定位，不能据此确认当时已有可导航位置。

**修正与复验：** Ranger模板和本轮配置的HTTP超时改为15秒，整个后台作业另留5秒处理证据；metadata分别记录 `api_timeout_s` 和 `adapter_timeout_s`。每视角的合法本轮目标检出会立即结束扫描；有效检出不会被此前视角的API_ERROR抹去。无有效检出时仍保留完整性及API错误检查，超时/取消的旧回复不用于导航。提前结束的部分扫描不记为整点已观察。新R4进程直接加载源码，结束旧CLI后按2.6创建新实验目录重跑即可，不需重启硬件/Nav2；本轮未自动发送运动或模型请求。

本次相关回归367项通过，覆盖本轮目标检出后不再旋转、非本轮目标不提前结束、默认完整扫描、缺深度不接近、此前API_ERROR不抹去合法检出，以及停稳未确认时不发送接近目标。全量为525项通过、19项失败、1项跳过；失败名称与此前路径修正回归完全一致，无新增失败，详见[本次全量日志](../build/ranger-local/early-target-regression.log)。真机接近仍待新一轮2.6验收。

### 2.6.3 已定位目标却没有安全停靠候选（2026-10-17修正）

**现象与原因：** `single-B-20261017_212008` 第3个视角确认B、深度定位有效，提前停止扫描后报告 `APPROACH_FAILED / no safe standoff candidate`。停稳机器人位置约 `(2.614,-0.056)`，目标约 `(2.886,1.010)`，距离1.099米，大于成功半径0.81米。旧协作接近流程在0.81米圆周筛8个候选，未选出后直接失败；当前只读costmap采样复现8个候选的车身覆盖区域均含253/254禁止栅格，快照位于 `build/ranger-local/standoff-diagnostics/20261017_212521/`。历史失败时未保存逐候选明细，快照属于后续复现。

**修正与复验：** Ranger单机改为原VLM_Nav的目标参考点规划→距离取消→停稳验证流程。目标参考点可以位于物体上，交给Nav2按既有0.76米planner tolerance生成路径；不再用协作层对目标点/圆周候选的整车栅格检查提前否决请求。Nav2的障碍层、车身碰撞与局部控制检查照常执行，协作层保留地图/TF新鲜度、非空规划路径、保护区边界、任务取消、执行路径跟踪及真实反馈停稳条件。无路径仍失败并记录规划原因，原生导航SUCCEEDED但停稳距离仍大于0.81米也不能判成功。新R4沿用2.6命令和新实验目录，无需重启硬件/Nav2；本次没有自动运行运动或模型请求。

本次相关回归379项通过，覆盖目标参考点位于占据区而Nav2返回容差路径、坐标转换、距离取消、已在成功/取消半径内的处理、固定参考点重试、停稳距离不达标、规划无路径/空路径、地图过期、任务取消及停止未确认。全量为537项通过、19项原有失败、1项跳过，失败清单与上一轮完全一致，见[目标参考点接近回归日志](../build/ranger-local/direct-reference-regression.log)。真实接近效果仍需新一轮2.6验收。

**随后现场验收通过：** `single-B-20261017_221059` 在 `ranger_view_02` 找到B/纸巾并完成接近，用户确认 `single experiment exit=0`。事件记录中首次停稳距离为0.914510米，按距离契约重试1次；第二次停稳距离0.767151米≤0.81米，`ARRIVAL_VERIFIED.decision=SUCCESS`，最终 `TARGET_APPROACHED`、`stop_confirmed=true`、`own_motion_terminal=true`。证据写入无失败且无待处理VLM证据。完整输出见[现场控制台日志](../build/ranger-local/acceptance-20261015_161341/single-B-20261017_221059.console.log)，详情见同名实验目录下 `events.jsonl` / `result.json`。据此记录阶段①单机导航接近闭环通过；阶段②静止通信和阶段③双机任务闭环仍待后续验收，不据此产生公共READY。

### 2.7 中止和回退

R4 首选 Ctrl+C，让 CLI 执行原生取消、零速和反馈停稳确认。未确认停止时按现场实体停止流程处理，不打开另一任务，不删除 `/tmp/ranger-coop-acceptance-motion.lock` 解锁。旧 `scripts/stop.sh` 只做旧节点disabled和一次零速，不等于新CLI取消完成。

确认新任务停止、退出新导航后，可在新终端仅加载旧 `scripts/common.sh`，按原成功入口回退；不要在新导航仍运行时执行：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash --legacy
bash scripts/start_fixed_map_navigation.sh \
  --map shared_maps/lab_ranger_20261006/lab.yaml '本轮实际目标描述'
```

此命令会启动旧硬件/导航流程。保留旧 install、日志和 bag，不 reset 仓库。

## 3. 阶段②：两端静止通信

**截至2026-10-18：静止状态传输子项通过，完整静止通信待验。** 双方 PAUSED 状态连续收发及人工退出 Go2 agent 后的停流核对已完成，证据和失败重试见3.5。

仅在阶段①记录通过后进入。两台实体保持静止，独立探索CLI已正常退出；不发 start，不录制假 READY，不桥接传感器/TF/cmd_vel/action。静止通信分“真实阻塞状态传输”和“真实有效定位状态/观察共享”，前者通过不能替代后者。

### 3.1 本阶段前置缺项

先人工核实两端真实共享网卡/IP、两端相同新消息schema，以及实际 domain_bridge 安装。2026-10-18 已完成两端依赖、共享网卡及三个新消息定义一致性核对，见3.5。以下命令保留供复验。不要因为单机已通过就使用旧 `multi_agent_msgs` bridge。

Ranger R0（自动退出）：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
ip -brief link
ip -4 -brief address
ros2 pkg prefix multi_agent_task_msgs
ros2 interface show multi_agent_task_msgs/msg/AgentState
ros2 interface show multi_agent_task_msgs/msg/Observation
ros2 interface show multi_agent_task_msgs/msg/Command
ros2 pkg prefix domain_bridge
```

缺包先按本机软件安装流程准备，再加载其实际依赖 overlay，**最后再次加载 ranger-local overlay**。本文不预设安装来源或未核实的 install 路径。

两端均需经过检查的按域网卡绑定 DDS XML。Ranger 的 Domain0 绑定本机实际本地域接口、Domain40 绑定本机共享Wi-Fi；Go2 保留原 Domain0=enp4s0、Domain40=wlo1 约定并核实当下接口。不要把仅有一个网卡的 Go2 本地域 XML 用于双域 bridge，也不要未经核实复制 Go2 XML到Ranger。

Ranger 文件准备在本轮配置目录（人工按实际接口编写/审查后）：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
export RANGER_BRIDGE_DDS_XML="$RANGER_CONFIG/cyclonedds_ranger_multi_agent_bridge.xml"
test -r "$RANGER_BRIDGE_DDS_XML"
```

文件尚不存在时此命令应失败，停在这里准备真实配置，不继续启动。

2026-10-18 本轮 `acceptance-20261015_161341/config` 已准备两份 Ranger XML：`cyclonedds_ranger_local.xml` 供阻塞 agent 和本机查看终端使用，Domain0 仅选择 `lo`、关闭组播并以 `127.0.0.1` 单播发现；`cyclonedds_ranger_multi_agent_bridge.xml` 供 bridge 使用，Domain0 同前，Domain40 仅选择 `wlo1`，使用 SPDP 组播发现和单播数据。现场 `wlo1=192.168.1.111/24`，`enp5s0` 无链路。已通过安装的 CycloneDDS 0.10.5 创建/销毁无话题 DDS domain 检查，日志确认 Domain0 选择 lo、Domain40 选择 wlo1；这不代表跨端通信通过。按域配置字段参见 [CycloneDDS 0.10.5 配置定义](https://github.com/eclipse-cyclonedds/cyclonedds/blob/0.10.5/etc/cyclonedds.xsd)。

以下本机 XML 仅用于本节阻塞状态传输。它不会更改其他已运行进程的发现配置；完整 live 验收前，仍须核对全部本机节点的 Domain0 隔离和可达性，不能据此声称已有硬件节点均已隔离。

### 3.2 Go2 端环境与软件核对（G0，人工在Go2电脑执行）

以下是已审查 Go2 源码中的路径；若现场更新，先用现场真实仓库/环境替换并记录 commit/导入路径，不能假定它仍与同步基线完全一致。Ranger不自动SSH操作Go2。

```bash
source /home/isee-pst/unitree_ros2/VLM_Nav/CoVLM_env.bash
git rev-parse HEAD
git status --short
ros2 pkg prefix multi_agent_nav
ros2 pkg prefix multi_agent_task_msgs
ros2 pkg prefix domain_bridge
ros2 interface show multi_agent_task_msgs/msg/AgentState
ip -brief link
```

G0 人工选择和记录 **Go2实际公共配置目录** 到 `GO2_CONFIG`，其中包含与 Ranger 一致的共享身份/场景及本机Go2 profile；不要用Ranger配置覆盖Go2硬件参数。

```bash
source /home/isee-pst/unitree_ros2/VLM_Nav/CoVLM_env.bash
read -r -p 'Go2 实际公共配置目录绝对路径: ' GO2_CONFIG
export GO2_CONFIG
test -r "$GO2_CONFIG/alignment.yaml"
test -r "$GO2_CONFIG/scene.yaml"
test -r "$GO2_CONFIG/bridge_go2_pending.yaml"
```

其他 Go2 终端的首行同样 source 已有 CoVLM_env.bash；需用配置时再执行 read/export/test 命令块，输入 G0 确认的同一 GO2_CONFIG。该远端脚本由 Go2 维护，本次不修改远端文件。

### 3.3 启动两端定向 task bridge（R5/G1，保持运行）

先确认没有现有新task bridge。核对 pending YAML 的实际内容，名字 pending 不代表验收通过：Ranger state/observation 是0→40，Go2 state/observation及共享command是40→0；Go2方向相反；全部 Reliable/Volatile，仅这5个任务话题。

Ranger R5（新终端，先加载公共环境，再设置本终端专用 bridge 环境）：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
export RANGER_BRIDGE_DDS_XML="$RANGER_CONFIG/cyclonedds_ranger_multi_agent_bridge.xml"
test -r "$RANGER_BRIDGE_DDS_XML"
cat "$RANGER_CONFIG/bridge_ranger_pending.yaml"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="file://$RANGER_BRIDGE_DDS_XML"
unset ROS_DOMAIN_ID
ros2 run domain_bridge domain_bridge "$RANGER_CONFIG/bridge_ranger_pending.yaml"
```

Go2 G1（新终端，先加载 Go2 公共环境并选择已确认配置）：

```bash
source /home/isee-pst/unitree_ros2/VLM_Nav/CoVLM_env.bash
read -r -p 'Go2 实际公共配置目录绝对路径: ' GO2_CONFIG
export GO2_CONFIG
test -r "$GO2_CONFIG/alignment.yaml" && test -r "$GO2_CONFIG/scene.yaml" && test -r "$GO2_CONFIG/bridge_go2_pending.yaml"
cat "$GO2_CONFIG/bridge_go2_pending.yaml"
test -r "$PWD/config/cyclonedds_go2_multi_agent_bridge.xml"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="file://$PWD/config/cyclonedds_go2_multi_agent_bridge.xml"
unset ROS_DOMAIN_ID
ros2 run domain_bridge domain_bridge "$GO2_CONFIG/bridge_go2_pending.yaml"
```

这两个专用终端只运行 bridge；不要将此双域环境复制到硬件终端。

### 3.4 阻塞状态传输检查（两端无coordinator，无start）

当前完整live gates未通过时，真实 AgentNode 会仅发布 PAUSED/closed-gates 状态，**不会创建运动 adapter**。先核对 gates 确实非空，才可利用这个明确阻塞分支检查传输；这不是模拟正常定位或READY：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
# Ranger R0，自动退出。必须输出非空的真实缺项列表。
export COOP_CONFIG="$RANGER_CONFIG"
python3 - <<'PY'
import os
from multi_agent_nav.config import load_config, live_gates
errors = live_gates(load_config(os.environ['COOP_CONFIG']), local_robot='ranger')
print('\n'.join(errors))
assert errors, '门槛已打开：不能使用本节的阻塞状态传输流程'
PY
```

Ranger R6（Domain0，本轮配置；保持运行）：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$RANGER_CONFIG/cyclonedds_ranger_local.xml"
ros2 run multi_agent_nav multi_agent_agent_node --ros-args \
  -r __node:=multi_agent_ranger_agent \
  -p robot_id:=ranger -p simulation:=false \
  -p topic_prefix:=/multi_agent -p config_dir:="$RANGER_CONFIG"
```

Go2 G0 用同一 Python 命令将 COOP_CONFIG 设为 GO2_CONFIG、local_robot改为go2，确认真实 gates 非空；G2 再启动同一入口，将 node名/robot_id 改为 Go2：

```bash
source /home/isee-pst/unitree_ros2/VLM_Nav/CoVLM_env.bash
read -r -p 'Go2 实际公共配置目录绝对路径: ' GO2_CONFIG
export GO2_CONFIG
test -r "$GO2_CONFIG/alignment.yaml" && test -r "$GO2_CONFIG/scene.yaml" && test -r "$GO2_CONFIG/bridge_go2_pending.yaml"
# Go2 G2，保持运行；此前已确认Go2 live gates非空。
ros2 run multi_agent_nav multi_agent_agent_node --ros-args \
  -r __node:=multi_agent_go2_agent \
  -p robot_id:=go2 -p simulation:=false \
  -p topic_prefix:=/multi_agent -p config_dir:="$GO2_CONFIG"
```

Ranger R0 查看对端（自动退出；无消息超时返回124）：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$RANGER_CONFIG/cyclonedds_ranger_local.xml"
timeout 12 ros2 topic echo /multi_agent/go2/state \
  multi_agent_task_msgs/msg/AgentState --once \
  --qos-reliability reliable --qos-durability volatile
ros2 topic info /multi_agent/go2/state --verbose
```

Go2 G0 同样检查 `/multi_agent/ranger/state`。核对 robot_id、schema_version、map_id/version、alignment_id、simulation=false及真实 PAUSED reason；当前应 ready=false、deposited=false，pose_valid可能false。再短时观察接收频率，排除仅发现了订阅者/旧缓存：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
# Ranger执行；Go2对称改成 /multi_agent/ranger/state。
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$RANGER_CONFIG/cyclonedds_ranger_local.xml"
timeout 12 ros2 topic hz /multi_agent/go2/state
```

此处 timeout124 是采样结束方式，不直接判失败；根据实际连续消息间隔评估，与 `scene.lost_timeout=3s` 比较。完成后 Ctrl+C 退出阻塞agent，保持实体静止。

**只有传输检查可在当前阻塞分支进行。** 要验收“定位有效的真实状态和目标Observation共享”，须先完成 Ranger live分支、平台化live_gates及真实地图对齐；随后两端仅运行真实agent、保持任务IDLE，不启动coordinator任务。观察必须来自当下实际图像/TF及当前身份，当前CLI没有独立的公共静止Observation发布入口；补齐该入口/调度前，不用 ros2 topic pub 构造目标来宣称通过。

阶段②完整通过条件：双方真实状态与有效观察双向到达，身份/坐标/接收时效一致，无运动、无伪造 READY；桥接白名单内没有重复回环或低层话题。仅PAUSED状态传输通过时，记录“通信传输子项通过，完整静止通信待验”，不进入阶段③。

### 3.5 现场验收记录（2026-10-18）

**结论：静止状态传输子项通过；完整静止通信及阶段③仍待验。** 现场操作者为用户，姓名未记录。本节依据本次会话中用户回传的控制台输出和反方向检查确认填写；未提供两端原始日志的保存路径，不将摘要记为已归档的完整日志。

本轮实际身份与环境：

| 项目 | 已核对内容 |
|---|---|
| Ranger 源码与 overlay | `ranger-reuse-go2-framework`，HEAD `251f9fb39c25849dac00e81aec97fa5c45d98c1d`，保留未提交接入修改；公共包来自 `/home/isee601/ws/VLM_Nav/build/ranger-local/install` |
| Ranger 配置 | `/home/isee601/ws/VLM_Nav/build/ranger-local/acceptance-20261015_161341/config` |
| Go2 源码与实际 overlay | 回传 HEAD `eb32b4527c37cd0b278c1790f7c0bdde7bf5ad75`；最终核验的 `agent.py`、`runner.py` 来自 `/home/isee-pst/unitree_ros2/VLM_Nav/build/go2-task-control-20261017/install/multi_agent_nav/local/lib/python3.10/dist-packages/multi_agent_nav/`，不以此前 `go2-arrival-budget-20261015` 导入结果代替 |
| Go2 配置 | `/home/isee-pst/unitree_ros2/VLM_Nav/build/go2-task-control-20261017/install/multi_agent_nav/share/multi_agent_nav/config` |
| 网卡与域 | Ranger：Domain0=`lo`，Domain40=`wlo1 / 192.168.1.111/24`；Go2：Domain0=`enp4s0 / 192.168.123.222/24`，Domain40=`wlo1 / 192.168.1.118/24` |
| 消息与阻塞分支 | 双方 `AgentState`、`Observation`、`Command` 的 `ros2 interface show` SHA-256 对照一致；Ranger gates 非空（本次本机核对11项），Go2 最终 overlay 核对14项；不为通过通信测试修改 gates |
| 本轮检查入口 | Ranger：`scripts/check_ranger_comm.sh`；Go2：`scripts/check_go2_comm.sh`（用户在Go2执行，本机未保存该文件）；共用 `scripts/check_multi_agent_comm.py`。Ranger `preflight` 已退出0并输出 `PREFLIGHT_OK` |

用户回传的 Ranger 侧 `bash scripts/check_ranger_comm.sh check` 结果：

| 本地域接收话题 | 12秒接收数 / stamp推进数 | 最大接收间隔 | 唯一发布端 | 结果 |
|---|---|---|---|---|
| `/multi_agent/ranger/state` | 24 / 23 | 0.500秒 | `/multi_agent_ranger_agent`，RELIABLE / VOLATILE | 本机状态连续到达 |
| `/multi_agent/go2/state` | 24 / 23 | 0.586秒 | `/ranger_cooperative_task_bridge_0`，RELIABLE / VOLATILE | 对端状态经本机导入桥连续到达 |

两条最新状态均为 `schema_version=1`、`simulation=false`、`run_state=PAUSED`、`reason="live configuration gates closed"`、`ready=false`、`deposited=false`；最大接收间隔小于 `scene.lost_timeout=3s`。阻塞态 `sequence=0`，新鲜度依据连续接收和 `stamp` 推进确认。地图身份分别为 Ranger 的 `lab_ranger_20261006 / sha256:3bebc0aad79b9e3add860b33c59a1c7f98231341182103c8a20f1cdd9b43badb`、Go2 的 `lab_rear120_20261001_IwQjMX / sha256:c9f354a26471032e7ff10b5898db627b65fce7526c28124891833b6a3a164c02`。

用户随后确认 Go2 侧反方向检查同样输出 `DATA_OK`；未回传该侧逐项采样数值，因此不补写其接收数和间隔。人工退出 Go2 agent 后，Ranger 停流检查最终回传：

```text
STOP_OBSERVED：对端状态流已停止；结合人工退出记录确认来源。
DATA_OK：本机PAUSED及对端经本机导入桥的新鲜状态满足采样判据。双向结果及人工停流记录需两端共同确认；定位/Observation/任务未验收。
```

失败与重试记录保留如下：

- Go2 最初未连接机器人时，环境加载报告 `GO2_NET_IFACE=enp4s0 has no 192.168.123.x address`；重新接线后确认 `enp4s0=192.168.123.222/24`，环境检查恢复。
- Ranger `preflight` 最初报告环境脚本用法错误：`source` 继承了外层的 `preflight` 参数。通信入口改为显式传入 `local`，并在 RMW/DDS 变量为空时使用 CycloneDDS 和本轮本地 XML；复验 `PREFLIGHT_OK`、退出0。
- 第一次停流检查未在提示后的时间窗口内退出 Go2 agent，未通过停流核对；第二次在开始采样前已退出 Go2 agent，报告“至少需3条状态、stamp推进2次”和“状态接收超时”。随后按重新启动 Go2 agent、先连续采样、提示后再人工退出的顺序重试，最终回传上述 `STOP_OBSERVED` 与 `DATA_OK`。此前失败不作为成功证据。

本轮仍有 `pose_valid=false`、`localization_valid=false`、`alignment_id=""`；即使状态字段含 `common_frame_id=lab_ref`、`stopped=true`，也不据此认定实测地图对齐、有效定位或实车反馈停稳已通过。尚未验证真实 Observation 双向共享或公共任务 command 的实际业务闭环。下一步按第4节前置项补齐 Ranger live 分支、平台化 live_gates、真实标定与现场配置，再验收阶段②剩余内容；不启动阶段③任务。

## 4. 阶段③：双机任务闭环

**本阶段当前不可直接启动。** 先完成并验证这些实际缺项，然后重新执行阶段②完整验收：

1. AgentNode 的 Ranger live 分支使用同一个 `create_runtime`，AgentMachine/Runner/消息继续共用；不能把单机 TARGET_APPROACHED 字符串映射为 READY。
2. live_gates 按 local_robot 检查本机地图/健康/文件，去掉无条件 Go2 本机耦合；两端共享身份和scene仍严格检查。
3. 两图实测对应点与独立检验点完成 SE(2)拟合；填写 alignment_id、map_id/version和可复算report。Ranger transform保持真实测量，不能使用local_only单位阵冒充。
4. scene.enabled、两端真实观察/等待/退出点、门和basket多边形/通道出口以及ABC描述准备完毕；测量场景符合实机 footprint 与停止余量。
5. 两端同消息schema、同任务配置身份、唯一goal拥有者、真实停稳/执行路径/动态障碍检测；共享C采用同一 observation_id。实体机器人在场，禁止fake对端。

标定工具命令（R0，离线、自动退出、不直接写入配置）：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
read -r -p '实测标定参考 YAML 绝对路径: ' ALIGNMENT_REFERENCES
ros2 run multi_agent_nav multi_agent_calibration "$ALIGNMENT_REFERENCES" \
  > "$ACCEPTANCE_RUN/alignment-report.yaml"
```

输入字段为 source_points、common_points、held_out_pairs、thresholds、identities；至少3个不共线拟合点及未复用的独立验证点。阈值由现场精度需求填写，工具输出validated=true并人工复核后才合入双方配置。

两端分别核对完整live gates（R0/G0，自动退出）。Ranger使用RANGER_CONFIG，Go2对称改为GO2_CONFIG及local_robot='go2'：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
export COOP_CONFIG="$RANGER_CONFIG"
python3 - <<'PY'
import os
from multi_agent_nav.config import load_config, live_gates
errors = live_gates(load_config(os.environ['COOP_CONFIG']), local_robot='ranger')
print('\n'.join(errors) if errors else 'live_gates=[]')
raise SystemExit(bool(errors))
PY
```

必须退出0。软件缺项未修复时即使编辑YAML也不能越过本阶段。

### 4.1 启动真实agent与唯一coordinator（保持运行，不自动start）

R1/R2以及Go2既有健康底层保持运行，R5/G1定向bridge保持运行，独立探索CLI和阻塞检查agent均已退出。使用阶段3.4的两个 agent 命令启动修复后的真实分支。确认两端正常IDLE、当前定位有效、stopped=true、ready=false，无旧任务残留。

Go2 G3 加载同一现场环境/配置，单独启动唯一coordinator；Ranger不启动coordinator：

```bash
source /home/isee-pst/unitree_ros2/VLM_Nav/CoVLM_env.bash
read -r -p 'Go2 实际公共配置目录绝对路径: ' GO2_CONFIG
export GO2_CONFIG
test -r "$GO2_CONFIG/alignment.yaml" && test -r "$GO2_CONFIG/scene.yaml" && test -r "$GO2_CONFIG/bridge_go2_pending.yaml"
ros2 run multi_agent_nav multi_agent_coordinator_node --ros-args \
  -r __node:=multi_agent_coordinator \
  -p simulation:=false -p topic_prefix:=/multi_agent \
  -p config_dir:="$GO2_CONFIG"
```

Go2 G0 启动前核对状态（自动退出）：

```bash
source /home/isee-pst/unitree_ros2/VLM_Nav/CoVLM_env.bash
timeout 12 ros2 topic echo /multi_agent/coordinator/status std_msgs/msg/String --once
timeout 12 ros2 topic echo /multi_agent/go2/state multi_agent_task_msgs/msg/AgentState --once
timeout 12 ros2 topic echo /multi_agent/ranger/state multi_agent_task_msgs/msg/AgentState --once
```

### 4.2 显式开始一轮（Go2 G0，实际运动/API）

人工确认上述全部通过、本轮共享区域已清场后执行；场景ID当前为lab，若现场变更须使用双方确认的实际ID。

```bash
source /home/isee-pst/unitree_ros2/VLM_Nav/CoVLM_env.bash
ros2 run multi_agent_nav multi_agent_start_task start --scenario lab --prefix /multi_agent
```

必须返回 accepted=true 和新 task_id。Ranger本机不另发开始请求。双方状态、观察、goal和token均核对该task_id及当前command/goal身份。

Go2 G4 观察状态（保持运行，Ctrl+C只结束显示，不暂停任务）：

```bash
source /home/isee-pst/unitree_ros2/VLM_Nav/CoVLM_env.bash
ros2 topic echo /multi_agent/coordinator/status std_msgs/msg/String
```

人工按序核对：

1. SEARCH_AB：Go2只接近A，Ranger只接近B；各自当前主任务的原生 NavigateToPose 成功、反馈停稳并满足公共任务机条件后才能READY。
2. READY屏障：两端同task_id、有效且新鲜的READY都成立，才进入SEARCH_C；不能用主动取消后的本地距离成功替代。
3. 同一C：两端使用同一条经身份验证、位置有效的C observation，不能各自选择不同篮子。
4. 资源：门/通道/basket token 对应当前task/command；无许可不进入，不因坐标中心出去就释放。
5. DEPOSIT：Go2先完成放置任务并整机退出basket区域、确认释放，再允许Ranger进入、放置并退出；此处“放置”为现有导航任务语义，不宣称机械臂抓放。
6. DONE：双方deposited/exited及coordinator DONE一致，原生动作终态、实体停稳与资源释放都有记录，无旧goal/旧观察被应用。

当前状态/观察topic为volatile；需要保存闭环证据时，在开始任务前将所需**高层文本**输出重定向到本轮目录，并在完成后正常结束这些显示进程。不默认启动bag或完整点云/RGB-D诊断。两端本地均记录，coordinator/status不在跨域白名单内，在Go2本机查看。

### 4.3 暂停、失败与退出

正常软件暂停（Go2 G0，自动退出，随后等待两端停稳）：

```bash
source /home/isee-pst/unitree_ros2/VLM_Nav/CoVLM_env.bash
ros2 run multi_agent_nav multi_agent_start_task pause --prefix /multi_agent
```

service success只表示已发出暂停，不等于实体停止。查看两端run_state、stopped、原生取消终态和资源状态；实体停止未确认时走现场停止流程。任务未安全停止前不要退出bridge/agent或重新start。确认停止后再按coordinator→agent→bridge→必要底层的顺序退出各终端。

## 5. 人工验收记录

本表按已有现场证据填写；后续继续补充原始日志路径和身份，保留失败记录。“待验”须有对应验收证据才能改为“通过”。

| 阶段 | 日期、操作者、源码/配置身份 | 证据目录 / task_id | 结论 | 失败项与下一步 |
|---|---|---|---|---|
| ① Ranger 单机现场闭环 | 2026-10-17，用户现场执行；Ranger HEAD `251f9fb`及未提交接入修改，本轮配置见2.6 | `build/ranger-local/acceptance-20261015_161341/single-B-20261017_221059.console.log`；`single experiment exit=0` | 通过 | 目标B接近、距离验证与停稳通过，详见2.6.3 |
| ② 静止状态传输子项 | 2026-10-18，用户现场执行；两端源码、实际overlay及配置身份见3.5 | 3.5节记录用户回传的双向 `DATA_OK` 和人工停流 `STOP_OBSERVED`；原始日志路径未提供；本子项未启动任务 | 通过 | 双向新鲜PAUSED状态收发、发布来源/QoS及Go2 agent人工停流核对完成；失败重试已保留 |
| ② 完整静止通信 | 截至2026-10-18未完成 | 状态传输证据见3.5；有效定位/Observation共享证据待补 | 待验 | 补齐真实live分支、平台化live_gates、地图对齐及现场配置，验证有效状态与真实Observation双向共享 |
| ③ 双机任务闭环 | 截至2026-10-18未执行 | 尚无task_id及闭环证据 | 待验 | 先完成阶段②完整验收及第4节全部前置项 |

详细本机接口、软件回归与回退补充见 [ranger_local_runbook.md](multi_agent/ranger_local_runbook.md)；本轮软件接入证据见 [ranger_port_status.md](multi_agent/ranger_port_status.md)。本文件仅对已有证据的阶段①单机现场闭环和阶段②状态传输子项记录通过，不代表完整静止通信或双机任务闭环通过。
