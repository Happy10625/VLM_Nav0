# Ranger 本机代码核查（2026-10-09）

本轮仅静态核查和文档编写。依据用户提供的 `/home/isee601/Notes/ranger_codex_audit_and_plan.md`，先完成核查，再生成[修改方案](../superpowers/plans/2026-10-09-ranger-cooperative-navigation.md)。没有运行或 source 启动脚本，没有启停节点、查询现场 ROS 图、发速度/导航目标、启动 bridge、SSH/ping Go2、测试通信或调用 VLM API；没有安装、构建、运行测试或修改产品代码。

证据等级：**静态**指本轮读取的文件、配置和本机文件状态；**历史**指迁移/运行记录；**用户验收**指用户确认的独立地图定位、旋转找物、导航已跑通；**待验**指本轮没有执行的现场检查。历史测试不能计为本轮结果。

## 1. 仓库与原有改动快照

| 项目 | 创建文档前的结果 |
|---|---|
| `pwd -P` / `git rev-parse --show-toplevel` | `/home/isee601/ws/VLM_Nav`，两者相同 |
| 分支 / HEAD | `main` / `251f9fb39c25849dac00e81aec97fa5c45d98c1d` |
| upstream | `origin/main`；本地记录为无 ahead/behind，仅反映已有 remote-tracking 信息 |
| remote | `https://github.com/Happy10625/VLM_Nav0.git`，未联网 |
| 本地分支 | 只有 `main` |
| worktree | 只有 `/home/isee601/ws/VLM_Nav`，同一 HEAD、`refs/heads/main` |
| 暂存 | `git diff --cached --stat` / `--name-status` 均为空 |
| 未暂存 | 下列 6 个文件；27 行增加、25 行删除 |
| 未跟踪 | 开始时 `git status --short --untracked-files=normal` 没有未跟踪项 |
| 最近提交 | `251f9fb 10.6`、`b01a764 8.20`、`4165cb2 refactor: replace sparse obstacle chain with standard Nav2 pipeline`、`1213bb5 init7.31`（`git log -8` 实际仅 4 条） |

原有状态（` M` 表示仅工作区修改）：

```text
 M README.md
 M docs/小车的VLM_Nav启动.md
 M docs/运行与测试日志.md
 M scripts/common.sh
 M scripts/start_ranger_agent.sh
 M scripts/start_ranger_multi_agent_bridge.sh
```

| 原有文件 | `git diff --numstat`（增加/删除） | 内容概要 |
|---|---|---|
| `README.md` | 11/9 | 用户路径迁移及环境变量说明 |
| `docs/小车的VLM_Nav启动.md` | 4/4 | 本机路径 |
| `docs/运行与测试日志.md` | 4/4 | 本机路径 |
| `scripts/common.sh` | 5/5 | 5 个 overlay 路径迁移到 isee601 |
| `scripts/start_ranger_agent.sh` | 1/1 | workspace install 路径迁移 |
| `scripts/start_ranger_multi_agent_bridge.sh` | 2/2 | 配置和 install 路径迁移 |

本轮保留上述全部内容，没有切分支、reset、clean、stash、pull、push、commit。写文档前另对全部 98 个 tracked 文件保存 SHA-256 快照到 `/tmp/ranger_audit_tracked_before_20261009.json`，用于交付末尾逐文件检查。两份目标文档原先不存在。

指引检查范围：`/AGENTS.md`、`/home/AGENTS.md`、`/home/isee601/AGENTS.md`、`/home/isee601/ws/AGENTS.md`、仓库 `AGENTS.md`、`/home/isee601/.codex/AGENTS.md`，以及仓库内/目标 docs 祖先目录的 `AGENTS.md`；未找到适用文件。仓库 `.agents/`、`.codex/` 为空。未使用或寻找不存在的 writing-plans 技能；说明文件要求本轮顺序执行，没有委派子代理。

## 2. 真实单机启动链与参数优先级

`docs/小车的VLM_Nav启动.md:1` 明确记录成功操作入口：

```text
scripts/start_fixed_map_navigation.sh --map shared_maps/lab_ranger_20261006/lab.yaml <目标描述>
 ├─ fixed_map.validate_map_yaml（校验 YAML、相对 image 路径、图像可读性）
 ├─ scripts/start_hardware.sh
 │   ├─ scripts/01_camera.sh → realsense2_camera/rs_launch.py
 │   ├─ scripts/02_ranger.sh → ranger_bringup/ranger_mini_v3.launch.py
 │   ├─ scripts/03_livox.sh → livox_ros_driver2/msg_MID360_launch.py
 │   └─ scripts/04_fastlio.sh → fast_lio/mapping.launch.py config_file:=mid360.yaml
 └─ scripts/start_navigation_validation.sh --localization-mode amcl --map ...
     └─ gnome-terminal → scripts/terminal_runner.sh → --component system
         └─ ros2 launch vlm_nav system.launch.py enabled:=false
             ├─ mapping.launch.py → cloud_to_scan + map_server + amcl + localization lifecycle
             ├─ navigation.launch.py → nav2_bringup/navigation_launch.py
             └─ vlm_navigation.launch.py → fastlio_odom_adapter + 2 静态 TF
                                         + obstacle_cloud_filter + vlm_navigator
```

定位入口证据：`scripts/start_fixed_map_navigation.sh:69,79,103,110`；导航子进程参数 `scripts/start_navigation_validation.sh:29,42,202`；launch 组合 `launch/system.launch.py:24`。`--map` 必需，无隐式默认地图。调用者先做 realpath，`map_server` 的最终 `yaml_filename` 在 `launch/mapping.launch.py:76` 覆盖 `config/ranger_amcl.yaml` 中的空值。`amcl_scan_topic` 默认 `/scan`，在 `mapping.launch.py:83` 覆盖 AMCL 配置。

环境链：每个硬件脚本/导航 component 读取 `scripts/common.sh:7–20`：Humble → `/home/isee601/ros2_ws/install`（Livox）→ `/home/isee601/rs515/ros2_ws/install`（RealSense）→ `/home/isee601/agilex_ws/install`（Ranger）→ `/home/isee601/ws/install_fastlio` → `/home/isee601/ws/VLM_Nav/install`，文件缺失只警告。五套路径当前均存在。ROS 包通过安装前缀解析，不能只读源码而假定被加载；本轮比对了所有 launch/config 和两个主要 Python 节点，安装副本与源码一致，多处为 symlink，FAST-LIO 和 RealSense launch/config 也解析到当前源码。

`launch/navigation.launch.py:24` 读取已安装的 Nav2 默认 YAML，然后递归 deep_merge **一个**选定 overlay：AMCL 用 `config/nav2_amcl_overrides.yaml`，SLAM 用 `config/nav2_overrides.yaml`；不是二者累加。显式 `overrides_file` 优先于模式选择。合并写临时 `/tmp/vlm_nav2_*.yaml`，传给上游非 composition、autostart Nav2。`system.launch.py` 尚未暴露 `overrides_file`。

VLM/适配/过滤共用 `config/robot.yaml`，`vlm_navigation.launch.py:70` 的参数字典最后覆盖 enabled、目标描述、scan_first、easy_case_mode、allow_frontier_after_scan、arrival_odom_topic。固定地图强制 `scan_first=true`、`easy_case_mode=false`、`allow_frontier_after_scan=false`。相机 TF 数字硬编码在该 launch 中，不从 robot YAML 读取。

| 入口 | 用途与运动触发 |
|---|---|
| `start_fixed_map_navigation.sh` | 成功文档指定的固定地图组合入口；启动 disabled，后续 ARM 才进入旧循环 |
| `start_navigation_validation.sh` | 默认 SLAM，也可显式 AMCL；不启动底层硬件，打开系统/RViz/状态/诊断终端 |
| `start_hardware.sh`、01～04 | 底层硬件；会查询 ROS 图/配置 CAN/启节点，本轮未执行 |
| `mapping.launch.py` | `slam` 启 SLAM Toolbox；`amcl` 启固定 map_server/AMCL，二者互斥 |
| `start_easy_case_validation.sh`、`easy_case.launch.py` | 空场测试，无静态/障碍/voxel 层；不适合协同 |
| `arm.sh:6–12` | 先运行 `check_system.sh`（含真实 VLM API 请求），确认 ARM 后设置 `/vlm_nav enabled=true` |
| `stop.sh:7–8` | disabled + 一次零 `/cmd_vel`；不等待取消终态/停稳确认 |
| `start_ranger_agent.sh` | 旧 `/odom` 状态转发入口，非新 robot_agent；依赖缺失，见 §6 |
| `start_ranger_multi_agent_bridge.sh` | 旧高层里程计状态 bridge，非任务桥 |
| `check_system.sh`、`qwen_latency_probe.py` | ROS 图诊断/真实 API；不能把它们当纯静态核查运行 |
| `test_no_hardware.sh` | 执行 pytest；本轮只读取，未执行（测试集含 ROS runtime 测试） |

旧自主导航权在 `VLMNavigator.navigation_tick` / `handle_worker_result` / `on_navigation_result`，参数 enabled=true 即触发扫描、单目标确认、接近及可能的探索。enabled=false 确实禁用旧循环，但也禁用其 RGB-D 处理（`on_rgbd:1147`），因此不能靠 disabled 旧节点直接提供协同感知技能。

## 3. 地图身份与定位 TF

| 文件（仓库相对路径） | 分辨率、origin | YAML SHA-256 | image SHA-256 |
|---|---|---|---|
| `shared_maps/lab_ranger_20261006/lab.yaml` / `lab.pgm` | 0.05 m；[-2.44, -9.87, 0] | `3ac7c5aa5f7ddc59423bdb877af905bcdc52171dda377a9ce2936a97ace9b9df` | `ff6b7cd32e87705bd0ea230ce127d9fdf687eeab471dd3df09609e6c76240daf` |
| `shared_maps/lab_rear120_20261001_IwQjMX/lab.yaml` / `lab.pgm` | 0.05 m；[-6.52, -5.34, 0] | `b928cdb28dab483674c49ab269ee9818cf8cf5d4fd147f2314d193e93877532f` | `90ab030608cba960b42922507a851760a7de2405b27665a65781c40e99eabe36` |

二者均 image=lab.pgm、mode=trinary、negate=0、occupied_thresh=0.65、free_thresh=0.25。frame 不在地图 YAML 中声明，由 map_server/AMCL/本机配置使用 `map`。第二份地图是历史候选，不能仅凭名字认定就是**本次指定 Go2 map**；缺对端正式地图 ID/版本/哈希。origin 相减不是两图标定。

固定地图 TF owner（静态代码）：

- AMCL：`map→odom`，`config/ranger_amcl.yaml:5–9,22`，OmniMotionModel；set_initial_pose=false，旧操作文档要求 RViz 初始定位。
- FAST-LIO：`camera_init→body`，外部 `FAST_LIO_ROS2/src/laserMapping.cpp:637–665`。
- 本项目：静态单位 `odom→camera_init`（`vlm_navigation.launch.py:35`）；`FastLioOdomAdapter.on_odom:101` 将源 pose 乘 `T_body_base`，发布 `/fastlio/odom` 的 `odom/base_link` 与 `odom→base_link`。
- Ranger 底盘 `/odom` 为另一条底盘里程计；02 脚本关闭其 odom TF，不能将其 pose 当 `map` pose。
- 相机安装：`base_link→camera_link` 在 `vlm_navigation.launch.py:42`；相机驱动提供 optical 子链。现文件值 x=-0.16、y=0、z=1.185、roll=0、pitch=0.45204034、yaw=0；这是已有数值，不是本轮标定结果。
- `robot.yaml:8–9` 的 base_to_body_xyz=[0.299,-0.02329,0.32088]、rpy=[0,0,π] 为已有配置，未看到显式 calibration_valid 标记/标定证书。

`VLMNavigator.robot_pose:1744` 使用 `lookup_transform(map,base_link,Time())`，是最新 map 位姿。原代码记录 TF 查询是否成功/超时，未形成公共地图版本校验、AMCL 跳变/协方差门槛。运行日志 `docs/运行与测试日志.md:454–579` 记载 AMCL 跳变及后续三次成功；`ranger_amcl.yaml:1` 仍写“not yet validated”。用户本次验收是较新事实，不能因旧注释否认已跑通，也不能据此宣称联合区域监护已验收。

## 4. 本机能力与可复用边界

| 能力 | 已读真实接口、输入输出/坐标 | 复用与缺口 |
|---|---|---|
| 相机 | `01_camera.sh:14` → RealSense wrapper；RGB 1280×720×30，原始深度640×480×30，align_depth/enable_sync=true；`realsense.yaml` 两流 global_time_enabled=true。外部 `rs_launch.py:24,26` 默认启用 depth/color，未指定 serial_no/device_type | **当前配置确实要求 RGB-D**；不是单目配置。实物型号/实际流见下文历史证据，实时内参/时间戳仍待验 |
| RGB-D 同步 | `VLMNavigator.__init__:242–269` CameraInfo + RGB/深度 sensor QoS；ApproximateTimeSynchronizer queue=10、slop=0.10 | 提取为可被公共 agent 调度的感知技能；不能直接开旧 enabled |
| 深度/内参 | `on_rgbd:1147` 要求 RGB/对齐深度/CameraInfo 同尺寸，深度 frame 与 camera_color_optical_frame 匹配；K[0,4,2,5]；`image_to_depth_m:1125` 支持32FC1/16UC1/MONO16 | 不填猜测内参；需 CameraInfo 新鲜度、相机外参身份和实测误差门槛 |
| MID-360 | `03_livox.sh` → 外部 `ros2_ws/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py:8–19`：CustomMsg、10 Hz、livox_frame、MID360_config.json；`src/lddc.cpp:481,575,630` 点云/IMU 话题 `/livox/lidar`、`/livox/imu`，IMU 为sensor_msgs/Imu | 原输入是 livox_ros_driver2/CustomMsg，不能误写为 PointCloud2。不加 Go2 传感器依赖 |
| LIO 与障碍 | 外部 `FAST_LIO_ROS2/config/mid360.yaml:15–48` 输入上述两话题；lidar_type=1，blind=0.5，scan_rate=10；输出 `/Odometry`（camera_init/body）及 `/cloud_registered_body` PointCloud2（body） | 参数沿用当前安装 symlink；已有雷达/IMU 外参只作配置证据，不新标定 |
| 点云→scan | `src/obstacle_cloud_filter.cpp:55–71,104–122` 在点云 stamp 查TF，输出 `/vlm_nav/obstacle_cloud`（base_link）；TF→PassThrough→负CropBox→VoxelGrid。`robot.yaml:19–31` 高度0.05–1.50、box x±0.36/y±0.25、leaf0.05 | 参数有实机 TODO。`mapping.launch.py:12` 转 `/scan`（base_link，高度0.05–1.50，range0.50–20，角步0.0087266）给 AMCL；不是 VLM 的深度来源 |
| VLM API | `OpenAICompatibleVLMClient.infer(rgb,target)`，`vlm_client.py:264–404`，单目标7字段 JSON；qwen3-vl-flash 默认，DASHSCOPE_* 优先，BASE_URL 或 WORKSPACE_ID 构造endpoint；备用 OPENAI_* | HTTP timeout来自robot.yaml的3s、max_retries=0；不记录密钥/实际私有endpoint。当前不是ABC批量识别，需要同帧逐目标调用适配 |
| 异步推理 | `LatestFrameWorker` 后台线程+latest待处理槽；`ensure_worker:1277`，result_timer 0.05s；main使用4线程executor | API不直接占ROS回调，但3目标结果不能沿用latest覆盖槽而漏掉某目标；需要显式同帧ABC批次身份 |
| 像素投影 | `resolve_pending_rgbd:1218` 按深度消息stamp、map←camera optical的确切图像时刻TF生成snapshot；`ground_pixel_with_reason:1657` → `geometry.depth_at_pixel_with_reason:72` → `project_pixel:121` | 深度中值/有效样本/范围检查，当前信任0.20–6.0m；图像TF等最多5s。识别成功但深度/TF无效只能position_valid=false；没有已实现的MID-360到图像联合定位替代路径 |
| 目标确认 | `confirm_target:1694`、`TargetTracker`，3帧、0.35m一致；target_reference_position为物体语义参考点 | 需ABC分别tracker；物体位置与停车点分开。可复用geometry，不套完整旧状态机 |
| 深度恢复 | `recover_target_depth:1925`、`send_spin`小角度重观测、`build_target_probe_candidates:1843`本机射线观察点 | 仅恢复观察技能，不把射线方向/机器人位置当物体地图坐标；公共agent决定何时调用 |
| 路径 | `plan_then_navigate:2378`、`request_next_plan:2426`，ComputePathToPose(use_start=false)；`on_plan_result:2468` 检查 action成功与非空path | 旧函数会规划后直接导航，协同需拆为“规划返回path”和“许可后执行”。/plan见RViz配置与历史录制线索；本轮未确认实际重规划发布者/QoS |
| 接近 | `navigation_tick:2191` 将物体参考x/y直接作为候选；`on_plan_result:2529` approach走完整路径，再发同一goal；Nav2 planner tolerance0.76、controller0.05 | 当前没有独立通用停车点技能。geometry中的旧snap/classify函数不能据存在就认定当前approach使用。协同本机选择/验停车点，保留单机现行为 |
| 导航 | `send_navigation_goal:2706` → `/navigate_to_pose`，PoseStamped(map)，异步handle/result，goal_token；`on_navigation_result:2779` | 当前nav反馈未注册，path更新不是Marker的保证。需公共task/command/goal身份、反馈和执行路径监护 |
| 扫描 | `begin_scan:1764`、`send_spin:2727` → `/spin`，相对yaw/time_allowance；8个yaw，每次成功后`prepare_stationary_observation:1797`，停0.30s、取新帧、等待确切结果 | 不是直接发非零旋转cmd_vel；需取消+实测停稳后采样，扫描许可必须含旋转footprint |
| 取消/停止 | `cancel_motion:2885` 增token、cancel_goal_async后立刻清handle；`publish_stop:896` 连发3零Twist | 普通取消没有等待cancel响应/terminal或停车。pending goal接受后补取消已有保护，但需要完整取消句柄生命周期 |
| 接近终态 | `begin_approach_stop:2285`、`advance_approach_stop:2316` 等action terminal、鲜活odom、连续3样本、距离≤0.81；超时3/5s | 会接受 CANCELED 或 SUCCEEDED，甚至已在范围内没有active goal也可本地SUCCEEDED。不是新的READY条件，见§5 |
| 底盘 | 外部 `agilex_ws/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:168,391` 订绝对 `/cmd_vel` Twist→SetMotionCommand→CAN；发布 `/odom` twist由反馈linear/angular/mode产生（289–373） | 本机未见ROS层cmd超时停机/仲裁实现；底盘固件watchdog未核实。已有footprint为0.72×0.50m矩形+padding0.03，实际附件/整机轮廓待验 |

相机型号证据不是目录名推测：本轮找到 `/home/isee601/migration_archive/Ranger-VLM_Nav_aux_last_session_20261008/迁移情况清单.md` 的已完成实机验收：**当时 L515、USB3.2、wrapper4.51.1/SDK2.51.1，RGB和对齐深度1280×720，全局时间参数true**。该记录和当前配置一致，但脚本并未锁定具体设备；本轮没枚举USB、没读实时流，因此应写“历史验收为L515、当前配置RGB-D，现场型号/流待复核”，不能写“本轮检测到L515”。

## 5. 协同前必须处理的行为差异

### 5.1 READY 与旧 SUCCEEDED 不等价

旧 approach在0.89m开始主动取消，在action CANCELED/SUCCEEDED且停稳、最终≤0.81m后可置本地SUCCEEDED（`vlm_navigator.py:2251–2372,2798–2819`）。此外仅原地距离已满足也可进入成功。`~/Ranger_migration/.../core/docs/使用日志.md:2380` 也记录停车环曾删除，圆/方容差存在历史讨论。

新要求只有**当前 Ranger B 取物接近主任务的 Nav2 STATUS_SUCCEEDED**才能READY；不能监听 `/vlm_nav/state` 的字符串推断READY。协同必须自己保存真实action status/UUID与公共任务身份，选择本机可达停车点并正常完成对应Nav2 action，再验证停稳/接近条件。主动取消的任务不计READY。旧单机语义保持不变。

### 5.2 当前停稳观测源存在静态缺口

外部 FAST-LIO `laserMapping.cpp` 中 `/Odometry` 的发布函数填pose/frame/stamp，但搜索及上下文未见任何 twist赋值。`fastlio_odom_adapter.py:125–151` 只转换源twist，所以 `/fastlio/odom` 的fresh stamp不能证明twist是有效速度。现approach停止检查订的正是该话题。不能由三个零值推断实体已停。

后续协同用**经现场对照确认有效的底盘 `/odom.twist`**作停稳源，map pose仍取TF；可辅以定位增量/速度输出，但任何速度源失效都拒绝替换运动/释放资源。Ranger底盘twist的源码有反馈赋值，历史迁移记录有静止零速；动态准确性、时效性、阈值与刹停余量仍需现场验收。本轮不修改原arrival参数，也不修改外部LIO代码。

### 5.3 固定地图并未配置动态障碍层

`nav2_amcl_overrides.yaml:18,44` 明确两costmap plugins仅 `[static_layer,inflation_layer]`，注释也写单机监督测试。SLAM overlay中的 obstacle/voxel设置不会自动叠加到AMCL。即使过滤点云持续发布，也不能据此声称固定地图costmap能感知另一台整机。

后续需独立协同Nav2 profile保留静态地图+加本机动态障碍层；不覆盖验收过的单机profile。对方机身是否进入costmap、制动是否有效必须真机验收。共享区域监管也不能替代本机障碍检测。

### 5.4 速度链与资源监护尚缺

已安装 `/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py:122,152,183`：controller `/cmd_vel_nav` → smoother → `/cmd_vel`；behavior_server仍直接 `/cmd_vel`；VLM停指令也直接同话题；smoother默认OPEN_LOOP、velocity_timeout=1s。这不是完整速度仲裁，零速发布可能被其他发布者覆盖。需协同速度门控及唯一底盘输入的验收，不能只依赖一次零速。

没有找到门/C许可、footprint退出判断、真实执行路径随重规划更新监护、失联保留占用等实现。Nav2 NavigateToPose feedback含current_pose/distance_remaining，不含完整路径（本机已安装action定义）；旧 `/vlm_nav/markers` 为可视化，不能充当带goal身份的实际执行路径。

## 6. 公共包、交接与 Domain 核查

### 6.1 查找范围和结论

已查仓库、`/home/isee601/ws`、common.sh指定的三个外部工作区源码/安装线索、Notes、Documents、Downloads、Ranger_migration核心迁移包、migration_archive、松灵小车2的相关文件清单/文档；没有递归扫描整个home，没有读取密钥、会话原件、bag或图像内容。

- 现仓库只有旧bridge YAML与两个旧启动脚本，未找到 `multi_agent_msgs` / `multi_agent_comm` 源码或消息生成包。
- `/home/isee601/ws/install/setup.bash` 不存在：两个旧协同脚本source该路径，会在进入节点前失败。当前有效VLM overlay在`VLM_Nav/install`，不能因此擅自用它“修复”缺失公共包。
- `/opt/ros/humble` 包索引未找到domain_bridge或multi_agent包；当前已知安装前缀无发现其实现证据。不能声称机器绝对没有任意未搜索位置的副本。
- 找到迁移文档 **9.19历史记录**（`/home/isee601/Ranger_migration/Ranger-VLM_Nav_core_20261007_100709/core/docs/使用日志.md:2391–2454`）：旧ranger_agent_node把 `/odom` stamp/pose/twist 转成 ranger/odom 的state，nav_state=IDLE、task_id空、healthy=true。它只证实曾有里程计消息工程；源码、精确.msg schema、来源commit、QoS生产端实现未随现工作区找到。
- `ranger_go2_codex_plan.md`、`go2_codex_implementation_plan.md` 未在上述明确候选位置找到；没有新公共RobotAdapter接口或Go2 coordinator/RulePlanner实施交接记录。

结论：**尚未取得对端实现证据**。本轮说明中的 `multi_agent_task_msgs`、AgentState/Observation/Command是设计约定，不是本机已安装事实；不新建另一套公共消息，不猜字段类型、enum或函数签名。后续实现前取得公共源/commit、真实.msg/IDL、RobotAdapter和agent factory接口、topic/QoS及去重/重传/许可语义。旧multi_agent_msgs的里程计state不是READY。

### 6.2 实际本机半程配置

| 项目 | 静态事实 | 采用依据/差距 |
|---|---|---|
| 本地Domain | `start_ranger_agent.sh:10` 显式0；common.sh和单机脚本不设置域 | 本机现有文件支持0，不采用早期30/31；既有进程实际env未检查 |
| 共享Domain | `multi_agent_bridge_ranger.yaml:2–3` from0→to40 | 与本次较新Domain0↔40要求一致；不能据此证明Go2半程 |
| Ranger旧state | `/multi_agent/agents/ranger/state` →40；`multi_agent_msgs/msg/AgentState` | keep_last depth1、best_effort、volatile |
| Go2旧state | `/multi_agent/agents/go2/state` 40→0（reversed:true）同类型/QoS | 仅两个旧状态topic，无Command/Observation |
| 网络脚本 | `start_ranger_multi_agent_bridge.sh:4–27` 要求本机wlo1、UP、192.168.1.*；检查Wi-Fi省电 | `/sys/class/net` 本轮仅见enp5s0/lo/wlo1；没有查看链路/IP或发包。接口存在不证明bridge绑定它 |
| DDS/RMW | bridge脚本仅unset Domain/Discovery Server/FastDDS profiles；未设置RMW/Cyclone URI、接口白名单或域特定发现策略 | unset可能抹掉已有隔离，不能沿用为新桥接启动策略 |
| 当前审计shell | Domain/RMW/CYCLONEDDS_URI/Discovery Server/profile未设置；ROS_LOCALHOST_ONLY=0；AMENT_PREFIX_PATH=/opt/ros/humble；无venv/conda | 仅审计shell，非运行栈证明；未打印完整环境。bashrc/profile中未发现这些DDS赋值 |
| 安装中间件 |包索引有rmw_fastrtps_cpp，未见rmw_cyclonedds_cpp；本机CMake默认优先FastDDS | 新配置需按真实可用RMW落实，不照抄Go2 Cyclone网卡XML |
| 早期30↔40↔31 | 本机相关脚本/config/docs未找到生效配置 | 仅用户提供的历史上下文，不擅改实际域 |

若两台本地均Domain0而都能经无线发现，`/map`、TF、Nav2/速度等可**绕过bridge白名单**直接混入。现脚本没有证明已隔离。后续必须让Domain0发现范围只在本机、Domain40才进入共享网络，同时bridge本地participant能访问本机0、共享participant能访问40。`ROS_LOCALHOST_ONLY=1`套整个bridge进程可能同时屏蔽40，不是已验证解法；单独namespace也不隔离绝对话题。隔离配置能否按participant/domain选择须结合取得的bridge/RMW版本验证，在此之前禁止live跨端联通。

## 7. 环境、已有验证与阻塞阶段

ROS文件/安装：Humble（`/opt/ros/humble`）；本次shell `python3=/usr/bin/python3`、3.10.12；节点wrapper均`#!/usr/bin/env python3`，脚本未启虚拟环境。package.xml声明ament_cmake，CMake用ament_python_install_package安装Python及C++过滤器；setup.py不是主构建链。`requirements.txt`为openai>=1.40，实际SDK安装版本本轮未盘点/升级。

历史迁移清单（2026-10-08）记录无硬件145项、colcon159项0失败、launch解析与受控运动/取消停车通过；当时API和VLM自动任务仍推迟。本次用户明确原有VLM_Nav及独立地图功能已验收，是之后提供的较新背景。没有本轮新测试结果，也没有联合通信/运动通过的证据。

| 缺口 | 精确阻塞阶段 | 不阻塞的工作 |
|---|---|---|
| 新公共包/Go2 commit、schema与adapter签名 | 公共绑定实现、消息构建、联调；禁止猜出wire兼容协议 | 静态审计、Ranger几何/状态规则测试计划 |
| 指定Go2地图身份、匹配地标、SE(2)实测误差 | live跨图坐标导航、共享区域进入/退出 | synthetic转换、position_valid=false线索/状态逻辑 |
| ABC完整描述、C实例身份、门/C/等待/退出区域 | live任务调度与许可判定 | fake场景/故障测试 |
| 相机实时型号/尺寸/K/时间与安装标定 | 新协同可导航物体位置现场验收 | 复用静态RGB-D接口、无位置识别测试 |
| /odom速度反馈有效性、停止余量、底盘watchdog | 取消/替换运动、现场READY/资源释放 | 离线迟到结果/停稳无效测试 |
| 动态costmap对另一台机身效果、有效整机footprint | 双机近距离/窄门/C现场运动 | 本机单独profile准备 |
| 实际执行路径更新来源/QoS/时效 | live资源区路径交叉监护 | 用fake更新路径测试 |
| domain_bridge版本、公共QoS、DDS发现隔离 | 高层跨端消息现场验收 | 本机离线allowlist与方向检查 |

这些缺现场数据不阻止本轮报告和方案完成。后续实现任务、定向桥接、验证命令及旧单机回退见配套方案。

## 8. 交付范围复核

两份文档写入后，本轮逐一复核98个原tracked文件SHA-256，全部与写文档前一致；原有6项未暂存修改保持27增加/25删除，暂存diff仍空。`git status --short --untracked-files=all` 只额外出现本报告和配套计划两个未跟踪文件，`git diff --check` 无错误。没有修改原README、脚本、配置、Python/C++代码、地图或现有运行日志。

本轮没有新ROS/离线测试结果。产品实现、配置标定、依赖同步和现场操作均留到后续明确安排。
