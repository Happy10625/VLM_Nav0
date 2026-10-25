# VLM_Nav

独立的 ROS 2 Humble 视觉语言导航包。它读取 RealSense 对齐 RGB-D，
把原始 RGB 图像发送给 OpenAI 兼容 VLM，将返回的目标/语义证据像素结合
深度和图像时刻 TF 投影到 `map`，再通过 Nav2 的路径规划与导航
动作控制 Ranger。VLM 从不直接产生速度命令。

本目录借鉴 `Co-NavGPT2` 和其中的 `co_nav2_nav`，但运行和构建不要求修改
那些源码。Livox、FAST_LIO、RealSense 和 Ranger 驱动仍由现有工作空间提供。

协作 Ranger 在现场验证标志核实后，可用以下入口完成预检、硬件/定位、健康检查、任务桥、agent及state/peer窗口启动：

```bash
bash /home/isee601/ws/VLM_Nav/scripts/start_ranger_cooperative.sh
```

只需在 RViz 核对本机地图和位姿后输入 `4`，随后自动启动或复用 Ranger 任务桥、启动 agent 并打开 state/peer。不等待 Go2 的步骤确认；双方完整入口可先后独立运行。Go2 完整入口完成双方准备采样后自动 start 一次，Ranger 不发送 start；人工分组件路径则由 Go2 采样通过后手工 start 一次。已有旧 agent 须先停稳并正常退出，预检或本机健康检查失败会停止后续启动。具体流程见[全流程启动入口](docs/双机协同启动与验收.md#21-ranger-全流程启动入口)。

场地边界使用与定位地图对齐的 Keepout 掩膜：两个 costmap 使用不按车体尺寸内缩的边界，轨迹按 footprint + padding 检查；VLM候选中心另按加 padding 后的完整旋转半径向内筛选。更新后须正常重启旧 Nav2/agent，掩膜文件和三项静止验收见[场地Keepout说明](docs/multi_agent/ranger_keepout_20261022.md)。

协同 VLM 记录保存在本机 `experiment_records_root/cooperative/<机器人>/` 下：
扫描使用 `scans/YYYY-MM-DD/000001/`，选点使用 `selections/YYYY-MM-DD/000001/`。
日期按记录生成时的本机日期计算，两类记录分别按生成顺序使用六位序号；同日多轮实验及进程重启后接着编号，次日从 `000001` 开始。
扫描目录保存 `input.jpg`、`result.png`、`request.json`、`response.txt` 和 `metadata.json`；选点目录保存地图输入图及请求、响应和元数据。
任务、决策和扫描 ID 保留在元数据中，旧哈希目录不自动搬迁。修改后正常重启 agent，新记录使用此规则。

对齐深度定位保留目标像素周围 11×11 区域内 0.20～6.00 m 的有限有效值，至少需要 8 个样本；计算中位数后，取距离中位数最近的 6 个样本求平均作为目标深度，不再使用 ±0.20 m 的一致性门槛。协同扫描识别到目标，但深度样本不足或超过可靠距离时，该次目标观察按 `UNCERTAIN` 处理。Ranger 保持原视角，包含首次最多观察三帧：每次等待停稳后的新 RGB-D 帧，重新调用 VLM 并使用该帧深度定位；任一帧取得有效目标位置就结束扫描，三次深度均失败才继续下一旋转视角（初始原地观察则继续选点探索）。重试沿用扫描超时、健康检查和取消规则，每次请求单独保存图像与结果，`metadata.json` 中的 `depth_attempt` 为本视角第几次尝试；最多增加两次 VLM 请求。Go2 沿用下一视角扫描逻辑。原始 VLM 图片与返回值仍保留，记录中的视觉 `DETECTED` 不代表已获得有效位置。

协同 Ranger 每轮新任务先把当前位置作为首次观察点 `START`（记录 ID 为 `task_start`），保持当前朝向、不旋转，等待新图像并向 VLM 请求识别；已检出但深度不可靠时按上述三帧规则重试。起点获得有效目标位置时直接进入目标接近；未检出或三帧深度均不可靠则让 VLM 选择候选观察点并前往，候选点到达后仍按原有流程环视。起点观察不要求完整旋转空间，也不占用候选点。同一任务的后续搜索指令不重复起点观察；新任务按新的起始位置重新观察。此流程修改重启 Ranger agent 后生效。

Ranger 的协同与新单机探索接近均直接将目标本机地图坐标交给 Nav2，不再生成目标周围的停靠候选。沿用旧版原生容差分工：`GridBased.tolerance=0.76 m`、位置到达容差 `0.05 m`，对应名义 `0.81 m` 停靠范围；不要将两项都设成停靠距离。目标本身允许位于物体占据区，上层检查 Nav2 返回的实际路径终点、代价地图和距离预算，不要求接近终点具有整圈旋转空间。Ranger 等待本次 Nav2 导航 `SUCCEEDED` 和真实停稳，再核对单机 ≤0.81 m / 协同 ≤1.00 m；不再以 0.89 m 距离主动取消导航，已在距离内也须完成本次原生导航。Go2 策略不变。原版独立 `VLMNavigator` 的距离取消流程保持原实现。

Ranger 路径跟踪允许同时平动和转动；平动候选的角速度上限为 0.29 rad/s，最终 `/cmd_vel` 有非零平动时要求 `abs(angular.z) < 0.3 rad/s`。原地旋转保留 0.4 rad/s 上限，扫描／恢复动作仍使用 behavior_server 的 0.25 rad/s 上限。导航层不再保留直行／旋转模式锁存或等待旧轴完全停稳的切换规则。
加减速由 Ranger 的 DWB 候选生成器负责，候选按预测轨迹接受现有 `ObstacleFootprintCritic` 检查，位置到达容差仍为 0.05 m。预测包含 Mini V3 驱动在 `abs(v/w) < 0.47644 m` 时丢弃平动、改为原地旋转的转换，因此小半径混合指令仍可能实际原地转动。
实际连接为 `controller_server / behavior_server → /cmd_vel_nav → ranger_command_guard → /cmd_vel → 底盘`，该 Ranger 配置不启动 `velocity_smoother`。guard 透传合法指令；平动角速度达到或超过 0.3 rad/s、1 s 无指令、0.5 s 无新鲜底盘 `/odom` 或控制器停用时发零速。显式停用后锁存到 ACTIVE；SIGINT/SIGTERM 退出前发零速。任务取消和轨迹碰撞检查继续生效。
导航执行中若 15 s 内移动不足 0.1 m，或 DWB 无有效局部轨迹，Ranger 会先检查剩余 180° 转动范围内的矩形 footprint；允许时原地掉头一次，再重试路径跟踪。转动期间每周期复查，角度采样间隔不超过 0.01 rad，并保持每秒重新规划路径。掉头受阻、30 s 内未完成或再次跟踪失败时停止并返回导航失败；其他错误不触发盲目掉头。FollowPath 切换到 Spin 时会先发零速，但不会立即把整个导航任务判为失败。指令中断、里程计失效、heartbeat 丢失、任务期限和取消的停车机制保留。
修改后需确认两机停稳，再正常结束并重启 Ranger R2 Nav2 以加载新控制器和 guard，核对定位后建立新任务；若尚未加载之前的深度观察修改，也需重启 Ranger agent。实际制动距离与转向效果仍需低速现场验证。

单独启动硬件、Nav2、轻量RViz和R5只读诊断终端可使用：

```bash
bash /home/isee601/ws/VLM_Nav/scripts/start_ranger_stack.sh
```

入口自动加载协同环境，从本轮配置的 `robot_ranger.yaml` 读取地图路径。启动后保留各组件终端，在RViz设置真实初始位姿，再执行下面的只读检查；已有完整实例时复用，不自动开始探索。具体组件及排障步骤见 [双机协同启动与验收](docs/双机协同启动与验收.md#4-本机硬件与定位组件终端保持运行)。

协作 Ranger 单机验收使用中文只读检查入口，完整诊断保存在日志文件中：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
bash "$REPO/scripts/check_ranger.sh" --offline       # 仅配置
bash "$REPO/scripts/check_ranger.sh" --hardware-only # 仅硬件，尚未启动Nav2时使用
bash "$REPO/scripts/check_ranger.sh"                # 配置、硬件及定位/Nav2
```

任务状态和结果使用简明诊断面板，默认自动跟随当前实验；需要单次摘要时加 `--once`：

```bash
source /home/isee601/ws/VLM_Nav/RangerVLM_env.bash
bash "$REPO/scripts/monitor_ranger.sh"
```

按当前验收阶段选择其中一条命令。入口不发送运动或VLM请求；输出每项通过/失败及处理建议，退出0表示所选范围全部通过，退出1表示有失败项。步骤见 [协作Ranger单机验收](docs/协作Ranger单机验收.md#25-配置与只读健康检查r0自动退出)。

## 数据流

```text
RGB + aligned depth + CameraInfo
        │
        ├─ 5 Hz latest-frame worker ─ VLM JSON pixels
        │                              │
        └─ depth + image-time TF ──────┘
                         │
                         ▼
                  map-frame candidates
                         │
                ComputePathToPose
                         │
                 NavigateToPose
                         │
                    Nav2 /cmd_vel

Livox → FAST_LIO → /cloud_registered_body → tf2 转到 base_link
      → PCL PassThrough → CropBox(negative) → VoxelGrid
      → /vlm_nav/obstacle_cloud
           ├→ Nav2 清除射线（逐帧）
           ├→ odom 中 XY 栅格连续 3 × 0.8 s 确认
           │    → /vlm_nav/confirmed_obstacle_cloud → Nav2 障碍标记
           └→ /scan → SLAM Toolbox → /map
```

## 安装与构建

```bash
cd /home/isee601/ws
python3 -m pip install -r VLM_Nav/requirements.txt
source /opt/ros/humble/setup.bash
colcon --log-base VLM_Nav/log build \
  --base-paths VLM_Nav \
  --build-base VLM_Nav/build \
  --install-base VLM_Nav/install
source VLM_Nav/install/setup.bash
```

启动脚本使用 `/home/isee601/ws/VLM_Nav/install`。修改源码后必须重新构建该
安装目录并重启旧的 launch 进程；启动器会在打开终端前检查
`obstacle_cloud_filter` 是否存在，避免旧 overlay 静默覆盖新版本。

API 凭据只能放在环境变量中：

```bash
export DASHSCOPE_API_KEY="sk-..."
export DASHSCOPE_WORKSPACE_ID="你的百炼业务空间ID"
export DASHSCOPE_MODEL="qwen3-vl-flash"  # 可省略，这是默认模型
# 检查
test -n "${DASHSCOPE_API_KEY:-}" && echo "API key 已设置"
```

北京地域的端点会由 `DASHSCOPE_WORKSPACE_ID` 自动组成。也可以不设置该变量，
而直接设置完整端点：

```bash
export DASHSCOPE_BASE_URL="https://你的业务空间ID.cn-beijing.maas.aliyuncs.com/compatible-mode/v1"
```

目前通过设置~/.bashrc来配置环境变量,从而不需要每次运行脚本时输入APIkey等信息.

客户端通过 OpenAI Python SDK 的 `chat.completions` 接口调用
`qwen3-vl-flash`，使用非流式 JSON 输出并显式设置
`enable_thinking=false`，避免推理内容增加导航延迟。旧的
`OPENAI_API_KEY`、`OPENAI_BASE_URL` 和 `OPENAI_MODEL` 仍保留为兼容回退。
发送给 VLM 的图片、ARM 记录图和 RViz 调试图均不绘制坐标栅格。Qwen3-VL
按照其原生 0～1000 相对坐标返回目标与语义证据位置，节点将其换算成相机图像的
实际像素后再做标注、深度查询和导航；例如 1280×720 图像最终使用
`u=0～1279`、`v=0～719`。其他兼容模型直接返回实际像素坐标。节点也兼容
Qwen 偶发返回的严格二元素坐标数组 `[u,v]`，换算后仍执行整数类型和边界校验。

相机运行后，可使用实机 RGB 帧连续测试图片约束、JSON 响应和端到端延迟：

```bash
source /opt/ros/humble/setup.bash
source /home/isee601/ws/VLM_Nav/install/setup.bash
python3 /home/isee601/ws/VLM_Nav/scripts/qwen_latency_probe.py \
  --samples 5 --timeout 8.0 --target chair --mode both
```

`--mode both` 会分别测试单图目标识别和“8 方向拼图＋候选地图”的双图前沿
选择，并独立打印成功率及延迟统计。

## 启动

### 启动总步骤

启动共三步：

- 启动硬件
- 启动建图和导航
- ARM开启实验

### 启动相机、底盘、雷达和 FAST_LIO

开各项器件使用的脚本：

```bash
VLM_Nav/scripts/01_camera.sh
VLM_Nav/scripts/02_ranger.sh
VLM_Nav/scripts/03_livox.sh
VLM_Nav/scripts/04_fastlio.sh
```

**也可以**使用严格顺序启动器。它会逐项等待话题、拒绝重复发布者并检查
FAST_LIO 初始位置，但不会 ARM 或驱动小车：

```bash
cd /home/isee601/ws
./VLM_Nav/scripts/start_hardware.sh
```

`02_ranger.sh` 在 `can0` 为 DOWN 时会请求 sudo 密码，并固定使用
`publish_odom_tf:=false`，避免与 FAST_LIO 的 `odom → base_link` 冲突。

### 启动建图、Nav2 和 VLM 导航

**一键打开**系统、RViz、状态和诊断四个终端：

```bash
cd /home/isee601/ws
unset ALL_PROXY all_proxy
./VLM_Nav/scripts/start_navigation_validation.sh "放置了可乐的椅子"
```

该脚本始终以 `enabled=false` 启动 VLM_Nav，并拒绝重复启动已有的
`/vlm_nav` 节点。

首次使用必须校准 `config/robot.yaml` 中的雷达/车体外参，以及
`vlm_navigation.launch.py` 中的相机静态外参。若相机驱动已经发布同一静态
TF，请使用 `publish_camera_tf:=false`，避免重复 TF 发布者。

### 检查与ARM实验

保持停车状态执行检查：

```bash
VLM_Nav/scripts/check_system.sh

# 对于TF时间戳问题，着重测试
ros2 topic hz /livox/imu
ros2 topic hz /Odometry
ros2 run tf2_ros tf2_echo camera_init body
```

该脚本除节点、话题和 TF 外，还会从实时相机取一帧发起真实 VLM 请求，
验证从请求发出到有效输出返回的端到端延迟。默认执行 1 次，最大允许延迟
为 4 秒；请求失败、没有有效输出或超出阈值都会使检查失败并禁止 ARM。

阈值和采样次数可按网络环境调整：

```bash
VLM_LATENCY_MAX_SECONDS=5 \
VLM_LATENCY_TIMEOUT_SECONDS=10 \
VLM_LATENCY_SAMPLES=3 \
VLM_Nav/scripts/check_system.sh
```

也可将自定义 RViz 配置作为第一个参数传入：

```bash
VLM_Nav/scripts/start_rviz.sh /path/to/custom.rviz
```

在 RViz 中重点确认 `/map`、`/scan`、TF、全局/局部代价地图和规划路径正常，
验证期间保持 `enabled=false`。
默认配置及 Ranger 的“固定地图轻量验证.rviz”均默认开启 Global Costmap 和 Local Costmap，分别订阅 `/global_costmap/costmap`、`/local_costmap/costmap`，以半透明代价地图配色叠加显示；已打开的 RViz 需重新加载配置。

在实体急停可用、场地封闭、人工监护并完成手动 Nav2 验证后：

```bash
VLM_Nav/scripts/arm.sh
```

停车：

```bash
VLM_Nav/scripts/stop.sh
```

任务目标只能在未使能时修改：

```bash
ros2 param set /vlm_nav enabled false
ros2 param set /vlm_nav target_description "red fire extinguisher"
ros2 param set /vlm_nav enabled true
```

## 运行接口

```bash
ros2 topic echo (接口)
```

- 状态：`/vlm_nav/state`
- 最近一次完整 VLM/直达阶段事件：`/vlm_nav/output_text`
- 地图标记：`/vlm_nav/markers`
- VLM 像素调试图：`/vlm_nav/debug_image`
- 延迟、丢帧和错误计数：`/vlm_nav/diagnostics`
- 标准 Behavior Server 代价地图：`/local_costmap/costmap_raw`
- 逐帧清除/扫描转换点云：`/vlm_nav/obstacle_cloud`
- 确认后的障碍标记点云：`/vlm_nav/confirmed_obstacle_cloud`
- 点云过滤诊断：`/diagnostics`（硬件 ID `obstacle_cloud_filter`）
- Nav2 动作：`/compute_path_to_pose`、`/navigate_to_pose`、`/spin`

地图标记显示 VLM 目标参考位置、Nav2 目标和已验证路径；不再生成或显示停靠环候选。

每次 ARM（`enabled` 从 `false` 切到 `true`）会建立一个独立的排障目录。
每次 VLM 请求完成后，节点保存发送给 VLM 的图片，并叠加目标与语义证据标记：

```text
~/.ros/vlm_nav/arm_records/arm_YYYYmmdd_HHMMSS_ffffff/
```

目标图片标记 `TARGET` 和 `EVIDENCE`；前沿请求会同时保存场景图和地图图，
地图上的绿色箭头标出
`ROBOT -> FRONTIER N`。文件名包含请求序号、请求类型和处理结论，便于把
误识别、过期结果或 API 错误与现场画面对齐。

节点只保留最近 3 次 ARM 的目录，旧目录自动删除。每个目录中的
`events.jsonl` 记录 VLM 响应和 Easy Case 的确认、对准、规划、执行与失败
事件；`/vlm_nav/output_text` 同时发布最近一次完整结果或阶段事件，便于实时
观察。
可通过 `config/robot.yaml` 的 `vlm_image_record_path` 修改保存位置，通过
`vlm_image_record_keep_arms` 修改保留次数。

前沿响应中的 `reason` 会在提示词中限制为少于 200 字符；若模型仍超出限制，
节点保留有效的前沿编号和置信度，并只把理由截断到 199 字符。
`/vlm_nav/diagnostics` 只保留相机、扫描、VLM、目标落地、导航及最终失败原因等
关键聚合状态。

默认 `start_rviz.sh` 配置已经订阅 `/vlm_nav/markers` 和
`/vlm_nav/debug_image`，并通过 `Camera RGB (live)` 面板直接显示
`/camera/color/image_raw` 实时画面。RViz 中：

- `Camera RGB (live)`：相机原始 RGB 实时画面，不依赖 VLM 是否启用；
- 红色球体和文字：VLM 识别并经 RGB-D 投影后的目标；
- 车体上方黄色文字：目标描述、导航状态、处理结论、置信度和 API 延迟；
- `VLM Annotated Image`：带目标和语义证据像素标记的原始相机图。

状态包括 `DISARMED`、`SCANNING`、`FRONTIER_SELECTING`、`EXPLORING`、
`TARGET_CONFIRMING`、`TARGET_REOBSERVING`、`TARGET_ALIGNING`、
`APPROACHING`、`APPROACH_STOPPING`、`SENSOR_WAITING`、`SUCCEEDED`、`API_ERROR` 和 `FAILED`。
连续三次 API 失败、TF 超时或 Nav2 失败都会取消运动并发送零速度。

目标像素的深度稀疏或混杂时，节点进入 `TARGET_REOBSERVING`，按照 VLM 目标
像素的方位做最多 `10°` 的小角度 Spin，停车稳定后重新获取 RGB-D 与 VLM
结果。连续三次仍不能取得可靠深度，或目标深度明确超过 `6m` 时，仅在目标尚未
可靠落地并完成三帧确认前，节点沿目标像素射线生成最远 `2m` 的主动观察
subgoal。候选点不经过占据地图吸附、footprint 或邻域启发式，最终可达性完全由
Nav2 `ComputePathToPose` 决定；到达后停车重新观测，最多执行三次中继接近。
目标参考位置一旦确认，后续深度暂时失败也不会再插入 subgoal，而是直接继续
对该参考位置进行规划。

完整扫描未找到目标后，SLAM 占据栅格只负责生成安全、可达的前沿候选。节点把
8 方向扫描拼图和带编号的前沿地图发送给 Qwen，由 Qwen 返回唯一候选编号、
置信度和理由；不再用面积/距离评分自动决定前沿。Nav2 仍有路径安全否决权。

前沿路径采用滚动闭环：Nav2 路径按弧长等距采样为 16 段，路径超过 1 米时只
执行前 8 段，然后用当前 RGB-D 和更新后的地图重新让 Qwen 选择。到达实际前沿
后重新执行 8 方向扫描。RViz 的 `VLM Frontier Map`、`VLM Scan Montage` 和
MarkerArray 会显示候选编号、VLM 选择、完整路径、已承诺半段和复评点。

`max_travel_radius` 继续约束 target probe 和 frontier 的滚动执行范围。目标接近
直接以 `target_reference_position` 调用 `ComputePathToPose` 和 `NavigateToPose`，
不再生成停靠候选，也不会把已验证路径截断成另一个终点。

到目标参考位置的距离首次达到 `0.89m` 以内时，节点先取消当前
`NavigateToPose`，进入 `APPROACH_STOPPING`。只有动作进入 `CANCELED` 或
`SUCCEEDED` 终态，并且 `/fastlio/odom` 连续 3 帧满足线速度不超过 `0.03m/s`、
角速度不超过 `0.05rad/s`，节点才重新读取距离。复核仍不超过 `0.81m` 才进入
`SUCCEEDED`；Nav2 自行返回成功也必须经过同一复核。第一次复核超界会重新规划
一次；这次重试允许继续驶入 `0.81m` 成功边界，第二次停车复核仍超界则进入
`FAILED`。

FAST_LIO 点云在进入 SLAM 和 Nav2 前只执行标准预处理：按点云时间等待
`body → base_link` TF、`pcl_ros::transformPointCloud`、PCL z PassThrough、
negative CropBox 自身剔除及 VoxelGrid。没有 XY 邻域、离群点、聚类、时序或
自定义 range 判定。range 仍由 Nav2 observation source 和
pointcloud_to_laserscan 控制。高度、CropBox 六边界与 `0.05m` voxel 都是初始值，
必须实机校准。Behavior Server 直接读取 `/local_costmap/costmap_raw`，不再复制
或改写 Nav2 costmap。

目标 approach 和 target probe 都不使用 `/map`、自定义 footprint、邻域吸附或
N 格阈值否决候选，最终几何可达性由 Nav2 决定。`/map` 仍服务 SLAM、frontier
与 free/occupied/unknown 语义；通用地图几何 helper 保留给正式地图流程使用。

### 首次实机障碍链验收

```bash
ros2 topic info -v /cloud_registered_body
ros2 run tf2_ros tf2_echo base_link body
ros2 topic hz /vlm_nav/obstacle_cloud
ros2 topic echo /diagnostics
ros2 param get /behavior_server costmap_topic
```

源 topic 的 QoS 必须与 sensor-data subscriber 兼容。`base_link ↔ body` 必须持续
存在、数值稳定并表示固定刚体关系；若缺失、漂移或异常，应停止 filter 调参，
单独处理 FAST-LIO TF 架构。第一次验证必须从新的 SLAM Toolbox 建图会话开始，
不加载或复用旧地图，避免旧 `/map` 噪声经 global StaticLayer 继续影响规划。

## 测试

不需要硬件的测试：

```bash
cd /home/isee601/ws/VLM_Nav
./scripts/test_no_hardware.sh
```

该脚本会设置 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`，避免系统自带的
`pytest 6.2.5` 与 `~/.local` 中面向新版 pytest 的 AnyIO 等插件发生冲突。
它不会卸载或修改任何系统 Python 包。

若要同时运行 Nav2 动作门控测试，先执行：

```bash
source /opt/ros/humble/setup.bash
source /home/isee601/ws/VLM_Nav/install/setup.bash
cd /home/isee601/ws/VLM_Nav
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  PYTHONPATH=.:${PYTHONPATH:-} \
  python3 -m pytest -q test
```

Humble 环境中的完整构建和 CTest/pytest 注册验证：

```bash
colcon build --base-paths VLM_Nav0 --packages-select vlm_nav --symlink-install
colcon test --packages-select vlm_nav --event-handlers console_direct+
colcon test-result --verbose
python3 -m pytest -q VLM_Nav0/test
```

后续 TODO：评估 Co-NavGPT2 的 `mask + depth → object point cloud` 目标定位方法，
用于未来替换当前单 pixel 深度定位；本阶段不迁移其 obstacle_map、FMMPlanner 或
离散运动栈。`easy_case` 历史代码清理也留到后续 legacy cleanup。

上线顺序应为：离线测试 → ROS bag/假 VLM → 实时传感器但不使能 →
手动 Nav2 → 封闭空旷区域低速 ARM。Livox 无法可靠检测台阶落差，测试区域
必须物理隔离楼梯、坑洞和平台边缘。
