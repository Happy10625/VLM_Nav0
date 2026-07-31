# VLM_Nav

独立的 ROS 2 Humble 视觉语言导航包。它读取 RealSense 对齐 RGB-D，
把带像素坐标网格的 RGB 图像发送给 OpenAI 兼容 VLM，将返回的目标/地面
像素结合深度和图像时刻 TF 投影到 `map`，再通过 Nav2 的路径规划与导航
动作控制 Ranger。VLM 从不直接产生速度命令。

本目录借鉴 `Co-NavGPT2` 和其中的 `co_nav2_nav`，但运行和构建不要求修改
那些源码。Livox、FAST_LIO、RealSense 和 Ranger 驱动仍由现有工作空间提供。

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

Livox → FAST_LIO → /cloud_registered_body → 车体尺度稀疏点过滤
      → /vlm_nav/filtered_obstacle_cloud → /scan → SLAM Toolbox → /map
```

## 安装与构建

```bash
cd /home/isee-cdh/ws
python3 -m pip install -r VLM_Nav/requirements.txt
source /opt/ros/humble/setup.bash
colcon --log-base VLM_Nav/log build \
  --base-paths VLM_Nav \
  --build-base VLM_Nav/build \
  --install-base VLM_Nav/install
source VLM_Nav/install/setup.bash
```

启动脚本使用 `/home/isee-cdh/ws/VLM_Nav/install`。修改源码后必须重新构建该
安装目录并重启旧的 launch 进程；启动器会在打开终端前检查
`sparse_obstacle_filter` 是否存在，避免旧 overlay 静默覆盖新版本。

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

客户端通过 OpenAI Python SDK 的 `chat.completions` 接口调用
`qwen3-vl-flash`，使用非流式 JSON 输出并显式设置
`enable_thinking=false`，避免推理内容增加导航延迟。旧的
`OPENAI_API_KEY`、`OPENAI_BASE_URL` 和 `OPENAI_MODEL` 仍保留为兼容回退。
Qwen3-VL 的目标和路径点使用 0～1000 归一化坐标，节点校验后转换为相机
真实像素；其他兼容模型仍使用实际像素坐标。发送图和 RViz 调试图中的坐标网格
会随模型使用相同坐标制。

相机运行后，可使用实机 RGB 帧连续测试图片约束、JSON 响应和端到端延迟：

```bash
source /opt/ros/humble/setup.bash
source /home/isee-cdh/ws/VLM_Nav/install/setup.bash
python3 /home/isee-cdh/ws/VLM_Nav/scripts/qwen_latency_probe.py \
  --samples 5 --timeout 8.0 --target chair --mode both
```

`--mode both` 会分别测试单图目标识别和“8 方向拼图＋候选地图”的双图前沿
选择，并独立打印成功率及延迟统计。

## 启动

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
cd /home/isee-cdh/ws
./VLM_Nav/scripts/start_hardware.sh
```

`02_ranger.sh` 在 `can0` 为 DOWN 时会请求 sudo 密码，并固定使用
`publish_odom_tf:=false`，避免与 FAST_LIO 的 `odom → base_link` 冲突。

### 启动建图、Nav2 和 VLM 导航

**一键打开**系统、RViz、状态和诊断四个终端：

```bash
cd /home/isee-cdh/ws
unset ALL_PROXY all_proxy
./VLM_Nav/scripts/start_navigation_validation.sh "chair"
```

该脚本始终以 `enabled=false` 启动 VLM_Nav，并拒绝重复启动已有的
`/vlm_nav` 节点。

### 椅子直达 Easy Case

完全清空且封闭的测试场地可使用独立的直达入口：

```bash
cd /home/isee-cdh/ws
unset ALL_PROXY all_proxy
./VLM_Nav/scripts/start_easy_case_validation.sh "chair"
```

该入口同样以 `enabled=false` 启动，但将 `easy_case_mode` 设为 `true`，
并使用 `config/nav2_easy_case.yaml`。全局和局部 costmap 均为滚动空白地图且
`plugins: []`，因此不会避让人、物体、墙、台阶或坑洞。Livox、FAST_LIO 和
SLAM 仍运行，只用于定位、TF 和地图观测。

ARM 前除通用检查外，应确认：

```bash
ros2 param get /vlm_nav easy_case_mode
ros2 param get /global_costmap/global_costmap plugins
ros2 param get /local_costmap/local_costmap plugins
```

预期依次为 `True`、`[]`、`[]`。Easy Case 采用
`SCANNING → TARGET_CONFIRMING → TARGET_ALIGNING → APPROACHING → SUCCEEDED`：
三帧确认后先通过 Spin 正对椅子，再忽略 VLM 中间 waypoints 和地图停靠点分类，
将车体中心导航到距椅子约 `0.81m` 的唯一停靠点，使车头距椅子约 `0.50m`。
Nav2 规划路径横向偏离直线超过 `0.10m` 时拒绝执行。完整扫描没有找到目标、
对准失败、直线路径失败、目标丢失或任务超过 `120s` 均直接进入 `FAILED`，
不会进入前沿探索。

此模式仅用于 easy case：场地必须完全清空，并物理隔离楼梯、坑洞和平台边缘；
必须保证实体急停可用并全程有人监护。`easy_case_mode` 只能在
`enabled=false` 时修改，普通的 `start_navigation_validation.sh` 不受影响。

如果还要启动Rviz:

```bash
/home/isee-cdh/ws/VLM_Nav/scripts/start_rviz.sh
```

首次使用必须校准 `config/robot.yaml` 中的雷达/车体外参，以及
`vlm_navigation.launch.py` 中的相机静态外参。若相机驱动已经发布同一静态
TF，请使用 `publish_camera_tf:=false`，避免重复 TF 发布者。

保持停车状态执行检查：

```bash
VLM_Nav/scripts/check_system.sh
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
- Spin 稀疏过滤代价地图：`/vlm_nav/behavior_costmap_raw`
- 稀疏过滤统计：`/sparse_costmap_filter/status`
- 规划用过滤点云：`/vlm_nav/filtered_obstacle_cloud`
- 障碍点过滤统计：`/sparse_obstacle_filter/status`
- Nav2 动作：`/compute_path_to_pose`、`/navigate_to_pose`、`/spin`

每次 ARM（`enabled` 从 `false` 切到 `true`）会建立一个独立的排障目录。
每次 VLM 请求完成后，节点保存发送给 VLM 的图片，并在图片上叠加 VLM
给出的路径：

```text
~/.ros/vlm_nav/arm_records/arm_YYYYmmdd_HHMMSS_ffffff/
```

目标图片中的 `W1`、`W2`、`W3` 按 VLM 返回顺序标记，黄色箭头连接到
`TARGET`；前沿请求会同时保存场景图和地图图，地图上的绿色箭头标出
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
- 绿色圆柱：VLM 给出的可行地面导航点；
- 青色连线：按近到远连接导航点并最终指向目标；
- 车体上方黄色文字：目标描述、导航状态、处理结论、置信度和 API 延迟；
- `VLM Annotated Image`：带目标像素及路径点编号的原始相机图。

状态包括 `DISARMED`、`SCANNING`、`FRONTIER_SELECTING`、`EXPLORING`、
`TARGET_CONFIRMING`、`TARGET_REOBSERVING`、`TARGET_ALIGNING`、
`APPROACHING`、`SENSOR_WAITING`、`SUCCEEDED`、`API_ERROR` 和 `FAILED`。
连续三次 API 失败、TF 超时或 Nav2 失败都会取消运动并发送零速度。滚动检查点
未重新看到目标时，节点会恢复 8 方向扫描和前沿探索，而不是立即结束任务。

目标像素的深度稀疏或混杂时，节点进入 `TARGET_REOBSERVING`，按照 VLM 目标
像素的方位做最多 `10°` 的小角度 Spin，停车稳定后重新获取 RGB-D 与 VLM
结果。连续三次仍不能取得可靠深度，或目标深度明确超过 `6m` 时，节点优先从
VLM 路径点截取 `2m` 内的中继点；若路径点也超出量程，则沿目标像素射线生成
逐渐变近的候选点。候选点必须先通过占据地图检查和 Nav2 路径验证，到达后重新
观测目标，最多执行三次中继接近。

完整扫描未找到目标后，SLAM 占据栅格只负责生成安全、可达的前沿候选。节点把
8 方向扫描拼图和带编号的前沿地图发送给 Qwen，由 Qwen 返回唯一候选编号、
置信度和理由；不再用面积/距离评分自动决定前沿。Nav2 仍有路径安全否决权。

前沿路径采用滚动闭环：Nav2 路径按弧长等距采样为 16 段，路径超过 1 米时只
执行前 8 段，然后用当前 RGB-D 和更新后的地图重新让 Qwen 选择。到达实际前沿
后重新执行 8 方向扫描。RViz 的 `VLM Frontier Map`、`VLM Scan Montage` 和
MarkerArray 会显示候选编号、VLM 选择、完整路径、已承诺半段和复评点。

`max_travel_radius` 是每次 Nav2 规划的滚动执行半径，不是整个任务相对启动点的
累计活动范围。若 Nav2 验证通过的目标接近路径或前沿路径离开本轮圆形窗口，
节点在第一次越界处插值截断，只执行圆内路径；到达检查点后等待一帧更新的 VLM
结果，再以当前位置为新圆心规划下一段。初始任务原点只保留用于累计距离诊断。

FAST_LIO 点云在进入 SLAM 和 Nav2 前先执行车体尺度稀疏过滤。默认把 XY 平面按
`0.05m` 栅格去重，并对每个障碍格统计车体大小 `0.72m × 0.50m` 窗口内的
障碍格数：少于 3 格的单点/双点返回视为噪声，3 格及以上保留为真实障碍。
SLAM、全局/局部规划器和控制器都订阅过滤后的
`/vlm_nav/filtered_obstacle_cloud`，因此噪声点不会阻止通过或目标停靠。

停靠候选还会按面向目标的实际停车朝向，用 `0.72m × 0.50m` 车体 footprint
复核占据地图。footprint 内少于 3 个占用格时仍接受该停靠区域，不再因单点或
双点噪声把停靠点标为不可靠；达到 3 格时才否决。对应参数为
`standoff_footprint_length`、`standoff_footprint_width` 和
`standoff_min_occupied_cells`。地图边界、未知区域策略和 Nav2 路径验证仍然
有效。

Spin 另外使用 `/vlm_nav/behavior_costmap_raw` 做末端兜底过滤。点云过滤参数位于
`config/robot.yaml` 的 `sparse_obstacle_filter` 段，代价地图过滤参数位于
`sparse_costmap_filter` 段。实车运行时可检查：

```bash
ros2 topic echo /sparse_costmap_filter/status
ros2 topic echo /sparse_obstacle_filter/status
ros2 topic hz /vlm_nav/filtered_obstacle_cloud
ros2 topic hz /vlm_nav/behavior_costmap_raw
ros2 param get /behavior_server costmap_topic
```

最后一条应返回 `/vlm_nav/behavior_costmap_raw`。点云状态中的
`removed_points` 表示本帧规划链路忽略的稀疏返回；代价地图状态中的
`lethal_cells_removed` 表示 Spin 兜底过滤移除的致命栅格数。

## 测试

不需要硬件的测试：

```bash
cd /home/isee-cdh/ws/VLM_Nav
./scripts/test_no_hardware.sh
```

该脚本会设置 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`，避免系统自带的
`pytest 6.2.5` 与 `~/.local` 中面向新版 pytest 的 AnyIO 等插件发生冲突。
它不会卸载或修改任何系统 Python 包。

若要同时运行 Nav2 动作门控测试，先执行：

```bash
source /opt/ros/humble/setup.bash
source /home/isee-cdh/ws/VLM_Nav/install/setup.bash
cd /home/isee-cdh/ws/VLM_Nav
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  PYTHONPATH=.:${PYTHONPATH:-} \
  python3 -m pytest -q test
```

上线顺序应为：离线测试 → ROS bag/假 VLM → 实时传感器但不使能 →
手动 Nav2 → 封闭空旷区域低速 ARM。Livox 无法可靠检测台阶落差，测试区域
必须物理隔离楼梯、坑洞和平台边缘。
