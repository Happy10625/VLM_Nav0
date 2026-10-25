# Ranger 场地 Keepout 接入与静止验收

本轮按 [场地多边形.md](../场地多边形.md) 的14个 `map` 坐标顶点生成 Ranger 专用掩膜；没有用候选点截图，未修改原定位地图。Go2 必须用其本机原始地图和本机框选顶点另行生成，不能复制 Ranger 的坐标或掩膜。

## 已生成的文件

- 定位地图仍为 `shared_maps/20261018161152/lab.yaml` / `lab.pgm`，摘要保持 `8b1a3f0587afd56e20431668647cf2624d83897a587090ef808424eb6049fc30`。
- [keepout_mask.yaml](../../shared_maps/20261018161152/keepout-no-inset-20261024/keepout_mask.yaml) / [keepout_mask.pgm](../../shared_maps/20261018161152/keepout-no-inset-20261024/keepout_mask.pgm)：777×681，resolution=0.05，origin=[-22.1,-24.2,0]，mode=trinary，negate=0。白色是场地允许区，黑色是禁行区；没有灰色未知像素。车体轮廓还须通过 Nav2 碰撞检查。
- 原 `keepout/` 目录和 [keepout_preview.png](../../shared_maps/20261018161152/keepout/keepout_preview.png) 保留为旧版内缩掩膜的历史证据，不代表当前配置。

2026-10-24 已将导航与候选中心的边界约束分开：导航掩膜生成器固定 `boundary_margin_m=0.0`，候选中心单独按加 padding 后的完整旋转半径内缩筛选。导航掩膜仅保留栅格离散化保护，保证整个白色单元位于原场地多边形内（当前分辨率对应约0.0354m的半对角线）。轨迹安全间距由 Nav2 的完整 footprint 和原有0.03m padding 检查；原障碍、膨胀参数和可达性检查保留。

依据：[Nav2 Keepout 教程](https://docs.nav2.org/rolling/tutorials/general_tutorials/navigation2_with_keepout_filter/navigation2_with_keepout_filter/)、[Humble KeepoutFilter 实现](https://api.nav2.org/nav2-humble/html/keepout__filter_8cpp_source.html)。本机使用 Humble 已有的接口，不使用 rolling 才有的 lethal override 等参数。

## 当前配置和启动

当前验收目录 `build/ranger-local/acceptance-20261015_161341/config/robot_ranger.yaml` 已增加：

```yaml
keepout_mask_yaml: /home/isee601/ws/VLM_Nav/shared_maps/20261018161152/keepout-no-inset-20261024/keepout_mask.yaml
keepout_mask_digest: 64e8cdff4265015cc3a671ba2d0b3197dffcea3cf66e80184c7b41f155b3b91e
```

原配置备份为同目录 `robot_ranger.before-keepout-20261022.yaml`。其他自定义 COOP_CONFIG 副本也须正确设置这两个字段；不能把旧图对应的掩膜填入新图配置。

既有完整入口命令不变（推荐，自动传递同一配置中的掩膜）：

```bash
bash /home/isee601/ws/VLM_Nav/scripts/start_ranger_cooperative.sh
```

底层启动器读取同一 robot_ranger 配置，将原地图、掩膜路径和摘要传递到 `ranger_local → system → navigation`。后者仅在启用掩膜时新增：

| 节点/参数 | 用途 |
|---|---|
| `/keepout_mask_server` | 独立 Map Server；发布 `/keepout_filter_mask`，frame=map，不替换定位 `/map` |
| `/costmap_filter_info_server` | 发布 `/costmap_filter_info`，type=0、base=0.0、multiplier=1.0，指向同一 mask topic |
| `/lifecycle_manager_keepout` | 管理上述两节点，autostart=true |
| global_costmap 和 local_costmap | 两者均追加 filters=[keepout_filter]，插件为 nav2_costmap_2d::KeepoutFilter，订阅同一 filter info |

当前Ranger的origin yaw=0。Humble Keepout内部按轴对齐栅格查询，因此启动器拒绝非零yaw的地图；Go2若遇到非零yaw，须先做其本机适配，不能手工把origin改0或直接套用本掩膜。

Go2接入时只在自己的既有导航配置上增加同样的过滤器和服务器，不替换为Ranger的地图、Nav2 profile或硬件参数。

任务 agent 在收到与固定摘要对应的**完整掩膜数据和正确过滤信息**之前，nav_ready=false。文件变化或摘要不匹配会阻塞任务；完整摘要仍走已有单worker，不回到定时器中做耗时读取。掩膜话题不加入双机bridge。

**旧 Nav2/agent 进程不会热更新。** 先按原入口暂停任务，确认真实停稳及原生动作终态，正常退出旧 agent 和旧 R2 导航进程，再加载新版本。完整启动器会拒绝复用缺少三个 Keepout 节点的旧导航栈；不要只给运行中的旧栈补三个节点来绕过此检查，因为旧 costmap 还没有启用过滤器。健康硬件可保留，重新定位按原流程核对。

## 候选生成

`GeneratedCandidates` 在原静态障碍/旋转空间检查之后，对本机导航掩膜做候选中心的旋转圆检查：完整圆盘必须位于白区，碰到黑色栅格或超出覆盖范围的候选被删除，再进入原可达性筛选、P编号、绘图和VLM请求。这等价于仅为候选点内缩允许区域，不生成或发布第二张导航掩膜，也不改写原占用图或导航掩膜；掩膜路径和图像仍纳入任务资产摘要。

候选半径与 [Nav2 的 padding 方式](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/footprint.cpp#L133-L140) 对齐，先对顶点的非零 x/y 坐标加 padding，再取外接半径。Ranger 当前为 `hypot(0.36+0.03, 0.25+0.03) ≈ 0.4801m`。当前静态候选由仅检查中心时的 **64** 个变为 **57** 个，所有候选旋转圆均留在导航掩膜白区。运行时仍会继续按规划路径及实时 costmap 筛选，这57个不是保证可达的点。已有 selections 图像不会被改写；重启后的新任务会重新生成VLM输入。

## 重制掩膜

使用同一程序读取人工点选文档，不覆盖已有文件；重制时指定新目录：

```bash
source /home/isee601/ws/VLM_Nav/Cooperative_env.bash ranger
python3 "$REPO/scripts/make_keepout_mask.py" \
  --config-dir "$COOP_CONFIG" \
  --robot ranger \
  --polygon "$REPO/docs/场地多边形.md" \
  --output-dir /实际/新掩膜目录
```

程序输出新的 keepout_mask_yaml 和 keepout_mask_digest；核对图形后更新本机配置并正常重启 Nav2/agent。保持原始图像尺寸和原YAML几何，不旋转、裁剪或缩放定位图；不手改摘要来跳过几何/来源检查。Go2 使用 --robot go2 和其本机的多边形文档、配置，保留自己的 footprint 及硬件参数。

## 三项静止验收

首次验收只用 Ranger 本机硬件/Nav2/RViz入口，暂不启动任务agent或会自动start的Go2完整编排：

```bash
source /home/isee601/ws/VLM_Nav/Cooperative_env.bash ranger
bash "$REPO/scripts/start_ranger_stack.sh"
```

先按上文正常退出旧任务和旧R2；若Go2完整入口还在等待，Ranger agent上线就可能触发它自动请求任务，因此静止验收期间不要混用两端任务完整入口。仅Nav2入口不会启动agent、bridge或请求导航。

1. **VLM输入没有场外候选。** 当前离线几何核对已通过：57个候选的完整旋转圆全部位于白区。现场重启后检查新 `experiments/cooperative/ranger/selections/<本轮决策ID>/occupancy.png`、`semantic.png`，不能拿旧图作为新版本证据。
2. **两个 costmap 都有场外禁行。** 不请求任务start，保持停稳；核对两 mask server 的 lifecycle 为 active、两个 costmap 的 filters 包含 keepout_filter。RViz 分别显示 `/global_costmap/costmap`、`/local_costmap/costmap`，可另加 `/keepout_filter_mask` 对照；local costmap 是滚动窗口，须选择窗口覆盖到场地边界的位置观察，不能要求它显示整个场地。
3. **场内两点的规划路径不越界。** 保持停稳，用显式场内 start/goal 调用 `/compute_path_to_pose` 的 ComputePathToPose，设置 use_start=true、planner_id=GridBased；这只请求路径，不能调用 NavigateToPose 或 start。在RViz核对发布到 `/plan` 的完整路径和掩膜叠加，尤其检查凹角/窄通道。无路径应记为不可达，不能为得到路径而扩大白区或关闭 Keepout。

第2项可先执行以下只读查询，再在RViz看两张costmap；掩膜Map显示使用RELIABLE/TRANSIENT_LOCAL接收静态数据：

```bash
ros2 lifecycle get /keepout_mask_server
ros2 lifecycle get /costmap_filter_info_server
ros2 param get /global_costmap/global_costmap filters
ros2 param get /local_costmap/local_costmap filters
```

第3项的只规划示例（两个点来自当前白区候选；更换地图或掩膜后须重新选点）：

```bash
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{start: {header: {frame_id: map}, pose: {position: {x: -5.575, y: -12.675}, orientation: {w: 1.0}}}, goal: {header: {frame_id: map}, pose: {position: {x: 6.425, y: 3.325}, orientation: {w: 1.0}}}, planner_id: GridBased, use_start: true}"
```

这不是导航执行命令；确认返回路径完整留在允许区，不发送NavigateToPose或任务start。

本轮没有启动 Nav2、bridge、Go2连接或真实运动；第2、3项仍需新版本进程启动后的静止验收，不能用配置检查代替现场结果。

## 2026-10-22 历史验证记录（旧版内缩掩膜）

完整相关离线测试 **400 passed，3项原生ROS节点用例按离线约束未执行**。结果以 `/tmp/ranger-keepout-final.log` 和 JUnit `/tmp/ranger-keepout.xml` 为准，测试参数保存于 `/tmp/ranger-keepout-command.txt`。静态几何核对 `/tmp/ranger-keepout-audit.json` 记录1059→57、全部点在白区、定位图摘要不变；预检 `/tmp/ranger-keepout-preflight.json` 的 live_gates=[]、motion_sent=false。现有33个协同Python模块、三个导航launch以及新增keepout模块已核对与实际overlay一致；已有进程仍须正常重启才能加载。
