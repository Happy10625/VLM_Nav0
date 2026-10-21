# Ranger 接入状态（2026-10-15）

2026-10-17更新：按用户明确要求，Ranger默认具备运动许可，移除新增许可话题订阅、三项motion_permission配置及对应静态/在线门槛。以下2026-10-15审查和缺项记录为历史结果，涉及额外运动许可的要求已被此变更取代；定位、输入时效、动作服务、互斥和停稳检查保留。

本次变更验证：

- Ranger定向回归先RED再GREEN：25项通过；覆盖无额外订阅、无需许可字段的离线就绪，以及RGB-D、里程计、costmap、动作服务、任务互斥/终态检查。
- 公共及Ranger回归306 passed、1 skipped。运行范围为公共tests（不含 `test_dynamic_costmap_config.py`、`test_field_preparation.py`）及根包Ranger launch、arrival、geometry、navigation_gate测试。全范围尝试为362 passed、1 skipped、19 failed，涉及本机缺少Go2专用文件/地图、Go2配置预期差异及ROS上下文连带失败，不作为全量通过；未为这些测试修改Go2部署配置或伪造地图。
- 当前实验配置 `acceptance-20261015_161341/config` 仅移除三项许可字段；已有观察点、stop_margin、相机及costmap验证值保留。原CLI的 `--offline --require-ready` 已退出0，`single_robot_experiment_gates=[]`、`motion_sent=false`，记录为该目录下 `no-extra-permission-offline.log`。
- 安装为symlink overlay，原入口已直接加载更新。15秒只读复测 `platform_readiness={}`，无许可失败项；退出1来自 `localization_valid=false`（pose_age约2.657秒）、`rgbd_ready=false`、costmap的TF查询失败。记录为该目录下 `no-extra-permission-inspect.log`；未发送goal、速度或API请求。
- Python语法、验收文档bash语法及 `git diff --check` 通过。

计划：`/home/isee601/Downloads/2026-10-15-ranger-reuse-go2-framework.md`。
本机基线：`251f9fb39c25849dac00e81aec97fa5c45d98c1d`；工作分支 `ranger-reuse-go2-framework`。
公共源码：`pppppst/VLM_Nav0@eb32b4527c37cd0b278c1790f7c0bdde7bf5ad75`。
保留开始时 6 项用户修改及已有未跟踪文档，不自动提交。

## 实施记录

- 任务0：已取得两公共包；根包依赖合并中。
- 任务1：共用 Nav2/vision、薄平台 adapter。
- 任务2：共用单机入口、目标ID、平台配置。
- 任务3：local_only 身份、Ranger 地图与选点。
- 任务4：启动集成、隔离构建及离线验证。

## 实施裁决

- 现有文档已提供设计和计划，用户已授权按此实施，不重做设计审批。
- 保留绝对路径，在当前目录创建工作分支；不另建 worktree。
- 本轮 local_only 恒等变换仅表示 Ranger 自己的地图；真实 alignment 保持未验证。
- 现场观察点、语义标注和运动许可验证没有证据时保持缺项，不能造数据。

## 原入口与运行环境

旧入口 `scripts/start_fixed_map_navigation.sh --map shared_maps/lab_ranger_20261006/lab.yaml <描述>` 保留。
相机为本机 RGB / aligned_depth_to_color；TF 沿用原 launch。
map pose 来自 map→base_link；停止反馈使用底盘 /odom.twist（源码有真实反馈赋值，动态现场对照仍待验）。
ROS Humble / /usr/bin/python3；外部依赖沿用 scripts/common.sh。

## 接口交叉核对

- 任务1→任务2：factory 只接收已知 robot_id，使用同一 FrameAdapter；保留 go2.py 的类/客户端导入边界。
- 任务2→任务3：目标ID传到扫描描述、结果过滤、历史、接近及事件；Ranger local_config 深拷贝，不修改真实 alignment。
- 任务3→任务4：源码根 config/robot.yaml 提供观察参数；新 overlay 最后 source，旧 launch 默认行为保留。
- 根包合并裁决：保留 Ranger target_lost、目标参考点生命周期和 compact diagnostics；raw-depth 支持只在显式启用时加载 SDK。恢复 Ranger 原 scan_yaws（45°…360°），Go2 adapter 的四方向配置保持原样。
- 同步的 Go2 部署资源仅 robot_go2.yaml、cyclonedds_go2.xml、go2_exploration.rviz，供兼容入口/回归使用；不引入 SSH/硬件启动脚本。
- 未同步 Go2 相机 SSH 脚本，对应一条 upstream 部署测试明确 skip；没有删除公共算法回归。
- /plan 暂为候选接收话题，path_source_verified=false；保持 5 秒接收断流检查，未增加 header 1 秒门槛。
- 停车 margin 未有本机测量，配置 null；定位/相机/costmap 与底盘许可验证仍按明确缺项拒绝运动。

## 已验证（本轮）

- 原有受影响纯 Python 基线：56 passed。
- 新 adapter/本地探索关键测试先失败后通过：目标B、A/C不终止、八个真实目标方向与新帧顺序、aligned-depth disabled observer、local_only 身份、旋转/非零origin合成地图与部分ABC错误。
- 受影响回归第一次稳定结果：277 passed，1 skipped（Go2 SSH部署脚本不在本轮同步范围）；包括旧 navigation_gate/geometry/launch 回归。最终结果见后续记录。
- 隔离构建：3 packages finished；BUILD_TESTING=OFF；未运行 bridge parser / 完整 colcon test。
- 安装检查发现 upstream entry.in 无执行权限导致 symlink-install 的 CLI 不可执行；已修复模板权限并增量构建。
- 全部操作限软件：未运行硬件、coordinator、bridge、API请求或真机运动。

## 最终代码审查

独立只读 reviewer 已核对目标ID、八方向、observer、平台门槛、本地身份、取消及 launch，认可本轮单机软件范围。审查发现许可超时字段缺少有限值校验；按失效许可可能继续有效的实际影响修复：配置层和运行时均拒绝非正/非有限/布尔值，6项回归 RED→GREEN。没有遗留审查问题。

范围裁决：AgentNode 的 Ranger live 分支、真实 bridge/标定/观察点资料、原有无UUID路径关联限制仍是计划第9节/现场事项，不伪装成本轮通过。影响是当前交付可做本机软件准备和后续获授权单机实验，不能直接开展双机联动。未自动提交或合并。

## 最终交付证据

- 任务0–4的软件实现已完成；同步文件及与 Go2 基线逐文件 SHA-256/权限差异见 [ranger_sync_manifest.json](ranger_sync_manifest.json)。原先6项用户修改逐字节保持。
- 最终受影响及公共核心回归：**338 passed，1 skipped，2.04秒**；日志 `build/ranger-local/final-tests.log`。其中跳过的是本轮未同步的 Go2 相机SSH部署脚本测试。新增平台测试共22项（adapter14、本地探索8），其余为同步/原有回归；未运行现场和桥接测试。
- 最终隔离构建：**3 packages finished**，日志 `build/ranger-local/final-build.log`；PCL CMP0074 开发警告保留，无构建错误。
- 在 `/tmp` 工作目录中验证，三包导入全部来自 `build/ranger-local/install`，Python 3.10.12，实际加载生成消息 type support 成功（未用当前目录掩盖旧 overlay）。安装入口列表同时包含 Go2/Ranger CLI。
- 已运行安装后的 Ranger offline require-ready：退出 **1（预期）**，目标 **B**、frame **ranger_local_map**、motion_sent=false。具体缺项为真实运动许可来源验证、实测stop_margin、costmap来源验证、相机外参验证和Ranger观察点；没有Go2/对端门槛。
- `bash -n`、`compileall`、`git diff --check` 通过。
- 操作步骤、终端/环境/预期结果与回退见 [ranger_local_runbook.md](ranger_local_runbook.md)。语义标注仍待提供，现阶段默认 fixed_viewpoints；真实双机Agent/READY/bridge/资源联合验收未开展。
