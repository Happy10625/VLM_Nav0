# Ranger 可选 RoboRSI / Codex CLI 入口

本实现对应 `/home/isee601/Downloads/Ranger_RoboRSI_CodexCLI_Implementation_Plan.md` 的阶段 A。
**真实后端保持关闭；模型与真机均未验收。** 不将开关改为 true 后直接运行机器人。

## 当前版本和边界

- Ranger：`/home/isee601/ws/VLM_Nav`，分支 `ranger-reuse-go2-framework`，HEAD `a1c1b4c136b285b98732570a63ef2f1c3548eff3` 加保留的 dirty 工作树。
- 框架：`/home/isee601/ws/SenseNova-RoboRSI`，detached HEAD `0208cd72ec7c885a7219040cd2be459d127e374e`，干净源码。
- 框架 Python：`/home/isee601/ws/SenseNova-RoboRSI/.venv/bin/python`，独立 venv；ROS 执行器仍使用 `/usr/bin/python3`（3.10.12）。
- Codex：固定 `/home/isee601/.codex/packages/standalone/releases/0.166.0-x86_64-unknown-linux-musl/bin/codex`；版本行 `codex-cli 0.166.0`；SHA-256 `b8a11fedf24b2dfb59a70c9e52b030f9a508dcd49cd2dd0d41956f4c785156b8`。
- 显式模型 `gpt-6-astra`、reasoning `high`；未验证登录或模型可用性，不复制认证文件。
- 配置：`/home/isee601/ws/VLM_Nav/tools/ranger_agent/config.yaml`。`real_backend_enabled: false`、`capability_verification: null`；仅接入层配置，不含导航参数。

安装只使用框架基础包、pytest和PyYAML，没有安装上游runtime/test extra、升级系统Python/ROS或本机Codex。
框架以专用venv内的editable方式安装（`pip install -e /home/isee601/ws/SenseNova-RoboRSI pytest PyYAML`）；不是系统安装。
框架基础依赖会带入venv内的`openai-codex`及其CLI包0.163.0；执行始终指定上述独立0.166.0二进制，不使用venv中候选。
依赖范围解析结果见 `build/ranger-roborsi-validation/20261105/framework-freeze.txt`；固定框架commit不代表所有依赖版本固定。
首次pip命令继承ROS PYTHONPATH导致无关ROS依赖告警；清除PYTHONPATH/PYTHONHOME后`pip check`通过，框架导入来自专用目录。

## 两种入口

原日常入口继续独立使用，不检查框架、Codex或新配置：

```bash
cd /home/isee601/ws/VLM_Nav
bash scripts/start_ranger_single_robot.sh --allow-motion --target-id B
```

上面是未来按原现场授权执行的命令，实施期间没有执行。原硬件、定位和Nav2需预先就绪。
是否还有用户日常附加参数仍需在现场对照前确认。不要把`start_ranger_local.sh`裸调用视作同一配置。

新入口用法（阶段 B/C完成前只可查看帮助，实际调用会报告后端关闭）：

```bash
bash /home/isee601/ws/VLM_Nav/scripts/start_ranger_roborsi.sh --help
# 将来明确授权后、能力验证与配置启用完成后才运行：
bash /home/isee601/ws/VLM_Nav/scripts/start_ranger_roborsi.sh --allow-motion --target-id B
```

支持`--target`、`--config-dir`、`--timeout`、`--strategy`、`--search-hint`，不接受任意shell或原参数透传。
没有`--allow-motion`不替用户补许可。目标不指定时从所选scene读取；有效目标的前后空格、中文及符号逐字符保留。
两入口交替运行；不会替用户终止已有拥有者。现有运动锁和owner检查不变，无新增ARM/DISARM。

新入口只打印诊断日志和状态文件路径；有图形环境时尝试打开独立诊断终端，展示状态、原因、业务结果、停止与证据字段。
无图形终端时相同诊断文件仍存在，不将终端可用性变成运动门槛。诊断终端不会创建ROS节点；关闭它不取消搜索。
bootstrap失败（包含后端关闭）也写`session.json`，不会启动ROS或调用模型。

## 配置与环境

新入口使用绝对`/usr/bin/python3`准备宿主；获准启用后才source原`RangerVLM_env.bash local`。
其环境逻辑仍是`common.sh`依赖链、最后`build/ranger-local/install/setup.bash`和acceptance marker。
所选配置优先级：显式`--config-dir` > 原环境解析的RANGER_CONFIG > 原源码配置目录。
原环境会清空继承的RANGER_CONFIG，不能仅靠export强制覆盖。

当前marker：`/home/isee601/ws/VLM_Nav/build/ranger-local/current-acceptance-run.txt`，指向
`/home/isee601/ws/VLM_Nav/build/ranger-local/acceptance-20261015_161341`。
主配置四份YAML来自该目录的`config/`，观察/靠近文件为仓库`config/robot.yaml`；地图与Keepout仍是现有20261018161152路径。
新宿主固定选定目录、目标、ID和所引用配置/地图摘要，启动原脚本时显式传同一目录。
marker之后改变不会换目录；所选配置发生变化则报告不一致，不切换或回写。
没有显式传timeout/strategy时仍由原脚本/配置决定；宿主只核对有效任务期限不超过外层planner预算，不改任务期限。

ROS环境只通过内存和私有memfd交给框架宿主，读完即关闭FD；不写入manifest或模型上下文。
启动框架使用绝对venv解释器、清理过PYTHONPATH/PYTHONHOME的环境，不activate。原视觉凭据仅保留在搜索环境中。
CLI环境保留必要用户/桌面上下文，不额外继承ROS变量及视觉API凭据。
搜索恢复原环境并通过原完整脚本执行；PATH中的python3必须仍指向/usr/bin/python3。

当前关键11个Python安装文件都是源码软链接，与直接源码SHA-256一致；acceptance四份YAML是普通文件。
以后有意修改共享Python实现，两个入口重启任务后读取同一源码；新增安装项/C++变化需按原流程构建到同一`build/ranger-local/install`并重新source/重启。
acceptance配置需单独管理，不能用源码模板覆盖；Nav2配置改变需重启相应节点。没有热更新承诺，也不把本次源码摘要作为永久算法锁。

## 单次启动、结果与取消

宿主是CLI和搜索的共同父进程，搜索不是CLI后代；框架只清理CLI及其后代。
工具只有`start_find_target(target)`、`get_task_status(task_id)`、`cancel_task(task_id)`。
同session文件锁和原子已消费记录先于spawn，线程/多宿主并发最多一次；spawn失败、执行失败或宿主重启都不补发。
模型不能修改spec、目标ID、路径、期限、策略或环境。原同视角重试、深度恢复、靠近重试和RESUME_SEARCH完全保留。

**固定框架接口调整：**使用`EmbodiedAgent.start_episode(skills=[...], deferred_tools=[...], include_finish=True)`，而非字面`run()`。
指定源码`embodied_agent.py:755–804`提供可取消episode；二者共用`_run`与静态skills加载。
`run()`不提供取消句柄，不能满足“信号处理器只置标志，主循环取消planner”的要求。没有改框架源码或使用API-only runtime/skill_paths。
真实LocalToolSpec注册/分发由假Planner测试；没有用假分发器替代框架。
`action_timeout_s=30`按配置传入，但该固定版本仅在评估connector使用此字段，普通`start_episode`并不据此给本地handler设置硬超时。
本实现的start/status/cancel不等待整轮搜索；30秒取消退出观察由宿主单独实现。不能把字段值写成已经验证的工具硬时限。

SIGINT/SIGTERM/SIGHUP处理器只置位；主循环向自己启动且PID出生身份/进程组仍匹配的搜索发送一次SIGINT。
模型提前finish、框架失败/超时也走同一收尾。新增30秒是宿主退出观察上限，原取消5秒、靠近3+5秒等预算不变。
超时只记STOP_UNCONFIRMED及仍活动身份，不TERM/KILL搜索、不清锁、不重启导航、不再启动一轮。
CLI自身清理由框架完成；它可能有额外清理耗时，不能把“30秒”描述成整棵进程树必定退出时间。
宿主SIGKILL、掉电、外部服务管理器整树强杀没有停车保证；没有新增watchdog。现场部署前核实终端/服务管理器的行为。

每轮目录：`/home/isee601/ws/VLM_Nav/build/ranger-roborsi-runs/<session_id>/`：

- `manifest.json`：固定任务/配置/源码身份，不含继承环境或凭据。
- `session.json`、宿主`result.json`：原子写入的管理状态与最终摘要。
- `diagnostics.log`、`executor.log`：框架/原执行器输出；主终端只给路径。
- `search/`：原程序自行创建的result/events/视觉/选点证据，宿主不改写；不受原十份实验清理规则管理。
- `planner/`、`framework-identity.json`：框架轨迹与版本核对。

不使用旧current-single-experiment.txt指针。运行中半行JSON只表示未就绪；进程退出后才解析最终result和关联run_id/序号的SESSION_END。
业务成功要求退出0、原TARGET_APPROACHED及stop_confirmed严格true；不增加“Nav2必须SUCCEEDED”、距离或速度判据。
executor_success和evidence_status分开：REPORTED_OK仅表示原标志及最小文件检查通过，INCOMPLETE表示写失败/pending/缺损，UNKNOWN保留超时/取消视觉工作不可追踪的不确定性。
原结果符合业务成功条件、但events缺失或SESSION_END不一致时，保留executor_success与原停止报告，同时标记证据INCOMPLETE；宿主不会以完整成功退出。
已有语义输入图片允许来自会话目录外，但必须属于固定配置摘要；新生成证据仍须位于本轮search目录。
启动机会必须先成功落盘才允许spawn；已经持有搜索后，宿主状态落盘失败标记HOST_STATE_IO_FAILED，仍用内存中的身份和一次取消标记完成SIGINT收尾。
pending=false和进程退出都不是所有discard后台证据完整的独立证明；模型自称成功不参与判定。framework_error可以和原业务成功同时存在。
缺失停止字段不会被解释成未运动。晚到取消不抹去已完成成功。停止未知按原现场流程处理，不删除`/tmp/ranger-coop-acceptance-motion.lock`。

## 能力限制与后续验收

本机0.166.0接受候选`--disable shell_tool/unified_exec/code_mode/code_mode_host/multi_agent`参数；离线features查询中shell_tool、code_mode、code_mode_host、multi_agent为false，**unified_exec仍为true**。
不能将接受参数等同于已关闭执行。**没有验证实际模型工具清单或所有间接执行通道**。代码会禁用用户配置中的其他命名MCP服务；不修改CODEX_HOME或全局配置。
`real_backend_enabled=false`是方案明确要求，不能凭提示词或features列表放开。

阶段 B：在无法接触机器人、没有真实执行器的环境中，经另行授权测试真实CLI工具暴露、重复start、shell/代码/委派旁路、提前finish和清理。
验证记录须绑定resolved CLI路径、完整版本、SHA-256、框架commit、有效cli_args/overrides的规范JSON摘要以及证据文件摘要。
`capability_verification`指向该本地记录：verified=true、cli_path、cli_version、cli_sha256、framework_commit、policy_sha256、evidence_path、evidence_sha256。
这是操作者验收证据的绑定，不会自动证明记录内容的真实性；版本/二进制/有效配置变化会使记录失效。
没有生成可用于真实运行的假验证记录。

阶段 C：另获授权后确认用户日常附加参数、当前dirty小角度深度重观察行为、运行中Nav2/map_server配置与能力限制，再由操作者启用。
旧/新入口分别做同目标同配置实验；本轮不运行任何真实模型、ROS节点或机器人搜索。

## 离线验证与恢复

```bash
cd /home/isee601/ws/VLM_Nav
env -u PYTHONPATH -u PYTHONHOME PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /home/isee601/ws/SenseNova-RoboRSI/.venv/bin/python -m pytest \
  test/test_ranger_roborsi_session.py test/test_ranger_roborsi_runtime.py -q
bash -n scripts/start_ranger_roborsi.sh
```

新增测试最终为51 passed（`review-green.log`）。原回归为117 passed（`original-regression.log`），实际命令：

```bash
cd /home/isee601/ws/VLM_Nav
set +u
source build/ranger-local/install/setup.bash
PYTHONPATH="$PWD/cooperative_ws/src/multi_agent_nav:$PWD:$PYTHONPATH" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /usr/bin/python3 -m pytest test/test_ranger_single_robot_parity.py \
  test/test_ranger_depth_probe.py test/test_ranger_approach_transport.py -q
```

这些用例使用假Node/Future/客户端，无ROS节点或模型请求。完整记录在`build/ranger-roborsi-validation/20261105/`。
全局`git diff --check`原有AGENTS.md:33尾随空格仍存在；不把原dirty格式问题计为本轮修复或整体检查通过。

仓库外私有备份：`/home/isee601/ws/ranger-roborsi-backups/20261105-092309`，包含原diff、dirty文件、配置与完整摘要。
原入口从未修改，继续使用不需要Git回退。若以后需要回滚接入，只精确处理新增适配文件；禁止reset/clean掉原有工作。
