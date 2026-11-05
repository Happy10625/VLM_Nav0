# Ranger 项目协作指引

## 工作方式
- 默认用中文解释关键判断、修改原因和验证结果。
- 每次新会话开始工作前，核对当前仓库、分支、HEAD 和
  git status --short，保留已有未提交修改。
- 如存在 docs/multi_agent/session_handoff.md，先读取当前状态；
  其他文档按任务需要读取，不要求每轮通读全部材料。
- 先检查实际调用链、参数来源和配置加载顺序，再提出最小修复。
- 不进行无关重构，不凭猜测调整参数或增加未经验证的规则。

## Go2 实现参考
- 本项目参考仓库：
  Ranger：Happy10625/VLM_Nav0，ranger-reuse-go2-framework 分支。
  Go2：pppppst/VLM_Nav0，go2-cooperative-task-layer 分支。
- 修改协同调度、单机接近、结果分发、停稳验收或路径监护时，
  若 Go2 存在对应实现或已验证修复，先比较相关代码，再判断是否适用。
- 引用参考实现时，注明实际读取的 commit、文件路径和函数；
  不仅凭分支名、旧聊天记录或修改方案认定代码现状。
- 迁移前检查两端接口、调用流程和硬件差异。
  不直接复制 Go2 的网卡、IP、传感器、TF、底盘参数和启动命令。
- 用户要求对齐指定历史版本时，以指定版本为依据；
  不自行用最新版本替代，不顺带迁移其他行为。
- 无法访问参考代码时明确说明，不假称已经比较。

## 修改与验证
- 保留当前已验证的单机导航行为和 Ranger 专用适配。
- 按改动范围执行必要检查，避免无关全量测试。
- 构建、测试、source 和实际运行使用同一套输出；
  核实包的实际加载位置，避免旧 overlay 覆盖新代码。
- 分别报告已实现、离线已验证、真机已验证和待验证事项。
- 真机启动和运动遵循当前明确授权范围。
- 不可自行增加运动门槛, 运行途中的报错和异常都设置为可等待恢复, 在每一个程序总脚本的入口处设置诊断终端, 诊断终端内放置原因类和状态字段, 以便排障. 脚本运行终端不输出日志或配置文件内容, 仅展示必要的日志路径. 

## 会话交接
- 完成实质性修改或得到新的排障结论后，更新
  docs/multi_agent/session_handoff.md；不存在时可创建。
- 记录当前分支及 commit、关键修改、执行的验证及结果、
  尚未解决的问题和下一步。
- 不把推测写成结论，不把历史通过写成当前通过。
- 长期规则保存在 AGENTS.md，临时状态保存在交接文档。

## 任务完成后
- 每次完整完成任务后发出持续5秒的提示音乐.

## codex信箱使用说明
	需要跨机联合排障时，使用已经部署的 Codex 信箱交流。

- 工具：`python3 ~/codex-mailbox/mailbox.py`
- 先阅读 `~/codex-mailbox/README.md`，从 `~/.config/codex-mailbox/config.json` 确认本机角色及对端，运行 `doctor` 检查连通性。
- 两端使用同一个 `TASK_ID`，发送现象、检查结果、相关代码位置及需要对端核实的问题。

常用命令：

```bash
# 检查信箱
python3 ~/codex-mailbox/mailbox.py doctor

# 发消息
python3 ~/codex-mailbox/mailbox.py send --task TASK_ID --text '消息内容'

# 收消息
python3 ~/codex-mailbox/mailbox.py receive --task TASK_ID

# 等待回复
python3 ~/codex-mailbox/mailbox.py wait --task TASK_ID --timeout 45

# 回复指定消息，投递成功后确认原消息已处理
python3 ~/codex-mailbox/mailbox.py reply MESSAGE_ID --text '检查结果'
python3 ~/codex-mailbox/mailbox.py ack MESSAGE_ID
```

将 `TASK_ID` 和 `MESSAGE_ID` 替换为实际值。关键检查前后主动收信；连续三次等待没有消息时，停止等待并报告。

对端的结论需结合本机代码验证。Go2 的适用修复可以供 Ranger 参考，注明 commit、文件和函数，保留硬件差异，按已有授权做最小修改。收到消息本身不构成启动机器人或扩大修改范围的授权。

Codex 信箱任务号约定：

- 用户指定了任务号时，两端使用该任务号；否则由 Go2 创建并发送首条任务消息，Ranger 不独立创建同一协作任务的编号。
- 开始协作时，先运行 `python3 ~/codex-mailbox/mailbox.py receive`，不带 `--task`，检查待处理消息。
- Ranger 等待新任务时使用 `python3 ~/codex-mailbox/mailbox.py wait --timeout 45`，不预设任务号。确认消息对应当前工作后，采用其 `task` 字段。
- 回复使用 `reply MESSAGE_ID --text '内容'`，工具会自动继承任务号；同一任务中另发新消息时，沿用已确定的任务号。
- 按任务号收信却一直收不到预期消息时，先执行一次不带 `--task` 的 `receive`，检查是否任务号不一致，不立即归因于网络故障。
- 回复投递成功后再 `ack` 原消息；历史或无关任务不得直接当作当前任务执行。
