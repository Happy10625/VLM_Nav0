# Ranger 私有坐标协同接入

依据：`docs/ranger_handoff.md` 的 2026-10-20 覆盖节、`docs/协同VLM_Nav框架.md`。以这两份用户指定的规格实施，保留已有未提交改动，不覆盖硬件实现。

## 范围与接口

- `models.py`、`agent.py`：私有命令位置禁止携带坐标；同步组合为 action=SYNC、kind=SYNC_LOCAL_C，不增加消息字段或 action。仅本机缓存目标，线上 CACHED 清零坐标；任务切换清缓存。
- `planner.py`、`coordinator.py`：只派发任务意图；双方 READY 后找本机 C，双方缓存后按 Go2→Ranger 顺序放入/整机退出。保留原许可父命令/token。
- `go2_exploration.py`：抽出同一 `explore_steps` 生成器；原单机入口同步推进并保持 0.81 m/原取消逻辑，Runner 每 tick 推进。协同强制接近导航成功、停稳、≤1 m，并验证真实规划终点预算。
- `runner.py`：推进公共核心，VLM/规划/扫描等待期间持续通信；取消、旧任务/命令结果隔离，接入门/篮子许可与执行路径监护。
- `adapters/base.py`、`fake.py`、`nav2.py`：补齐异步容差读取、路径验收回调、全局/局部覆盖边界和 motion_guard；复用 Ranger 已有 RGB-D/TF/底盘实现。
- `frame_adapter.py`、`config.py`、`mock.py`、`agent_node.py`、factory：私有模式不依赖跨机标定，仍检查本机地图和健康；启用 Ranger 分支并传入配置与 1 m 半径。
- `config/scene.yaml`：明确 `share_c_coordinates: false`。不猜测家具标注、制动余量、网卡、门/篮子多边形或验证标志。

## 实施顺序和验证

1. 先加无 ROS 的私有模式回归：身份/坐标边界、READY、缓存、顺序、令牌、迟到结果。
2. 实现公共状态机及协调器私有分支，原共享模式测试继续通过。
3. 将本机探索抽为生成器，保留原单机测试；补充协同严格接近/终点预算/异步取消测试。
4. 接入 Runner、Ranger live factory/节点和配置门槛；使用 fake 双机执行完整任务，增加丢心跳和门退出验证。
5. 运行公共包和单机 Python 回归、shell/语法检查，记录 ROS/真机相关的未验证项。
6. 在 `/tmp/ranger-cooperative-20261020` 完成并测试后，用带原始摘要校验的脚本申请写回源码。当前可写范围仅 docs 和 /tmp。

## 已知边界

不启动机器人、bridge、远端通信或真实 API；不提交现有工作区改动。现场参数未填写/未验证时保留 live 启动门槛。

## 执行结果

全部六步已完成。新增 37 项回归通过；公共测试 382 通过、原有 13 项失败集合不变；单机测试 191 通过。隔离构建成功，独立审查发现的扫描配置和 guard 异常问题已修复并回归。经沙箱写入授权后，21 个源码/测试文件已写回并校验；原仓库联合复验 573 通过、13 项原有失败。详细记录见 `docs/multi_agent/ranger_private_c_20261020.md`。
