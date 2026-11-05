# Ranger / Go2 等待恢复兼容同步

授权：用户明确允许代码同步和离线测试，以及7个纯状态机文件同步前后快照通过Codex信箱共享。禁止机器人、桥接、ARM和运动；保留原dirty，不提交。

1. 保存Ranger实际工作树旧快照（已完成，/tmp/ranger-mailbox-go2-resilient-20261105/before-sync），读取Go2实际commit/工作树并记录SHA，不按分支名假定已同步。
2. models/agent/coordinator统一显式软原因、hard_fault优先、TaskClock预算；默认开关false，Ranger私有任务profile/scene启用并完整接线，真实健康/动作期限不冻结。
3. runner/exploration区分任务预算与单动作时钟；保留任务/成功命令/目标和Ranger扫描checkpoint、三次旋转计数。Nav2恢复失败不丢检查点，已收到hard状态撤销恢复。
4. events/最终结果等必要证据失败禁止完整成功验收，展示失败不影响取消/停车；不宣称缺失记录自动补齐。
5. 先失败测试、定向回归、双方实际新旧状态机回放；证据记录和核心状态机独立实施审查，汇总实际加载位置和剩余异步传播窗口。无wire schema变化、无Sport移植、无新增硬件门槛。
6. 发新快照给Go2并确认回放结论，更新交接。仅声称离线已验证，不声称现场或全局原子恢复通过。

完成状态：实现、限定离线回归、双方实际版本互验及交接已完成。Ranger v2 11/11；Go2报告v3 17/17+新反例3/3。707通过、8原基线失败、1现场初始化测试剔除。未真机验收；详情见session_handoff与diagnostics/go2-wait-compatibility-20261105。
