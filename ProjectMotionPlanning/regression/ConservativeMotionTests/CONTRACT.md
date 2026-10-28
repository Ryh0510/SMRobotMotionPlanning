# 规划保守运动模式候选契约

Owner SMRobotMotionPlanning/ProjectMotionPlanning；ai/candidate。使用真实 420-red4600-tool 项目和公共运行时。显式开启保守模式时，零预算/覆盖不完整不得变 valid；不存在机器人不得 Clear；边验证不改变已有快照关节状态；旧采样模式入口保留。授权必须在宿主机正常通过，不跳过失败。未覆盖 Windows 和完整 OMPL 性能/所有 detector 组合。Core 的真实 FK 圆弧/工具偏置/ACM 回归在 CollisionQueryTests；这里保护规划委托与错误映射。
