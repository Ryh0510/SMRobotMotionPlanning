# 外部工件与轨迹评估

## TCP、用户坐标系与外部智能体接口（新增）

完整双后端操作说明见 `D:\RWS\quick_placement\ai\RWS_BACKEND.md`。

- `tool_attachment_id` 自动读取已保存的主 TCP，组合 link→mount→asset→TCP；缺少 TCP 不默认回退法兰。此选项与手工 `tcp_link/link_from_tcp` 互斥。
- `frame` 支持 `user/world/workpiece`，缺省为 user；用户坐标系默认为 WORLD 原点并随项目保存。外部工件可传 `user_pose` 或 `world_pose`，不能同时指定。
- `control_points_only=true` 只验证原始点，禁止内部插值查询；兼容旧 JSON 调用时此选项缺省 false。新 Python `make_request` 和 RWS 智能体明确默认 true，独立原始点 gate 通过后需用户明确确认才进入插值优化。
- `return_candidates=true` 返回候选层、圈数、构型分类可用性、拒绝原因及最近距离。`continuous_path_count` 为1..30，`joint_weights` 控制加权关节变化图的排序；`ik_max_candidates_per_point` 缺省64，截断明确标记，不宣称穷尽解析根。
- `planning_status(job_id, after_sequence=N)` 返回带序号的逐点 IK 事件；这些流式候选在最终报告前尚未通过碰撞/奇异筛选。长结果支持 `planning_result(job_id, result_file=绝对路径)` 本机文件导出。
- Python `load_trajectory` 支持 `.mod/.paq/.ls`，PAQ 样本 `.txt` 也可读取；仅原始控制点，不自动插值。源程序工具/坐标必须与所选 RWS 工具和用户坐标系对应。

## 本次检查结论

`SMRobotMotionPlanning` 是运动规划核心，但原来的单点多逆解、图排序、CDF 修复与导入器并不等于完整的智能体闭环。

| 项目 | 当前能力 / 本次改动 |
| --- | --- |
| 实际模型逆解 | 新入口强制使用场景机器人 FK；不回退到默认 ABB DH。现有多逆解器限定六个转动关节，数值多起点搜索不保证穷尽 |
| 全轨迹连续分支 | 图在排序前过滤节点和连边；增加实际未折返关节角跳变、时间/速度上限、节点和线段约束回调 |
| 中间碰撞 | 新评估器对关节线性插值采样，每段至少检查一个内部点，并检查间距、奇异值、TCP 偏离 |
| 连续轴圈数 | `MotionValidationOptions.preserveJointTurns=true` 检查实际圈数；CDF 修复已启用，且不复用最短弧缓存；默认 false 保留原规划器最短弧语义；新增采样预算与非有限输入拒绝 |
| 外部 STL | 私有文档中按 id 新增/替换工件，显式 mesh scale、世界位姿；不修改打开的 GUI 工作站 |
| 外部轨迹 | C++ 内存结构与独立 JSON CLI；显式 world/workpiece 坐标系、米/弧度/秒、TCP link 与工具变换 |
| 碰撞配置 | 使用指定检测器，要求有效检测对；CLI 可按显式机器人 link 列表创建工件检测对；不暗中启用关闭的全局碰撞或机器人 |
| 智能体反馈 | `candidate_counts`、拒绝状态/边数量、失败类型与首次失败点/段、`retry_workpiece_pose`；搜索失败不宣称已证明不可达 |
| 通用避障优化 | `optimize_avoidance=true` 接入多检测器 CDF 修复。显式 `detectorIds` 通用模式不创建 burnner、不覆盖原检测对；旧默认仅供历史流程兼容 |
| 位姿自动搜索 | `pose_search` 在位置/姿态硬边界内搜索；支持标定后的 UF2 Y 轴竖直约束、候选数量、超时与取消。每个候选执行 IK→连续路径→避障修复→独立复检，失败后继续下一个位姿 |
| GUI 当前场景 | `planning_submit` 在 GUI 线程捕获未保存的关节、基座、工件位姿和配置；计算在独立后台场景进行。提供状态、结果、取消、释放命令；不自动应用结果 |

## 调用

核心接口：`ProjectMotionPlanning/ExternalTrajectoryEvaluation.h`，同步调用 `ProjectExternalTrajectoryEvaluator::evaluate(document, basePath, request)`。
每次使用独立场景，适合后台任务。不要放在 GUI 线程的 10/20 ms 控制循环中。

构建与命令行：

```powershell
cmake --build D:\RWS\build --config Release --target RwsTrajectoryEvaluate
D:\RWS\build\SceneTools\bin\RwsTrajectoryEvaluate.exe station.sys.json request.json result.json
```

退出码：0 评估通过，1 评估未通过，2 输入/运行错误。结果写入 `result.json`。
CLI 读取保存的项目文件。读取 GUI 未保存状态请使用下面的异步接口，二者共用 JSON 输入与规划实现。

Python 入口位于 `D:\RWS\quick_placement\rwssdk\planning.py`：

```python
from rwssdk import evaluate_trajectory
report = evaluate_trajectory(
    r"D:\RWS\RWS\data\scene\fanuc.sys.json",
    r"D:\jobs\request.json",
    timeout=120,
)
if report["success"]:
    joint_points = report["joint_points"]
elif report.get("retry_workpiece_pose"):
    # 智能体在既定约束范围内生成下一工件位姿，再评估。
    pass
```

也可传 Python dict；STL 相对路径以 Python 当前工作目录为基准，传请求文件则以请求文件目录为基准。
这不是实时关节控制接口，也不向活动机器人发送返回的轨迹。

## JSON 请求 v1

以下位置/姿态仅说明格式，需要替换成实际工艺点和模型 link 名称：

```json
{
  "version": 1,
  "robot_id": "m710ic_50_drake",
  "joint_names": ["J1", "J2", "J3", "J4", "J5", "J6"],
  "seed_joints_rad": [0, 0, 0, 0, 0, 0],
  "tcp_link": "填写实际末端link",
  "link_from_tcp": {"position_m": [0, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]},
  "detector_ids": [],
  "workpiece_detector_id": "external_robot_part",
  "workpiece_collision_links": ["填写需要检测的实际link"],
  "workpiece": {
    "id": "external_part",
    "stl_path": "part.stl",
    "mesh_scale_to_m": 0.001,
    "world_pose": {"position_m": [1, 0, 0.5], "quaternion_xyzw": [0, 0, 0, 1]}
  },
  "frame": "workpiece",
  "workpiece_id": "external_part",
  "points": [
    {"time_s": 0, "position_m": [0, 0, 0.1], "quaternion_xyzw": [0, 0, 0, 1]},
    {"time_s": 1, "position_m": [0.01, 0, 0.1], "quaternion_xyzw": [0, 0, 0, 1]}
  ],
  "minimum_clearance_m": 0.01,
  "validation_joint_step_rad": 0.02,
  "max_joint_delta_rad": 0.5,
  "minimum_singular_value": 0.0001,
  "jacobian_length_scale_m": 1,
  "max_tcp_position_deviation_m": 0.001,
  "max_tcp_orientation_deviation_rad": 0.01
}
```

可选：`search_lower_rad` / `search_upper_rad`（连续轴必须提供有限搜索窗）、`max_joint_velocities_rad_s`（不超过模型已知上限）、`ik_seed_count`、`ik_max_iterations`。
未提供速度上限时使用模型速度限制；缺失时拒绝评估，不悄悄视为无限速。
`detector_ids` 可追加已有的自碰撞、夹具、工具和环境检测器；仅创建机器人—工件检测对不覆盖这些关系。
`mesh_scale_to_m` 只影响网格，不缩放已按米提供的轨迹或工件世界平移。
workpiece 坐标系指 STL 本地坐标系；如工艺使用 UF2，调用前必须应用 STL→UF2 的标定变换，不可直接混用。

## 边界与后续工作

- `success` 表示所返回路径通过配置的采样几何/运动学约束，不是控制器可执行证书。加速度、jerk、驱动器动力学和控制器插补还需检查与时间参数化。
- 采样可能漏掉采样间的细小碰撞或奇异区间，不等于连续碰撞检测。减小步长提高覆盖率但不能提供数学上的连续保证。
- 图连边是关节线性插补；期望工艺路径是笛卡尔位置线性与姿态 slerp。稀疏工艺点可能需要加密，不能通过放宽偏离阈值掩盖工艺不一致。
- 当前只评价工艺轨迹本身，不包含当前关节状态到首点、末点到安全位的进退路径。
- `ProjectExternalTrajectoryEvaluator` 是单个位姿评估/优化层；`ProjectExternalWorkpiecePlanner` 是受约束位姿搜索层。未启用 `pose_search` 时不会搜索新位姿。
- 位姿搜索是有限候选搜索，避障是局部优化，不保证找到全局解。优化后额外复检全部请求检测器、奇异性、关节速度和按原时间对应的 TCP 位置/姿态；未通过不发布优化轨迹。
- 优化不改变工艺时间；若原定时间过紧，结果可被速度检查拒绝。尚未提供带工艺时间重映射的联合时间优化。

## 异步读取 GUI 未保存场景

新程序 `D:\RWS\build\SceneTools\bin\RobotQtViewerToolFix.exe` 使用 `--rws-sdk-dir` 启用桥接。已运行的旧程序不会动态加载本次新代码；不要与另一个实例共用同一个通信目录。

```python
import time
from rwssdk import FastClient

with FastClient(r"D:\RWS\sdk-channel-planning") as rws:
    started = rws.submit_planning(r"D:\jobs\request.json")
    job_id = started["job_id"]
    # started 包含 snapshot_id / snapshot_robots / snapshot_objects，可核对实际捕获的状态。
    while True:
        state = rws.planning_status(job_id)
        if state["done"]:
            report = rws.planning_result(job_id)["report"]
            break
        time.sleep(0.1)
    rws.forget_planning(job_id)
```

`cancel_planning(job_id)` 请求停止，仍需等待 `done=true`，再获取取消报告或释放任务。
一次允许一个活动任务，保留最多 8 个已完成任务；用 `forget_planning` 释放记录。关闭桥接时请求取消并等待工作线程退出。
任务开始后的 GUI 改动不影响该任务的文档快照，任务也不会改变 GUI 的关节/工件状态。
`use_snapshot_seed=true` 或省略 `seed_joints_rad` 时，使用快照中的实际关节作为逆解初值；这不等于已经规划当前姿态到工艺首点的接近路径。
对象/机器人网格仍从快照所引用的文件读取，不是磁盘资产的完整版本归档。
内嵌 dict 受 64 KiB 请求上限约束；大轨迹用绝对 `request.json` 路径（文件上限 16 MiB，后台读取）。TCP 响应上限仍为 16 MiB。

## 优化和位姿搜索参数

在 v1 请求中增加：

```json
{
  "optimize_avoidance": true,
  "optimization_iterations": 3,
  "optimization_seed_paths": 3,
  "frame": "workpiece",
  "workpiece_id": "external_part",
  "use_snapshot_seed": true,
  "pose_search": {
    "enabled": true,
    "max_candidates": 24,
    "max_duration_s": 120,
    "lower_position_m": [0.8, -0.2, 0.3],
    "upper_position_m": [1.2, 0.2, 0.7],
    "lower_rpy_rad": [1.5707963267948966, 0, -3.141592653589793],
    "upper_rpy_rad": [1.5707963267948966, 0, 3.141592653589793],
    "require_y_vertical": true,
    "ground_normal_world": [0, 0, 1],
    "axis_tolerance_rad": 0.000017453292519943296,
    "workpiece_from_constraint_frame": {
      "position_m": [0, 0, 0],
      "quaternion_xyzw": [0, 0, 0, 1]
    }
  }
}
```

边界是工件/STL 世界位姿的 xyz 与 roll/pitch/yaw（米、弧度），不是增量。以上姿态范围适用于 STL 轴就是 UF2 轴的情形；实际标定不同，必须填写真实 STL-from-UF2 变换并调整工件姿态范围。
Y 轴约束检查 `world_R_workpiece * workpiece_R_UF2 * Y` 与指定地面法向的夹角，默认硬约束为 Y 轴朝 WORLD +Z，容差 0.001°。没有标定时默认单位变换，不会猜测 STL 与 UF2 的关系。
先检查当前候选，随后按确定性低差异序列生成候选；超出任一硬边界的候选不执行优化。启用位姿搜索必须将轨迹绑定到同一工件坐标系，不能把固定世界轨迹误当成随工件移动的轨迹。
结果包含 `selected_workpiece_pose`、`pose_attempts`、`poses_rejected_by_constraints`、`timed_out`。超时不标记为用户取消，也不宣称无解。

## 回归

聚焦测试开关 `-DBuildExternalTrajectoryTests=ON`，目标 `ExternalTrajectoryTests` / `ExternalIkGraphTests`。
Python `python -m unittest discover -s rwssdk/tests -v` 包括外部 JSON CLI→实际模型→STL 检测对→报告的端到端测试；CLI 未构建时该集成测试跳过。
