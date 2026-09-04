# 架构与两期实施清单

## Phase A：先复用、先成功

硬件保留：现有单臂 Dynamixel GELLO 的 6 个关节电机、夹爪电机、Dynamixel 总线、电源、线缆和相机；follower 是现有 xArm6 与标准 xArm Gripper。新项目只读取 Dynamixel raw encoder，不导入或调用当前 Evo-RL 项目的 teleoperator、Piper、录制脚本。

运行数据流：

```text
GELLO raw counts
  -> per-joint sign / gain calibration
  -> relative target around explicit session zero
  -> joint range + speed + per-cycle-delta limiter
  -> xArm servo-j (mode 1)

GELLO gripper raw
  -> [0, 1] normalized independent channel
  -> pulse deadband / open-close mapping
  -> standard xArm Gripper
```

### 必须完成的验证顺序

1. DXL 只读：一个一个弯动关节，确认 `name -> ID -> raw 增减方向`。
2. xArm 只读：确认 IP、固件、标准夹爪已被控制器识别、无 error/warn code。
3. P0 对齐：将两边手动放到可逃逸、远离关节极限的对应姿态；运行时只记住它，不自动把 xArm 移过去。
4. 多姿态标定：每个关节至少采集 3 个小幅单关节动作，拟合 `xArm rad delta / DXL count delta` 的中位数，并人工确认符号。示例 YAML 的全 2π gain 不具备安全性。
5. 空载低速：初始限制维持 0.70 rad/s、0.035 rad/cycle、30 Hz；逐一验证 J1--J6，再验证夹爪。
6. 加入 raw recorder：每帧保存 `leader_raw`、`target_before_limit`、`target_after_limit`、`xarm_actual`、`gripper_target`、`gripper_actual`、时间戳与事件；相机视频按每 episode 单独写入。录制与数据集导出保持两个命令，避免训练格式反向约束实时控制。

默认 `xarm6_standard_gripper.yaml` 的关节硬限位来自 UFACTORY 维护的 [xArm6 URDF](https://github.com/xArm-Developer/xarm_ros/blob/master/xarm_description/urdf/xarm6/xarm6.urdf.xacro)。它们不是任务空间安全围栏：实际运行仍应在 xArm Studio/控制器中单独启用并核验 reduced mode、碰撞检测和实体急停。

## Phase B：xArm6-like 被动 leader

目标是动作映射更自然，而非复刻 xArm6 的外观或专有结构。保持六个旋转自由度，并让机械轴顺序与最终 xArm6 关节映射一致；Dynamixel 只作为带读数的编码器，不作为承力/驱动来源。

### 可复用与应重构的边界

| 保留 | 重构 / 新打印 |
| --- | --- |
| 6 个 DXL、夹爪 DXL、USB/串口板、电源、线缆、软件 `JointVector(6)+gripper` 接口 | 关节外壳、连杆、底座、承力轴/轴承座、端部手柄/触发器、线缆固定与应力释放 |
| 当前标定工具和软件 profile 结构 | xArm6-like 机械关节零位、每关节符号/倍率标定文件 |

### 机械原则

- 关节由独立钢轴/轴承承受人手力矩；DXL 输出轴只传递编码器角度，不能当作悬臂承力轴。
- 底座采用加厚板、侧装支架和宽支撑面；将重心投影保持在底座内。所有横向连杆至少留一个可加配重或安装孔位。
- 每节连杆布置封闭或半封闭线槽、最小弯曲半径和两端 strain relief；关节旁留检修口，避免重打印才能换线。
- 手部仅保留现有夹爪 DXL：新打印一个可调行程的扳机/夹爪把手，配置文件中定义 `raw_closed/raw_open`，不把它当作第七个 xArm 关节。
- 先完成单臂、无力反馈。双臂同步、力反馈、电机反驱均是独立后续项目，不能掺入 Phase A/B 验收条件。

## 可参考的开源机械资料

[GELLO mechanical](https://github.com/wuphilipp/gello_mechanical) 提供 MIT 许可的打印件/BOM，可借鉴关节壳体、走线和装配方式；其成品目标是 xArm7，不能把其 xArm7 运动学或零件直接当作 xArm6 leader。 [GELLO software](https://github.com/wuphilipp/gello_software) 和论文 [GELLO: A Generalist Low-Cost End-Effector-Less Teleoperation System](https://arxiv.org/abs/2309.13037) 可用于理解被动 leader 的总体方法。

[BiDex assembly](https://bidex-teleop.github.io/assembly) 明确列出 xArm6 GELLO 相关装配资料，是查找可打印 xArm6 adapter 的线索；下载或复用前必须在对应仓库中核验文件许可证、版本和适配尺寸。不得从 UFACTORY 的专有 xArm CAD 反向复制结构。
