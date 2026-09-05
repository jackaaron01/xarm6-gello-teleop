# xArm6 GELLO 遥操作进度记录

最后更新：2026-09-05

## 代码仓库与发布状态

- 项目已在本目录建立独立 Git 仓库，默认分支为 `main`；它与上级 Evo-RL 工作树相互独立。
- 当前公开基线提交：`1f49762 初始提交：xArm6 GELLO 遥操作`。
- GitHub 远端：`https://github.com/jackaaron01/xarm6-gello-teleop.git`。
- 首次 HTTPS 推送曾收到 GitHub `403 Permission denied`。本地的旧凭据缓存已清除；必须使用属于 `jackaaron01`、且对该仓库具有 `Contents: Read and write` 权限的 Fine-grained Personal Access Token（PAT）重新推送。
- PAT 绝不能写进 README、配置文件、提交历史或聊天记录。推送成功后可改用 SSH key，避免日常输入 Token。

## 已确认的硬件与网络

- 项目目录：`/home/aaron/workspace/evo-rl-chaozhi/xarm6-gello-teleop`
- leader：现有单臂 Dynamixel GELLO，USB2DXL/FT232R 串口板。
- 稳定串口：已在本机验证；真实设备路径仅保存在被 Git 忽略的 `configs/hardware/gello_ids_1_7.local.yaml`，公开配置使用示例值。
- Dynamixel：ID 1--6 为 XL330-M288（model 1200），ID 7 为 XL330-M077（model 1190）。
- leader 扭矩关闭，1--6 已为 extended position mode，ID 7 为 position mode；可作为被动 leader 读取。
- xArm6：已验证真实内网地址与路由器映射地址均可连接；具体地址不提交到公开仓库。本地开发与性能测试优先使用真实内网地址。
- xArm SDK：`xarm-python-sdk 1.18.4`；xArm 固件输出为 `v2.6.0`。

## 已确认的 leader 关节 ID

| 逻辑轴 | 物理轴 | DXL ID | 映射状态 |
|---|---|---:|---|
| `shoulder_pan` | J1 | 1 | 已确认 |
| `shoulder_lift` | J2 | 2 | 已确认 |
| `elbow_flex` | J3 | 3 | 已确认 |
| `wrist_1` | J4 | 4 | 已确认 |
| `wrist_2` | J5 | 5 | 已确认 |
| `wrist_3` | J6 | 6 | 已确认 |
| `gripper` | leader 扳机 | 7 | 已确认 |

## 夹爪标定

已由 `scripts/calibrate_gripper.py` 完成：

```text
leader 扳机：松开 raw_open=1857，扣下 raw_closed=2438
标准 xArm Gripper：pulse 0=闭合，pulse 850=张开
首次测试速度：200 r/min
```

对应配置已写入：

- `configs/hardware/xarm6_standard_gripper.yaml`
- `configs/hardware/gello_ids_1_7.yaml`

夹爪尚未接入任何实机 teleop 命令；当前单轴测试明确不使能、不发送夹爪指令。

## 标定姿态对与候选标定

P0 文件：`results/pose_pairs/p0.json`

P0 xArm 关节（rad）：

```text
J1 -0.000886
J2 -0.607182
J3 -0.690500
J4 -0.000033
J5 +1.397376
J6 +0.000012
```

当前候选文件：`configs/calibration/gello_to_xarm6.candidate.yaml`

```text
轴          sign    gain_rad_per_turn
J1          -1      7.1135
J2          +1      4.4989
J3          -1     10.4508  （2026-09-05 实机单轴方向修正，后续重点复核）
J4          -1      4.6426
J5          +1      4.3863  （2026-09-05 实机单轴方向修正，后续重点复核）
J6          -1      5.6068
```

所有轴均使用 `20 raw counts` 死区，过滤 passive leader 的小幅随动。

### 样本取舍

- J3：原始样本 `+115 raw / +0.3414 rad`，重复样本 `+160 raw / +0.3414 rad`；比例取中位数。2026-09-05 的首次实机单轴测试显示该符号与期望物理方向相反，当前候选配置已将 J3 `sign` 手动修正为 `-1`。后续需在正确期望方向下重采 J3 姿态对，确认比例是否仍合适。
- J5：旧样本仅 `-96/-124 raw` 且与大行程样本方向冲突，未纳入最新候选文件。
- J5 最新有效样本：`+327 raw / -0.3502 rad`。2026-09-05 的首次实机单轴测试显示候选方向与期望物理方向相反，当前候选配置已将 J5 `sign` 手动修正为 `+1`；后续需在正确期望方向下重采姿态对，确认比例。

## 软件已实现

- `scripts/gello_joint_direction_check.py`：中文、只读的 DXL ID/关节方向检查。
- `scripts/calibrate_gripper.py`：leader 扳机与标准 xArm Gripper 的低速端点校准。
- `scripts/record_pose_pair.py`：只读采集 leader/xArm 姿态对；支持 `--reference-p0 --target-joint` 单轴样本校验，未通过不写入输出文件。
- `scripts/fit_relative_calibration.py`：可为同一关节提供多组姿态对，方向一致时取 gain 中位数。
- `scripts/preview_mapping.py`：只读映射预览；使用 Phase-A 主导关节门控，抑制被动 leader 副轴随动。
- `scripts/teleop_single_axis.py`：首次实机单轴 servo 测试。只允许指定 xArm 轴运动，锁定其它五轴，夹爪不动作；两次 Enter 确认、自动停止、Ctrl-C 停止。
- xArm 进入 servo mode 后会等待控制箱报告实际 mode=1，再发送第一条关节目标，避免启动阶段的 `mode: 1 (0)` SDK 警告。

普通 `xarm6-gello teleop` 仍不可直接使用：它尚未接入 passive leader 的主导关节门控，不要运行。

## 延迟与性能优化

FTDI 初始 `latency_timer=16 ms` 导致 leader 读取平均约 16.7 ms。用户已使用管理员权限将：

```text
/sys/bus/usb-serial/devices/<serial-device>/latency_timer = 1
```

优化后 leader 读取约 2.05 ms。

推荐本地单轴测试档位：

```text
xArm IP：本机内网地址（不公开）
rate：50 Hz
max velocity：0.20 rad/s
```

在 J4 上已得到稳定结果：实际 49.8 Hz，超时 1/499，leader 读取平均 2.06 ms，xArm servo 发送平均 1.50 ms。

`teleop_single_axis.py` 在控制箱发生长响应后会丢弃过期节拍、从最新 leader 读取重新开始，不会突发补发旧 servo 目标。

## 已完成的实机单轴测试

- J1：可动，初始低频振动通过提高 FTDI 低延迟与平滑节拍解决。
- J2：可动，40 Hz 时实际 39.7 Hz，链路稳定。
- J4：可动，50 Hz/0.20 rad/s 时稳定通过。
- J6：能动、`diagnose-xarm` 无错误，但 servo 调用存在明显长尾（最大 457 ms、178 ms 等）；首版暂时固定 J6，不纳入实时控制。
- J3：方向修正后复测通过；50 Hz/0.15 rad/s 时实际 48.4 Hz，leader 读取平均 2.06 ms，超时 3/484。
- J5：方向修正后复测通过；50 Hz/0.15 rad/s 时实际 50.0 Hz，零超时（0/500），leader 读取平均 2.06 ms，xArm servo 发送平均 1.41 ms，最大 12.47 ms。

## 当前安全边界

1. 每次实机动作前，在 xArm Studio 确认 reduced mode、空工作区、实体急停可达。
2. 本地测试用真实 IP；远程使用路由器映射时从 `--rate-hz 40` 开始，不要直接要求 100 Hz。
3. 仅执行 `teleop_single_axis.py` 的已验证轴；不要执行普通 `teleop`。
4. 单轴测试结束后 xArm 会停在最终目标，不会自动回 P0；下一次前由 xArm Studio 低速回到安全姿态。
5. xArm 物理上是 6 个关节轴（J1--J6）加一个标准夹爪。当前 J1--J5 已通过单轴实机验证；J6 因历史 servo 长尾暂时冻结，夹爪也暂时冻结。

## 下一步：J6 延迟诊断与六轴联动准备

J1--J5 已完成单轴实机验证。下一步先诊断 J6 的历史 servo 长尾，再把它纳入受主导关节门控保护的六轴 Phase-A 联动测试；夹爪仍不接入该测试。

```bash
cd /home/aaron/workspace/evo-rl-chaozhi/xarm6-gello-teleop

.conda/bin/python scripts/teleop_single_axis.py \
  --axis wrist_3 \
  --leader configs/hardware/gello_ids_1_7.local.yaml \
  --xarm-ip <XARM_IP> \
  --xarm configs/hardware/xarm6_standard_gripper.yaml \
  --calibration configs/calibration/gello_to_xarm6.candidate.yaml \
  --rate-hz 50 \
  --max-velocity-rad-s 0.15
```

控制箱应在程序打印“单轴低速测试已开始”前完成 mode=1 确认。只有 J6 在稳定频率与可接受长尾下通过后，才开放六轴 Phase-A 联动；若方向不符或再次出现明显卡顿，立即 Ctrl-C。

## 后续架构路线

- Phase A：复用当前 leader，采用受限、主导关节门控的低速测试；先验证可信单轴，不追求多轴同步。
- Phase B：制作低耦合、6 旋转关节、轴序与 xArm6 接近的被动 3D 打印 leader；保留现有 Dynamixel 作为编码器和现有扳机电机。
- 双臂、力反馈、自然多轴同步遥操作均在 Phase B 的机械耦合问题解决后再开展。
