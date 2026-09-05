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
- `scripts/teleop_six_axis_gated.py`：Phase-A 六轴顺序门控测试。J1--J6 均可选择，但同一时刻只允许一个 leader 轴改变目标；停住 0.35 秒后锁存该目标，再选择下一轴。夹爪始终冻结。
- `scripts/teleop_multi_axis_limited.py`：显式指定的同时多轴测试。仅 `--axes` 列出的关节会同时跟随，其它轴锁定在 session zero；夹爪冻结。`--duration-s 0` 可持续运行至 Ctrl-C；退出时以低速返回明确配置的 P0，回程时再次 Ctrl-C 可取消回程并停止。
- xArm 进入 servo mode 后会等待控制箱报告实际 mode=1，再发送第一条关节目标，避免启动阶段的 `mode: 1 (0)` SDK 警告。
- `safe` 与 `responsive` 运动档位：默认 `safe` 保持 `0.004 rad/周期`、`0.20 rad/s`；显式 `--profile responsive` 才使用 `0.005 rad/周期`、`0.25 rad/s`。两项限制必须同步提高，单独传入 `--max-velocity-rad-s 0.25` 不会绕过 `safe` 档的步长上限。
- `teleop_single_axis.py --diagnostics-output`：可选地将超过发送时延阈值的周期写入本地 JSON，包含 tick、leader raw 增量、请求目标、限幅目标和发送耗时；不增加任何硬件读取或控制命令。
- 单轴与六轴脚本的 `Ctrl-C`：信号处理只设置停止请求，不在信号回调中同步调用 xArm SDK；当前发送调用返回后，循环停止发送新目标，再在主流程中发送一次受控停止。这避免控制箱偶发长响应时在信号回调中重复阻塞。

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
- J6：方向正确。50 Hz/0.15 rad/s 时仍有可感知延迟（最大发送 186.23 ms）；提高到 0.20 rad/s 后复测实际 49.7 Hz、超时 1/498、leader 读取平均 2.05 ms、xArm servo 发送平均 1.46 ms（最大 48.43 ms），可纳入受门控保护的六轴测试。
- J6 responsive：首次复测方向正确、仅有轻微卡顿、无其它异常；当时实际 45.5 Hz、超时 9/456、xArm servo 最大 183.41 ms。随后使用长尾诊断重测，实际 50.0 Hz、零超时（0/500）、leader 读取平均 2.06 ms、xArm servo 平均 1.41 ms、最大 15.62 ms，零个超过 30 ms 的事件。长尾当前判定为偶发控制箱/链路抖动，而非 J6 固有瓶颈；响应档可进入受控六轴门控验证。
- 六轴顺序门控：J1--J6 已在同一 20 秒 session 内依次完成“选择→锁存”；实际 48.5 Hz、超时 6/971、leader 读取平均 2.06 ms、xArm servo 发送平均 2.00 ms。期间存在一次 183.84 ms 发送长尾，脚本已丢弃过期节拍；该模式目前仅作为受控顺序测试，不开放自由同时六轴跟随。
- 六轴顺序门控的操作者验收：J1--J6 方向全部正确，锁存后姿态保持，无漂移或异常动作；仅存在轻微延迟感。当前 `0.20 rad/s` 与每周期 `0.004 rad` 限制共同形成约 `0.20 rad/s` 的实际速度上限，后续响应优化必须同时调整两者并重新做小行程安全验证。
- 六轴 responsive：40 秒 session 中 J1--J6 均至少完成一次“选择→锁存”，允许同一轴在后续再次被选择；脚本正常按时退出。实际 48.3 Hz、超时 11/1933、leader 读取平均 2.05 ms、xArm servo 发送平均 2.13 ms，但最大 198.57 ms；responsive 六轴功能可用，控制箱偶发长尾仍需保守看待。该运行未触发 Ctrl-C，停止修复尚未做实机回归验证。
- 软件停止回归：responsive 六轴 session 中单次 Ctrl-C 后，脚本打印停止请求、停止发送新目标、执行 xArm stop 并正常返回终端；实际 50.0 Hz、零超时（0/618）、leader 读取平均 2.06 ms、xArm servo 发送平均 1.58 ms（最大 15.16 ms）。停止修复已通过实机验证。
- 显式同时多轴 J1+J2：方向正确，J3--J6 保持锁定，无其它异常；操作者感到轻微卡顿。实际 49.5 Hz、超时 2/496、leader 读取平均 2.07 ms、xArm servo 发送平均 1.57 ms（最大 55.47 ms）。safe 档下通过，继续验证其它关节组合。
- J3：方向修正后复测通过；50 Hz/0.15 rad/s 时实际 48.4 Hz，leader 读取平均 2.06 ms，超时 3/484。
- J5：方向修正后复测通过；50 Hz/0.15 rad/s 时实际 50.0 Hz，零超时（0/500），leader 读取平均 2.06 ms，xArm servo 发送平均 1.41 ms，最大 12.47 ms。
- 持续显式六轴回程验证（2026-09-05）：两次以全部 J1--J6 同时跟随、`safe`/50 Hz、`duration-s 0` 运行；首次 Ctrl-C 均能退出跟随、低速回到 P0 `[0,-35,-40,0,80,0]` 并正常停止。第 1 次实际 48.3 Hz、超时 13/5565、leader 读取平均 2.06 ms、xArm 发送平均 1.56 ms、最大 178.49 ms；第 2 次实际 48.4 Hz、超时 1/2421、leader 读取平均 2.06 ms、xArm 发送平均 1.44 ms、最大 25.75 ms。回程功能通过；偶发控制箱发送长尾仍存在。

## 持续六轴测试观察与待优化项

操作者在持续、较大范围的六轴联动中报告以下问题；它们是下一阶段的优化输入，不应以“单轴方向已通过”替代验证：

1. **大范围姿态偏差**：小范围时方向正确，但 leader 与 xArm 的相对位姿在大范围运动时逐渐不一致。当前映射是每轴的 session-zero 相对比例映射，采样点有限，未校正零位、比例的全行程误差或关节限位差。
2. **leader/xArm 结构差异引起的可操作性偏差**：现有复用 leader 的关节轴线、连杆长度、支架刚度、重心和副轴耦合与 xArm6 不同；手动驱动一个轴时，非目标 Dynamixel 编码器仍会变化。软件目前将六个 raw 值独立映射，无法从耦合位移中识别真正的人为意图。
3. **可感知延迟**：稳态传输并不慢（FTDI 读取约 2.06 ms、xArm SDK 发送均值约 1.44--1.56 ms），但当前控制周期是 50 Hz，且 safe 档限制为 0.004 rad/周期、0.20 rad/s；这些速度限制会产生“跟不上”的感觉。另有偶发 100 ms 以上的控制箱发送长尾，会造成真实的瞬时延迟。
4. **感觉多轴像依次而非同步执行**：软件每个周期调用一次 `set_servo_angle_j` 并发送完整的六轴目标，不存在 Python 按 J1→J6 依次下发的逻辑。因此该体感优先怀疑为 leader 机械耦合、各轴映射比例/限速不同，或 xArm 各关节自身的动态响应差异；需要记录“leader raw / 请求目标 / 限幅目标 / xArm 实际角度”的时间序列后再判断。

建议的处理顺序：先增加六轴运行日志与实际关节反馈，对大行程采集多点标定并拟合每轴的比例/零位；然后以小幅阶跃同时命令两轴，测量实际同步误差；最后再评估 80--100 Hz 或更低延迟控制接口。C++ 可能减少 Python 循环和序列化开销，但不能解决运动学不匹配、被动副轴耦合或控制箱偶发长尾，因此不是当前的第一优先级。

## 当前安全边界

1. 每次实机动作前，在 xArm Studio 确认 reduced mode、空工作区、实体急停可达。
2. 本地测试用真实 IP；远程使用路由器映射时从 `--rate-hz 40` 开始，不要直接要求 100 Hz。
3. 仅执行 `teleop_single_axis.py` 的已验证轴；不要执行普通 `teleop`。
4. 单轴测试结束后 xArm 会停在最终目标，不会自动回 P0；下一次前由 xArm Studio 低速回到安全姿态。
5. xArm 物理上是 6 个关节轴（J1--J6）加一个标准夹爪。J1--J6 已通过单轴实机验证；首次六轴联动只允许顺序主导关节门控，夹爪继续冻结。
6. `Ctrl-C` 是软件受控停止请求，不代替实体急停。持续多轴模式中，首次 Ctrl-C 会结束跟随并以 0.15 rad/s 上限回到 P0 `[0,-35,-40,0,80,0]`；回程时第二次 Ctrl-C 取消回程并停止。若 xArm 仍在运动、终端卡住或状态不明确，优先使用实体急停或 xArm Studio Stop；确认机器人停止后，再在另一终端用 `pgrep -af teleop_six_axis_gated.py` 找到进程并以 `kill -KILL <PID>` 清理卡住的 Python 进程。

## 下一步：大行程标定与同步诊断

Phase-A 的 J1--J6 单轴映射、safe/responsive 顺序门控、显式六轴同时跟随、持续运行与 Ctrl-C 后受控回 P0 均已通过实机验证。全六轴连续运行仍仅限此显式受限脚本；被动 leader 的副轴耦合会被误解为操作者意图，因此暂不启用普通全轴映射，夹爪仍冻结。

```bash
cd /home/aaron/workspace/evo-rl-chaozhi/xarm6-gello-teleop

.conda/bin/python scripts/teleop_multi_axis_limited.py \
  --axes elbow_flex,wrist_1 \
  --leader configs/hardware/gello_ids_1_7.local.yaml \
  --xarm-ip <XARM_IP> \
  --xarm configs/hardware/xarm6_standard_gripper.yaml \
  --calibration configs/calibration/gello_to_xarm6.candidate.yaml \
  --rate-hz 50 \
  --profile safe
```

下一轮不再重复方向测试，而是记录大行程六轴数据并对比 xArm 实际反馈，量化：(a) 各轴映射的比例/零位误差，(b) 非目标轴 raw 耦合，(c) 多轴实际到达时间差，以及 (d) SDK 发送长尾。结果将决定是改进软件映射、调整 leader 机械结构，还是再评估 C++/更高频率控制。

## 后续架构路线

- Phase A：复用当前 leader，采用受限、主导关节门控的低速测试；先验证可信单轴，不追求多轴同步。
- Phase B：制作低耦合、6 旋转关节、轴序与 xArm6 接近的被动 3D 打印 leader；保留现有 Dynamixel 作为编码器和现有扳机电机。
- 双臂、力反馈、自然多轴同步遥操作均在 Phase B 的机械耦合问题解决后再开展。
