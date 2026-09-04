# xArm6 GELLO Teleoperation

一个与 Evo-RL/LeRobot 运行时代码完全独立的单臂遥操作项目：复用现有 Dynamixel GELLO（6 个关节编码器 + 现有夹爪电机），控制带**标准 xArm Gripper** 的 xArm6。

> 安全边界：本项目不替代实体急停、围栏、碰撞检测或厂商的安全评估。首次运行时清空工作空间、降低速度、保持实体急停可达，并只在低风险 P0 姿态附近验证。

## 当前范围

Phase A 先实现关节空间遥操作：GELLO 的相对编码器增量经过每关节标定，生成 xArm6 的 6 轴弧度目标；夹爪以单独的归一化通道控制。连接不会移动机械臂，必须显式执行 `arm` 后才允许流式命令。

Phase B 仅更换 leader 的打印机械结构：仍用同一套 Dynamixel、串口板、电源和软件接口，做 6 旋转关节、轴序和可操作姿态接近 xArm6 的被动 leader。它不是 xArm6 外形或 CAD 的复刻。

## 安装

```bash
cd /home/aaron/workspace/evo-rl-chaozhi/xarm6-gello-teleop
python -m venv .venv
.venv/bin/pip install -e '.[dev,recording]'
```

## 代码仓库与 GitHub 推送

本项目在目录内维护独立 Git 仓库，远端为 [jackaaron01/xarm6-gello-teleop](https://github.com/jackaaron01/xarm6-gello-teleop)。首次通过 HTTPS 推送时，需要 GitHub Personal Access Token（PAT），不能使用 GitHub 账户登录密码。

创建 Fine-grained PAT 时，选择资源所有者 `jackaaron01`，并仅授权仓库 `xarm6-gello-teleop`；在 **Repository permissions** 中，`Contents: Read and write` 已足够执行 `git push`，`Metadata: Read-only` 会自动包含。不要把 Token 保存到项目文件或提交到 Git。

```bash
cd /home/aaron/workspace/evo-rl-chaozhi/xarm6-gello-teleop
git push -u origin main
```

提示用户名时输入 `jackaaron01`，提示密码时粘贴 PAT。若此前输过错误 Token，可先清理 GitHub 的缓存凭据：

```bash
git credential-cache exit
```

```bash
printf 'protocol=https\nhost=github.com\nusername=jackaaron01\n\n' | git credential reject
```

长期使用建议配置 SSH key 后将远端切换为 SSH；这样日常 `git push` 不需要重复输入 Token。

编辑硬件文件；不要把示例串口、IP、ID 直接用于实机：

```text
configs/hardware/gello_ids_1_7.yaml
configs/hardware/xarm6_standard_gripper.yaml
configs/calibration/gello_to_xarm6.example.yaml
```

公开仓库中的 `gello_ids_1_7.yaml` 是示例文件。首次配置本机时，复制出被 Git 忽略的本地 profile，再填入真实稳定串口路径；后续实机命令将 `--leader` 改为这个 `.local.yaml` 文件。

```bash
cp configs/hardware/gello_ids_1_7.yaml configs/hardware/gello_ids_1_7.local.yaml
```

## 首次硬件流程

1. 实体急停、降低碰撞灵敏度和 xArm 的 reduced mode 先在 xArm Studio/厂商流程中确认。
2. 用 `xarm6-gello diagnose-leader ...` 核对每个 DXL ID 和关节顺序；此命令关闭 leader 扭矩，只读编码器。
3. 用 `xarm6-gello diagnose-xarm ...` 读取 xArm 状态和 6 轴角度；此命令不使能运动。
4. 完成多姿态标定，生成独立的 `configs/calibration/gello_to_xarm6.yaml`；不要使用示例标定控制真实机械臂。
5. 将 xArm 和 leader 放在安全 P0 姿态，运行 `xarm6-gello teleop ...`。程序显示预检结果后，必须在终端输入 `ARM` 才会进入 xArm servo mode。

### 只读检查 GELLO 关节顺序

在连接 xArm 或做相对标定前，运行以下脚本。它先对 1--7 号电机做三轮 ping，确认型号和扭矩状态，再依次要求手动移动一个关节，报告所有编码器的增量。它不会写入任何 Dynamixel 寄存器或运动目标。

```bash
python scripts/gello_joint_direction_check.py \
  --port <LEADER_PORT> \
  --output results/gello_joint_direction_check.json
```

重测单个关节时追加 `--joint wrist_1`（可替换为任一关节名称），例如：

```bash
python scripts/gello_joint_direction_check.py \
  --port <LEADER_PORT> \
  --joint wrist_1
```

脚本会以中文提示操作。每一步用手固定被测关节相邻的两段连杆，再仅转动该关节约 10--30°。被动串联 leader 的其它自由关节会随重力或连杆动作变化，因此脚本以**变化量最大的 ID**作为候选，而不要求其它编码器静止。对同一关节重复两次且主导 ID 一致，才接受该候选映射。目标增量的 `+/-` 是这个实际 GELLO 的 raw 正方向；它还不是 xArm 的正方向，xArm 对齐时再将其写为每关节 `sign` 标定。

### 校准 leader 扳机和标准 xArm Gripper

六轴映射确认后，再运行下列独立脚本。它对 Dynamixel ID 7 **只读**，记录松开和扣下扳机的 raw 位置；对 xArm 则需要两次输入确认，随后仅以低速 200 r/min 移动**夹爪本身**到 pulse 0 和 850。它不会使能或移动 xArm 的六个关节。

先清空夹爪周边、移开手指、保持急停可达；不要夹持工件。`--xarm-ip` 可以填写真实内网 IP，也可以填写已验证可用的路由器映射 IP，但不要加 `:502`。

```bash
.conda/bin/python scripts/calibrate_gripper.py \
  --leader-port <LEADER_PORT> \
  --xarm-ip <XARM_IP> \
  --output results/gripper_calibration.json
```

在两个端点分别按实际观察输入 `OPEN` 或 `CLOSED`；程序会输出 `raw_open/raw_closed` 和 `open_pulse/closed_pulse`，并保存 JSON。夹爪在第二端点停止，这是有意设计，避免未经确认的额外“返回”动作。

本机已确认的夹爪结果为：leader 松开/扣下分别为 `1857/2438` raw，xArm 的 `0/850` pulse 分别为闭合/张开。它们会在后续六轴多姿态标定生成正式 `gello_to_xarm6.yaml` 时写入；在此之前，不要用示例标定文件启动 `teleop`。

### P0 与六轴多姿态标定

`record_pose_pair.py` 完全只读：它读取全部 7 个 leader 编码器和当前 xArm 六轴角度，不会使能或移动 xArm/夹爪。先让被动 leader 的姿态尽量接近 xArm 当前的安全姿态，记录 P0：

```bash
.conda/bin/python scripts/record_pose_pair.py \
  --label p0 \
  --leader-port <LEADER_PORT> \
  --xarm-ip <XARM_IP> \
  --output results/pose_pairs/p0.json
```

之后通过 xArm Studio **一次只低速调整一个 xArm 关节约 15--30°**，让 leader 的同一物理关节近似跟随，保持两者静止，然后记录同名姿态对。例如 J1：

```bash
.conda/bin/python scripts/record_pose_pair.py \
  --label shoulder_pan \
  --leader-port <LEADER_PORT> \
  --xarm-ip <XARM_IP> \
  --output results/pose_pairs/shoulder_pan.json
```

对 J1--J6 各记录一个文件。注意：每次先由人在 xArm Studio 完成安全的单轴移动，采集脚本本身只读取；不要用本项目控制 xArm 移到这些姿态。

六个文件齐全后，用下列脚本计算**候选**标定（它也不连接硬件）：

```bash
.conda/bin/python scripts/fit_relative_calibration.py \
  --p0 results/pose_pairs/p0.json \
  --joint-pair shoulder_pan=results/pose_pairs/shoulder_pan.json \
  --joint-pair shoulder_lift=results/pose_pairs/shoulder_lift.json \
  --joint-pair elbow_flex=results/pose_pairs/elbow_flex.json \
  --joint-pair wrist_1=results/pose_pairs/wrist_1.json \
  --joint-pair wrist_2=results/pose_pairs/wrist_2.json \
  --joint-pair wrist_3=results/pose_pairs/wrist_3.json \
  --gripper-calibration results/gripper_calibration.json \
  --output configs/calibration/gello_to_xarm6.candidate.yaml
```

候选文件必须先经过只读映射预览验证；不要直接用于 `teleop`。

对 J3/J5 这类 leader 实际行程较小的关节，可以为同一关节提供多组姿态对；拟合脚本要求各组方向一致，并取比例的中位数。例如追加 J3/J5 的高质量重复采样：

```text
--joint-pair elbow_flex=results/pose_pairs/elbow_flex.json
--joint-pair elbow_flex=results/pose_pairs/elbow_flex_repeat.json
--joint-pair wrist_2=results/pose_pairs/wrist_2.json
--joint-pair wrist_2=results/pose_pairs/wrist_2_repeat.json
```

重复单轴采集时应使用 `--reference-p0` 与 `--target-joint`。这会在保存前验证：目标 xArm 轴确实相对 P0 移动、其它 xArm 轴没有明显偏移、leader 行程足够大；不通过的样本不会覆盖输出文件。

### 候选标定的只读映射预览

下列脚本将当前 leader/xArm 姿态设为本次 session zero；随后你仅手动移动 leader 的一个关节 5--15°。它会打印按照候选标定计算出的 xArm 目标、关节范围和 xArm 的实际角度，但**绝不会发送任何 xArm 或夹爪命令**。首次建议只测试 J1，再分别重跑脚本测试其它轴。

```bash
.conda/bin/python scripts/preview_mapping.py \
  --leader-port <LEADER_PORT> \
  --xarm-ip <XARM_IP> \
  --xarm configs/hardware/xarm6_standard_gripper.yaml \
  --calibration configs/calibration/gello_to_xarm6.candidate.yaml
```

预览采用 Phase-A **主导关节门控**：一次手动动作只让 raw 变化量最大的关节产生预测目标，其它轴即使有被动随动也会保持在 session zero。这是复用现有 passive leader 时的安全策略；更自然的多轴同时操控留待 Phase B 的低耦合 leader 机械结构。通过六轴预览后，仍需以一次低速、单关节、可中止的实机动作验证后，才允许使用 `teleop`。

候选标定默认对每轴使用 `20 raw counts` 死区：这会滤除 passive leader 未触及关节的微小随动，避免它们被误映射成 xArm 的副轴运动。它不改变大幅手动移动的方向与比例。

### 首次实机动作：单轴 J1 测试

当前复用的 passive leader 存在可观测的关节随动，因此在完成六轴验证前，禁止使用普通 `teleop`。可使用下列**单轴**命令逐轴验证：它只向所选轴发送目标，其余五轴固定在启动时的 xArm 角度，夹爪不使能且不发送命令。该命令默认以 100 Hz 流式发送小步伺服目标，默认最高变化率为 0.20 rad/s（安全上限 0.25 rad/s）；两次按 Enter 后开始，默认 10 秒自动停止，`Ctrl-C` 会立即调用 xArm stop。可通过 `--max-velocity-rad-s` 降低速度；结束时会打印实际循环频率与超时周期。

若实际频率明显低于请求频率或每周期均超时，不要继续提高请求频率。先用控制箱真实内网 IP 测试；经路由器映射的连接通常应从 `--rate-hz 40` 开始，以消除命令调用超时造成的抖动和滞后。

单轴测试若遇到控制箱的偶发长响应，会主动丢弃过期节拍并从最新 leader 位置重新开始下一周期，不会在一次阻塞后突发补发多条旧 servo 目标。

```bash
.conda/bin/python scripts/teleop_single_axis.py \
  --axis shoulder_pan \
  --leader configs/hardware/gello_ids_1_7.yaml \
  --xarm configs/hardware/xarm6_standard_gripper.yaml \
  --calibration configs/calibration/gello_to_xarm6.candidate.yaml \
  --xarm-ip <XARM_IP>
```

仅在 xArm Studio 已确认 reduced mode、工作区清空、实体急停可达时执行。测试时仅手动移动 leader J1，别碰其它关节或扳机。

标准 xArm Gripper 使用厂商 SDK 的 `set_gripper_*` API；其位置范围通常是 -100 到 850 pulse，最终以固件和实机开合方向为准，均由 `xarm6_standard_gripper.yaml` 配置。xArm 的关节流控制使用厂商所述 servo motion mode（mode 1）及 `set_servo_angle_j`。参考：[xArm Python SDK](https://github.com/xArm-Developer/xArm-Python-SDK)、[servo-j 示例](https://github.com/xArm-Developer/xArm-Python-SDK/blob/master/example/wrapper/common/7001-servo_j.py)、[标准夹爪示例](https://github.com/xArm-Developer/xArm-Python-SDK/blob/master/example/wrapper/common/5004-set_gripper.py)。

`--xarm-ip` 只填写 IP，例如 `--xarm-ip 192.168.1.100`；不要附加 `:502`。SDK 会自行使用 xArm 的默认网络通信方式，附加端口会被误判为串口路径。

## 设计约束

- 内部关节单位一律为弧度；只在 DXL 驱动边界使用 raw counts。
- leader raw、标定后的关节目标、限幅后的目标、xArm 实际反馈应分别记录；录制器会在下一小版本接入，核心接口已预留。
- 每次指令均经过关节范围、每周期最大变化和速度限制；leader 数据过期或 xArm 返回错误时进入故障状态并停止发送新目标。
- 不自动执行回零、P0 或夹爪开合。任何移动到起始姿态的功能必须作为独立、人工确认的命令实现。

更完整的两期机械与软件改造清单见 [docs/architecture.md](docs/architecture.md)。
