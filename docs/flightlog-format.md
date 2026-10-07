# 飞行日志读取规则 (Actual.dat v2)

本文档是分析飞行日志的**唯一权威格式参考**。读日志前先看这里，不要凭记忆数列。
列序以 `Source/Core/BLController.cs` 的 `V2Columns` 常量与行写入 `String.Format` 为准；
每次加列必须同步更新本文档。

## 文件位置

- 实时日志： `<KSP>\Logs\BoosterGuidance\<船名>_<UTC时间>.Actual.dat` （分析前先快照到
  `.claude/flightlog_snapshots/`，同名会话会截断覆盖）。
- 头三行自描述： `# format v2` / `# build <md5前8位>` （跑的是哪个构建） /
  `# columns <完整列单>`。
- `#` 开头的行全是事件注释，不是数据。

## 坐标系 （最大的坑）

所有位置/速度/加速度都在**目标点切平面系**: 原点在靶心，Y 轴朝天 (Targets.SetUpTransform)。
- `y` = 相对靶心基准面的高度，**不是海拔**; 远距 (300km 量级） 会因切平面近似变负。
- 触地行 y≈-19 之类的小负数 = 船体质心低于靶心基准面，正常。
- 速度同理： `vy` 是相对该平面的垂直分量。

## 40 列清单 (awk 1-based)

| # | 列名 | 含义 / 单位 | 坑 |
|---|------|------------|-----|
| 1 | t | 日志开始起的秒数 | 非均匀采样！见下"采样率" |
| 2 | phase | Unset/TurnAround/BoostBack/Coasting/ReentryBurn/AeroDescent/LandingBurn | |
| 3-5 | x y z | 位置 m （靶心切面系） | y 见上 |
| 6-8 | vx vy vz | 速度 m/s | **水平速度 vh=sqrt(vx²+vz²) 要自己算，没有现成列** |
| 9-11 | ax ay az | **仅推力**加速度 m/s² | 熄火时恒为 0 — 即使真实气动减速很大。气动要看 25/26 列 |
| 12 | att_err | 姿态误差 deg | |
| 13-14 | amin amax | 最小/最大推力加速度 m/s² | |
| 15 | steer_gain | 随阶段变： AD-manual=滑翔角； 其他=PID kp | |
| 16 | target_error | **预测落点**误差 m （含预测噪声， 会跳） | ≠ 船到靶距离！ |
| 17 | totalMass | kg | |
| 18-19 | traj_x traj_z | Trajectories 预测落点 （切面系） | |
| 20-21 | own_x own_z | 自家仿真的预测落点 | NaN=无新鲜预测 |
| 22 | tiltDeg | 姿态与**天顶**夹角 deg | 不是迎角！不是与逆行方向夹角！ |
| 23-24 | tiltAz steerAz | 姿态/指令方位角 deg （切面系） | |
| 25 | aTotH | **总**水平加速度 （沿水平速度方向， 带符号： 负=减速） m/s² | 速度斜率+0.3s EMA; 测气动效果用它 |
| 26 | aTotV | 总垂直加速度 m/s² | 同上 |
| 27-29 | omx omy omz | 角速度 deg/s （船体系） | |
| 30 | glideSlope | atan2（水平距靶， y) deg | |
| 31-33 | aLatReq aMeasH thrFloor | vh-kill 需求/实测横向权威/油门地板 | **仅当 vh-kill floor 当拍主控时非 NaN** |
| 34 | thr | 油门 0-1 | |
| 35 | fuelKg | 可用推进剂 kg | |
| 36 | engOn | 未熄火发动机数 | |
| 37 | trajAge | Trajectories 预测年龄 s | |
| 38 | wallT | 墙钟 HH:mm:ss | 用来对齐 KSP.log 时间窗 |
| 39 | owner | 当拍转向主控 (vh-kill-floor/vh-kill-slam/AERO-cut/AD-glide/AD-cancel/AD-glide+dragbrake/term-far-return/term-falcon/term-plain...) | **仲裁**: TagOwner（油门地板/事件）无条件覆盖，TagOwnerSteer（舵律）只填空位 — 同拍双写时油门侧永远赢（`steerOwnerTick` 每拍清零， 空位回退 phase 名） |
| 40 | tags | 异常标记， 竖线分隔 | TERRJMP=预测误差跳>200m; TRAJJMP=落点跳>200m; THRJMP=油门跳>0.3; FLAMEOUT; TILT=LB段tilt>15°; OS-SNAP |

## 事件行 (# 开头， 不被采样门限过滤）

- `# phase X->Y t=.. y=.. vy=.. vh=.. dist=.. terr=.. thr=.. fuelKg=..`
  **dist=船到靶水平距离， terr=预测落点误差 — 两个不同的量**, 都只在事件行里有。
- `# engines on/off t=.. n=..`

## 采样率 （读时间序列前必看）

- y<200m: 每拍全速； y<2000m 或相位切换后 10s 内或 vh-kill 活跃或有 tag: 0.1s; 其他： 1s。
- 不要假设固定 dt; 用 t 列差分。

## KSP.log 联动

- 行格式： `[LOG HH:MM:SS.mmm] BoosterGuidance: ...`。
- 按时间窗过滤的正确姿势： 先从 Actual.dat 38 列 (wallT) 查出飞行的墙钟区间，
  再用字面前缀 `\[LOG 03:5[6-9]` 这种正则 grep KSP.log。
- 5s 节奏的频道心跳： `[AeroDescent] ADAPT CAP` （滑翔上限/实测权威）, `[AeroDescent] DRAG-BRAKE`
  （批次十七阻力刹车）, `[AeroDescent] AD-CANCEL`, `[LandingBurn] vh-kill floor` 等。

## 常用 awk 配方 （列号照抄上面表格）

```bash
F="path/to/xxx.Actual.dat"
# 相位切换
grep "^# phase" "$F"
# 主控占有统计
awk 'NR>1 && $1!~/^#/ {print $39}' "$F" | sort | uniq -c | sort -rn
# 时间序列采样 (例: LB 段每 ~1s: t vh vy y terr thr owner)
awk 'NR>1 && $2=="LandingBurn" {vh=sqrt($6*$6+$8*$8);
     printf "%s vh=%.0f vy=%.0f y=%.0f terr=%.0f thr=%s own=%s\n", $1,vh,$7,$4,$16,$34,$39}' "$F"
```

## Simulate.dat

多run格式， 与 Actual.dat **不同框不同语义** （其 y 不是高度， 加速度列恒 0);
来源是被丢弃的 LogSimulation 重跑， 不是红十字预测。分析前先读
`Source/Core/Simulate.cs` 的写入处确认列义。
