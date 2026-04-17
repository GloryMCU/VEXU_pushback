# Autonomous Routine 说明

## 文件位置

- 入口绑定：`src/hardware/robots/basic_robot.cpp`
- 自动逻辑声明：`src/control/autonomous/routine.h`
- 自动逻辑实现：`src/control/autonomous/routine.cpp`

## 当前自动流程

`run_routine()` 里按顺序执行 4 段动作：

1. 前进 `600 mm`
2. 左转 `90°`
3. 前进 `467 mm`
4. 后退 `1027 mm`

对应的轮子目标圈数为：

- `600 / 212.8 = 2.820 rev`
- `467 / 212.8 = 2.195 rev`
- `1027 / 212.8 = 4.826 rev`

## 模块职责

`basic_robot.cpp` 只负责两件事：

- 把 `competition.autonomous(...)` 绑定到自动入口
- 在自动开始时调用 `robots::autonomous::run_routine(...)`

真正的运动控制细节全部放在 `control/autonomous/routine.cpp`，避免把功能实现塞进机器人装配文件。

## 命名空间

自动代码使用：

```cpp
namespace basic::hardware::robots::autonomous
```

这样可以和现有的：

- `basic::hardware`
- `basic::hardware::robots`

分层保持一致，同时避免把自动流程混进手动底盘控制接口。

## 直线控制怎么做

直线段由 `drive_distance_mm()` 负责。

核心思路：

1. 用
   `目标距离 / 212.8`
   换算出目标圈数。
2. 读取 8 个底盘电机的当前位置，作为起始参考值。
3. 循环中继续读取 8 个底盘电机的圈数。
4. 对每个电机计算
   `当前圈数 - 起始圈数`
   的绝对值。
5. 对 8 个电机取平均，作为当前已经走过的圈数。
6. 用
   `剩余圈数 * 比例系数`
   计算速度。
7. 记录这段直线开始时的 IMU 朝向，循环中持续比较当前朝向和起始朝向的偏差。
8. 用朝向偏差乘以保直比例系数，生成左右轮差速修正，让车尽量沿着起始朝向走直线。
9. 当朝向偏差很小的时候，不做修正，避免 IMU 微小抖动导致电机来回波动。
10. 速度限制在最小值和最大值之间，保证：
   - 距离远时不会太快
   - 临近目标时还能继续推进，不会太早停住
11. 到达容差后停车，并用 `hold` 刹车保持。

为什么取 8 个电机的平均值：

- 这台底盘是 8 电机驱动
- 单个电机可能有轻微偏差
- 平均值比只看某一个电机更稳

## 转向控制怎么做

转向由 `turn_deg()` 负责。
正向为右向

核心思路：

1. 开始转向前调用 `inertial.resetRotation()`
2. `turn_deg()` 约定：
   - 正角度表示左转
   - 负角度表示右转
3. 左右轮反向驱动，实现原地转向
4. 用 IMU 的 `rotation(deg)` 读取当前有符号转角
5. 用
   `目标角度 - 当前转角`
   得到转向误差
6. 用
   `误差角度 * 比例系数`
   计算转向速度
7. 远离目标时使用正常最小转速，接近目标时切到更低的靠近速度，减少来回摆动
8. 进入角度容差后立即结束本次转向
9. 如果超过预估超时时间还没进入容差，会强制退出这次转向，避免整段自动卡死

这里没有用轮子圈数直接算转角，原因是转角不仅和轮子转了多少有关，还和底盘轮距有关。当前仓库里没有单独维护这类底盘几何参数，所以直接用 IMU 更稳。

## 自动停止条件

自动逻辑每次循环都会检查：

- `competition.isEnabled()`
- `competition.isAutonomous()`

只要比赛状态退出自动阶段，动作循环就会立刻停止，不会继续往下跑。

## 当前关键参数

这些参数都在 `routine.cpp` 顶部集中定义：

- `kMillimetersPerWheelRevolution = 212.8`
  - 每转一圈对应的直线距离
- `kAutonomousLoopDelayMs = 10`
  - 自动控制循环周期
- `kAutonomousSettleDelayMs = 150`
  - 每段动作结束后的稳定时间
- `kDriveToleranceRevolutions = 0.03`
  - 直线圈数容差
- `kTurnToleranceDegrees = 1.5`
  - 转向角度容差
- `kDriveProportionalGain = 30.0`
  - 直线比例系数
- `kDriveMinSpeedPct = 12.0`
  - 直线最小速度
- `kDriveMaxSpeedPct = 22.5`
  - 直线最大速度
- `kDriveHeadingProportionalGain = 1.0`
  - 直线保直比例系数
- `kDriveHeadingCorrectionMaxPct = 6.0`
  - 直线保直允许施加的最大左右差速修正
- `kDriveHeadingDeadbandDegrees = 1.0`
  - 直线保直的角度死区，小偏差时不纠偏
- `kTurnProportionalGain = 0.6`
  - 转向比例系数
- `kTurnMinSpeedPct = 10.0`
  - 远离目标时的转向最小速度
- `kTurnApproachMinSpeedPct = 4.0`
  - 接近目标时的转向最小速度
- `kTurnMaxSpeedPct = 30.0`
  - 转向最大速度
- `kTurnApproachWindowDegrees = 12.0`
  - 进入这个角度范围后切到更低的靠近速度
- `kTurnBaseTimeoutMs = 1000`
  - 转向超时的基础时间
- `kTurnTimeoutPerDegreeMs = 50`
  - 每一度额外增加的转向超时时间

## 怎么调参数

如果实车表现不理想，优先按下面思路调：

- 直线冲过头
  - 先减小 `kDriveMaxSpeedPct`
  - 再减小 `kDriveProportionalGain`
  - 必要时增大 `kDriveToleranceRevolutions`

- 直线走不动或者末端太慢
  - 增大 `kDriveMinSpeedPct`

- 直线容易跑偏
  - 先增大 `kDriveHeadingProportionalGain`
  - 再增大 `kDriveHeadingCorrectionMaxPct`

- 直线纠偏过猛，出现左右摆
  - 先减小 `kDriveHeadingProportionalGain`
  - 再减小 `kDriveHeadingCorrectionMaxPct`

- 直线电机有明显来回抽动
  - 先增大 `kDriveHeadingDeadbandDegrees`
  - 还可以略微减小 `kDriveHeadingProportionalGain`

- 转向冲过头
  - 先减小 `kTurnMaxSpeedPct`
  - 再减小 `kTurnProportionalGain`
  - 必要时增大 `kTurnToleranceDegrees`
  - 还可以减小 `kTurnApproachMinSpeedPct`

- 转向停得太早
  - 减小 `kTurnToleranceDegrees`
  - 或者略微增大 `kTurnApproachMinSpeedPct`

- 转向经常超时退出
  - 先增大 `kTurnBaseTimeoutMs`
  - 再增大 `kTurnTimeoutPerDegreeMs`

## 当前限制

这版自动控制是一个轻量级可用版本，不是完整运动学控制器。当前已知限制：

- 直线已经加了基于 IMU 的轻量保直，但还没有更完整的速度规划或轨迹控制
- 没有加速度规划，只有简单比例减速
- 没有把动作序列抽象成通用指令表，当前是固定写死的 4 段流程

如果后面要继续扩展，建议下一步做这几件事：

1. 给直线增加 IMU 保直
2. 把动作序列改成可配置的 step 列表
3. 给转向补完成状态上报，区分正常结束和超时退出
4. 视需要再上更完整的 PID 或 ADRC
