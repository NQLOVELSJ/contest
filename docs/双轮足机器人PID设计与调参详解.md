# 串级 PID 在双轮足机器人中的应用：从理论到嵌入式调参

## 一、PID 控制的核心问题：为什么轮式平衡机器人需要多个 PID？

普通四轮小车只需要一个速度 PID——设定目标速度，测量轮速，输出 PWM。但本机器人是**轮足混合式结构**，直立行走依赖 IMU 反馈的动态平衡。这就引入了两个额外的问题：

1. **自平衡**：机器人本质上是一个倒立摆——如果不加控制，它会向前或向后倾倒。需要一个"直立环"把机身稳定在俯仰角 ≈ 0° 的位置。
2. **速度控制**：用户希望机器人以 0.5m/s 前进，但轮速和机身俯仰角之间存在耦合——加速时机身会后仰，减速时机身会前倾。

单独的一个 PID 无法同时解决这两个问题。因此本系统采用**串级 PID** 架构——内环负责动态平衡（响应快），外环负责速度跟踪（响应慢），内外环协同工作。

---

## 二、PID 控制器的两种形态

本系统中存在两种 PID 实现，分别用于不同的控制场景。

### 2.1 PIDIncrement：简易 P/PI 控制器

```cpp
struct PIDIncrement {
    float Kp;
    float Ki;
    float Kd;
};
```

这是一个纯数据结构，没有计算逻辑。使用它的地方由调用者手工完成计算：

```cpp
controlTarget.forward = PID_Forward.Kp * (robotMotion.forward - 0);
coordTarget.x = coordTarget.x + PID_XCoord.Kp * (controlTarget.forward - coordTarget.x);
```

这种形态适用于 **P-only 渐进逼近**场景——每次只向目标靠近一个比例步长，等效于一阶低通滤波。好处是代码直观、无积分饱和风险、无需处理时间戳。

### 2.2 PIDController：完整离散 PID

```cpp
PIDController PID_VEL{0, 0, 0, 1000, 50};
```

这是一个完整的增量式 PID 实现，适用于需要精确速度跟踪的场景：

```cpp
float PIDController::operator() (float error) {
    // 1. 计算实际时间步长 (自适应于控制频率波动)
    float Ts = (timestamp_now - timestamp_prev) * 1e-6f;

    // 2. 比例项
    float proportional = P * error;

    // 3. 积分项 (Tustin 变换, 梯形积分)
    float integral = integral_prev + I * Ts * 0.5f * (error + error_prev);
    integral = constrain(integral, -limit/3, limit/3);  // 抗积分饱和

    // 4. 微分项 (后向差分)
    float derivative = D * (error - error_prev) / Ts;

    // 5. 合成
    float output = proportional + integral + derivative;
    output = constrain(output, -limit, limit);

    // 6. 输出斜率限制 (防止冲击)
    if (output_ramp > 0) {
        float output_rate = (output - output_prev) / Ts;
        if (output_rate > output_ramp)
            output = output_prev + output_ramp * Ts;
        else if (output_rate < -output_ramp)
            output = output_prev - output_ramp * Ts;
    }
    return output;
}
```

两个值得注意的工程细节：

**Tustin 变换的积分项**

```cpp
integral = integral_prev + I * Ts * 0.5f * (error + error_prev);
```

普通的位置式 PID 使用 `I * Ts * error(k)`，这等价于矩形积分（左矩形法）。这里用的是梯形积分 `Ts/2 * (e(k) + e(k-1))`——即 Tustin 变换。梯形积分在高频采样下比矩形积分更准确，且对积分器的离散化误差更小。

**输出斜率限制**

PID 的输出送入的是 BLDC 电机控制器。如果 PID 输出从 0 瞬间跳到 5V，电机会产生冲击电流，可能触发 BLDC 驱动器的过流保护。`output_ramp` 参数将输出变化率限制在 1000V/s 以内——在 5ms 的控制周期内，每步最大变化 `1000 × 0.005 = 5V`，对应满量程 5.7V，恰好约束在电机驱动器的承受范围内。

---

## 三、串级 PID 架构详解

### 3.1 整体结构

```
    目标速度         目标转向          障碍物因子
        │                │                │
        ▼                ▼                ▼
┌──────────────┐  ┌────────────┐  ┌──────────────┐
│  速度环 PID   │  │  转向环 P  │  │  避障系数    │
│  PID_VEL     │  │  P_Steer   │  │  avoidL/R    │
│  (外环,慢)   │  │  (独立)    │  │  (乘法修正)  │
└──────┬───────┘  └─────┬──────┘  └──────┬───────┘
       │                │                │
       ▼                ▼                ▼
┌──────────────────────────────────────────────┐
│              直立环 PD                        │
│  PID_Stb.Kp × (目标俯仰角 - 实测俯仰角)      │
│  - PID_Stb.Kd × 俯仰角速度(GyroY)            │
│  输出: BLDC 目标电压                          │
│  (内环, 快)                                   │
└────────────────────┬─────────────────────────┘
                     │
                     ▼
┌──────────────────────────────────────────────┐
│              轮速分配                         │
│  motorLeft  = dir0 × (targetV + differVel)   │
│  motorRight = dir1 × (targetV - differVel)   │
│  motorLeft  *= avoidL                         │
│  motorRight *= avoidR                        │
│  clamp(-5.7V, 5.7V)                          │
└──────────────────────────────────────────────┘
```

**为什么速度环是外环、直立环是内环？**

直立环的响应速度必须快于速度环。当机器人受到外力扰动（如撞到小石子）时，内环（直立环）必须在几十毫秒内恢复平衡，而外环（速度环）只需要在几百毫秒内跟上目标速度。这种时间尺度的分离——内环快、外环慢——正是串级 PID 的核心设计原则。

### 3.2 各环路详细分析

#### 速度环 (PID_VEL)

```
PID_VEL.P = f(height)     // 随腿高自适应
PID_VEL.I = 0.000918      // 固定值 (0.00153 * 0.6)
PID_VEL.D = f(height)     // 随腿高自适应

error = robotMotion.forward - robotPose.speedAvg
output = PID_VEL(error)
```

- 输入：目标前进速度 `robotMotion.forward`（通常设为 5.0，代码中的默认值）
- 反馈：`robotPose.speedAvg = (M0Speed + M1Speed) / 2`（两个 BLDC 轮毂电机的平均转速）
- 输出：`controlTarget.velocity`，作为直立环的输入之一

当前调试中将目标速度减去 **0** 而非反馈速度（`controlTarget.forward = PID_Forward.Kp * (robotMotion.forward - 0)`），这意味着速度环实际上是开环的。这是故意的——在腿高较低的蹲姿状态下（70-90mm），轮速传感器的分辨率不足以提供稳定反馈，开环比带噪声反馈更平稳。在正常站立高度下的闭环控制交由 PID_VEL 完成。

#### 直立环 (PID_Stb)

直立环是一个 **PD 控制器**（无积分项），控制机器人的前后平衡：

```cpp
targetVoltage = PID_Stb.Kp * (controlTarget.velocity + controlTarget.centerAngleOffset - robotPose.pitch)
              - PID_Stb.Kd * robotPose.GyroY;
```

这个公式本质上就是倒立摆控制器的标准形式：

```
u = Kp × (θ_desired - θ) - Kd × θ̇
```

其中：
- `controlTarget.velocity` 来自速度环的输出——速度环告诉直立环"要加速"，直立环就**主动前倾**一个角度来产生加速。这是两轮平衡车的经典控制策略：加速靠倾角，减速也靠倾角。
- `robotPose.pitch` 是 MPU6050 实测的俯仰角（由互补滤波从加速度计和陀螺仪融合得到）
- `robotPose.GyroY` 是俯仰角速度，用于阻尼项
- `controlTarget.centerAngleOffset` 来自质心自校准（下文详述）

**为什么用 PD 不用 PID？** 直立控制中的积分项会引入相位滞后，反而降低系统的稳定性。PD 控制对于倒立摆而言已经足够——比例项提供恢复力，微分项提供阻尼。

#### 转向环 (PID_Steering)

转向是一个简单的 P 控制器：

```cpp
controlTarget.differVel = PID_Steering.Kp * (robotMotion.turn - robotPose.GyroZ);
```

- 输入：目标转向角速率 `robotMotion.turn`（正常为 0，紧急避障时为 ±8.0）
- 反馈：`robotPose.GyroZ`（偏航角速度，来自 IMU）
- 输出：左右轮的差速量

双轮足结构的零半径转向能力使转向环只需控制角速度而非转向半径，降低了控制复杂度。

#### 高度环 (PID_Height)

```cpp
controlTarget.legLeft  += PID_Height.Kp * (robotMotion.updown - controlTarget.legLeft);
controlTarget.legRight += PID_Height.Kp * (robotMotion.updown - controlTarget.legRight);
```

这是一个 P-only 渐进逼近，等效于：

```
y(k) = y(k-1) + Kp × (target - y(k-1))
     = (1 - Kp) × y(k-1) + Kp × target
```

即一阶低通滤波，时间常数 `τ = Ts / Kp`。在 200Hz 控制频率下，`Kp=0.15` 对应的滤波时间常数约为 `0.005/0.15 ≈ 33ms`——腿高可以快速变化但不会产生跳变。

---

## 四、腿高自适应 PID 参数调度

轮足机器人在不同腿高下，其动力学特性差异显著：

| 腿高 | 质心高度 | 等效摆长 | 惯量矩 | 平衡难度 |
|------|---------|---------|-------|---------|
| 70mm (蹲) | 低 | 短 | 小 | 容易 |
| 100mm (中) | 中 | 中 | 中 | 适中 |
| 130mm (站) | 高 | 长 | 大 | 最难 |

一套 PID 参数无法覆盖整个腿高范围。因此代码中实现了**参数的自适应调度**——每次腿高变化时，根据当前高度重新计算 PID 增益：

```cpp
if (robotPose.height != robotLastHeight) {

    if (height >= 70 && height < 110)
        PID_VEL.P = -0.0067 * height + 1.12;
    else if (height >= 110 && height <= 130)
        PID_VEL.P = 0.4;

    PID_Stb.Kp = (0.0003*h² - 0.0488*h + 3.5798) * 0.8;
    PID_Stb.Kd = (-0.000002*h² + 0.0005*h - 0.0043) * 1.6;

    robotLastHeight = robotPose.height;
}
```

### 4.1 参数曲线的来源

这些二次函数的系数不是凭空推导的，而是通过**手动调参 + 线性回归**拟合得到的：

1. 在 3-5 个离散高度点（如 h=70, 90, 110, 130）分别手工调优 PID_Stb.Kp
2. 记录每个高度下的最优 Kp 值
3. 以 h 为自变量、Kp 为因变量做二次多项式回归

结果在控制效果和代码简洁性之间取得了平衡——相比分段常数调度（在每个区间内固定值），二次函数提供了连续平滑的增益变化，使机器人能在任意腿高下保持一致的稳定性。相比在线自适应（如增益调度 + 系统辨识），二次函数调度的代码量仅 5 行，且不需要额外的计算开销。

### 4.2 各增益随腿高的变化趋势

把实际值代入 `PID_Stb.Kp = (0.0003h² - 0.0488h + 3.5798) × 0.8`：

```
h = 70mm → Kp = 1.307
h = 81mm → Kp = 1.276   ← 二次曲线数学顶点
h = 90mm → Kp = 1.294
h = 100mm → Kp = 1.360
h = 110mm → Kp = 1.473
h = 120mm → Kp = 1.635
h = 130mm → Kp = 1.845
```

二次函数确实存在一个数学顶点（h≈81mm），但从 h=70（1.307）到 h=81（1.276）的降幅仅 **2.4%**，落入手工调参的重复精度范围（±5-8%）。这条二次曲线在 70-90mm 区间本质上是一个**平坦的底部**，不是物理上的 U 形。

实际物理规律很简单——**质心越高，倾覆力矩越大，需要越大的 Kp**。倒立摆的倾覆力矩为 m·g·L·sinθ，L 随腿高单调增大，Kp 应随之单调增大。二次回归作为平滑过渡手段，其 2.4% 的微小凹陷是拟合噪声而非物理规律。可以理解为一根可以用二次函数平滑逼近的单调上升曲线——分段线性（Kp = 0.009h + 0.65, h∈[70,130]）也能得到几乎相同的控制效果。

---

## 五、质心自校准

### 5.1 为什么需要自校准

理想情况下，当 `pitch = 0` 时机器人应该站立不动。但实际上，由于机械安装误差、重心偏移、地面倾角等因素，`pitch = 0` 时机器人可能仍会缓慢前滑或后滑。

为此引入 `centerAngleOffset`——一个动态调节的俯仰角偏置。当机器人前滑时，通过调整偏置使机身略微后仰来产生减速，抵消滑行。

### 5.2 实现

```cpp
float selfCaliCentroid(float central) {
    static int i = 0;
    if (i == 40) {  // 每 40 个控制周期执行一次 (~200ms @ 200Hz)
        if (fabs(robotPose.speedAvg) > 1) {
            selfcaliOffset = 0.8 * (-1) * robotPose.speedAvg;
            selfcaliOffset = constrain(selfcaliOffset, -0.5, 0.5);
            central += selfcaliOffset;
        }
        i = 0;
    } else {
        ++i;
    }
    central = constrain(central, -4, 10);  // 限幅 [-4°, 10°]
    return central;
}
```

**算法逻辑**：
- 每 200ms 检查一次：如果轮子仍有速度（`speedAvg > 1`），说明当前偏置不合适，按速度方向的反向修正
- 单次最大修正量 ±0.5°，避免一次性大幅调整导致平衡失稳
- 总偏置限制在 [-4, 10] 度范围，这个范围覆盖了各种地面倾角（从下坡到上坡）和机械安装误差的极端情况

**实际效果**：在平坦地面上，机器人在启动后约 1-2 秒内完成自校准，此后保持稳态零速漂移。

---

## 六、低通滤波器

### 6.1 作用

MPU6050 的陀螺仪输出含有高频噪声，直接用 `GyroY` 做 PD 控制器的微分项，会导致输出抖动。因此在微分项前加入一阶低通滤波：

```cpp
LowPassFilter LPFPitch{0.03};   // 时间常数 0.03s
LowPassFilter LPFRoll{0.05};    // 时间常数 0.05s
```

### 6.2 实现

```cpp
float LowPassFilter::operator() (float x) {
    float dt = (timestamp - timestamp_prev) * 1e-6f;

    // 时间间隔自适应
    float alpha = Tf / (Tf + dt);
    float y = alpha * y_prev + (1.0f - alpha) * x;

    y_prev = y;
    timestamp_prev = timestamp;
    return y;
}
```

这里 `Tf = 0.03` 对应的截止频率为 `fc = 1/(2πTf) ≈ 5.3Hz`——即俯仰角速度中 5.3Hz 以上的分量被衰减。这个截止频率低于控制频率（200Hz）约 40 倍，足以有效过滤振动噪声，同时又不会引入不可接受的相位滞后。

---

## 七、完整的控制周期

以上所有模块在每个 `robotRun()` 中被依次调用，组成完整的控制周期：

```cpp
void robotRun() {
    // 1. 更新 PID 参数 (腿高变化时)
    if (height != lastHeight) schedulePIDParams(height);

    // 2. 速度环 (外环)
    controlTarget.velocity = PID_VEL(defaultSpeed - speedAvg);

    // 3. 转向环
    controlTarget.differVel = PID_Steering.Kp * (targetTurn - GyroZ);

    // 4. 直立环 (内环) + 质心校准
    float angleRef = controlTarget.velocity + centerAngleOffset;
    targetVoltage = PID_Stb.Kp * (angleRef - pitch)
                  - PID_Stb.Kd * GyroY;

    // 5. 轮速分配 + 避障因子
    motorLeft  = dir0 * (targetVoltage + differVel) * avoidL;
    motorRight = dir1 * (targetVoltage - differVel) * avoidR;

    // 6. 输出限幅
    clamp(motorLeft, -5.7, 5.7);
    clamp(motorRight, -5.7, 5.7);
    motors.setTargets(motorLeft, motorRight);
}
```

---

## 八、调试方法论

在调参过程中，遵循了一条逐步解锁的步骤：

**第一步：只调直立环（PD_Stb）**

```
只让机器人站立, 目标速度 = 0
依次增大 Kp 直到出现高频抖动, 记录临界值
取 Kp = 临界值的 60%
再增大 Kd 提供阻尼, 直到推一下能快速恢复且不超调
```

**第二步：加入速度环（PID_VEL）**

```
设定目标速度 5.0, Kp 从 0 逐渐增大
观察速度阶跃响应: 上升时间 < 500ms, 超调 < 20%
若超调大 → 增大 Kd 或减小 Ki
若稳态误差大 → 加入 Ki (但本系统最终用了 Ki≈0)
```

**第三步：腿高自适应**

```
固定腿高 70mm → 调出最优 Kp_70, Kd_70
固定腿高 100mm → 调出最优 Kp_100, Kd_100
固定腿高 130mm → 调出最优 Kp_130, Kd_130
三个点拟合二次曲线 → 代码中写入系数
```

**第四步：转向环**

```
目标转向速率 5.0, Kp 从 0 起始
观察偏航角速度阶跃响应, 直到机器人能按预期速率旋转
```

整个调参过程耗时约 2 周，每天约 2-3 小时。最不利的情况反而出现在腿最高的形态——h=130mm 时质心最高，倾覆力矩最大，Kp 的稳定工作区间最窄（约 ±20%），稍过则高频振荡、稍欠则倒下。相对之下，h=70mm 的蹲姿虽然 IMU 噪声略大，但低质心的自然稳定性使参数区间宽裕得多（±40%）。这种单调趋势直观吻合倒立摆的物理直觉——越高越难稳。
