# 双足导盲轮足机器人：多传感融合与全局-局部分层导航系统设计

## 一、项目概述

本作品设计并实现了一款面向视障人士辅助出行的轮足混合式导盲机器人。系统以 **ESP32-S3** 和 **OpenMV H7 Plus** 双芯片为核心，融合全局路径规划、多传感器局部避障、力控随行和语音交互，构建了一套完整的室内外辅助导航系统。

**硬件平台参数：**

| 组件 | 型号 | 数量 |
|------|------|------|
| 运动控制 MCU | ESP32-S3 DevKitC-1 (240MHz 双核) | 1 |
| 视觉/规划 MCU | OpenMV H7 Plus (Cortex-M7 480MHz, 32MB SDRAM) | 1 |
| 姿态传感器 | MPU6050 (I2C, 400kHz) | 1 |
| 超声波测距 | 高精度超声波模块 (温度补偿, ±0.5cm) | 1 |
| 视觉传感器 | OV5640 (QVGA, FOMO 检测) | 1 |
| 探测云台 | Pan 舵机 (超声波模块 + OpenMV 共架) | 1 |
| 腿部执行器 | MG996R 舵机 × 4 (PCA9685 驱动) | 4 |
| 行走执行器 | BLDC 轮毂电机 × 2 (FOC 控制) | 2 |
| 人机交互 | 多维力传感器手柄 + 离线 ASR 语音 | 1 |
| 供电 | 3S 5000mAh LiPo (约 1.5h 续航) | 1 |
| 通信 | ESP32↔OpenMV: UART @ 921600 baud | 1 |

---

## 二、系统架构

系统采用双芯片分工架构，OpenMV H7 Plus 负责上游感知与规划，ESP32-S3 负责下游控制与交互：

```
┌──────────────────────────────────────────────────────────────────┐
│ OpenMV H7 Plus (10Hz 任务循环)                                   │
│                                                                  │
│  [1] Camera snapshot (QVGA 240×240)                              │
│  [2] 畸变校正 + 图像预处理                                        │
│  [3] TFLite FOMO 推理 (椅子/柱子/手, conf≥0.5)                   │
│  [4] A* 全局路径规划 (100×100 栅格, 8方向)                        │
│  [5] Pan 舵机→超声波指向障碍物 + 超声测距                         │
│  [6] 球坐标系解算 (servoAngle × distance → x,y)                   │
│  [7] 双通道冗余决策 (视觉 AND/OR 超声) + 势场避障合成              │
│  [8] 二进制帧编码 → UART 发送                                     │
│                                                                  │
│  单帧耗时: 畸变(3ms) + TFLite(30ms) + A*(20ms) + 超声/解算(3ms)  │
└──────────────────────────────┬───────────────────────────────────┘
                               │ UART @ 921600, 单向
                               │ OpenMV→ESP32: avoidL_f32 | avoidR_f32 | crc_u8
                               │
┌──────────────────────────────┴───────────────────────────────────┐
│ ESP32-S3 (50-200Hz 控制循环)                                     │
│                                                                  │
│  [1] 接收 OpenMV 避障因子 → 解析                                 │
│  [2] 安全层: 看门狗超时 (300ms 保持→1000ms 停止)                 │
│  [3] 导纳控制: 力传感器 → 运动映射                                │
│  [4] IMU 互补滤波 → pitch/roll/yaw + 角速度                       │
│  [5] 串级 PID: 速度环→直立环→转向环                               │
│  [6] 5连杆逆运动学 → 4舵机 PWM 输出                               │
│  [7] FOC 轮毂电机电压输出                                         │
│  [8] 离线 ASR 轮询 + TTS 播报                                    │
│  [9] WiFi AP + WebSocket 状态广播 (1Hz)                         │
└──────────────────────────────────────────────────────────────────┘
```

### 关键设计决策：为什么将规划部署在 OpenMV

A* 全局路径规划需要维护一个栅格地图。经过实测对比：

| 部署位置 | 内存瓶颈 | 最大栅格规模 | 单次规划耗时 |
|---------|---------|-------------|-------------|
| ESP32-S3 (4MB PSRAM) | PSRAM 延迟较高 | ~150×150 | ~30ms (C++) |
| OpenMV H7 Plus (32MB SDRAM) | 无 | ~500×500+ | ~20-50ms (MicroPython) |

虽然 ESP32 运行 C++ 的 A* 执行速度更快，但 OpenMV 的 32MB SDRAM 可以承载更大规模的地图，为后续楼层级导航（1000×1000 栅格）预留了扩展空间。此外，将视觉推理和路径规划放在同一芯片上，避免了视觉特征数据跨芯片传输的延迟。

---

## 三、障碍物检测与坐标解算

这是本系统最关键的传感器融合环节，涉及视觉→伺服→测距→坐标变换的完整链路。

### 3.1 工作流程

```
① OpenMV 检测到障碍物 (像素坐标 cx, cy, 置信度≥0.5)
         ↓
② 根据像素水平偏移计算云台目标角度:
   servo_target = PAN_CENTER + (cx - 120) / 120 * 30°
         ↓
③ Pan 舵机旋转 → 超声波传感器指向障碍物
         ↓
④ OpenMV 触发超声测距 (TRIG/ECHO GPIO) → 读取距离 r
         ↓
⑤ OpenMV 球坐标系解算 (全部在 OpenMV 单芯片内完成):
   x = r · sin(θ)
   y = r · cos(θ)
   z = (1 - cy / 240) × 100 cm
         ↓
⑥ 双通道决策 → 势场导航 → 避障因子 → UART 发送 ESP32
```

OpenMV 的 pan 舵机与超声波传感器同轴安装，舵机旋转时超声波同步指向目标方向。这种"视觉粗定位+超声波精测距"的串行架构可以有效降低传感器的硬件成本，同时避免多传感器同时旋转带来的机械干涉。

### 3.2 畸变校正

OV5640 在 QVGA 分辨率下存在桶形畸变，影响像素→角度映射精度。在校正前，240×240 图像边缘像素的角度偏差可达 8°，校正后降至约 1°：

```python
img = sensor.snapshot()
img.lens_corr(1.5)  # OpenMV 内置畸变校正
```

### 3.3 FOMO 目标检测

采用 TensorFlow Lite Micro 部署 FOMO 模型，在 240×240 输入上检测 chair、column、hands 三类：

```python
def fomo_post_process(model, inputs, outputs):
    ob, oh, ow, oc = model.output_shape[0]
    x_scale = inputs[0].roi[2] / ow
    y_scale = inputs[0].roi[3] / oh
    scale = min(x_scale, y_scale)

    for i in range(1, oc):
        detection_list = outputs[0][:, :, i]
        img = image.Image(detection_list * 255)
        blobs = img.find_blobs(threshold_list, pixels_threshold=1)
        for b in blobs:
            score = img.get_statistics(roi=b.rect()).l_mean() / 255.0
            if score >= 0.5:
                x = int(b.cx() * scale)
                yield (labels[i], x, int(b.cy() * scale), b.w(), b.h(), score)
```

检测结果的特征图分辨率为输入图像的 1/8，每个输出像素对应 8×8 的感受野，再通过坐标缩放映射回输入空间。

### 3.4 球坐标系解算

超声波传感器与 OpenMV 共架于同一 pan 舵机，因此**超声的触发、回波读取、坐标解算全部在 OpenMV 单芯片内闭环**，不经过 ESP32。OpenMV 同时控制舵机角度 θ 和读取超声距离 r，两者在本地合并即可得到障碍物坐标：

```python
def measure_and_solve(pixel_cy):
    # === OpenMV 本地超声测距 (TRIG=GPIO_P0, ECHO=GPIO_P1) ===
    trig_pin.value(1)
    time.sleep_us(20)
    trig_pin.value(0)
    pulse_us = time_pulse_us(echo_pin, 1, 6000)  # 最大等待 6ms (~100cm)
    distance_cm = pulse_us / 58.0                 # 声速换算

    if distance_cm <= 0 or distance_cm >= DANGER_RADIUS:
        return None, None

    # === OpenMV 本地坐标解算 ===
    servo_angle = get_current_pan_angle()          # OpenMV 直接控制舵机
    pan_rad = math.radians(servo_angle - PAN_SERVO_OFFSET)

    x = math.sin(pan_rad) * distance_cm           # 横向偏移 (cm)
    y = math.cos(pan_rad) * distance_cm           # 前向距离 (cm)

    height_ratio = 1.0 - (pixel_cy / IMAGE_HEIGHT)
    z = height_ratio * 100.0

    return (x, y, z), distance_cm
```

超声触发→回波读出的整个时序由 OpenMV 的 `time_pulse_us()` 硬件定时器完成，精度 ±2μs（约 ±0.03cm），比 Arduino 的 `pulseIn()` 更稳定，且与 TFLite 推理在同一进程内顺序执行，无需跨芯片同步。

这种解算方式假设地面平坦，实际测试中在 ±3° 倾角范围内高度估算偏差小于 5cm，对避障决策影响可忽略。如果后续引入 IMU 俯仰角补偿，可以进一步提升在坡道上的精度。

### 3.5 双通道冗余避障决策

障碍物是否触发避障由一条**双通道判定规则**决定：

```
避障触发条件:

  [正常态] 视觉看到障碍物 AND 超声测距 < 危险阈值 (100cm)
      → 激活势场避障, 输出左右轮避障因子

  [安全态] 超声测距 < 紧急阈值 (30cm) 
      → 直接进入紧急转向状态机, 视觉丢失不阻塞刹车

  注: 仅有视觉但无超声 → 视为虚警, 不避障
      仅有超声但无视觉 → 触发紧急避障 (优先级最高)
```

**AND 逻辑（正常态）**降低了单传感器虚警：墙壁漫反射导致超声测到短距离但视野正常时不会误转向；图像误识别出障碍物但超声显示前方空旷时也不会绕行。

**OR 逻辑（安全态）**确保不会因视觉失效而漏检。超声从触发到中断读出仅需约 3ms（对应 50cm 内障碍），而 OpenMV 的视觉链路包含 30ms 推理 + 100ms 周期 ≈ 130ms 总延迟。当行人突然横穿时，超声先触发紧急转向状态的**快通道保安全**，视觉随后提供障碍物精确角度的**慢通道保精度**——形成天然的多时间尺度冗余：

```
t = 0ms     行人进入超声波波束 (距离 80cm)
t = 3ms     OpenMV 超声回波读出, 触发紧急标志
t = 3-55ms  OpenMV 继续完成当前帧: TFLite → A* → 球坐标 → 势场
t = 55ms    OpenMV 帧发送至 ESP32 (含紧急标志 + 避障因子)
t = 55-56ms ESP32 解析帧 → 紧急转向执行, 轮速归零
t = 56ms+   ESP32 执行 PID 控制, 根据避障因子规划精细绕行
```

这个机制依赖一个前提：**OpenMV 的计算速度必须足够快，使其视觉输出能跟上超声触发的紧急动作节奏。** 10Hz 的帧率（100ms）对于以 1m/s 移动的行人来说足够——行人在 100ms 内移动 10cm，在超声 10-15° 波束角覆盖范围内仍然可以稳定跟踪。

---

## 四、全局路径规划

### 4.1 A* 算法实现

基于 100×100 栅格地图，8 方向邻域 + 欧几里得距离启发式：

```python
def a_star(grid, start, goal):
    open_heap = [(0, start)]
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    parent = {}

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal:
            path = []
            while current in parent:
                path.append(current)
                current = parent[current]
            path.append(start)
            return path[::-1]

        for dx, dy in [(1,0), (-1,0), (0,1), (0,-1),
                       (1,1), (-1,1), (1,-1), (-1,-1)]:
            nx, ny = current[0] + dx, current[1] + dy
            if not (0 <= nx < 100 and 0 <= ny < 100):
                continue
            if grid[ny][nx] == 1:
                continue

            move_cost = math.sqrt(dx*dx + dy*dy)
            tentative_g = g_score[current] + move_cost

            if tentative_g < g_score.get((nx, ny), float('inf')):
                parent[(nx, ny)] = current
                g_score[(nx, ny)] = tentative_g
                f = tentative_g + heuristic((nx, ny), goal)
                f_score[(nx, ny)] = f
                heapq.heappush(open_heap, (f, (nx, ny)))
    return []
```

100×100 的地图中，一次重规划约 20-50ms（取决于障碍物密度和起点-终点距离）。在 10Hz 的规划频率下，OpenMV 端还有约 30ms 的余量用于势场合成与串口发送。

### 4.2 路径跟踪

```
全局路径 P = [p₀, p₁, ..., pₙ]    A* 输出
局部目标 g = p[k]                   取前方第 k 个路径点
引力方向 = normalize(g - robot_pos)  势场法输入
```

---

## 五、局部避障与运动控制

### 5.1 势场法避障

避障决策在 OpenMV 端进行，视觉检测结果和超声波距离均在 OpenMV 本地获取，无需跨芯片传输：

```python
def potential_field_force(distance, angle):
    # distance: OpenMV 本地超声测距结果 (trig/echo GPIO)
    # angle:    pan 舵机当前偏转角 (OpenMV 直接控制)
```

OpenMV 端持有完整的感知上下文——视觉坐标、舵机角度、超声距离、球坐标解算结果全部在单芯片内闭环。势场法直接从本地内存读取障碍物 (x, y)，无需任何跨芯片数据交换。

### 5.2 串级 PID 控制

ESP32-S3 端运行三环串级 PID：

```cpp
void robotRun() {
    // 速度环
    controlTarget.velocity = PID_VEL(forwardSpeed - speedAvg);

    // 转向环
    controlTarget.differVel = PID_Steering.Kp * (turnRate - GyroZ);

    // 直立环 (角度-角速度 PD)
    targetVoltage = PID_Stb.Kp * (velocity + centerOffset - pitch)
                  - PID_Stb.Kd * GyroY;

    // 轮速分配
    motorLeft  = dir0 * (targetVoltage + differVel) * avoidanceFactorLeft;
    motorRight = dir1 * (targetVoltage - differVel) * avoidanceFactorRight;

    motors.setTargets(clamp(motorLeft, -5.7, 5.7),
                      clamp(motorRight, -5.7, 5.7));
}
```

PID 参数随腿高自适应调节：

```cpp
if (height >= 70 && height < 110) {
    PID_VEL.P = -0.0067 * height + 1.12;
    PID_Stb.Kp = (0.0003*h*h - 0.0488*h + 3.5798) * 0.8;
}
```

### 5.3 5连杆逆运动学

腿部机构简化为五连杆模型，计算 4 个舵机角度：

```cpp
void inverseKinematics() {
    float a1 = 2 * x * L1, b1 = 2 * yL * L1;
    float c1 = x*x + yL*yL + L1*L1 - L2*L2;

    IKParam.alpha1 = 2 * atan2(b1 + sqrt(a1*a1 + b1*b1 - c1*c1), a1 + c1);
    IKParam.beta1  = 2 * atan2(e1 - sqrt(d1*d1 + e1*e1 - f1*f1), d1 + f1);

    servoLeftRear  = map(alpha1ToDeg, 0, 180, 103, 327);
    servoLeftFront = map(beta1ToDeg,  0, 180, 103, 327);
}
```

### 5.4 航向恢复算法

在引入 A* 全局规划之前，避障转向缺乏全局参考，会产生不可控的偏航累积。为此设计了一个积分补偿机制：在避障过程中记录左右轮速系数的差值累积 `ΔK_sum`，避障结束后以反向平均量恢复航向。引入 A* 后，全局路径本身就提供了持续的方向引导，该算法被移除，路径跟踪完全交由势场法的引力项处理。

---

## 六、人机交互

### 6.1 力控随行

采用导纳控制框架实现人机物理交互：

```
力 F_push ∈ [0.5N, 20N] → 目标速度 v = F_push / D_viscous
力矩 M_turn ∈ [0.1N·m, 3N·m] → 目标转向角速率 ω = M_turn / D_rotational

D_viscous = 5.0, D_rotational = 3.0
F_push < 0.5N → 视为噪声, 不响应
F_push > 20N 或 F_brake > 15N → 触发急停
```

### 6.2 AI 语音交互

基于 ESP-SR 框架的离线 ASR，定义了 20 个导盲场景命令词，安静室内识别率约 92%：

```
运动控制: "前进", "停止", "左转", "右转", "慢点", "快点"
导航控制: "去门口", "回起点", "当前位置"
状态查询: "还有多少电", "前方有没有障碍"
系统控制: "关机", "休眠"
```

TTS 音频资源预存于 Flash，由 ESP32 通过 I2S DAC 播放。播报事件包括：障碍物提示、电量不足警告、命令复述确认。

### 6.3 Web 远程监控

ESP32 开启热点（SSID: MyRobot），手机通过 WebSocket 连接后实时查看超声波距离、障碍物状态、舵机角度等信息。

---

## 七、安全架构

超声波传感器部署在 OpenMV 端后，ESP32 不再直接连接任何测距硬件。因此安全层完全依赖**通信看门狗**——若 OpenMV 整体死机（视觉+超声全链路失效），ESP32 通过帧超时检测触发保护：

```
┌────────────────────────────────────────────────────────┐
│ ESP32 安全层 (优先级: 高于一切控制输出)                  │
│                                                        │
│ 条件 1: 超过 300ms 未收到 OpenMV 帧                    │
│   → 保持模式: 维持最后有效速度 + 减速至零               │
│                                                        │
│ 条件 2: 超过 1000ms 未收到 OpenMV 帧                   │
│   → 安全停止: 电机电压归零, 舵机锁位                    │
│                                                        │
│ 条件 3: IMU 检测到俯仰角 > 45° (摔倒)                  │
│   → 紧急断电: 电机 PWM 输出 0, 舵机关闭                │
│                                                        │
│ 条件 4: 力传感器检测到 > 20N 推力 (急停意图)            │
│   → 紧急制动: 反向输出 100% 扭矩 200ms                 │
└────────────────────────────────────────────────────────┘
```

OpenMV 单芯片承载了从视觉到超声的全部感知链路，理论上故障面扩大。但实际中时间窗口是安全的：OpenMV 导致看门狗超时的典型场景是 MicroPython GC 卡顿（150-200ms），300ms 阈值可以过滤；而真正死机时 1 秒内完全停止，在导盲场景的典型速度 0.5m/s 下，1 秒滑行距离约 50cm，仍在安全边界内。条件 3 和条件 4 保留在 ESP32 端独立运行，作为关键安全冗余。

---

## 八、项目文件结构

```
├── src/
│   ├── main.cpp                  # ESP32-S3 主程序 (~1100 行)
│   ├── bipedal_data.h            # 全局数据结构定义
│   └── index_html.h              # Web UI (HTML+JS 内嵌)
│
├── lib/
│   ├── SF_Servo/                 # PCA9685 舵机驱动
│   ├── SF_IMU/                   # MPU6050 驱动 + 互补滤波
│   ├── SF_BLDC/                  # BLDC FOC 控制 (静态库)
│   ├── SBUS/                     # SBUS 协议解析
│   ├── pid/                      # 离散 PID 控制器
│   └── lowpass_filter/           # 一阶低通滤波器
│
├── openmv_test/
│   ├── main.py                   # OpenMV 主程序 (187 行)
│   ├── trained.tflite            # FOMO 模型 (量化后 ~200KB)
│   └── labels.txt                # 标签: background, chair, column, hands
│
├── platformio.ini                # PlatformIO 构建配置
└── README.md
```

**数据流时序（以一次 100ms 周期为例）：**

```
t=0ms     OpenMV: snapshot + 畸变校正 (3ms)
t=3ms     TFLite 推理 (30ms)
t=33ms    A* 重规划 (20ms)
t=53ms    Pan 舵机更新 + TRIG 超声脉冲 → 等待回波 (~1ms)
t=54ms    OpenMV: time_pulse_us() 读取回波 → 距离 r
t=55ms    OpenMV: 球坐标系解算 (x,y,z) (0.1ms)
t=55ms    OpenMV: 双通道决策 + 势场合力 (2ms)
t=57ms    OpenMV: 避障因子帧编码 → UART 发送 (0.2ms)
          ↓
t=58ms    ESP32: 帧接收完成 → 提取避障因子
t=58-100ms ESP32: 执行 8-16 次 50-200Hz 控制循环
t=100ms   下一周期
```

---

## 九、调试与验证

**超声波测距精度对比（各 500 次采样，目标距离 50cm）：**

| 指标 | 普通 HC-SR04 | 高精度超声波 (带温度补偿) |
|------|-------------|------------------------|
| 均值误差 | +2.8cm | +0.4cm |
| 标准差 | 3.1cm | 0.5cm |
| 有效测距帧率 | 20Hz (被 PWM 触发间隔限制) | **10Hz** (与 OpenMV 视觉周期同步) |
| 最小可测角分辨率 | — | 约 1° (配合舵机) |

**电池续航实测：**

| 模式 | 电流 (5V 等效) | 实测续航 |
|------|---------------|---------|
| 待机 (WiFi 开启) | ~450mA | ~9h |
| 直行无避障 | ~2.1A | ~2h |
| 全功能运行 (避障+语音+WiFi) | ~3.8A | ~1.3h |

---

## 十、总结

本项目的核心技术积累主要体现在三个层面：

- **系统架构层面**：双芯片分工将感知规划与运动控制解耦，关键数据流在单芯片内部闭环，跨芯片通信仅传递紧凑的二进制控制帧，延迟和可靠性均可控。
- **感知规划层面**：OpenMV 单芯片闭环完成视觉推理、畸变校正、超声测距、球坐标系解算、A* 路径规划与势场避障的完整链路。所有感知数据在本地内存中流转，跨芯片总线仅传输最终的控制指令（左右轮避障因子），规避了中间结果跨芯片传输的延迟与同步问题。
- **工程实践层面**：经历了从"走起来就行"到"稳定可靠地走"的转变。中位数滤波、帧校验与 XOR 校验和、看门狗超时、突变抑制、互补滤波——这些"不性感"的工程细节构成了系统稳定运行的基础。

项目已开源，源代码见 [GitHub 仓库]。
