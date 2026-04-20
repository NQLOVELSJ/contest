# 双足导盲机器人项目

## 项目简介

这是一个基于ESP32-S3微控制器的智能双足导盲机器人系统。项目采用PlatformIO开发环境，结合多种传感器和通信模块，实现机器人的自主导航、障碍物检测和远程监控功能。

## 核心功能

### 🤖 机器人控制
- **双足行走控制**：基于逆运动学的腿部控制算法
- **姿态稳定**：MPU6050传感器实时姿态检测与PID控制
- **遥控操作**：SBUS协议遥控器控制

### 🎯 智能导航
- **超声波避障**：HC-SR04超声波传感器检测前方障碍物
- **视觉识别**：OpenMV摄像头模块进行视觉识别
- **势场导航**：高级势场导航算法实现智能避障

### 📡 通信与监控
- **WiFi Web服务器**：ESPAsyncWebServer提供Web界面
- **实时状态监控**：WebSocket实时传输机器人状态数据
- **蓝牙音频**：btAudio库支持蓝牙音频传输

### 🔧 系统特性
- **多传感器融合**：IMU、超声波、摄像头数据融合
- **低通滤波**：传感器数据平滑处理
- **看门狗保护**：ESP32硬件看门狗确保系统稳定性

## 技术栈

### 硬件平台
- **主控芯片**：ESP32-S3 DevKitC-1
- **传感器**：MPU6050陀螺仪、HC-SR04超声波、OpenMV摄像头
- **执行器**：舵机、BLDC电机
- **通信**：SBUS遥控器、WiFi、蓝牙

### 软件框架
- **开发环境**：PlatformIO
- **框架**：Arduino Framework
- **核心库**：
  - `SF_Servo` - 舵机控制库
  - `SF_IMU` - IMU传感器库
  - `SF_BLDC` - BLDC电机控制库
  - `ESPAsyncWebServer` - 异步Web服务器
  - `ArduinoJson` - JSON数据处理
  - `NewPing` - 超声波测距库
  - `btAudio` - 蓝牙音频库

## 项目结构

```
contest/
├── src/
│   ├── main.cpp              # 主程序文件
│   ├── bipedal_data.h        # 机器人数据结构定义
│   ├── index_html.h          # Web界面HTML代码
│   └── contest.code-workspace # VS Code工作区配置
├── platformio.ini            # PlatformIO项目配置
├── lib/                      # 自定义库目录
│   └── SF_BLDC/              # BLDC电机控制库
└── .pio/                     # PlatformIO构建目录
```

## 快速开始

### 环境准备

1. **安装PlatformIO**：
   - 通过VS Code安装PlatformIO IDE扩展
   - 或使用命令行：`pip install platformio`

2. **克隆项目**：
   ```bash
   git clone <repository-url>
   cd contest
   ```

### 硬件连接

#### 传感器连接
- **MPU6050**：I2C接口（默认SDA=GPIO21, SCL=GPIO22）
- **HC-SR04**：TRIG=GPIO37, ECHO=GPIO38
- **OpenMV摄像头**：TX=GPIO45, RX=GPIO48
- **SBUS遥控器**：RX=GPIO41

#### 执行器连接
- **舵机**：I2C接口控制
- **BLDC电机**：Serial2接口控制

### 编译与烧录

1. **编译项目**：
   ```bash
   pio run
   ```

2. **烧录固件**：
   ```bash
   pio run --target upload
   ```

3. **监视串口输出**：
   ```bash
   pio device monitor
   ```

### 使用说明

#### 启动机器人
1. 给机器人上电
2. 等待WiFi热点启动（默认SSID：Robot-AP）
3. 连接机器人WiFi热点
4. 打开浏览器访问 `http://192.168.4.1`

#### Web界面操作
- **实时状态**：查看机器人姿态、距离等数据
- **声音警报**：点击"启动声音通知"启用音频警报
- **远程控制**：通过Web界面进行基本控制

#### 遥控器操作
- **通道1**：前进/后退控制
- **通道2**：转向控制
- **通道3**：模式切换
- **通道4-6**：舵机控制

## 配置说明

### 机器人参数配置
在 `bipedal_data.h` 中可调整以下参数：

```cpp
// 机器人尺寸参数
#define L1  60    // 腿部长度1
#define L2  100   // 腿部长度2
#define L3  100   // 腿部长度3
#define L4  60    // 腿部长度4
#define L5  40    // 腿部长度5

// 舵机偏移量
#define LFSERVO_OFFSET -2   // 左前舵机偏移
#define LRSERVO_OFFSET -8   // 左后舵机偏移
#define RFSERVO_OFFSET -11  // 右前舵机偏移
#define RRSERVO_OFFSET -1   // 右后舵机偏移

// 避障参数
const float DANGER_ZONE_START = 30.0f;  // 危险区域开始距离
const float DANGER_ZONE_END = 10.0f;    // 危险区域结束距离
```

### WiFi配置
在 `main.cpp` 中可修改网络配置：

```cpp
// WiFi热点配置
const char* ssid = "Robot-AP";
const char* password = "12345678";
```

## 开发指南

### 添加新功能

1. **传感器集成**：
   - 在 `main.cpp` 中添加传感器初始化代码
   - 实现数据读取和处理函数
   - 更新Web界面显示新数据

2. **算法优化**：
   - 修改 `inverseKinematics()` 函数优化运动控制
   - 调整PID参数改善姿态稳定性
   - 优化避障算法提高导航精度

### 调试技巧

- 使用串口监视器查看调试信息
- 通过Web界面实时监控传感器数据
- 利用PlatformIO的调试功能进行代码调试

## 故障排除

### 常见问题

1. **编译错误**：
   - 检查库依赖是否正确安装
   - 确认平台和板卡配置正确

2. **连接问题**：
   - 确认硬件连接正确
   - 检查电源供应是否稳定

3. **控制异常**：
   - 检查舵机零点校准
   - 验证传感器数据准确性

### 调试工具

- **串口调试**：使用 `Serial.print()` 输出调试信息
- **Web界面**：实时查看传感器数据和机器人状态
- **PlatformIO调试器**：设置断点进行代码调试

## 贡献指南

欢迎提交Issue和Pull Request来改进项目！

1. Fork本项目
2. 创建功能分支：`git checkout -b feature/AmazingFeature`
3. 提交更改：`git commit -m 'Add some AmazingFeature'`
4. 推送分支：`git push origin feature/AmazingFeature`
5. 提交Pull Request

## 许可证

本项目采用MIT许可证 - 查看 [LICENSE](LICENSE) 文件了解详情

## 联系方式

- 项目维护者：[您的姓名]
- 邮箱：[您的邮箱]
- 项目地址：[GitHub仓库地址]

## 更新日志

### v1.0.0 (2024-XX-XX)
- 初始版本发布
- 基础双足行走功能
- Web监控界面
- 超声波避障功能

---

**注意**：本项目仍在积极开发中，功能可能会有所变动。建议定期查看更新日志了解最新进展。