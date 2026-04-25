#include <Arduino.h>
#include "SF_Servo.h"
#include "SF_IMU.h"
#include "sbus.h"
#include "bipedal_data.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "SF_BLDC.h"
#include <cmath>
//tockn
#include <cstring>   // for strchr
#include <cstdlib>   // for atof, strtol
#include <iostream>
#include "MPU6050_tockn.h"
#include <NewPing.h>  // 添加NewPing库
//#include <BluetoothA2DPSink.h>  // 蓝牙音频库
#define ROLL_OFFSET 0
#include <SoftwareSerial.h>
#include <sys/unistd.h>

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "index_html.h"

#define MEASURE_INTERVAL 50  // 测量间隔（毫秒）
#define PAN_SERVO_OFFSET 140
unsigned long lastMeasureTime = 0;  // 上次测量时间
SoftwareSerial OpenMVSerial(45, 48); // TX=GPIO45，RX=GPIO48
#define OPENMV_SERIAL OpenMVSerial // 定义OpenMV串口
SF_Servo servos = SF_Servo(Wire); //实例化初始化舵机


#define DISABLE_HEADING_RECOVERY 0   // 1=禁用恢复, 0=启用恢复


//tockn
MPU6050 mpu6050(Wire, 0.03, 0.97); //实例化初始化传感器
bfs::SbusRx sbusRx(&Serial1);//实例化初始化接收机
SF_BLDC motors = SF_BLDC(Serial2);//实例化初始化电机

// 定义HC - SR04的引脚
#define TRIGGER_PIN 37
#define ECHO_PIN 38
#define MAX_DISTANCE 200 // 最大检测距离，单位：厘米
const float OBSTACLE_VALID_DIST_MAX = 100.0f;   // 超声波有效最大距离（cm），超过此值视为无实际障碍物
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // 实例化NewPing对象
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) // 限幅函数
// 舵机云台控制相关变量
// 处理 OpenMV 串口数据
#define MAX_OPENMV_BUFFER 64
#define OPENMV_TIMEOUT 1000  // 1秒超时

// 摄像头参数
const int IMAGE_WIDTH = 240;     // OpenMV图像宽度（像素）
const int IMAGE_HEIGHT = 240;    // OpenMV图像高度（像素）
const float CAMERA_FOV_H = 60.0; // 水平视场角（度）
const float CAMERA_FOV_V = 60.0; // 垂直视场角（度）
const float CAMERA_HEIGHT = 15.0; // 摄像头安装高度（cm）
const float ROBOT_HEIGHT = 20.0;  // 机器人高度（cm）
bool emergencyTurn = false; // 紧急转向标志
//转向参数
//float avoidanceFactorLeft = 1.0f;
//float avoidanceFactorRight = 1.0f;
const float MAX_AVOIDANCE_FACTOR = 2.5f;  // 最大加速因子（适度增加）
const float MIN_AVOIDANCE_FACTOR = 0.4f;   // 最小减速因子（适度减速）
const float AVOIDANCE_SMOOTHING = 0.3f;   // 平滑系数(0.0-1.0) - 较小的值更平滑
const float DANGER_ZONE_START = 30.0f;     // 开始紧急避障的距离(cm)
const float DANGER_ZONE_END = 10.0f;       // 最大避障强度的距离(cm)
// 障碍物位置预测变量
float lastCentryX = 0;
uint32_t lastObstacleTime = 0;
const float OBSTACLE_TRACKING_SMOOTHING = 0.2f; // 位置预测平滑系数

void getRCValue();//获取遥控器的值。
void getMPUValue();//获取 MPU6050 传感器的数据。
void getMotorValue();//获取电机的状态或值。
void setServoAngle(uint16_t servoLeftFront, uint16_t servoLeftRear, uint16_t servoRightFront, uint16_t servoRightRear);
//设置舵机的角度。
void setRobotparam();//设置机器人的参数。
void robotRun();//运行机器人。
void inverseKinematics();//计算机器人的逆运动学，以确定关节角度。
void legControl();//控制机器人的腿部动作。
float selfCaliCentroid(float central);//自校准机器人的重心，并返回一个浮点数值。
void advancedPotentialFieldNavigation();
void updateObstacleData(); // 更新障碍物信息
void read(); // 读取串行端口的数据
void setup(); // 初始化函数
//定义了一些全局变量，用于存储机器人状态、控制参数和其他数据。
std::array<int16_t, bfs::SbusRx::NUM_CH()> sbusData;
//array:系统自带的数组容器，这里用于存储遥控器的数据。
robotposeparam robotPose;
//robotposeparam:结构体,机器人姿态参数，用于存储机器人的姿态数据。
robotmotionparam robotMotion;
//robotmotionparam:结构体,机器人运动参数，用于存储机器人的运动数据。
robotmode robotMode;
//robotmode:结构体,机器人模式，用于存储机器人的模式数据。
motorstatus motorStatus;
//motorstatus:结构体,电机状态，用于存储电机的状态数据。
controlparam controlTarget;
//controlparam:结构体,控制参数，用于存储控制参数数据。
coordinate coordTarget;
//coordinate:结构体,坐标目标，用于存储坐标目标数据。
IKparam IKParam;
//IKparam:结构体,逆运动学参数，用于存储逆运动学参数数据。
motorstarget motorsTarget;
//motorstarget:结构体,电机目标，用于存储电机目标数据。
//以上结构体在bipedal_data.h中均有定义，英文与中文对应
float robotLastHeight;
//robotLastHeight:浮点数,机器人最后的高度，用于存储机器人的最后高度数据。
int RCLastCH3Value;
//RCLastCH3Value:整型数,遥控器最后的通道3值，用于存储遥控器的最后通道3值数据。
int RCLastCH2Value;
//RCLastCH2Value:整型数,遥控器最后的通道2值，用于存储遥控器的最后通道2值数据。
SF_BLDC_DATA  BLDCData;
//BLDCData:结构体,电机数据，用于存储电机数据。
float wheel_X_off=2.2; //轮子X坐标偏置
uint32_t currTime;//当前时间
uint32_t prevTime;//上一次时间
//debug
int16_t alpha1ToAngle,beta1ToAngle,alpha2ToAngle,beta2ToAngle;
//alpha1ToAngle:整型数,（舵机）角度1转角度，用于存储角度1转角度数据,其他同理
float targetVoltage;
//targetVoltage:浮点数,目标电压，用于存储目标电压数据。
//定义了一些 PIDIncrement 类型的变量，并为它们的 Kp、Ki 和 Kd 参数赋初始值。
//这些变量用于实现不同控制目标的 PID 控制器
PIDIncrement PID_Roll{ .Kp = 0, .Ki = 0, .Kd = 0 };//控制机器人的滚转角度   
PIDIncrement PID_Gyrox{ .Kp = 0, .Ki = 0, .Kd = 0 };//控制机器人的陀螺仪X轴角速度  
PIDIncrement PID_Hegiht{ .Kp = 0.15, .Ki = 0, .Kd = 0 };//控制机器人的高度        
PIDIncrement PID_Y{ .Kp = 0.4, .Ki = 0, .Kd = 0 };//  控制机器人的Y坐标        
PIDIncrement PID_XCoord{ .Kp = 0.34, .Ki = 0, .Kd = 0 };//控制机器人的X坐标     
PIDIncrement PID_Stb{ .Kp = 0, .Ki = 0, .Kd = 0 }; //控制机器人的直立    
PIDIncrement PID_Streeing{ .Kp = 0.05, .Ki = 0, .Kd = 0 };//控制机器人的转向 
PIDIncrement PID_Forward{ .Kp = -0.8, .Ki = 0, .Kd = 0 };//控制机器人的前进后退
// PIDIncrement PID_Forward{ .Kp = -0.5, .Ki = 0, .Kd = 0 };
LowPassFilter LPFPitch{0.03 };  // 俯仰角速度低通滤波,减少噪声
LowPassFilter LPFRoll{0.05 };// 滚转角速度低通滤波,减少噪声
float GyroXPModify;//存储陀螺仪X轴角速度修正后的数据

PIDController PID_VEL{ 0, 0, 0, 1000,50 };//用于控制机器人的速度,在bipedal_data.h中有定义

int16_t x,yL,yR;//存储机器人的坐标数据
// 防抖动函数，对输入数据进行低通滤波处理
//自校准机器人的重心，并返回一个浮点数值。

static int speedMagnification = 1;
static int loopTimes = 0;
static float centry_x = 0;//存储障碍物检测数据
static float centry_y = 0;
static float centry_z = 0;
static char  obstale = 'O' ;//存储障碍物检测数据
static float HC_distance = 0; // 存储HC-SR04传感器的距离数据
float avoidanceLeftAdjust = 0.0f;
float avoidanceRightAdjust = 0.0f;
static int num = 0;
static int speed =1;
void calculateAvoidanceForces() ;//计算避障力
void updateObstacleData(float x_pixel, float z_pixel); // 更新障碍物数据
void calculateTargetAngle(float obstacleX) ; // 计算舵机目标角度
void read_openmv_data(); // 读取OpenMV数据
// 1. 势场避障参数配置
const float ATTRACTIVE_FORCE_GAIN = 1.0f;   // 目标引力系数
const float REPULSIVE_FORCE_MAX = 3.0f;     // 最大斥力系数
const float DANGER_RADIUS = 100.0f;          // 障碍物危险半径(cm)
const float ROBOT_WIDTH = 15.0f;            // 机器人宽度(cm)
const float MAX_SPEED = 50.0f;              // 最大移动速度(cm/s)
const float MIN_DISTANCE_ESCAPE = 40.0f;    // 触发虚拟障碍物的最小距离
const float ESCAPE_FORCE = 1.8f;            // 虚拟障碍物的逃逸力

// 添加普通避障的乘积因子变量
float avoidanceCameraFactorLeft = 1.0f;
float avoidanceCameraFactorRight = 1.0f;
// 恢复状态和参数
bool isRecovering = false;
uint32_t avoidanceStartTime = 0;      // 避障开始时间
uint32_t avoidanceDuration = 0;       // 避障持续时间
uint32_t recoveryStartTime = 0;       // 恢复开始时间
uint32_t recoveryDuration = 0;        // 恢复持续时间
float recoveryFactorLeft = 1.0f;      // 恢复过程中左轮的因子
float recoveryFactorRight = 1.0f;     // 恢复过程中右轮的因子

// ========== 航向恢复相关变量 ==========
bool isRecoveringHeading = false;       // 是否正在恢复航向
float deltaK_sum = 0.0f;                // 避障期间 (K1-K2) 的累积和（即左轮系数减右轮系数）
int obstacleCycleCount = 0;             // 避障经历的周期数（loop次数）
int recoveryCycleCount = 0;             // 恢复已经进行的周期数
float targetDeltaK = 0.0f;              // 恢复期间需要保持的 (K1-K2) 目标差值

// 用于处理恢复中断的变量
float remainingDeltaK = 0.0f;           // 被打断时剩余的未恢复转向量（以 deltaK 积分表示）
int remainingCycles = 0;                // 被打断时剩余的需要恢复的周期数
//超声波测量
unsigned int lastPingTime = 0;
unsigned int pingDuration = 0;
bool pingPending = false;

// 避障保持计时器
uint32_t obstacleActiveUntil = 0;
bool obstacleActive = false;
const uint32_t OBSTACLE_HOLD_TIME = 200;   // 保持时间（毫秒）

// Web服务器和WebSocket对象（全局）
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

struct OpenMVData {
    char status;      // 状态: 'O'=有障碍物, 'N'=无障碍物
    float x_pixel;    // 障碍物水平像素坐标
    float y_pixel;    // 障碍物垂直像素坐标
    float servo_angle;// 舵机当前角度
    bool valid;       // 数据是否有效
};
OpenMVData openmvData = {'N', 0, 0, 90, false};
// 2. 障碍物数据结构
struct Obstacle {
    float x;        // 横向位置 (cm) - 机器人坐标系
    float y;        // 纵向距离 (cm)
    float z;        // 障碍物高度 (cm)
    float width;    // 障碍物宽度 (cm)
    float vx;       // 横向速度 (cm/s)
    float vy;       // 纵向速度 (cm/s)
    unsigned long lastUpdateTime; // 最后更新时间
};
// 紧急避障状态机
enum EmergencyState {
    EM_NONE = 0,        // 无紧急情况
    EM_STOP,            // 正在停止
    EM_TURN,            // 正在原地转向
    EM_RECOVER          // 转向后短暂直行恢复（可选）
};
EmergencyState emergencyState = EM_NONE;
unsigned long emergencyStateStart = 0;   // 进入当前状态的时刻
float emergencyTurnDirection = 1.0f;     // 1 = 左转，-1 = 右转
float emergencyTargetYaw = 0.0f;         // 目标偏航角（可选）
// 3. 全局变量 - 避障系统状态
Obstacle currentObstacle = {0, 0, 0, 0, 0, 0}; // 当前障碍物
bool obstacleDetected = false;                  // 障碍物检测标志
bool inLocalMinimum = false;                    // 局部极小值状态
uint32_t escapeStartTime = 0;                   // 逃逸开始时间
float lastObstacleX = 0;                        // 上一次障碍物X位置
float lastObstacleY = 0;                        // 上一次障碍物Y位置
static int openmv=0;
void read_openmv_data() {
    static char buffer[64];
    static uint8_t idx = 0;
    static bool inFrame = false;
    static uint32_t lastValidTime = 0;

    while (OPENMV_SERIAL.available()) {
        char c = OPENMV_SERIAL.read();

        // 可选：打印每个字符（调试用）
        //Serial.printf("%c", c);

        if (!inFrame && c == 'S') {
            inFrame = true;
            idx = 0;
            buffer[idx++] = c;
        } else if (inFrame) {
            buffer[idx++] = c;
            if (idx >= sizeof(buffer) - 1) {
                // 帧过长，丢弃
                inFrame = false;
                idx = 0;
                continue;
            }
            if (c == 'E') {
                buffer[idx] = '\0';
                // 调试：打印完整帧
                //Serial.printf("\nFrame: %s\n", buffer);

                // 解析帧：S,O,123,100,150,5AE
                // 使用 strtok 或手动分割
                char *token = strtok(buffer, ",");
                if (token && token[0] == 'S') {
                    token = strtok(NULL, ","); // status
                    if (token) {
                        char status = token[0];
                        token = strtok(NULL, ","); // x
                        if (token) {
                            float x = atof(token);
                            token = strtok(NULL, ","); // y
                            if (token) {
                                float y = atof(token);
                                token = strtok(NULL, ","); // angle
                                if (token) {
                                    float angle = atof(token);
                                    token = strtok(NULL, ","); // checksum + E
                                    if (token) {
                                        // token 形如 "5AE"，最后一位是 E
                                        int len = strlen(token);
                                        if (len >= 3 && token[len-1] == 'E') {
                                            char checksumStr[3] = {token[len-3], token[len-2], '\0'};
                                            uint8_t recvChecksum = (uint8_t)strtol(checksumStr, NULL, 16);
                                            // 构建数据部分计算校验和
                                            char dataPart[32];
                                            snprintf(dataPart, sizeof(dataPart), "%c,%.0f,%.0f,%.0f",
                                                     status, x, y, angle);
                                            uint8_t calcChecksum = 0;
                                            for (int i = 0; dataPart[i]; i++) {
                                                calcChecksum ^= (uint8_t)dataPart[i];
                                            }
                                            if (calcChecksum == recvChecksum) {
                                                openmvData.status = status;
                                                openmvData.x_pixel = x;
                                                openmvData.y_pixel = y;
                                                openmvData.servo_angle = angle;
                                                openmvData.valid = true;
                                                lastValidTime = millis();
                                                // 可选打印
                                                Serial.printf("OK: %c,%.1f,%.1f,%.1f\n", status, x, y, angle);
                                            } else {
                                                openmvData.valid = false;
                                            }
                                        } else {
                                            openmvData.valid = false;
                                        }
                                    } else {
                                        openmvData.valid = false;
                                    }
                                } else {
                                    openmvData.valid = false;
                                }
                            } else {
                                openmvData.valid = false;
                            }
                        } else {
                            openmvData.valid = false;
                        }
                    } else {
                        openmvData.valid = false;
                    }
                } else {
                    openmvData.valid = false;
                }
                inFrame = false;
                idx = 0;
            }
        }
    }

    if (millis() - lastValidTime > 500) {
        openmvData.valid = false;
        inFrame = false;
        idx = 0;
    }
}

void read() {
    static String rc_received_chars;
    while (Serial.available() > 0) {
        char inChar = (char)Serial.read();
        rc_received_chars += inChar;
        if (inChar == '\n') {
            const char* d = rc_received_chars.c_str();
            sscanf(d, "%f", &wheel_X_off);
            rc_received_chars = "";
        }
    } 
    
}
uint8_t cnt;//一个无符号 8 位整数，用于存储计数器的值。
uint32_t now_time;//一个无符号 32 位整数，用于存储当前时间。
uint32_t last_time=micros();//一个无符号 32 位整数，用于存储上一次时间。
static bool HC_first= true; // 用于标记HC-SR04是否触发紧急避障
//用于初始化机器人系统的各个组件
void setup() {
    Serial.begin(921600);//波特率设置为 921600
    OpenMVSerial.begin(9600, SWSERIAL_8N1, 45, 48, false, 256);
    // 稳定性优化
    OpenMVSerial.enableIntTx(false); // 禁用发送中断
    OpenMVSerial.setTimeout(10);     // 设置超时
    setRobotparam();
    Wire.begin(1,2,400000UL);
    //Serial.printf("System Started!");
    //tockn
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);
    servos.init();
    servos.setAngleRange(0,300);
    servos.setPluseRange(500,2500);
    sbusRx.Begin(SBUSPIN,-1);
    motors.init();
    motors.setModes(4,4);
    delay(6000);
    currTime = micros();
    coordTarget.x = 0;
    coordTarget.yLeft = ROBOT_LOWEST_FOR_MOT;  // 设置为最低腿高
    coordTarget.yRight = ROBOT_LOWEST_FOR_MOT; // 设置为最低腿高
    cnt = ROBOT_LOWEST_FOR_MOT;                // 设置为最低腿高
    robotMotion.updown = ROBOT_LOWEST_FOR_MOT; // 设置初始腿高为最低腿高
    robotMotion.forward = 5.0f;                   // 设置初始前进速度为0

    // --- 添加 WiFi 热点和 Web 服务器初始化 ---
    const char* ssid = "MyRobot";
    const char* password = "12345678";
    WiFi.softAP(ssid, password);
    Serial.printf("AP started! IP: %s\n", WiFi.softAPIP().toString().c_str());

    // 处理 WebSocket 事件
    ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
        if(type == WS_EVT_CONNECT) Serial.println("Client connected");
        else if(type == WS_EVT_DISCONNECT) Serial.println("Client disconnected");
    });
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
    });
    server.begin();

}
//主函数，不断循环运行机器人系统,在Arduino环境中，setup() 函数只会运行一次，而 loop() 函数会一直循环运行。
//static float multiplyFactorALL1 = 1.0f;
//static float multiplyFactorALL2 = 1.0f;
void loop() {
    unsigned long currentTime = millis();
    float multiplyFactor1 = 1.0f;
    float multiplyFactor2 = 1.0f;
    openmv=0;
    // ========== 超声波测距 + 中位数滤波 ==========
    #define MEDIAN_WINDOW 5  // 窗口大小

    if (millis() - lastMeasureTime > MEASURE_INTERVAL) {
        lastMeasureTime = millis();
        
        // 1. 获取原始距离
        unsigned int uS = sonar.ping();
        float rawDist = uS / US_ROUNDTRIP_CM;
        if (rawDist == 0) rawDist = MAX_DISTANCE;

        // 2. 中位数滤波环形缓冲区
        static float buf[MEDIAN_WINDOW];
        static uint8_t idx = 0;
        static bool bufFilled = false;
        
        buf[idx] = rawDist;
        idx = (idx + 1) % MEDIAN_WINDOW;
        if (idx == 0) bufFilled = true;
        
        // 3. 计算中位数
        if (bufFilled) {
            // 复制缓冲区并排序（简单冒泡，窗口小效率足够）
            float sorted[MEDIAN_WINDOW];
            memcpy(sorted, buf, sizeof(sorted));
            for (int i = 0; i < MEDIAN_WINDOW - 1; i++) {
                for (int j = i + 1; j < MEDIAN_WINDOW; j++) {
                    if (sorted[i] > sorted[j]) {
                        float tmp = sorted[i];
                        sorted[i] = sorted[j];
                        sorted[j] = tmp;
                    }
                }
            }
            HC_distance = sorted[MEDIAN_WINDOW / 2];  // 取中位数
        } else {
            // 缓冲区未满时，直接使用原始值（或前一帧值）
            HC_distance = rawDist;
        }

        // 4. 可选：突变抑制（与中位数滤波二选一，若采用中位数可删除此段）
        // 中位数已能有效抑制突变，故此处省略或保留均可
    }
    //Serial.printf("HC-SR04 Distance: %f cm\n", HC_distance);
    /*if (millis() - lastMeasureTime > MEASURE_INTERVAL) {
    lastMeasureTime = millis();
    unsigned int uS = sonar.ping();
    Serial.printf("Raw uS: %d ，", uS);
    // 暂时注释滤波逻辑
    HC_distance = (uS == 0) ? MAX_DISTANCE : uS / US_ROUNDTRIP_CM;
    }*/
    
    read_openmv_data();  // 读取OpenMV数据
    updateObstacleData(); // 更新障碍物信息
    // 3. 设置紧急避障标志
    //emergencyOverride = (HC_distance > 0 && HC_distance < DANGER_ZONE_START);    
    // ========== 紧急避障触发检测 ==========
    static bool wasEmergency = false;
    bool isEmergencyNow = (HC_distance > 0 && HC_distance < DANGER_ZONE_START);

    if (isEmergencyNow && !wasEmergency) {
        // 刚进入紧急避障：启动状态机
        emergencyState = EM_STOP;
        emergencyStateStart = millis();
        
        // 决定转向方向：根据摄像头障碍物位置，若无则默认左转
        if (openmvData.valid && openmvData.status == 'O') {
            emergencyTurnDirection = (openmvData.x_pixel > 120) ? 1.0f : -1.0f;
        } else {
            emergencyTurnDirection = 1.0f;  // 默认左转
        }
        
        // 清空航向恢复状态（避免干扰）
        isRecoveringHeading = false;
        deltaK_sum = 0.0f;
        obstacleCycleCount = 0;
    }
    wasEmergency = isEmergencyNow;
    calculateAvoidanceForces();// 计算避障力
    //read();//读取串行端口的数据
    getRCValue();//获取遥控器的值
    getMPUValue();//获取 MPU6050 传感器的数据
    getMotorValue();//获取电机的状态或值
    read();//读取串行端口的数据

    legControl();//控制机器人的腿部动作
    inverseKinematics();//计算机器人的逆运动学，以确定关节角度
    
    robotRun();//运行机器人系统
    // 广播障碍物信息。
    static unsigned long lastSend = 0;
    if (millis() - lastSend > 1000) {
        lastSend = millis();

        // 构建 JSON 格式的障碍物信息
        StaticJsonDocument<200> doc;
        doc["distance"] = HC_distance;
        doc["status"] = obstacleDetected ? "Obstacle" : "Clear";
        doc["angle"] = openmvData.servo_angle;
        doc["x_pixel"] = openmvData.x_pixel;
        // 可以添加更多数据

        String jsonString;
        serializeJson(doc, jsonString);

        // 通过 WebSocket 广播给所有连接的客户端（手机网页）
        ws.textAll(jsonString);
    }
    if (obstacleDetected) { // 假设这是你的障碍物标志
        StaticJsonDocument<200> doc;
        doc["type"] = "alert";
        doc["message"] = "obstacle_detected";
        
        String jsonString;
        serializeJson(doc, jsonString);
        ws.textAll(jsonString);
    }
    if (openmv==1){
        /*Serial.printf(" HC_distance: %f \n", HC_distance);
        Serial.printf(" 左轮因子: %f \n", avoidanceCameraFactorRight);
        Serial.printf(" 右轮因子: %f \n", avoidanceCameraFactorLeft);
        Serial.printf("----------------------------------\n");*/}
        
    now_time = micros();//获取当前时间
    float Ts = (now_time - last_time) * 1e-6f;//计算时间间隔
    if (Ts > 1.0) {
        //这一段代码检查时间间隔是否大于 1 秒。如果是，则更新 last_time 为当前时间 now_time。
        last_time = now_time;
        //轮子X坐标，左轮Y坐标，右轮Y坐标，机器人左右横滚角，机器人前后俯仰角，平均速度，右轮速度，左轮速度
        //Serial.printf("%f,%f,%f,%f,%f,%f,%f,%f\n",coordTarget.x,coordTarget.yLeft,coordTarget.yRight,robotPose.roll,robotPose.pitch,robotPose.speedAvg,motorStatus.M0SpdDir*motorStatus.M0Speed,motorStatus.M1SpdDir*motorStatus.M1Speed);
        //平均速度；右轮速度，左轮速度
        //Serial.printf("%f,%f,%f\n",robotPose.speedAvg,motorStatus.M0SpdDir*motorStatus.M0Speed,motorStatus.M1SpdDir*motorStatus.M1Speed);
        //将机器人的一些状态信息打印到串行端口(vofa+)
        //解释motorStatus.M0Speed是S1反馈的电机速度（轮子向前转动为正），如果反馈的速度与实际相反需要手动设置motorStatus.M0SpdDir取反，反之不用
    }
    
}
//用于设置机器人系统的各种参数和限制
void setRobotparam(){
    //设置机体的运动限制
    robotMotion.heightest = ROBOT_HIGHEST;
    robotMotion.lowest = ROBOT_LOWEST_FOR_MOT;
    robotMotion.forwardLimit = 15;
    robotMotion.rollLimit = 20;

    robotMode.motorEnable = true;//设置电机使能标志为 true，表示电机已使能。
    robotMode.servoEnable = true;//设置舵机使能标志为 true，表示舵机已使能。
    robotMode.printFlag = false;//设置打印标志为 false，表示不打印信息。
    robotMode.mode = ROBOTMODE_MOTION;//设置机器人模式为 ROBOTMODE_MOTION，表示机器人处于运动模式。

    //电机速度控制方向
    motorStatus.M0Dir = 1;
    motorStatus.M1Dir = -1;
    //电机速度反馈方向
    motorStatus.M0SpdDir = -1;
    motorStatus.M1SpdDir = 1;
    //运动学逆解会用到
}
// 读取遥控器
void getRCValue(){
    // 设置默认的遥控器值，确保在遥控器未开启时机器人也能稳定运行
    static bool firstRun = true;
    if (firstRun) {
        // 初始化为中间值
        RCValue[0] = (RCCHANNEL_MIN + RCCHANNEL_MAX) / 2;  // 转向通道设为中间值
        RCValue[1] = (RCCHANNEL_MIN + RCCHANNEL_MAX) / 2;  // 前进后退通道设为中间值
        RCValue[2] = (RCCHANNEL3_MIN + RCCHANNEL3_MAX) / 2; // 腿高通道设为中间值
        RCValue[3] = (RCCHANNEL_MIN + RCCHANNEL_MAX) / 2;  // 横滚通道设为中间值
        RCValue[4] = RCCHANNEL3_MIN;  // 使能通道设为最小值（使能状态）
        RCValue[5] = RCCHANNEL3_MIN;  // 模式通道设为最小值（运动模式）
        firstRun = false;
    }
    
    if(sbusRx.Read()){
        sbusData = sbusRx.ch();
        RCValue[0] = sbusData[0];
        RCValue[1] = sbusData[1];
        RCValue[2] = sbusData[2];
        RCValue[3] = sbusData[3];
        RCValue[4] = sbusData[4];
        RCValue[5] = sbusData[5];

        RCValue[0] = _constrain(RCValue[0], RCCHANNEL_MIN, RCCHANNEL_MAX);//限制遥控器的值在最大值和最小值之间,下面同理
        RCValue[1] = _constrain(RCValue[1], RCCHANNEL_MIN, RCCHANNEL_MAX);
        RCValue[2] = _constrain(RCValue[2], RCCHANNEL3_MIN, RCCHANNEL3_MAX);
        RCValue[3] = _constrain(RCValue[3], RCCHANNEL_MIN, RCCHANNEL_MAX);
        //Serial.printf("%d,%d,%d,%d,%d,%d\n",RCValue[0],RCValue[1],RCValue[2],RCValue[3],RCValue[4],RCValue[5]);
    }
}
void getMPUValue(){
    //用于从 MPU6050 传感器获取数据并更新机器人的姿态信息
    mpu6050.update();//更新 MPU6050 传感器的数据。
    //tockn
    //通过调用 mpu6050.getAngleX()、mpu6050.getAngleY() 和 mpu6050.getAngleZ() 方法获取机器人的俯仰角、横滚角和偏航角。
    robotPose.pitch = -mpu6050.getAngleX();// 摆放原因导致调换
    robotPose.roll = mpu6050.getAngleY()+ROLL_OFFSET;// 摆放原因导致调换
    robotPose.yaw = mpu6050.getAngleZ();
    robotPose.GyroX = mpu6050.getGyroY(); 
    robotPose.GyroY = -mpu6050.getGyroX();
    robotPose.GyroZ = -mpu6050.getGyroZ();
    // Serial.printf("%f,%f,%f,%f,%f,%f\n",robotPose.pitch,robotPose.roll,robotPose.yaw,robotPose.GyroX,robotPose.GyroY,robotPose.GyroZ);
}
void getMotorValue(){
    //用于获取电机的状态或值，并更新相关变量
    BLDCData = motors.getBLDCData();
    //调用 motors 对象的 getBLDCData 方法，获取无刷直流电机（BLDC）的数据，并将其存储在 BLDCData 变量中。
    motorStatus.M0Speed = BLDCData.M0_Vel;
    motorStatus.M1Speed = BLDCData.M1_Vel;
    //从 BLDCData 中提取电机速度数据，并存储到 motorStatus.M0Speed 和 motorStatus.M1Speed 变量中。
}
//用于设置四个舵机的角度
void setServoAngle(uint16_t servoLeftFront, uint16_t servoLeftRear, uint16_t servoRightFront, uint16_t servoRightRear){
    // servos.setPWM(LFSERVO_CH, 0, servoLeftFront);
    // servos.setPWM(LRSERVO_CH, 0, servoLeftRear);
    // servos.setPWM(RFSERVO_CH, 0, servoRightFront);
    // servos.setPWM(RRSERVO_CH, 0, servoRightRear);

    //TEST
    // Serial.printf("%d,%d,%d,%d\n",servoLeftFront,servoLeftRear,servoRightFront,servoRightRear);
    // Serial.printf("%d,%d,%d,%d\n",90+servoLeftFront,90+servoLeftRear,270-servoRightFront,270-servoRightRear);
    // Serial.println();
    servos.setAngle(3, 90+LFSERVO_OFFSET+servoLeftFront);
    servos.setAngle(4, 90+LRSERVO_OFFSET+servoLeftRear);
    servos.setAngle(5, 270+RFSERVO_OFFSET-servoRightFront);
    servos.setAngle(6, 270+RRSERVO_OFFSET-servoRightRear);
    //将角度值设置到舵机（实例）上，servoleftFront在逆解函数中被求出，其他同理
}
//腿高度控制
void legControl(){
    // ========== 紧急避障状态机（优先级最高） ==========
    const unsigned long EMERGENCY_TURN_DURATION = 5000;   // 紧急转向持续时间（ms）
    const float EMERGENCY_TURN_SPEED = 8.0f;             // 紧急转向速度（更强的转向）
    
    if (emergencyState != EM_NONE) {
        unsigned long now = millis();
        
        // 紧急避障期间：强制原地转向，不受任何干扰
        robotMotion.forward = 0.0f;           // 强制停止前进
        robotMotion.turn = emergencyTurnDirection * EMERGENCY_TURN_SPEED;  // 强制原地转向
        
        // 添加紧急避障调试信息
        /*Serial.printf("🚨 紧急避障中: 状态=%d, 转向方向=%s, 剩余时间=%lu ms\n",
                      emergencyState, 
                      emergencyTurnDirection > 0 ? "左转" : "右转",
                      EMERGENCY_TURN_DURATION - (now - emergencyStateStart));
        */
        // 检查转向是否完成
        if (millis() - emergencyStateStart > EMERGENCY_TURN_DURATION) {
            emergencyState = EM_NONE;          // 紧急避障结束
            //Serial.println("✅ 紧急避障完成，恢复正常行走");
        }
        
        // 注意：状态机运行期间，跳过所有其他控制逻辑
        // 直接设置电机控制，确保不受干扰
    } else {
        // 非紧急状态，使用原来的默认值
        robotMotion.turn = 0;
        robotMotion.forward = 5.0;
    }
    
    float e_H;
    float E_H;

    // 遥控器控制已被禁用，机器人将根据避障系统自主行走和避障

    // 禁用遥控器对腿高的控制，始终保持最低腿高
    robotMotion.updown = ROBOT_LOWEST_FOR_MOT;  // 固定为最低腿高
    robotMotion.roll = map(RCValue[3], RCCHANNEL_MIN, RCCHANNEL_MAX, -1*robotMotion.rollLimit, robotMotion.rollLimit);//横滚，左左右 --- R3

    // 禁用遥控器对电机和舵机使能状态的控制，始终保持使能状态
    robotMode.motorEnable = true;
    robotMode.servoEnable = true;

    // 禁用遥控器对控制模式的控制，始终保持ROBOTMODE_MOTION模式
    robotMode.mode = ROBOTMODE_MOTION;
    robotMotion.lowest = ROBOT_LOWEST_FOR_MOT;
    robotMode.printFlag = false;

    if(abs(RCValue[2] - RCLastCH3Value) >= 5){
        GyroXPModify = 0.05f;
        RCLastCH3Value = RCValue[2];
    }else{
        GyroXPModify = 0.05f;
    }

    // controlTarget.forward = PID_Forward.Kp*(robotMotion.forward - (motorStatus.M0SpdDir*motorStatus.M0Speed + motorStatus.M1SpdDir*motorStatus.M1Speed)/2);
    controlTarget.forward = PID_Forward.Kp*(robotMotion.forward - 0);
    controlTarget.forward = _constrain(controlTarget.forward, -20, 20);
    // coordTarget.x = coordTarget.x + PID_XCoord.Kp*(controlTarget.forward - coordTarget.x);

    // 禁用防侧倾能力
    e_H = 0;
    E_H = 0;
    // Serial.printf("%f,%f\n",e_H,E_H);

    controlTarget.legLeft = controlTarget.legLeft + PID_Hegiht.Kp * (robotMotion.updown - controlTarget.legLeft);
    controlTarget.legRight = controlTarget.legRight + PID_Hegiht.Kp * (robotMotion.updown - controlTarget.legRight);

    controlTarget.legRollLeft = _constrain(controlTarget.legRollLeft + E_H, -robotMotion.lowest, robotMotion.lowest);
    controlTarget.legRollRight = _constrain(controlTarget.legRollRight - E_H, -robotMotion.lowest, robotMotion.lowest);

    // coordTarget.yLeft = _constrain(coordTarget.yLeft + PID_Y.Kp * ((robotMotion.updown+controlTarget.legRollLeft)-coordTarget.yLeft),robotMotion.lowest, robotMotion.heightest);
    // coordTarget.yRight = _constrain(coordTarget.yRight + PID_Y.Kp * ((robotMotion.updown+controlTarget.legRollRight)-coordTarget.yRight),robotMotion.lowest, robotMotion.heightest);

    coordTarget.yLeft = _constrain(controlTarget.legRollLeft + controlTarget.legLeft, robotMotion.lowest, robotMotion.heightest);
    coordTarget.yRight = _constrain(controlTarget.legRollRight + controlTarget.legRight, robotMotion.lowest, robotMotion.heightest);

    coordTarget.x = coordTarget.x + PID_XCoord.Kp * (controlTarget.forward - coordTarget.x)+ wheel_X_off;

    robotPose.height = (coordTarget.yLeft + coordTarget.yRight)/2;

    //debug
    // coordTarget.x = 0;
    // coordTarget.yLeft = 100;
    // coordTarget.yRight = 100;
}
//逆解函数
void inverseKinematics(){
    // coordTarget.xLeft = coordTarget.x;
    // coordTarget.xRight = coordTarget.xLeft;

    IKParam.XLeft = coordTarget.x;
    IKParam.YLeft = coordTarget.yLeft;
    IKParam.XRight = coordTarget.x;
    IKParam.YRight = coordTarget.yRight;

    float a1 = 2 * IKParam.XLeft * L1;
    float b1 = 2 * IKParam.YLeft * L1;
    float c1 = IKParam.XLeft * IKParam.XLeft + IKParam.YLeft * IKParam.YLeft + L1 * L1 - L2 * L2;
    float d1 = 2 * L4 * (IKParam.XLeft - L5);
    float e1 = 2 * L4 * IKParam.YLeft;
    float f1 = ((IKParam.XLeft - L5) * (IKParam.XLeft - L5) + L4 * L4 + IKParam.YLeft * IKParam.YLeft - L3 * L3);

    IKParam.alpha1 = 2 * atan((b1 + sqrt((a1 * a1) + (b1 * b1) - (c1 * c1))) / (a1 + c1));
    IKParam.beta1 = 2 * atan((e1 - sqrt((d1 * d1) + e1 * e1 - (f1 * f1))) / (d1 + f1));

    //限制解算角度的范围
    if (IKParam.alpha1 < 0)
        IKParam.alpha1 = IKParam.alpha1 + 2 * PI;

    if (IKParam.beta1 < 0)
        IKParam.beta1 = 0;

    alpha1ToAngle = (int)((IKParam.alpha1 / 6.28) * 360);//弧度转角度
    beta1ToAngle = (int)((IKParam.beta1 / 6.28) * 360);

    motorsTarget.servoLeftRear = (int)map(alpha1ToAngle, 0, 180, 103, 327);  // 1号舵机 500~1500us(500~2500)
    motorsTarget.servoLeftFront = (int)map(beta1ToAngle, 0, 180, 103, 327);  // 2号舵机

    float a2 = 2 * IKParam.XRight * L1;
    float b2 = 2 * IKParam.YRight * L1;
    float c2 = IKParam.XRight * IKParam.XRight + IKParam.YRight * IKParam.YRight + L1 * L1 - L2 * L2;
    float d2 = 2 * L4 * (IKParam.XRight - L5);
    float e2 = 2 * L4 * IKParam.YRight;
    float f2 = ((IKParam.XRight - L5) * (IKParam.XRight - L5) + L4 * L4 + IKParam.YRight * IKParam.YRight - L3 * L3);

    IKParam.alpha2 = 2 * atan((b2 + sqrt((a2 * a2) + (b2 * b2) - (c2 * c2))) / (a2 + c2));
    IKParam.beta2 = 2 * atan((e2 - sqrt((d2 * d2) + e2 * e2 - (f2 * f2))) / (d2 + f2));

    if (IKParam.alpha2 < 0)
        IKParam.alpha2 = IKParam.alpha2 + 2 * PI;

    if (IKParam.beta2 < 0)
        IKParam.beta2 = 0;

    alpha2ToAngle = (int)((IKParam.alpha2 / 6.28) * 360);//todo
    beta2ToAngle = (int)((IKParam.beta2 / 6.28) * 360);

    motorsTarget.servoRightFront = (int)map(alpha2ToAngle, 0, 180, 103, 327);  // 1号舵机 500~1500us(500~2500)
    motorsTarget.servoRightRear = (int)map(beta2ToAngle, 0, 180, 103, 327);  // 2号舵机

    // motorsTarget.servoRightRear = (int)map(alpha2ToAngle, 0, 180, 103, 327);  // 1号舵机 500~1500us(500~2500)
    // motorsTarget.servoRightFront = (int)map(beta2ToAngle, 0, 180, 103, 327);  // 2号舵机

    if(robotMode.servoEnable){
        //debug
        setServoAngle(beta1ToAngle,alpha1ToAngle,beta2ToAngle,alpha2ToAngle);
        // setServoAngle(motorsTarget.servoLeftFront, motorsTarget.servoLeftRear, motorsTarget.servoRightFront, motorsTarget.servoRightRear);
        // Serial.printf("%d,%d,%d,%d\n",motorsTarget.servoLeftFront, motorsTarget.servoLeftRear, motorsTarget.servoRightFront, motorsTarget.servoRightRear);
    }
}
//机器人运行函数
// 定义一个结构体来存储 PID 参数
struct PIDParams {
    float P;
    float I;
    float D;
};

// 定义 PID_Stb 结构体
struct PIDStbParams {
    float Kp;
    float Kd;
    float originalKp; // 保存原始 Kp 值
    float originalKd; // 保存原始 Kd 值
};

PIDStbParams PID_Stb2;

// 假设定义最高腿高
const float MAX_LEG_HEIGHT = 80; // 可根据实际情况修改

// 计算目标 PID 参数的函数
PIDParams calculateTargetPIDParams(float height) {
    PIDParams targetParams;
    targetParams.P = (0.00002 * height * height - 0.007 * height + 0.669) * 1.8;
    targetParams.I = 0.00153 * 0.6;
    targetParams.D = (0.0000002 * height * height - 0.00001 * height - 0.01) * 0.1;
    return targetParams;
}

// 防抖动函数，对输入数据进行低通滤波处理
float antiJitterFilter2(float input, float& filteredValue, float alpha) {
    // 低通滤波公式：filteredValue = alpha * filteredValue + (1 - alpha) * input
    filteredValue = alpha * filteredValue + (1 - alpha) * input;
    return filteredValue;
}

// 变腿高稳定函数
/*void adjustLegHeightStably(float targetHeight, float& currentHeight, PIDParams& currentPIDParams, bool& isHighLegHeight) {
    const float stepSize = 0.5; // 每次调整的步长，可以根据实际情况调整
    const float maxDelta = 1.0; // 最大允许的高度变化量，避免变化过快
    // 计算高度差
    float deltaHeight = targetHeight - currentHeight;
    // 限制高度变化量
    if (abs(deltaHeight) > maxDelta) {
        deltaHeight = (deltaHeight > 0) ? maxDelta : -maxDelta;
    }
    // 逐步调整高度
    if (abs(deltaHeight) > stepSize) {
        currentHeight += (deltaHeight > 0) ? stepSize : -stepSize;
    } else {
        currentHeight = targetHeight;
    }
    // 计算目标 PID 参数
    PIDParams targetPIDParams = calculateTargetPIDParams(currentHeight);
    // 根据腿高调整 PID 参数调整步长，腿高越高，调整越慢
    float paramStep = 0.08;
    if (currentHeight > MAX_LEG_HEIGHT * 0.25) { // 提前开始减慢调整速度
        paramStep = 0.05;
    }
    if (currentHeight > MAX_LEG_HEIGHT * 0.5) {
        paramStep = 0.02;
    }
    // 逐步调整 PID 参数
    currentPIDParams.P += (targetPIDParams.P - currentPIDParams.P) * paramStep;
    currentPIDParams.I += (targetPIDParams.I - currentPIDParams.I) * paramStep;
    currentPIDParams.D += (targetPIDParams.D - currentPIDParams.D) * paramStep;
    // 当腿高较高时，增加额外的稳定性限制
    if (currentHeight > MAX_LEG_HEIGHT * 0.5) { 
        if (!isHighLegHeight) {         // 记录高腿高状态
            isHighLegHeight = true;           // 保存原始参数
            PID_Stb2.originalKp = (0.0003 * currentHeight * currentHeight - 0.0488 * currentHeight + 3.5798) * 0.8;
            PID_Stb2.originalKd = (-0.000002 * currentHeight * currentHeight + 0.0005 * currentHeight - 0.0043) * 1.6;            // 调整参数
            PID_Stb.Kp = PID_Stb2.originalKp * 0.8; // 适当降低比例系数，减少响应的剧烈程度
            PID_Stb.Kd = PID_Stb2.originalKd * 1.2; // 适当增加微分系数，增强对变化的抑制
        }
    } else {
        if (isHighLegHeight) {
            // 还原参数
            isHighLegHeight = false;
            PID_Stb.Kp = PID_Stb2.originalKp;
            PID_Stb.Kd = PID_Stb2.originalKd;
        }
    }

    // 更新其他相关参数
    if (!isHighLegHeight) {
        PID_Stb.Kp = (0.0003 * currentHeight * currentHeight - 0.0488 * currentHeight + 3.5798) * 0.8;
        PID_Stb.Kd = (-0.000002 * currentHeight * currentHeight + 0.0005 * currentHeight - 0.0043) * 1.6;
    }
    PID_Roll.Kp = (0.001 * currentHeight * currentHeight - 0.2281 * currentHeight + 17.495) * 1.3;
    PID_Gyrox.Kp = (0.0000009 * currentHeight * currentHeight - 0.0005 * currentHeight + 0.091) * GyroXPModify;
    robotMotion.turnLimit = (0.000009 * currentHeight * currentHeight - 0.005 * currentHeight + 120.5104) * 1;
}*/
void robotRun() {
   float velLeft,velRight;
   float wheelControl;
  
  if(robotPose.height != robotLastHeight){
    if(70 <= robotPose.height < 110)
      PID_VEL.P = -0.0067 * robotPose.height + 1.12;
    else if(110 <= robotPose.height <= 130)
      PID_VEL.P = 0.4;
    // PID_VEL.P = (0.00002 * robotPose.height *  robotPose.height - 0.007 *  robotPose.height + 0.669) * 1.8;//1.8
    PID_VEL.I = 0.00153 * 0.6;  //1
    PID_VEL.D = (0.0000002 * robotPose.height * robotPose.height - 0.00001 * robotPose.height - 0.01) * 0.1;

    PID_Stb.Kp = (0.0003 * robotPose.height * robotPose.height - 0.0488 * robotPose.height + 3.5798) * 0.8;     //0.8
    PID_Stb.Kd = (-0.000002 * robotPose.height * robotPose.height + 0.0005 * robotPose.height - 0.0043) * 1.6;  //1.6

    PID_Roll.Kp = (0.001 * robotPose.height * robotPose.height - 0.2281 * robotPose.height + 17.495) * 1.3;

    PID_Gyrox.Kp = (0.0000009 * robotPose.height * robotPose.height - 0.0005 * robotPose.height + 0.091) * GyroXPModify;


    robotMotion.turnLimit = (0.000009 * robotPose.height * robotPose.height - 0.005 * robotPose.height + 120.5104) * 1;

    robotLastHeight = robotPose.height;
  }
  
  // 初始代码
  robotPose.speedAvg = (motorStatus.M0SpdDir*motorStatus.M0Speed + motorStatus.M1SpdDir*motorStatus.M1Speed)/2;

  // 调试代码
  // robotPose.speedAvg = (motorStatus.M0Dir*motorStatus.M0Speed*+ motorStatus.M1Dir*motorStatus.M1Speed)/2;


  if(RCLastCH2Value == RCValue[1]){
    controlTarget.centerAngleOffset = selfCaliCentroid(controlTarget.centerAngleOffset);
  }
  RCLastCH2Value = RCValue[1];

  // 调试代码
  // controlTarget.velocity = PID_VEL(robotPose.speedAvg);//速度环

  // 自主行走和避障模式
  // 使用默认速度进行前进，速度环控制
  float defaultSpeed = robotMotion.forward;  // 使用 legControl 中设置的默认速度;  
  controlTarget.velocity = PID_VEL(defaultSpeed - robotPose.speedAvg);//速度环，使用默认速度
  controlTarget.differVel = PID_Streeing.Kp*(robotMotion.turn-robotPose.GyroZ);//转向，robotMotion.turn已设为0
  targetVoltage = PID_Stb.Kp*(controlTarget.velocity + controlTarget.centerAngleOffset - robotPose.pitch) - PID_Stb.Kd * robotPose.GyroY;//直立环 输出控制的电机的目标速度

  // 调试代码
  // motorsTarget.motorLeft = motorStatus.M0Dir * (targetVoltage);
  // motorsTarget.motorRight = -motorStatus.M1Dir * (targetVoltage);

  // 初始代码
  motorsTarget.motorLeft = motorStatus.M0Dir * (targetVoltage + controlTarget.differVel);
  motorsTarget.motorRight = motorStatus.M1Dir * (targetVoltage - controlTarget.differVel);

    motorsTarget.motorLeft  *= avoidanceCameraFactorLeft;
    motorsTarget.motorRight *= avoidanceCameraFactorRight; //Serial.printf("左轮速%f,右轮速%f\n",motorsTarget.motorLeft,motorsTarget.motorRight);
  //Serial.printf("左轮系数，右轮系数\n",multiplyFactorALL1, multiplyFactorALL2);
  //Serial.printf("----------------------------------------------------------------\n");
  if (robotMode.motorEnable == 1 && robotPose.pitch <= 40 && robotPose.pitch >= -35) {
    motorsTarget.motorLeft = _constrain(motorsTarget.motorLeft, -5.7, 5.7);
    motorsTarget.motorRight = _constrain(motorsTarget.motorRight, -5.7, 5.7);
    motors.setTargets(motorsTarget.motorLeft,motorsTarget.motorRight);
    // motors.setTargets(2,2);
  } else {
    motors.setTargets(0,0);
  }
 
}

float selfCaliGain = 0.8;
float selfcaliOffset = 0;
#define SELF_CALI_RANGE 7
#define CENTER_ANGLE_OFFSET 3
float selfCaliCentroid(float central){
    static int i = 0;
    if(i == 40){
        if(fabs(robotPose.speedAvg) > 1){
            selfcaliOffset = selfCaliGain * -1 * robotPose.speedAvg;
            selfcaliOffset = _constrain(selfcaliOffset, -0.5, 0.5);
            central += selfcaliOffset;
        }
        i = 0;
    }else{
        ++i;   
    }
    central = _constrain(central, CENTER_ANGLE_OFFSET-SELF_CALI_RANGE, CENTER_ANGLE_OFFSET+SELF_CALI_RANGE);

    // central = _constrain(central, -10, 10);

    return central;
}

// 4. 核心函数：计算合力并生成控制指令
void calculateAvoidanceForces_OLD() {
// 检查避障保持计时器
if (obstacleActive && millis() > obstacleActiveUntil) {
    obstacleActive = false;
    obstacleDetected = false;   // 同步清除
        // 可选：恢复默认因子
    avoidanceCameraFactorLeft = 1.0f;
    avoidanceCameraFactorRight = 1.0f;
    // 如果有航向恢复逻辑，也可在此重置
    }

// 如果不在避障活跃状态，则直接退出（不执行避障计算）
if (!obstacleActive) {
    // 确保因子为1
    avoidanceCameraFactorLeft = 1.0f;
    avoidanceCameraFactorRight = 1.0f;
    return;
    }    
    
    // 紧急避障触发时，完全禁用航向恢复逻辑，直接使用紧急因子
if (HC_distance < DANGER_ZONE_START && HC_distance > 0) {
    // 清空所有航向恢复状态
    //isRecoveringHeading = false;
    //deltaK_sum = 0.0f;
    //obstacleCycleCount = 0;
    //remainingDeltaK = 0.0f;
    //remainingCycles = 0;
    // 紧急避障因子已在 loop 中通过 multiplyFactorALL1/2 应用，这里摄像头因子设为1
    //avoidanceCameraFactorLeft = 1.0f;
    //avoidanceCameraFactorRight = 1.0f;
    return;
}
    #if DISABLE_HEADING_RECOVERY
    // 完全禁用航向恢复：强制退出恢复模式并清空累积数据
    isRecoveringHeading = false;
    deltaK_sum = 0.0f;
    obstacleCycleCount = 0;
    recoveryCycleCount = 0;
    targetDeltaK = 0.0f;
    remainingDeltaK = 0.0f;
    remainingCycles = 0;
    #endif 
    
    // 1. 重置摄像头避障因子（默认无避障）
    float cameraFactorLeft = 1.0f;
    float cameraFactorRight = 1.0f;

    // 2. 处理航向恢复模式（优先级最高）
    if (isRecoveringHeading) {
        // 恢复期间：强制设置左右轮系数差为目标差值
        // 使用对称分配：左 = 1 + delta/2，右 = 1 - delta/2
        float delta = targetDeltaK;
        cameraFactorLeft = 1.0f + delta / 2.0f;
        cameraFactorRight = 1.0f - delta / 2.0f;
        
        // 限制范围，避免差速过大
        cameraFactorLeft = _constrain(cameraFactorLeft, 0.1f, 3.0f);
        cameraFactorRight = _constrain(cameraFactorRight, 0.1f, 3.0f);
        
        // 更新恢复计数器
        recoveryCycleCount++;
        
        // 恢复完成判断
        if (recoveryCycleCount >= obstacleCycleCount) {
            isRecoveringHeading = false;
            // 重置所有累积变量
            deltaK_sum = 0.0f;
            obstacleCycleCount = 0;
            targetDeltaK = 0.0f;
            remainingDeltaK = 0.0f;
            remainingCycles = 0;
            cameraFactorLeft = 1.0f;
            cameraFactorRight = 1.0f;
        }
        
        // 更新全局避障因子
        avoidanceCameraFactorLeft = cameraFactorLeft;
        avoidanceCameraFactorRight = cameraFactorRight;
        return;  // 恢复期间不再执行避障计算
    }

    // 3. 正常避障模式（无恢复进行中）
    // 首先检查是否有障碍物
    if (!obstacleDetected) {
        #if !DISABLE_HEADING_RECOVERY
        // 无障碍物，但之前可能有过避障（累积数据未清零），启动恢复
        if (obstacleCycleCount > 0 || remainingCycles > 0) {
            // 如果有中断剩余的未恢复量，合并到本次累积中
            if (remainingCycles > 0) {
                deltaK_sum += remainingDeltaK;
                obstacleCycleCount += remainingCycles;
                remainingDeltaK = 0.0f;
                remainingCycles = 0;
            }
            // 启动航向恢复
            isRecoveringHeading = true;
            recoveryCycleCount = 0;
            targetDeltaK = - (deltaK_sum / obstacleCycleCount);
            // 限制目标差值幅度，防止恢复过猛
            targetDeltaK = _constrain(targetDeltaK, -1.5f, 1.5f);
            // 注意：本次调用不设置 cameraFactor，因为恢复模式会在下次循环生效
            // 但为避免延迟，这里直接调用一次恢复设置
            cameraFactorLeft = 1.0f + targetDeltaK / 2.0f;
            cameraFactorRight = 1.0f - targetDeltaK / 2.0f;
            cameraFactorLeft = _constrain(cameraFactorLeft, 0.1f, 3.0f);
            cameraFactorRight = _constrain(cameraFactorRight, 0.1f, 3.0f);
            avoidanceCameraFactorLeft = cameraFactorLeft;
            avoidanceCameraFactorRight = cameraFactorRight;
            // 标记恢复状态，下次循环会进入恢复分支
            isRecoveringHeading = true;
            recoveryCycleCount = 1; // 本次已算一个周期
        } else {
            // 完全无障碍且无累积，恢复默认因子
            avoidanceCameraFactorLeft = 1.0f;
            avoidanceCameraFactorRight = 1.0f;
        }
        #else
        // 禁用恢复时，直接重置因子
        avoidanceCameraFactorLeft = 1.0f;
        avoidanceCameraFactorRight = 1.0f;
        #endif
        return;
    }

    // 4. 有障碍物且不在恢复模式：执行正常的避障计算
    // 注意：如果恢复模式被新障碍物打断，会在下面的中断检测中处理
    if (isRecoveringHeading && obstacleDetected) {
        // 恢复过程中突然出现障碍物：打断恢复，记录已恢复部分
        float recoveredDeltaK = targetDeltaK * recoveryCycleCount;
        remainingDeltaK = deltaK_sum + recoveredDeltaK;  // 未恢复的转向量（deltaK积分剩余）
        remainingCycles = obstacleCycleCount - recoveryCycleCount;
        // 退出恢复模式
        isRecoveringHeading = false;
        // 继续执行下面的避障计算（不清空 deltaK_sum 等，因为还要累加）
    }

    // 5. 基于势场的避障因子计算（原有逻辑）
    float attractiveForceX = 0.0f;
    float attractiveForceY = ATTRACTIVE_FORCE_GAIN;
    float repulsiveForceX = 0.0f;
    float repulsiveForceY = 0.0f;
    
    if (HC_distance < DANGER_RADIUS) {
        float distance = HC_distance;
        float heightFactor = 1.0f;
        heightFactor = 1.0f;  // 固定增益，不随高度变化
        float repulsiveGain = REPULSIVE_FORCE_MAX * (1.0f - distance / DANGER_RADIUS) * heightFactor;
        float dirX = -currentObstacle.x / distance;
        float dirY = -currentObstacle.y / distance;
        repulsiveForceX = dirX * repulsiveGain;
        repulsiveForceY = dirY * repulsiveGain;
        
        // 运动障碍物预测补偿
        if (fabs(currentObstacle.vx) > 1.0f || fabs(currentObstacle.vy) > 1.0f) {
            float predictedX = currentObstacle.x + currentObstacle.vx * 0.5f;
            float predictedY = currentObstacle.y + currentObstacle.vy * 0.5f;
            float predictedDistance = sqrt(predictedX * predictedX + predictedY * predictedY);
            if (predictedDistance < DANGER_RADIUS) {
                float extraRepulsive = 0.5f * repulsiveGain;
                repulsiveForceX -= (predictedX / predictedDistance) * extraRepulsive;
                repulsiveForceY -= (predictedY / predictedDistance) * extraRepulsive;
            }
        }
    }
    
    float totalForceX = attractiveForceX + repulsiveForceX;
    float totalForceY = attractiveForceY + repulsiveForceY;
    
    float baseSpeed = ATTRACTIVE_FORCE_GAIN * 0.2;
    float turnBias = 0.0f;
    if (obstacleDetected) {
        if (fabs(currentObstacle.x) < 0.2f * currentObstacle.y) {
            turnBias = -1.0f;
        } else if (currentObstacle.x < 0) {
            turnBias = 1.0f;
        } else {
            turnBias = -1.0f;
        }
    }
    float turnAmount = (totalForceX + turnBias) * 0.8f;
    
    // 根据转向量计算乘积因子
    float turnScale = 1.5f;  // 可调参数
    cameraFactorLeft = 1.0f - turnAmount * turnScale;
    cameraFactorRight = 1.0f + turnAmount * turnScale;
    cameraFactorLeft = _constrain(cameraFactorLeft, 0.1f, 3.0f);
    cameraFactorRight = _constrain(cameraFactorRight, 0.1f, 3.0f);
    
    // 6. 记录避障期间的 deltaK 累积和周期数（用于航向恢复）
    float currentDeltaK = cameraFactorLeft - cameraFactorRight;
    deltaK_sum += currentDeltaK;
    obstacleCycleCount++;
    
    // 7. 更新全局避障因子
    avoidanceCameraFactorLeft = cameraFactorLeft;
    avoidanceCameraFactorRight = cameraFactorRight;
    
    // 可选：调试输出
    // Serial.printf("避障: K1=%.2f, K2=%.2f, deltaK=%.2f, sum=%.2f, cnt=%d\n",
    //               cameraFactorLeft, cameraFactorRight, currentDeltaK, deltaK_sum, obstacleCycleCount);
}
void calculateAvoidanceForces() {
    // ========== 1. 紧急避障时，避障因子不干预，由状态机直接控制运动 ==========
    if (emergencyState != EM_NONE) {
        // 清空航向恢复状态
        isRecoveringHeading = false;
        deltaK_sum = 0.0f;
        obstacleCycleCount = 0;
        remainingDeltaK = 0.0f;
        remainingCycles = 0;
        targetDeltaK = 0.0f;
        recoveryCycleCount = 0;
        obstacleActive = false;
        obstacleDetected = false;
        // 避障因子保持中性
        avoidanceCameraFactorLeft = 1.0f;
        avoidanceCameraFactorRight = 1.0f;
        return;
    }

    // ========== 2. 检查避障保持是否超时 ==========
    if (obstacleActive && millis() > obstacleActiveUntil) {
        obstacleActive = false;   // 保持期结束
        // 注意：不立即清除 deltaK_sum 等，稍后会触发恢复
    }

    // ========== 3. 处理航向恢复模式（优先级高于避障） ==========
    if (isRecoveringHeading) {
        // 恢复期间，如果 obstacleActive 变为 true（新障碍物出现），则打断恢复
        if (obstacleActive) {
            // 记录已恢复的转向量，剩余部分留待下次
            float recoveredDeltaK = targetDeltaK * recoveryCycleCount;
            remainingDeltaK = deltaK_sum + recoveredDeltaK;
            remainingCycles = obstacleCycleCount - recoveryCycleCount;
            // 退出恢复模式
            isRecoveringHeading = false;
            // 注意：不清空 deltaK_sum 和 obstacleCycleCount，继续避障累积
        } else {
            // 正常恢复过程
            float delta = targetDeltaK;
            float leftFactor = 1.0f + delta / 2.0f;
            float rightFactor = 1.0f - delta / 2.0f;
            avoidanceCameraFactorLeft = _constrain(leftFactor, 0.07f, 3.5f);
            avoidanceCameraFactorRight = _constrain(rightFactor, 0.07f, 3.5f);
            
            recoveryCycleCount++;
            if (recoveryCycleCount >= obstacleCycleCount) {
                // 恢复完成
                isRecoveringHeading = false;
                deltaK_sum = 0.0f;
                obstacleCycleCount = 0;
                targetDeltaK = 0.0f;
                remainingDeltaK = 0.0f;
                remainingCycles = 0;
                avoidanceCameraFactorLeft = 1.0f;
                avoidanceCameraFactorRight = 1.0f;
            }
            return;
        }
    }

    // ========== 4. 无障碍物且不在恢复中：重置因子，但可能触发恢复 ==========
    if (!obstacleActive && !isRecoveringHeading) {
        // 如果之前有累积的转向量（避障刚刚结束），则启动恢复
        if (obstacleCycleCount > 0 || remainingCycles > 0) {
            // 合并剩余转向量
            if (remainingCycles > 0) {
                deltaK_sum += remainingDeltaK;
                obstacleCycleCount += remainingCycles;
                remainingDeltaK = 0.0f;
                remainingCycles = 0;
            }
            // 启动恢复
            isRecoveringHeading = true;
            recoveryCycleCount = 0;
            targetDeltaK = - (deltaK_sum / obstacleCycleCount);
            targetDeltaK = _constrain(targetDeltaK, -1.5f, 1.5f);
            // 立即应用一次恢复因子（避免延迟）
            float delta = targetDeltaK;
            float leftFactor = 1.0f + delta / 2.0f;
            float rightFactor = 1.0f - delta / 2.0f;
            avoidanceCameraFactorLeft = _constrain(leftFactor, 0.3f, 2.0f);
            avoidanceCameraFactorRight = _constrain(rightFactor, 0.3f, 2.0f);
            recoveryCycleCount = 1;
            return;
        } else {
            // 完全无障碍，因子为1
            avoidanceCameraFactorLeft = 1.0f;
            avoidanceCameraFactorRight = 1.0f;
            return;
        }
    }

    // ========== 5. 有障碍物（obstacleActive == true）：计算避障因子并记录积分 ==========
    // 使用超声波距离和 currentObstacle 计算转向量
    float distance = HC_distance;
    if (distance <= 0 || distance >= DANGER_RADIUS) {
        // 距离无效，不避障但保持活跃（因子1）
        avoidanceCameraFactorLeft = 1.0f;
        avoidanceCameraFactorRight = 1.0f;
        return;
    }

    // 势场计算（与之前相同）
    float heightFactor = 1.0f;
    if (currentObstacle.z < ROBOT_HEIGHT - 100) heightFactor = 0.5f;
    else if (currentObstacle.z > ROBOT_HEIGHT + 100) heightFactor = 1.5f;

    float repulsiveGain = REPULSIVE_FORCE_MAX * (1.0f - distance / DANGER_RADIUS) * heightFactor;
    float dirX = -currentObstacle.x / distance;
    float dirY = -currentObstacle.y / distance;
    float repulsiveForceX = dirX * repulsiveGain;
    float repulsiveForceY = dirY * repulsiveGain;

    float attractiveForceY = ATTRACTIVE_FORCE_GAIN;
    float totalForceX = repulsiveForceX;
    float totalForceY = attractiveForceY + repulsiveForceY;

    float turnBias = 0.0f;
    if (fabs(currentObstacle.x) < 0.1f * currentObstacle.y) {
        turnBias = -1.0f;
    } else if (currentObstacle.x < 0) {
        turnBias = 1.0f;
    } else {
        turnBias = -1.0f;
    }
    float turnAmount = (totalForceX + turnBias) * 0.8f;

    float cameraLeft = 1.0f - turnAmount * 1.5f;
    float cameraRight = 1.0f + turnAmount * 1.5f;
    cameraLeft = _constrain(cameraLeft, 0.1f, 3.0f);
    cameraRight = _constrain(cameraRight, 0.1f, 3.0f);

    // 低通滤波平滑（可选）
    static float smoothL = 1.0f, smoothR = 1.0f;
    float alpha = 0.4f;
    smoothL = alpha * smoothL + (1.0f - alpha) * cameraLeft;
    smoothR = alpha * smoothR + (1.0f - alpha) * cameraRight;
    avoidanceCameraFactorLeft = smoothL;
    avoidanceCameraFactorRight = smoothR;

    // 记录转向积分（用于航向恢复）
    float currentDeltaK = avoidanceCameraFactorLeft - avoidanceCameraFactorRight;
    deltaK_sum += currentDeltaK;
    obstacleCycleCount++;
}

void updateObstacleData() {
    // 1. 如果超声波距离本身大于阈值（如100cm），则即使摄像头有数据也不认为有障碍物
    //    注意：不清除 obstacleActive，让它自然超时（避免刚退出又激活）
    if (HC_distance > OBSTACLE_VALID_DIST_MAX) {
        // 可选：如果距离非常远，也可以将 obstacleDetected 置为 false
        obstacleDetected = false;
        return;
    }

    // 2. 摄像头数据无效时，不修改 obstacleActive
    if (!openmvData.valid) {
        return;
    }

    // 3. 根据摄像头状态和超声波距离联合判断
    if (openmvData.status == 'O') {
        // 只有当超声波距离有效且在阈值以内时，才激活避障
        if (HC_distance > 0 && HC_distance < OBSTACLE_VALID_DIST_MAX) {
            // 计算障碍物位置（使用超声波距离和摄像头角度）
            float distance = HC_distance;
            float panAngleRad = radians(openmvData.servo_angle - PAN_SERVO_OFFSET);
            currentObstacle.x = sin(panAngleRad) * distance;
            currentObstacle.y = cos(panAngleRad) * distance;
            float heightRatio = 1.0f - (openmvData.y_pixel / IMAGE_HEIGHT);
            currentObstacle.z = heightRatio * 100.0f;
            currentObstacle.lastUpdateTime = millis();

            // 激活避障保持
            obstacleActive = true;
            obstacleActiveUntil = millis() + OBSTACLE_HOLD_TIME;
            obstacleDetected = true;
        } else {
            // 摄像头检测到但超声波距离超阈值或无效：视为误识别，不激活避障
            // 保持 obstacleActive 原状（让它自然超时），但 obstacleDetected 置 false
            obstacleDetected = false;
        }
    } else if (openmvData.status == 'N') {
        // 摄像头明确报告无障碍物，但超声波可能仍有近距离目标（例如小障碍物未被摄像头识别）
        // 此时不改变 obstacleActive，让计时器自然超时，避免漏掉纯超声波避障
        obstacleDetected = false;
    }
}