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
#include <iostream>
#include "MPU6050_tockn.h"
#include <NewPing.h>  // 添加NewPing库
//#include <BluetoothA2DPSink.h>  // 蓝牙音频库
#define ROLL_OFFSET 0
#include <SoftwareSerial.h>
#include <sys/unistd.h>
#define MEASURE_INTERVAL 150  // 测量间隔（毫秒）
#define PAN_SERVO_OFFSET 140
unsigned long lastMeasureTime = 0;  // 上次测量时间
SoftwareSerial OpenMVSerial(47, 48); // TX=GPIO47，RX=GPIO48
#define OPENMV_SERIAL OpenMVSerial // 定义OpenMV串口
SF_Servo servos = SF_Servo(Wire); //实例化初始化舵机

//tockn
MPU6050 mpu6050(Wire, 0.03, 0.97); //实例化初始化传感器
bfs::SbusRx sbusRx(&Serial1);//实例化初始化接收机
SF_BLDC motors = SF_BLDC(Serial2);//实例化初始化电机

// 定义HC - SR04的引脚
#define TRIGGER_PIN 37
#define ECHO_PIN 38
#define MAX_DISTANCE 200 // 最大检测距离，单位：厘米
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
float avoidanceFactorLeft = 1.0f;
float avoidanceFactorRight = 1.0f;
const float MAX_AVOIDANCE_FACTOR = 5.2f;  // 最大加速因子（适度增加）
const float MIN_AVOIDANCE_FACTOR = 0.07f;   // 最小减速因子（适度减速）
const float AVOIDANCE_SMOOTHING = 0.08f;   // 平滑系数(0.0-1.0) - 较小的值更平滑
const float DANGER_ZONE_START = 20.0f;     // 开始避障的距离(cm)
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

// 3. 全局变量 - 避障系统状态
Obstacle currentObstacle = {0, 0, 0, 0, 0, 0}; // 当前障碍物
bool obstacleDetected = false;                  // 障碍物检测标志
bool inLocalMinimum = false;                    // 局部极小值状态
uint32_t escapeStartTime = 0;                   // 逃逸开始时间
float lastObstacleX = 0;                        // 上一次障碍物X位置
float lastObstacleY = 0;                        // 上一次障碍物Y位置
static int openmv=0;
// 主控端解析函数
// OpenMV 数据包格式：S,O,x,y,angle,checksumE
// 其中 checksum = 异或 (XOR) 计算 data 部分（不含S和E）的所有字符
uint8_t calc_checksum(const char* s) {
    uint8_t c = 0;
    while (*s) {
        c ^= (uint8_t)(*s);
        s++;
    }
    return c;
}

void read_openmv_data() {
    static char buffer[64];
    static uint8_t bufIndex = 0;
    static uint32_t lastOpenMVTime = 0;
    static bool inPacket = false;

    while (OpenMVSerial.available() > 0 && bufIndex < sizeof(buffer) - 1) {
        char c = OpenMVSerial.read();

        if (c == 'S') {
            bufIndex = 0;
            buffer[bufIndex++] = c;
            inPacket = true;
            continue;
        }

        if (!inPacket) {
            continue;
        }

        buffer[bufIndex++] = c;

        if (c == 'E') {
            buffer[bufIndex] = '\0';
            // 插入结束符后解析包
            // 例:S,O,120,150,90,3AE
            char status;
            int read = 0;
            float x, y, angle;
            unsigned int checksum;
            if (sscanf(buffer, "S,%c,%f,%f,%f,%xE", &status, &x, &y, &angle, &checksum) == 5) {
                // 校验范围
                if ((status == 'O' || status == 'N') && x >= 0 && x <= 240 && y >= 0 && y <= 240 && angle >= 10 && angle <= 210) {
                    // 计算校验和
                    // 解析 data 字符串（不含头S与尾E以及校验字段）
                    char dataPart[48] = {0};
                    int copyLen = snprintf(dataPart, sizeof(dataPart), "%c,%.1f,%.1f,%.1f", status, x, y, angle);
                    // 兼容整数类型格式
                    // 计算异或校验
                    uint8_t calc = 0;
                    for (int i = 0; i < copyLen; i++) {
                        calc ^= (uint8_t)dataPart[i];
                    }
                    if (calc == (uint8_t)checksum) {
                        openmvData.status = status;
                        openmvData.x_pixel = x;
                        openmvData.y_pixel = y;
                        openmvData.servo_angle = angle;
                        openmvData.valid = true;
                        openmv = 1;
                    }
                }
            }

            inPacket = false;
            bufIndex = 0;
            lastOpenMVTime = millis();
        }

        if (bufIndex >= sizeof(buffer) - 1) {
            // 保护：丢弃过长包
            inPacket = false;
            bufIndex = 0;
        }
    }

    if (millis() - lastOpenMVTime > OPENMV_TIMEOUT) {
        openmvData.valid = false;
        inPacket = false;
        bufIndex = 0;
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
    OpenMVSerial.begin(115200, SWSERIAL_8N1, 47, 48, false, 256);
    // 稳定性优化
    OpenMVSerial.enableIntTx(false); // 禁用发送中断
    OpenMVSerial.setTimeout(10);     // 设置超时
    setRobotparam();
    Wire.begin(1,2,400000UL);
    Serial.printf("System Started!");
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
    coordTarget.yLeft = ROBOT_LOWESR_FOR_CAL;
    coordTarget.yRight = ROBOT_LOWESR_FOR_CAL;
    cnt = ROBOT_LOWESR_FOR_CAL;
}
//主函数，不断循环运行机器人系统,在Arduino环境中，setup() 函数只会运行一次，而 loop() 函数会一直循环运行。
static float multiplyFactorALL1 = 1.0f;
static float multiplyFactorALL2 = 1.0f;
void loop() {
    unsigned long currentTime = millis();
    float multiplyFactor1 = 1.0f;
    float multiplyFactor2 = 1.0f;
    openmv=0;
    if (millis() - lastMeasureTime > MEASURE_INTERVAL) {
        lastMeasureTime = millis();
        unsigned int uS = sonar.ping(); 
        HC_distance = uS / US_ROUNDTRIP_CM;
        
        // 处理无响应情况
        if (uS == 0) {
            HC_distance = MAX_DISTANCE;
        }
        
        //Serial.printf("HC-SR04 Distance: %f cm\n", HC_distance);
    }
    
    // 根据距离计算避障因子
    float targetLeftFactor = 1.0f;
    float targetRightFactor = 1.0f;
    static bool ultrasonic_clear_long = false; // 超声波1米以上时屏蔽摄像头避障

    ultrasonic_clear_long = (HC_distance > 100.0f);

    if (HC_distance > 0 && HC_distance < DANGER_ZONE_START) {
        // 计算距离权重 (0-1之间)
        float distanceWeight = 1.0f - _constrain(
            (HC_distance - DANGER_ZONE_END) / (DANGER_ZONE_START - DANGER_ZONE_END), 
            0.0f, 1.0f
        );
        
        // 左转避障：左轮适度加速，右轮适度减速
        targetLeftFactor = 1.0f + distanceWeight * (MAX_AVOIDANCE_FACTOR - 1.0f);
        targetRightFactor = 1.0f - distanceWeight * (1.0f - MIN_AVOIDANCE_FACTOR);
        HC_first = false; // 紧急避障已触发
    } else if (HC_distance != 0) {
        HC_first = true;
    }
    
    // 平滑过渡避障因子
    avoidanceFactorLeft = avoidanceFactorLeft * (1 - AVOIDANCE_SMOOTHING) + 
                         targetLeftFactor * AVOIDANCE_SMOOTHING;
    
    avoidanceFactorRight = avoidanceFactorRight * (1 - AVOIDANCE_SMOOTHING) + 
                          targetRightFactor * AVOIDANCE_SMOOTHING;
    
    multiplyFactorALL1 = avoidanceFactorRight;
    multiplyFactorALL2 = avoidanceFactorLeft;
    
    
    read_openmv_data();  // 读取OpenMV数据
    updateObstacleData(); // 更新障碍物信息    
    
    read();//读取串行端口的数据
    getRCValue();//获取遥控器的值
    getMPUValue();//获取 MPU6050 传感器的数据
    getMotorValue();//获取电机的状态或值
    legControl();//控制机器人的腿部动作
    inverseKinematics();//计算机器人的逆运动学，以确定关节角度
    if (!ultrasonic_clear_long && HC_first) {
        calculateAvoidanceForces(); // 计算避障力
    } else if (ultrasonic_clear_long) {
        avoidanceCameraFactorLeft = 1.0f;
        avoidanceCameraFactorRight = 1.0f;
        obstacleDetected = false;
    }
    robotRun();//运行机器人系统
    if (openmv==1){
        Serial.printf(" HC_distance: %f \n", HC_distance);
        Serial.printf(" 左轮因子: %f \n", avoidanceCameraFactorRight);
        Serial.printf(" 右轮因子: %f \n", avoidanceCameraFactorLeft);
        Serial.printf("----------------------------------\n");}
        
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

    robotMode.motorEnable = false;//设置电机使能标志为 false，表示电机未使能。
    robotMode.servoEnable = false;//设置舵机使能标志为 false，表示舵机未使能。
    robotMode.printFlag = false;//设置打印标志为 false，表示不打印信息。
    robotMode.mode = ROBOTMODE_DIABLE;//设置机器人模式为 ROBOTMODE_DIABLE，表示机器人处于禁用状态。

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
    float e_H;
    float E_H;

    //根据RC 设定足轮的运动参数
    robotMotion.turn = map(RCValue[0], RCCHANNEL_MIN, RCCHANNEL_MAX, -1*robotMotion.turnLimit, robotMotion.turnLimit);//转弯，右左右 --- R1
    robotMotion.forward = map(RCValue[1], RCCHANNEL_MIN, RCCHANNEL_MAX, -1*robotMotion.forwardLimit, robotMotion.forwardLimit);//前进后退，右上下  ----  R0
    // Serial.printf("%f,%d\n",robotMotion.turn,RCValue[0]);
    if(robotMotion.forward >= 30) robotMotion.forward = 30;//正数是车身前倾，腿向后
    else if(robotMotion.forward <= -15) robotMotion.forward = -15;//负数是车身后倾，腿向前

    robotMotion.updown = ((int)map(RCValue[2], RCCHANNEL3_MIN, RCCHANNEL3_MAX, robotMotion.lowest, robotMotion.heightest));//变腿高，左上下 --- R2
    robotMotion.roll = map(RCValue[3], RCCHANNEL_MIN, RCCHANNEL_MAX, -1*robotMotion.rollLimit, robotMotion.rollLimit);//横滚，左左右 --- R3

    //根据RC 设定足轮的电机与舵机使能状态
    if(RCValue[4] == RCCHANNEL3_MIN){
        robotMode.motorEnable = true;
        robotMode.servoEnable = true;
        // Serial.println(111);
    }else if (RCValue[4] == RCCHANNEL3_MID){
        // robotMode.motorEnable = true;
        // robotMode.servoEnable = false;
        robotMode.motorEnable = false;
        robotMode.servoEnable = true;
        // Serial.println(222);
    }else if (RCValue[4] == RCCHANNEL3_MAX){
        robotMode.motorEnable = false;
        robotMode.servoEnable = false;
        // Serial.println(333);
    }

    //根据RC 设定足轮的控制模式
    if(RCValue[5] == RCCHANNEL3_MIN){
        robotMode.mode = ROBOTMODE_MOTION;
        robotMotion.lowest = ROBOT_LOWEST_FOR_MOT;
        robotMode.printFlag = false;
    }else if (RCValue[5] == RCCHANNEL3_MID){
        robotMode.printFlag = true;
    }else if (RCValue[5] == RCCHANNEL3_MAX){
        robotMode.mode = ROBOTMODE_CALIBRATION;
        robotMotion.forward = 0;
        robotMotion.updown = ROBOT_LOWESR_FOR_CAL;
        robotMotion.lowest = ROBOT_LOWESR_FOR_CAL;
        robotMode.printFlag = false;
    }

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

    if(robotMode.mode == ROBOTMODE_MOTION && robotMode.servoEnable == true){
        // e_H = PID_Roll.Kp * LPFRoll((robotMotion.roll - 3) - robotPose.roll);
        e_H = PID_Roll.Kp * LPFRoll(robotMotion.roll - robotPose.roll);
        E_H = PID_Gyrox.Kp * (e_H - robotPose.GyroX);
    }else{
        e_H = 0;
        E_H = 0;
    }
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
void adjustLegHeightStably(float targetHeight, float& currentHeight, PIDParams& currentPIDParams, bool& isHighLegHeight) {
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
}
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

  // 初始代码
  controlTarget.velocity = PID_VEL(robotMotion.forward*0.5f - robotPose.speedAvg);//速度环   forward是遥控器前进后退参数
  controlTarget.differVel = PID_Streeing.Kp*(robotMotion.turn-robotPose.GyroZ);//转向        turn是遥控器左右控制参数
  targetVoltage = PID_Stb.Kp*(controlTarget.velocity + controlTarget.centerAngleOffset - robotPose.pitch) - PID_Stb.Kd * robotPose.GyroY;//直立环 输出控制的电机的目标速度

  // 调试代码
  // motorsTarget.motorLeft = motorStatus.M0Dir * (targetVoltage);
  // motorsTarget.motorRight = -motorStatus.M1Dir * (targetVoltage);

  // 初始代码
  motorsTarget.motorLeft = motorStatus.M0Dir * (targetVoltage + controlTarget.differVel);
  motorsTarget.motorRight = motorStatus.M1Dir * (targetVoltage - controlTarget.differVel);

  motorsTarget.motorLeft = multiplyFactorALL1 * avoidanceCameraFactorLeft * motorsTarget.motorLeft;
    motorsTarget.motorRight = multiplyFactorALL2 * avoidanceCameraFactorRight * motorsTarget.motorRight;
  //Serial.printf("左轮速%f,右轮速%f\n",motorsTarget.motorLeft,motorsTarget.motorRight);
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
void calculateAvoidanceForces() {
    // 重置避障调整量
    avoidanceLeftAdjust = 0.0f;
    avoidanceRightAdjust = 0.0f;
    // 4.1 更新障碍物速度（如果检测到新数据）
    if (obstacleDetected && millis() - lastObstacleTime < 1000) {
        unsigned long currentTime = millis();
        float dt = (currentTime - currentObstacle.lastUpdateTime) / 1000.0f;
        
        if (dt > 0.01f) { // 最小时间间隔10ms
            currentObstacle.vx = (currentObstacle.x - lastObstacleX) / dt;
            currentObstacle.vy = (currentObstacle.y - lastObstacleY) / dt;
            
            // 保存当前数据用于下次计算
            lastObstacleX = currentObstacle.x;
            lastObstacleY = currentObstacle.y;
            currentObstacle.lastUpdateTime = currentTime;
        }
    }
    
    // 4.2 计算目标引力（前进方向）
    float attractiveForceX = 0.0f;
    float attractiveForceY = ATTRACTIVE_FORCE_GAIN; // Y轴为前进方向
    
    // 4.3 计算障碍物斥力（如果存在障碍物）
    float repulsiveForceX = 0.0f;
    float repulsiveForceY = 0.0f;
    
    if (obstacleDetected && HC_distance < DANGER_RADIUS) {
        // 计算障碍物距离
        float distance = HC_distance; // 优先使用超声波测距
        
        // 根据障碍物高度调整避障策略
            float heightFactor = 1.0f;
            
            // 如果障碍物高度低于机器人高度，可以尝试通过
            if (currentObstacle.z < ROBOT_HEIGHT - 100) {
                heightFactor = 0.5f; // 减小避让力度
                //Serial.println("Low obstacle, reduced avoidance");
            }
            // 如果障碍物高度高于机器人高度，需要完全避开
            else if (currentObstacle.z > ROBOT_HEIGHT + 100) {
                heightFactor = 1.5f; // 增加避让力度
                //Serial.println("High obstacle, increased avoidance");
            }
            
            // 动态斥力系数
            float repulsiveGain = REPULSIVE_FORCE_MAX * (1.0f - distance / DANGER_RADIUS) * heightFactor;
            
            // 斥力方向（远离障碍物）
            float dirX = -currentObstacle.x / distance;
            float dirY = -currentObstacle.y / distance;
            // 基本斥力
            repulsiveForceX = dirX * repulsiveGain;
            repulsiveForceY = dirY * repulsiveGain;
        
        // 运动障碍物预测补偿
        if (fabs(currentObstacle.vx) > 1.0f || fabs(currentObstacle.vy) > 1.0f) {
            // 预测0.5秒后的位置
            float predictedX = currentObstacle.x + currentObstacle.vx * 0.5f;
            float predictedY = currentObstacle.y + currentObstacle.vy * 0.5f;
            float predictedDistance = sqrt(predictedX * predictedX + predictedY * predictedY);          
            // 额外斥力补偿
            if (predictedDistance < DANGER_RADIUS) {
                float extraRepulsive = 0.5f * repulsiveGain;
                repulsiveForceX -= (predictedX / predictedDistance) * extraRepulsive;
                repulsiveForceY -= (predictedY / predictedDistance) * extraRepulsive;
            }
        }
    }
    
    // 4.4 计算合力
    float totalForceX = attractiveForceX + repulsiveForceX;
    float totalForceY = attractiveForceY + repulsiveForceY;
    
    // 4.5 计算基础前进速度（基于引力）
    float baseSpeed = ATTRACTIVE_FORCE_GAIN*0.2;
    
    // 4.6 根据障碍物位置调整速度和转向
    float speedAdjust = 1.0f;
    float turnBias = 0.0f;  // 默认转弯方向（正值为右转，负值为左转）
    
    // 如果有障碍物
    if (obstacleDetected) {
        // 障碍物在正前方（-10°到+10°范围内）
        if (fabs(currentObstacle.x) < 0.1f * currentObstacle.y) {
            // 减速并设置默认左转方向
            speedAdjust = 0.3f;  // 大幅减速
            turnBias = -1.0f;    // 默认左转
        }
        // 障碍物在左侧（x < 0）
        else if (currentObstacle.x < 0) {
            speedAdjust = 0.8f;  // 适度减速
            turnBias = 1.0f;     // 向右转避开左侧障碍物
        }
        // 障碍物在右侧（x > 0）
        else {
            speedAdjust = 0.8f;  // 适度减速
            turnBias = -1.0f;    // 向左转避开右侧障碍物
        }
    }
    
    // 4.7 计算转向量
    // 使用合力方向 + 默认转弯偏向
    float turnAmount = (totalForceX + turnBias) * 0.8f;
    
   // 4.8 计算摄像头避障的乘积因子 (范围0.1~3.0)
    float cameraFactorLeft = 1.0f;
    float cameraFactorRight = 1.0f;
    
    if (obstacleDetected) {
        // 根据转向量计算乘积因子
        cameraFactorLeft = 1.0f - turnAmount * 1.0f;
        cameraFactorRight = 1.0f + turnAmount * 1.0f;
        
        // 限制因子范围
        cameraFactorLeft = _constrain(cameraFactorLeft, 0.1f, 3.0f);
        cameraFactorRight = _constrain(cameraFactorRight, 0.1f, 3.0f);
        
        // 更新避障因子
        avoidanceCameraFactorLeft =cameraFactorLeft;
        avoidanceCameraFactorRight =cameraFactorRight;
       // Serial.print("avoidanceCameraFactorLeft =\n"); 
        //Serial.print(avoidanceCameraFactorLeft ); 
        //Serial.print("avoidanceCameraFactorRight =\n"); 
        //Serial.print(avoidanceCameraFactorRight ); 
        // 记录避障开始时间和持续时间
        if (avoidanceStartTime == 0) {
            avoidanceStartTime = millis();
        }
        avoidanceDuration = millis() - avoidanceStartTime;
        
        // 重置恢复状态
        isRecovering = false;
    } 
    else if (!isRecovering && avoidanceStartTime != 0) {
        // 障碍物消失，开始恢复过程
        isRecovering = true;
        recoveryStartTime = millis();
        recoveryDuration = avoidanceDuration;
        
        // 计算恢复因子（避障因子的倒数）
        recoveryFactorLeft = avoidanceCameraFactorRight;
        recoveryFactorRight = avoidanceCameraFactorLeft;
        
        // 限制恢复因子范围
        recoveryFactorLeft = _constrain(recoveryFactorLeft, 0.1f, 3.0f);
        recoveryFactorRight = _constrain(recoveryFactorRight, 0.1f, 3.0f);
        
        avoidanceStartTime = 0;
        
    }
    
    // 4.9 处理恢复过程
    if (isRecovering) {
        uint32_t elapsedRecoveryTime = millis() - recoveryStartTime;
        
        if (elapsedRecoveryTime < recoveryDuration) {
            // 应用恢复因子
            avoidanceCameraFactorLeft = recoveryFactorLeft;
            avoidanceCameraFactorRight = recoveryFactorRight;
        } else {
            // 恢复完成，重置因子
            avoidanceCameraFactorLeft = 1.0f;
            avoidanceCameraFactorRight = 1.0f;
            isRecovering = false;
        }
    }
    // 4.9 调试输出
    #ifdef DEBUG_AVOIDANCE
    Serial.print("AttF(引力):("); 
    Serial.print(attractiveForceX); 
    Serial.print(","); 
    Serial.print(attractiveForceY); 
    Serial.print(") ");
    
    Serial.print("RepF(斥力):("); 
    Serial.print(repulsiveForceX); 
    Serial.print(","); 
    Serial.print(repulsiveForceY); 
    Serial.print(") ");
    
    #endif
}
// 修复后的障碍物更新函数
void updateObstacleData() {
    if (!openmvData.valid || openmvData.status != 'O') {
        obstacleDetected = false;
        return;
    }
    
    // 使用超声波测量距离
    float distance = HC_distance;
    
    // 将舵机角度转换为弧度
    float panAngleRad = radians(openmvData.servo_angle - PAN_SERVO_OFFSET);
    
    // 计算世界坐标系中的位置
    currentObstacle.x = sin(panAngleRad) * distance;
    currentObstacle.y = cos(panAngleRad) * distance;
    
    // 正确的高度计算：图像顶部(y=0)对应更高高度
    float heightRatio = 1.0f - (openmvData.y_pixel / IMAGE_HEIGHT);
    currentObstacle.z = heightRatio * 100.0f; 
    
    currentObstacle.lastUpdateTime = millis();
    obstacleDetected = true;
    
    /*Serial.printf("Obstacle: X=%.1fcm, Y=%.1fcm, Z=%.1fcm, Angle=%.1f°，左轮速度=%0.1f,右轮速度=%0.1f\n", 
                 currentObstacle.x, currentObstacle.y, currentObstacle.z,
                 openmvData.servo_angle, avoidanceLeftAdjust,  avoidanceRightAdjust);*/
}