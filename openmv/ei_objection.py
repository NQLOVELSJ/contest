# Edge Impulse - OpenMV FOMO Object Detection Example
import sensor, image, time, ml, math, uos, gc
from pyb import UART, Servo

# UART 初始化
uart1 = UART(1, 9600)

# 舵机参数
pan_servo = Servo(1)
PAN_SERVO_OFFSET = 150
PAN_SERVO_MIN = 90
PAN_SERVO_MAX = 210
current_pan_angle = PAN_SERVO_OFFSET
target_pan_angle = PAN_SERVO_OFFSET
INVERT_SERVO_DIRECTION = True

def set_servo_angle(angle):
    if INVERT_SERVO_DIRECTION:
        pan_servo.angle(180 - angle)
    else:
        pan_servo.angle(angle)

set_servo_angle(PAN_SERVO_OFFSET)

def update_servo():
    global current_pan_angle
    angle_diff = target_pan_angle - current_pan_angle
    max_step = 5
    step = math.copysign(min(abs(angle_diff), max_step), angle_diff)
    current_pan_angle += step
    current_pan_angle = max(PAN_SERVO_MIN, min(current_pan_angle, PAN_SERVO_MAX))
    set_servo_angle(current_pan_angle)
    return current_pan_angle

def calculate_target_angle(obstacle_x):
    center_x = 120
    pixel_offset = obstacle_x - center_x
    FOV = 40
    angle_offset = pixel_offset * (FOV / center_x)
    return PAN_SERVO_OFFSET + angle_offset

def calculate_checksum(data_str):
    checksum = 0
    for c in data_str:
        checksum ^= ord(c)
    return checksum

# ===== 传感器初始化 =====
def init_sensor():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_windowing((240, 240))
    sensor.skip_frames(time=2000)
    sensor.set_vflip(True)
    sensor.set_hmirror(True)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False)
    sensor.set_brightness(3)

init_sensor()

# ===== 模型加载 =====
net = None
labels = None
min_confidence = 0.5
threshold_list = [(math.ceil(min_confidence * 255), 255)]

def load_model():
    global net, labels
    try:
        net = ml.Model("trained.tflite",
            load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
        labels = [line.rstrip('\n') for line in open("labels.txt")]
        return True
    except Exception as e:
        print("Model load failed: " + str(e))
        return False

if not load_model():
    raise Exception('Failed to load model')

colors = [
    (255, 0, 0), (0, 255, 0), (255, 255, 0),
    (0, 0, 255), (255, 0, 255), (0, 255, 255), (255, 255, 255)
]

def fomo_post_process(model, inputs, outputs):
    ob, oh, ow, oc = model.output_shape[0]
    x_scale = inputs[0].roi[2] / ow
    y_scale = inputs[0].roi[3] / oh
    scale = min(x_scale, y_scale)
    x_offset = ((inputs[0].roi[2] - (ow * scale)) / 2) + inputs[0].roi[0]
    y_offset = ((inputs[0].roi[3] - (ow * scale)) / 2) + inputs[0].roi[1]

    l = [[] for _ in range(oc)]
    for i in range(oc):
        img = image.Image(outputs[0][0, :, :, i] * 255.0)
        blobs = img.find_blobs(threshold_list, x_stride=1, y_stride=1,
                                area_threshold=1, pixels_threshold=1)
        for b in blobs:
            rect = b.rect()
            x, y, w, h = rect
            score = img.get_statistics(thresholds=threshold_list, roi=rect).l_mean() / 255.0
            x = int((x * scale) + x_offset)
            y = int((y * scale) + y_offset)
            w = int(w * scale)
            h = int(h * scale)
            l[i].append((x, y, w, h, score))
    return l

# ===== 发送和计时控制 =====
SEND_INTERVAL = 100     # 10Hz
OBSTACLE_TIMEOUT = 1000 # 1秒无检测才回中舵机
OBSTACLE_HOLD_FRAMES = 6 # 丢失后保持 O 状态的帧数（~600ms）

last_send_time = 0
last_obstacle_time = 0
obstacle_lost_count = 0
had_obstacle = False

# 持久化障碍物数据（丢失后短暂保持）
last_center_x = 0
last_center_y = 0
last_angle = PAN_SERVO_OFFSET

# 系统健康
gc_collect_interval = 50
loop_count = 0
sensor_error_count = 0
model_error_count = 0
MAX_SENSOR_ERRORS = 10
MAX_MODEL_ERRORS = 5

# 主动扫描状态机
SCAN_IDLE = 0
SCAN_RIGHT = 1
SCAN_RIGHT_HOLD = 2
SCAN_LEFT = 3
SCAN_LEFT_HOLD = 4
SCAN_RETURN = 5
SCAN_RIGHT_ANGLE = 100    # 右方扫描角度
SCAN_LEFT_ANGLE  = 200    # 左方扫描角度
SCAN_HOLD_MS     = 600    # 扫描停顿时长(ms)，让ESP32读超声波
SCAN_TRIGGER_MS  = 2500   # 无检测多久后触发扫描(ms)
SCAN_COOLDOWN_MS = 3000   # 一次扫描周期后冷却(ms)
scan_state = SCAN_IDLE
scan_phase_start = 0
last_scan_time = 0         # 上次扫描完成时间

clock = time.clock()
while True:
    clock.tick()
    current_time = time.ticks_ms()
    loop_count += 1

    # 定期垃圾回收
    if loop_count % gc_collect_interval == 0:
        gc.collect()

    # ===== 1. 图像采集（带校验和重试） =====
    img = None
    for retry in range(3):
        try:
            img = sensor.snapshot()
            if img is not None:
                sensor_error_count = 0
                break
        except Exception as e:
            print("snapshot error: " + str(e))
            time.sleep_ms(10)

    if img is None:
        sensor_error_count += 1
        if sensor_error_count >= MAX_SENSOR_ERRORS:
            print("Sensor persistent failure, reinit...")
            try:
                init_sensor()
            except:
                pass
            sensor_error_count = 0
        time.sleep_ms(50)
        continue  # 跳过本帧，不发送 N

    # ===== 2. 物体检测（带异常保护） =====
    obstacle_detected = False
    center_x = 0
    center_y = 0
    highest_score = 0

    try:
        predictions = net.predict([img], callback=fomo_post_process)
        for i, detection_list in enumerate(predictions):
            if i == 0:
                continue
            if not detection_list:
                continue
            for x, y, w, h, score in detection_list:
                if score < min_confidence:
                    continue
                if score > highest_score:
                    highest_score = score
                    center_x = math.floor(x + (w / 2))
                    center_y = math.floor(y + (h / 2))
                    img.draw_circle((center_x, center_y, 12), color=colors[i])
                    obstacle_detected = True
                    last_obstacle_time = current_time
        model_error_count = 0

    except Exception as e:
        print("predict error: " + str(e))
        model_error_count += 1
        gc.collect()
        if model_error_count >= MAX_MODEL_ERRORS:
            print("Model persistent failure, reloading...")
            try:
                load_model()
            except:
                pass
            model_error_count = 0
        time.sleep_ms(100)
        continue

    # ===== 3. 障碍物保持逻辑 =====
    # 短暂丢失检测时保持 O 状态，防止摄像头瞬时闪断
    if obstacle_detected:
        obstacle_lost_count = 0
        had_obstacle = True
        last_center_x = center_x
        last_center_y = center_y
    elif had_obstacle:
        obstacle_lost_count += 1
        # 保持最后已知位置（舵机继续跟踪）
        center_x = last_center_x
        center_y = last_center_y
        if obstacle_lost_count <= OBSTACLE_HOLD_FRAMES:
            # 在保持期内，仍视为有障碍物
            obstacle_detected = True
        else:
            had_obstacle = False
            obstacle_lost_count = 0

    # ===== 4. 舵机控制（含主动扫描） =====
    # 扫描状态机常量
    SCAN_IDLE = 0
    SCAN_RIGHT = 1
    SCAN_RIGHT_HOLD = 2
    SCAN_LEFT = 3
    SCAN_LEFT_HOLD = 4
    SCAN_RETURN = 5

    SCAN_RIGHT_ANGLE = 100    # 右方扫描角度
    SCAN_LEFT_ANGLE  = 200    # 左方扫描角度
    SCAN_HOLD_MS     = 600    # 扫描停顿时长(ms)，让ESP32读超声波
    SCAN_TRIGGER_MS  = 2500   # 无检测多久后触发扫描(ms)
    SCAN_COOLDOWN_MS = 3000   # 一次扫描周期后冷却(ms)

    if obstacle_detected:
        scan_state = SCAN_IDLE
        scan_phase_start = 0
        target_pan_angle = calculate_target_angle(center_x)
    else:
        idle_time = time.ticks_diff(current_time, last_obstacle_time)

        # 未触发扫描时：先回中
        if scan_state == SCAN_IDLE:
            if idle_time > SCAN_TRIGGER_MS:
                # 启动扫描
                scan_state = SCAN_RIGHT
                scan_phase_start = current_time
            elif idle_time > OBSTACLE_TIMEOUT:
                # 缓慢回中
                if abs(target_pan_angle - PAN_SERVO_OFFSET) > 1:
                    if target_pan_angle > PAN_SERVO_OFFSET:
                        target_pan_angle -= 2
                    else:
                        target_pan_angle += 2
                else:
                    target_pan_angle = PAN_SERVO_OFFSET

        # 扫描状态机
        if scan_state == SCAN_RIGHT:
            target_pan_angle = SCAN_RIGHT_ANGLE
            if abs(current_pan_angle - SCAN_RIGHT_ANGLE) < 3:
                scan_state = SCAN_RIGHT_HOLD
                scan_phase_start = current_time

        elif scan_state == SCAN_RIGHT_HOLD:
            target_pan_angle = SCAN_RIGHT_ANGLE
            if time.ticks_diff(current_time, scan_phase_start) > SCAN_HOLD_MS:
                scan_state = SCAN_LEFT
                scan_phase_start = current_time

        elif scan_state == SCAN_LEFT:
            target_pan_angle = SCAN_LEFT_ANGLE
            if abs(current_pan_angle - SCAN_LEFT_ANGLE) < 3:
                scan_state = SCAN_LEFT_HOLD
                scan_phase_start = current_time

        elif scan_state == SCAN_LEFT_HOLD:
            target_pan_angle = SCAN_LEFT_ANGLE
            if time.ticks_diff(current_time, scan_phase_start) > SCAN_HOLD_MS:
                scan_state = SCAN_RETURN
                scan_phase_start = current_time

        elif scan_state == SCAN_RETURN:
            if abs(target_pan_angle - PAN_SERVO_OFFSET) > 1:
                if target_pan_angle > PAN_SERVO_OFFSET:
                    target_pan_angle -= 2
                else:
                    target_pan_angle += 2
            else:
                target_pan_angle = PAN_SERVO_OFFSET
                # 扫描完成，冷却后重新扫描
                scan_state = SCAN_IDLE
                last_obstacle_time = current_time - (SCAN_TRIGGER_MS - SCAN_COOLDOWN_MS)
                # 效果：冷却约3秒后再触发下一轮扫描

    current_angle = update_servo()
    last_angle = current_angle

    # ===== 5. 数据发送 =====
    if time.ticks_diff(current_time, last_send_time) >= SEND_INTERVAL:
        if obstacle_detected:
            status = 'O'
            data_str = f"{status},{center_x},{center_y},{int(current_angle)}"
        else:
            status = 'N'
            data_str = f"{status},0,0,{int(current_angle)}"

        checksum = calculate_checksum(data_str)
        frame = f"S,{data_str},{checksum:02X}E"

        try:
            uart1.write(frame.encode())
        except OSError:
            pass  # UART buffer full, 丢弃本帧

        print(frame)
        last_send_time = current_time
