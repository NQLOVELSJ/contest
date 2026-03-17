# Edge Impulse - OpenMV FOMO Object Detection Example
import sensor, image, time, ml, math, uos, gc, binascii
from pyb import UART, Servo

# 初始化串口
uart = UART(3, 115200)
# 初始化串口（UART(1)对应P1(RX)、P0(TX)）
uart1 = UART(1, 115200)  # 波特率115200
# 舵机参数 - 修复安装方向问题
pan_servo = Servo(1)  # 使用P7引脚（Servo 1）
PAN_SERVO_OFFSET = 150    # 舵机正前方角度
PAN_SERVO_MIN = 90      # 逻辑最小角度（实际物理最大角度）
PAN_SERVO_MAX = 210     # 逻辑最大角度（实际物理最小角度）
current_pan_angle = PAN_SERVO_OFFSET
target_pan_angle = PAN_SERVO_OFFSET

# 关键修复1：添加方向反转参数
INVERT_SERVO_DIRECTION = True  # 根据实际安装方向调整
rx_buf = b''  # 接收缓存，用于拼接不完整的帧数据
# 帧格式定义
FRAME_HEADER = b'\xaa'  # 帧头：AA
FRAME_TAIL = b'\xfb'    # 帧尾：FB
EXPECTED_FRAME_LEN = 5  # 预期帧长度（AA+数据+FB共5字节）
VALID_THIRD_BYTE = 0x00  # 第三位有效值（00）

def obstacle_point(x):
    if x < 120:
        uart1.write(b'254\r\n')
    elif 120 < x < 240:
        uart1.write(b'255\r\n')
    elif x == 120:
        uart1.write(b'256\r\n')

# 语音识别-蓝牙播放函数
def read_language():
    global rx_buf
    if uart1.any():
        # 修复1：使用正确的串口uart1读取（而非uart）
        # 修复2：处理None情况，用or b''确保new_data始终是bytes类型
        new_data = uart1.read() or b''
        rx_buf += new_data
        print(f"接收原始字节: {binascii.hexlify(rx_buf).decode('utf-8')}")

        while True:
            header_idx = rx_buf.find(FRAME_HEADER)
            if header_idx == -1:
                rx_buf = b''
                break

            rx_buf = rx_buf[header_idx:]

            if len(rx_buf) >= EXPECTED_FRAME_LEN:
                if rx_buf[-1:] == FRAME_TAIL:
                    frame = rx_buf[:EXPECTED_FRAME_LEN]
                    rx_buf = rx_buf[EXPECTED_FRAME_LEN:]

                    frame_hex = binascii.hexlify(frame).decode('utf-8')
                    print(f"找到完整帧: {frame_hex}")

                    third_byte = frame[2]
                    if third_byte != VALID_THIRD_BYTE:
                        print(f"校验失败：第三位为0x{third_byte:02x}（要求0x00），跳过处理")
                        continue

                    valid_byte = frame[3]
                    print(f"有效位（16进制）: 0x{valid_byte:02x}")

                    mapped_value = valid_byte
                    print(f"映射后的十进制值: {mapped_value}")

                    send_data = f"{mapped_value}\r\n"
                    uart1.write(send_data.encode())
                    print(f"发送的数据: {send_data.strip()}（已添加\\r\\n结尾）")

                else:
                    rx_buf = rx_buf[1:]
            else:
                break

# 舵机角度映射函数（处理方向反转）
def set_servo_angle(angle):
    if INVERT_SERVO_DIRECTION:
        pan_servo.angle(180 - angle)
    else:
        pan_servo.angle(angle)

# 初始设置舵机位置
set_servo_angle(PAN_SERVO_OFFSET)

# 改进的舵机更新函数（添加方向反转）
def update_servo():
    global current_pan_angle
    angle_diff = target_pan_angle - current_pan_angle

    max_step = 5
    step = math.copysign(min(abs(angle_diff), max_step), angle_diff)

    current_pan_angle += step
    current_pan_angle = max(PAN_SERVO_MIN, min(current_pan_angle, PAN_SERVO_MAX))

    set_servo_angle(current_pan_angle)
    return current_pan_angle

# 目标角度计算（保持原逻辑）
def calculate_target_angle(obstacle_x):
    center_x = 120
    pixel_offset = obstacle_x - center_x

    FOV = 40
    angle_offset = pixel_offset * (FOV / center_x)

    return PAN_SERVO_OFFSET + angle_offset

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((240, 240))
sensor.skip_frames(time=1000)

net = None
labels = None
min_confidence = 0.5
threshold_list = [(math.ceil(min_confidence * 255), 255)]

try:
    net = ml.Model("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
except Exception as e:
    raise Exception('Failed to load "trained.tflite": ' + str(e))

try:
    labels = [line.rstrip('\n') for line in open("labels.txt")]
except Exception as e:
    raise Exception('Failed to load "labels.txt": ' + str(e))

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
        img = image.Image(outputs[0][0, :, :, i] * 255)
        blobs = img.find_blobs(threshold_list, x_stride=1, y_stride=1, area_threshold=1, pixels_threshold=1)
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

# 添加发送频率控制
OBSTACLE_REPORT_INTERVAL = 80
last_report_time = 0
last_obstacle_time = 0
OBSTACLE_TIMEOUT = 1000

clock = time.clock()
while True:
    clock.tick()
    current_time = time.ticks_ms()

    img = sensor.snapshot()
    obstacle_detected = False
    center_x = 0
    center_y = 0
    highest_score = 0
    read_language()

    # 物体检测部分
    for i, detection_list in enumerate(net.predict([img], callback=fomo_post_process)):
        if i == 0: continue
        if not detection_list: continue

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

    # 舵机控制部分
    if obstacle_detected:
        target_pan_angle = calculate_target_angle(center_x)
    else:
        if time.ticks_diff(current_time, last_obstacle_time) > OBSTACLE_TIMEOUT:
            if abs(target_pan_angle - PAN_SERVO_OFFSET) > 1:
                if target_pan_angle > PAN_SERVO_OFFSET:
                    target_pan_angle -= 1
                else:
                    target_pan_angle += 1
            else:
                target_pan_angle = PAN_SERVO_OFFSET

    current_angle = update_servo()

    # 数据发送部分
    if time.ticks_diff(current_time, last_report_time) > OBSTACLE_REPORT_INTERVAL:
        if obstacle_detected:
            data = f"O,{center_x},{center_y},{int(current_angle)},E\n"
            obstacle_point(center_x)
            uart.write(data)
            print(f"发送: {data.strip()}")
        else:
            data = f"N,0,0,{int(current_angle)},E\n"
        last_report_time = current_time

    print(f"{clock.fps():.1f} fps")
