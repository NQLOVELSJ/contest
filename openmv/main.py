import sensor, image, time, ml, math, uos, gc, binascii
from pyb import UART, Servo
uart = UART(3, 9600)
uart1 = UART(1, 9600)
pan_servo = Servo(1)
PAN_SERVO_OFFSET = 150
PAN_SERVO_MIN = 90
PAN_SERVO_MAX = 210
current_pan_angle = PAN_SERVO_OFFSET
target_pan_angle = PAN_SERVO_OFFSET
INVERT_SERVO_DIRECTION = True
rx_buf = b''
FRAME_HEADER = b'\xaa'
FRAME_TAIL = b'\xfb'
EXPECTED_FRAME_LEN = 5
VALID_THIRD_BYTE = 0x00
def obstacle_point(x):
	if x < 120:
		uart1.write(b'254\r\n')
	elif 120 < x < 240:
		uart1.write(b'255\r\n')
	elif x == 120:
		uart1.write(b'256\r\n')
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
		img = image.Image(outputs[0][0, :, :, i] * 255.0)
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
OBSTACLE_REPORT_INTERVAL = 80
last_report_time = 0
last_obstacle_time = 0
OBSTACLE_TIMEOUT = 1000
SEND_INTERVAL = 100
last_send_time = 0
clock = time.clock()
while True:
	clock.tick()
	current_time = time.ticks_ms()
	img = sensor.snapshot()
	obstacle_detected = False
	center_x = 0
	center_y = 0
	highest_score = 0
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
	if obstacle_detected:
		target_pan_angle = calculate_target_angle(center_x)
	else:
		if time.ticks_diff(current_time, last_obstacle_time) > OBSTACLE_TIMEOUT:
			if abs(target_pan_angle - PAN_SERVO_OFFSET) > 1:
				if target_pan_angle > PAN_SERVO_OFFSET:
					target_pan_angle -= 2
				else:
					target_pan_angle += 2
			else:
				target_pan_angle = PAN_SERVO_OFFSET
	current_angle = update_servo()
	if time.ticks_diff(current_time, last_send_time) >= SEND_INTERVAL:
		if obstacle_detected:
			status = 'O'
			data_str = f"{status},{center_x},{center_y},{int(current_angle)}"
		else:
			status = 'N'
			data_str = f"{status},0,0,{int(current_angle)}"
		checksum = calculate_checksum(data_str)
		frame = f"S,{data_str},{checksum:02X}E"
		uart1.write(frame.encode())
		print(frame)
		last_send_time = current_time