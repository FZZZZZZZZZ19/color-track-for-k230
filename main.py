 # K230 红色色块追踪 (双舵机) - 带低通滤波+积分分离PI+死区 (独立水平/垂直PI参数) + 激光常亮
import time, math
from machine import Pin, PWM, FPIOA
from media.sensor import *
from media.display import *
from media.media import *

# ==================== 1. 硬件与参数配置 ====================
# 舵机引脚配置 (保持不变)
fpioa = FPIOA()
fpioa.set_function(42, FPIOA.PWM0)
fpioa.set_function(43, FPIOA.PWM1)

# 初始化两个舵机，PWM频率为50Hz
servo_pan = PWM(0, 50, duty=0, enable=True)
servo_tilt = PWM(1, 50, duty=0, enable=True)

# 舵机初始角度 (居中)
PAN_CENTER = 0
TILT_CENTER = 0

# 屏幕中心坐标
CENTER_X = 320
CENTER_Y = 240

# 红色阈值 (LAB颜色空间) —— 唯一改动处
RED_THRESHOLD = (0, 80, 20, 80, 10, 80)

# ==================== 激光模块初始化 ====================
# 使用 GPIO47 控制激光，常亮
fpioa.set_function(44, FPIOA.GPIO44)   # 映射引脚为普通GPIO
laser = Pin(44, Pin.OUT, pull=Pin.PULL_NONE)
laser.value(1)  # 点亮激光
print("激光已开启")

# ==================== 关键优化参数（水平/垂直独立） ====================
# 水平舵机 (Pan) PI 参数
PAN_P_GAIN = 0.15
PAN_I_GAIN = 0.04

# 垂直舵机 (Tilt) PI 参数
TILT_P_GAIN = 0.16
TILT_I_GAIN = 0.06

# 低通滤波系数 (0.1~0.3)，用于平滑原始偏差
ALPHA = 0.11

# 死区阈值(像素)：误差小于此值时不做调整，避免中心点附近抖动
DEAD_ZONE = 5

# 积分分离阈值(像素)：误差大于此值时，不使用积分控制，防止过冲
I_SEPARATE_THRESH = 30

# ==================== 2. 状态变量 ====================
# 积分累加器
integral_pan = 0
integral_tilt = 0
# 滤波后的偏差值
filtered_offset_x = 0
filtered_offset_y = 0

# ==================== 3. 舵机角度控制函数 ====================
def set_servo_angle(servo, angle):
    """将舵机转动到指定角度 (-90° 到 +90°)"""
    angle = max(-90, min(90, angle))
    # 将角度(-90~+90)转换为占空比(2.5~12.5)
    duty = (angle + 90) / 180 * 10 + 2.5
    servo.duty(duty)

# 舵机归中
set_servo_angle(servo_pan, PAN_CENTER)
set_servo_angle(servo_tilt, TILT_CENTER)
print("舵机已归中")

# ==================== 4. 初始化摄像头 ====================
sensor = Sensor(id=2)
sensor.reset()
sensor.set_framesize(width=640, height=480)
sensor.set_pixformat(Sensor.RGB565)

# 初始化显示 (输出到IDE)
Display.init(Display.VIRT, width=640, height=480, to_ide=True)
MediaManager.init()
sensor.run()

print("摄像头初始化完成，开始红色色块追踪...")

# ==================== 5. 主循环 ====================
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()

    # 5.1 寻找红色色块 —— 改为红色阈值
    blobs = img.find_blobs([RED_THRESHOLD],
                           area_threshold=150,
                           pixels_threshold=150,
                           merge=True)

    if blobs:
        # 找到面积最大的色块作为追踪目标
        largest_blob = max(blobs, key=lambda b: b.area())

        # 在图像上绘制识别结果
        img.draw_rectangle(largest_blob.rect(), color=(0,255,0), thickness=2)
        img.draw_cross(largest_blob.cx(), largest_blob.cy(), color=(255,0,0), size=10, thickness=2)

        # 计算色块中心相对于屏幕中心的原始偏差
        offset_x = largest_blob.cx() - CENTER_X
        offset_y = largest_blob.cy() - CENTER_Y

        # === 死区处理 ===
        # 偏差在死区范围内则视为0，避免微小抖动
        if abs(offset_x) < DEAD_ZONE:
            offset_x = 0
        if abs(offset_y) < DEAD_ZONE:
            offset_y = 0

        # === 低通滤波 (平滑处理，解决抖动) ===
        filtered_offset_x = ALPHA * offset_x + (1 - ALPHA) * filtered_offset_x
        filtered_offset_y = ALPHA * offset_y + (1 - ALPHA) * filtered_offset_y

        # === 积分分离 (防止过冲) ===
        # 只在误差较小时启用积分，误差大时积分归零
        if abs(filtered_offset_x) < I_SEPARATE_THRESH:
            integral_pan += filtered_offset_x
        else:
            integral_pan = 0

        if abs(filtered_offset_y) < I_SEPARATE_THRESH:
            integral_tilt += filtered_offset_y
        else:
            integral_tilt = 0

        # 限制积分范围，防止过度累加
        integral_pan = max(-30, min(30, integral_pan))
        integral_tilt = max(-30, min(30, integral_tilt))

        # === PI控制计算（使用独立参数） ===
        # 水平舵机 (Pan)
        pan_correction = filtered_offset_x * PAN_P_GAIN + integral_pan * PAN_I_GAIN
        target_pan = PAN_CENTER + pan_correction

        # 垂直舵机 (Tilt)
        tilt_correction = filtered_offset_y * TILT_P_GAIN + integral_tilt * TILT_I_GAIN
        target_tilt = TILT_CENTER - tilt_correction

        # 控制舵机转动
        set_servo_angle(servo_pan, target_pan)
        set_servo_angle(servo_tilt, target_tilt)

    else:
        # 没有找到色块时，积分值缓慢衰减
        integral_pan *= 0.95
        integral_tilt *= 0.95

    # 显示图像到IDE
    Display.show_image(img)
形状匹配

# K230 形状引导颜色追踪 - 最终稳定版（find_rects + 无IDE显示）
import time, math
from machine import Pin, PWM, FPIOA
from media.sensor import *
from media.display import *
from media.media import *

# ==================== 1. 硬件与参数配置 ====================
fpioa = FPIOA()
fpioa.set_function(42, FPIOA.PWM0)
fpioa.set_function(43, FPIOA.PWM1)

servo_pan = PWM(0, 50, duty=0, enable=True)
servo_tilt = PWM(1, 50, duty=0, enable=True)

PAN_CENTER = 0
TILT_CENTER = 0
CENTER_X = 320
CENTER_Y = 240

fpioa.set_function(44, FPIOA.GPIO44)
laser = Pin(44, Pin.OUT, pull=Pin.PULL_NONE)
laser.value(1)
print("激光已开启")

# ==================== 控制参数（保持不变） ====================
PAN_P_GAIN = 0.15
PAN_I_GAIN = 0.04
TILT_P_GAIN = 0.16
TILT_I_GAIN = 0.06

ALPHA = 0.11
DEAD_ZONE = 5
I_SEPARATE_THRESH = 30

integral_pan = 0
integral_tilt = 0
filtered_offset_x = 0
filtered_offset_y = 0

def set_servo_angle(servo, angle):
    angle = max(-90, min(90, angle))
    duty = (angle + 90) / 180 * 10 + 2.5
    servo.duty(duty)

set_servo_angle(servo_pan, PAN_CENTER)
set_servo_angle(servo_tilt, TILT_CENTER)
print("舵机已归中")

# ==================== 初始化摄像头（不传IDE） ====================
sensor = Sensor(id=2)
sensor.reset()
sensor.set_framesize(width=640, height=480)
sensor.set_pixformat(Sensor.RGB565)

# 关键：to_ide=False，彻底关闭IDE图像传输
Display.init(Display.VIRT, width=640, height=480, to_ide=False)
MediaManager.init()
sensor.run()
print("摄像头初始化完成（无IDE显示）")

# ==================== 状态机 ====================
STATE_LEARN = 0
STATE_TRACK = 1

current_state = STATE_LEARN
learned_threshold = None
LEARN_DURATION = 30
learn_frame_count = 0

print("状态: 学习模式 - 请将圆形物体放入画面...")

clock = time.clock()

def compute_lab_threshold_from_roi(img, roi):
    x, y, w, h = roi
    x = max(0, min(x, img.width()-1))
    y = max(0, min(y, img.height()-1))
    w = max(1, min(w, img.width()-x))
    h = max(1, min(h, img.height()-y))
    
    stats = img.get_statistics(roi=(x, y, w, h))
    l_mean = stats.l_mean()
    a_mean = stats.a_mean()
    b_mean = stats.b_mean()
    l_std = stats.l_stdev()
    a_std = stats.a_stdev()
    b_std = stats.b_stdev()
    
    scale = 2.0
    l_min = max(0, int(l_mean - scale * l_std))
    l_max = min(100, int(l_mean + scale * l_std))
    a_min = max(-128, int(a_mean - scale * a_std))
    a_max = min(127, int(a_mean + scale * a_std))
    b_min = max(-128, int(b_mean - scale * b_std))
    b_max = min(127, int(b_mean + scale * b_std))
    
    return (l_min, l_max, a_min, a_max, b_min, b_max)

# ==================== 主循环 ====================
while True:
    clock.tick()
    img = sensor.snapshot()

    # ---------- 学习阶段 ----------
    if current_state == STATE_LEARN:
        circles = img.find_circles(threshold=3500, x_margin=10, y_margin=10,
                                   r_margin=10, r_min=20, r_max=200)
        if circles:
            largest_circle = max(circles, key=lambda c: c.r())
            r = largest_circle.r()
            cx = largest_circle.x()
            cy = largest_circle.y()
            roi_side = int(r * 1.2)
            roi_x = cx - roi_side // 2
            roi_y = cy - roi_side // 2
            
            learned_threshold = compute_lab_threshold_from_roi(img, (roi_x, roi_y, roi_side, roi_side))
            
            learn_frame_count += 1
            if learn_frame_count >= LEARN_DURATION:
                current_state = STATE_TRACK
                print("学习完成！切换到追踪模式。阈值:", learned_threshold)
        else:
            learn_frame_count = 0

    # ---------- 追踪阶段（使用 find_rects 检测任意角度矩形）----------
    elif current_state == STATE_TRACK:
        if learned_threshold is None:
            current_state = STATE_LEARN
            continue

        # 1. 寻找所有矩形
        rects = img.find_rects(threshold=10000)  # 阈值可根据实际调整，典型值8000~15000
        target_blob = None
        target_rect = None

        if rects:
            for rect in rects:
                roi = rect.rect()
                # 在矩形区域内寻找匹配颜色的色块
                blobs_in_rect = img.find_blobs([learned_threshold], roi=roi,
                                               area_threshold=100, pixels_threshold=100, merge=True)
                if blobs_in_rect:
                    target_rect = rect
                    target_blob = max(blobs_in_rect, key=lambda b: b.area())
                    break  # 找到第一个符合条件的矩形即可

        if target_blob is not None:
            offset_x = target_blob.cx() - CENTER_X
            offset_y = target_blob.cy() - CENTER_Y
            # 可选的串口调试信息
            # print("Target at:", target_blob.cx(), target_blob.cy())
        else:
            offset_x = 0
            offset_y = 0

        # PID追踪逻辑（完全保留）
        if abs(offset_x) < DEAD_ZONE:
            offset_x = 0
        if abs(offset_y) < DEAD_ZONE:
            offset_y = 0

        filtered_offset_x = ALPHA * offset_x + (1 - ALPHA) * filtered_offset_x
        filtered_offset_y = ALPHA * offset_y + (1 - ALPHA) * filtered_offset_y

        if abs(filtered_offset_x) < I_SEPARATE_THRESH:
            integral_pan += filtered_offset_x
        else:
            integral_pan = 0
        if abs(filtered_offset_y) < I_SEPARATE_THRESH:
            integral_tilt += filtered_offset_y
        else:
            integral_tilt = 0

        integral_pan = max(-30, min(30, integral_pan))
        integral_tilt = max(-30, min(30, integral_tilt))

        pan_correction = filtered_offset_x * PAN_P_GAIN + integral_pan * PAN_I_GAIN
        target_pan = PAN_CENTER + pan_correction
        tilt_correction = filtered_offset_y * TILT_P_GAIN + integral_tilt * TILT_I_GAIN
        target_tilt = TILT_CENTER - tilt_correction

        set_servo_angle(servo_pan, target_pan)
        set_servo_angle(servo_tilt, target_tilt)

    # 不再调用 Display.show_image()，彻底关闭IDE图像传输
    time.sleep_ms(5)