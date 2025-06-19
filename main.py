#!/usr/bin/env python3
# Robot DUSK - Main Control System
# By: WhayBot & Aquavivaaa
# License: MIT

import config
import time
import math
import busio
import board
import RPi.GPIO as GPIO
import pigpio
import random
from luma.oled.device import ssd1306
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from smbus2 import SMBus
from VL53L0X import VL53L0X
from mpu6050 import mpu6050
import adafruit_ina219

# ===== KONFIGURASI GPIO =====
GPIO.setmode(GPIO.BCM)
pi = pigpio.pi()

# ===== INISIALISASI HARDWARE =====
# I2C Bus Setup
i2c_bus0 = busio.I2C(board.SCL, board.SDA)
i2c_bus1 = SMBus(1)  # I2C Bus 1

# OLED Display
serial_oled_left = i2c(i2c_bus0, address=config.OLED_LEFT_ADDRESS)
oled_left = ssd1306(serial_oled_left, width=config.OLED_WIDTH, height=config.OLED_HEIGHT, rotate=0)
serial_oled_right = i2c(i2c_bus0, address=config.OLED_RIGHT_ADDRESS)
oled_right = ssd1306(serial_oled_right, width=config.OLED_WIDTH, height=config.OLED_HEIGHT, rotate=0)

# IMU Sensor
mpu = mpu6050(config.MPU6050_ADDRESS, bus=1)

# Voltage Sensor
ina219 = adafruit_ina219.INA219(i2c_bus0, address=config.INA219_ADDRESS)

# LiDAR Sensors
def init_vl53l0x():
    GPIO.setup(config.VL53L0X_LEFT_XSHUT, GPIO.OUT)
    GPIO.setup(config.VL53L0X_RIGHT_XSHUT, GPIO.OUT)
    
    GPIO.output(config.VL53L0X_LEFT_XSHUT, GPIO.LOW)
    GPIO.output(config.VL53L0X_RIGHT_XSHUT, GPIO.LOW)
    time.sleep(0.1)
    
    # Aktifkan sensor kiri
    GPIO.output(config.VL53L0X_LEFT_XSHUT, GPIO.HIGH)
    time.sleep(0.1)
    sensor_left = VL53L0X(i2c_bus1, config.VL53L0X_LEFT_ADDRESS)
    sensor_left.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
    
    # Aktifkan sensor kanan
    GPIO.output(config.VL53L0X_RIGHT_XSHUT, GPIO.HIGH)
    time.sleep(0.1)
    sensor_right = VL53L0X(i2c_bus1, config.VL53L0X_RIGHT_ADDRESS)
    sensor_right.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
    
    return sensor_left, sensor_right

vl53_left, vl53_right = init_vl53l0x()

# Setup GPIO Aktuator
GPIO.setup([
    config.IR_RECEIVER_LEFT, config.IR_RECEIVER_RIGHT,
    config.RELAY_VACUUM, config.RELAY_SWEEPER, 
    config.RELAY_MOP, config.RELAY_PUMP,
    config.MOTOR_L_IN1, config.MOTOR_L_IN2,
    config.MOTOR_R_IN1, config.MOTOR_R_IN2
], GPIO.OUT)

# Setup GPIO Encoder
GPIO.setup(config.ENCODER_LEFT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(config.ENCODER_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Motor Control Setup
pi.set_mode(config.MOTOR_L_ENA, pigpio.OUTPUT)
pi.set_mode(config.MOTOR_R_ENB, pigpio.OUTPUT)
pi.set_PWM_frequency(config.MOTOR_L_ENA, 1000)  # 1 kHz
pi.set_PWM_frequency(config.MOTOR_R_ENB, 1000)

# ESC Calibration
try:
    pi.set_servo_pulsewidth(config.ESC_PWM, 1500)  # Netral position
    time.sleep(2)
except pigpio.error:
    print("Warning: ESC initialization failed. Check wiring!")

# ===== KELAS UTAMA =====
class DUSKController:
    def __init__(self):
        self.mode = "CLEANING"  # CLEANING, DOCKING, CHARGING
        self.battery_low = False
        self.docking_found = False
        self.clean_pattern = "ZIGZAG"
        self.water_level = config.WATER_TANK_CAPACITY  # mL
        
        # Variabel animasi mata
        self.last_blink_time = time.time()
        self.last_eye_move_time = time.time()
        self.eye_open = True
        self.blink_state = 0
        self.eye_offset_x = 0
        self.eye_offset_y = 0
        self.blink_interval = random.uniform(3, 8)
        self.eye_move_interval = random.uniform(2, 5)
        
        # Variabel odometri
        self.x = 0.0  # Posisi X (cm)
        self.y = 0.0  # Posisi Y (cm)
        self.theta = 0.0  # Orientasi (radian)
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        self.last_left_count = 0
        self.last_right_count = 0
        self.last_odom_time = time.time()
        self.gyro_bias = self.calibrate_gyro()
        
        # Daftar event detector untuk encoder
        GPIO.add_event_detect(config.ENCODER_LEFT, GPIO.RISING, 
                             callback=lambda _: self.encoder_callback('left'))
        GPIO.add_event_detect(config.ENCODER_RIGHT, GPIO.RISING, 
                             callback=lambda _: self.encoder_callback('right'))
    
    def calibrate_gyro(self):
        """Kalibrasi gyro saat robot diam"""
        print("Calibrating gyro...")
        total = 0
        for _ in range(100):
            gyro_data = mpu.get_gyro_data()
            total += gyro_data['z']
            time.sleep(0.01)
        return total / 100
    
    def check_battery(self):
        voltage = ina219.bus_voltage
        if voltage < config.BATTERY_LOW_VOLTAGE:
            self.battery_low = True
            self.mode = "DOCKING"
        return voltage
    
    def read_distance_sensors(self):
        left_dist = vl53_left.get_distance() / 10.0  # mm to cm
        right_dist = vl53_right.get_distance() / 10.0
        return left_dist, right_dist
    
    def read_ir_docking(self):
        left_ir = GPIO.input(config.IR_RECEIVER_LEFT)
        right_ir = GPIO.input(config.IR_RECEIVER_RIGHT)
        return left_ir, right_ir
    
    def read_imu(self):
        return mpu.get_accel_data(), mpu.get_gyro_data()
    
    def encoder_callback(self, side):
        """Catat pulsa encoder (tanpa arah)"""
        if side == 'left':
            self.left_encoder_count += 1
        else:
            self.right_encoder_count += 1
    
    def update_odometry(self):
        """Fusion data encoder dan gyro untuk estimasi posisi"""
        # Hitung delta waktu
        current_time = time.time()
        dt = current_time - self.last_odom_time
        self.last_odom_time = current_time
        
        # Hitung delta pulsa encoder
        d_left = self.left_encoder_count - self.last_left_count
        d_right = self.right_encoder_count - self.last_right_count
        
        # Update last count
        self.last_left_count = self.left_encoder_count
        self.last_right_count = self.right_encoder_count
        
        # Hitung jarak tempuh per roda (cm)
        dist_left = d_left * config.CM_PER_PULSE
        dist_right = d_right * config.CM_PER_PULSE
        
        # Dapatkan yaw rate dari gyro (koreksi bias)
        _, gyro_data = self.read_imu()
        yaw_rate = gyro_data['z'] - self.gyro_bias
        
        # Hitung perubahan orientasi (radian)
        d_theta = math.radians(yaw_rate * dt)
        
        # Hitung jarak tempuh robot (rata-rata dua roda)
        dist_center = (dist_left + dist_right) / 2.0
        
        # Update posisi (model differensial drive)
        self.x += dist_center * math.cos(self.theta)
        self.y += dist_center * math.sin(self.theta)
        self.theta += d_theta
        
        # Normalisasi theta ke range -π sampai π
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        return self.x, self.y, math.degrees(self.theta)
    
    def set_motor_speed(self, left_speed, right_speed):
        # Batasi kecepatan antara -1.0 sampai 1.0
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))
        
        # Konversi ke duty cycle (0-100%)
        left_duty = int(abs(left_speed) * 10000)
        right_duty = int(abs(right_speed) * 10000)
        
        # Set arah dan kecepatan
        GPIO.output(config.MOTOR_L_IN1, GPIO.HIGH if left_speed >=0 else GPIO.LOW)
        GPIO.output(config.MOTOR_L_IN2, GPIO.LOW if left_speed >=0 else GPIO.HIGH)
        GPIO.output(config.MOTOR_R_IN1, GPIO.HIGH if right_speed >=0 else GPIO.LOW)
        GPIO.output(config.MOTOR_R_IN2, GPIO.LOW if right_speed >=0 else GPIO.HIGH)
        
        pi.hardware_PWM(config.MOTOR_L_ENA, 1000, left_duty)
        pi.hardware_PWM(config.MOTOR_R_ENB, 1000, right_duty)
    
    def set_vacuum(self, state):
        GPIO.output(config.RELAY_VACUUM, GPIO.HIGH if state else GPIO.LOW)
        if state:
            pi.set_servo_pulsewidth(config.ESC_PWM, config.VACUUM_POWER)  # Full speed
    
    def set_sweeper(self, state):
        GPIO.output(config.RELAY_SWEEPER, GPIO.HIGH if state else GPIO.LOW)
    
    def set_mop(self, state):
        GPIO.output(config.RELAY_MOP, GPIO.HIGH if state else GPIO.LOW)
    
    def set_water_pump(self, duration_ms):
        if self.water_level <= 0:
            return  # Stop jika air habis
            
        GPIO.output(config.RELAY_PUMP, GPIO.HIGH)
        time.sleep(duration_ms / 1000.0)
        GPIO.output(config.RELAY_PUMP, GPIO.LOW)
        self.water_level -= duration_ms * config.PUMP_FLOW_RATE
    
    def zigzag_navigation(self):
        """Pola navigasi zigzag dengan koreksi IMU"""
        _, gyro_data = self.read_imu()
        left_dist, right_dist = self.read_distance_sensors()
        
        # Hindari rintangan
        if min(left_dist, right_dist) < config.OBSTACLE_DISTANCE:
            if left_dist > right_dist:
                self.set_motor_speed(0.5, -0.5)  # Putar kanan
            else:
                self.set_motor_speed(-0.5, 0.5)  # Putar kiri
            time.sleep(1)
            return
        
        # Pola zigzag normal dengan koreksi gyro
        yaw = gyro_data['z'] - self.gyro_bias
        if yaw > 5:  # Derajat threshold
            self.set_motor_speed(config.ZIGZAG_SPEED_NORMAL + config.ZIGZAG_SPEED_CORRECTION, 
                                 config.ZIGZAG_SPEED_NORMAL)
        elif yaw < -5:
            self.set_motor_speed(config.ZIGZAG_SPEED_NORMAL,
                                 config.ZIGZAG_SPEED_NORMAL + config.ZIGZAG_SPEED_CORRECTION)
        else:
            self.set_motor_speed(config.ZIGZAG_SPEED_NORMAL, config.ZIGZAG_SPEED_NORMAL)
    
    def fine_tune_docking(self, left_ir, right_ir):
        """Presisi akhir berdasarkan sinyal IR"""
        print("Fine-tuning docking with IR...")
        while True:
            left_ir, right_ir = self.read_ir_docking()
            if left_ir and right_ir:
                self.set_motor_speed(-0.2, -0.2)  # Mundur perlahan
            elif left_ir:
                self.set_motor_speed(-0.1, -0.3)  # Koreksi kanan
            elif right_ir:
                self.set_motor_speed(-0.3, -0.1)  # Koreksi kiri
            else:
                break  # Sinyal hilang = sudah masuk dock
            
            # Cek jarak ke dock
            left_dist, _ = self.read_distance_sensors()
            if left_dist < 5:  # Sangat dekat
                break
                
            time.sleep(0.05)
    
    def return_to_home(self):
        """Kembali ke home (0,0) dengan odometri"""
        print("Starting RTH procedure...")
        last_update = time.time()
        
        while self.mode == "DOCKING":
            # Update odometri setiap 0.1 detik
            if time.time() - last_update > 0.1:
                x, y, theta = self.update_odometry()
                last_update = time.time()
                
                # Hitung vektor ke home
                dx = -x
                dy = -y
                distance_to_home = math.sqrt(dx**2 + dy**2)
                
                # Jika sudah dekat (<50cm), aktifkan docking IR
                if distance_to_home < 50:
                    left_ir, right_ir = self.read_ir_docking()
                    if left_ir or right_ir:
                        self.fine_tune_docking(left_ir, right_ir)
                        break
                
                # Hitung sudut menuju home
                target_angle = math.atan2(dy, dx)
                angle_error = target_angle - theta
                
                # Normalisasi error (-π to π)
                if angle_error > math.pi:
                    angle_error -= 2 * math.pi
                elif angle_error < -math.pi:
                    angle_error += 2 * math.pi
                
                # Kontrol PID sederhana
                Kp = 0.8
                steering = Kp * angle_error
                
                # Batasi steering
                steering = max(-0.5, min(0.5, steering))
                
                # Set kecepatan
                base_speed = 0.4
                left_speed = base_speed - steering
                right_speed = base_speed + steering
                
                self.set_motor_speed(left_speed, right_speed)
            
            time.sleep(0.01)
    
    def docking_procedure(self):
        """Prosedur kembali ke docking station"""
        # Pertama coba cari sinyal IR
        left_ir, right_ir = self.read_ir_docking()
        
        if left_ir or right_ir:
            # Jika sinyal IR terdeteksi, gunakan metode berbasis IR
            print("IR docking signal detected!")
            while self.mode == "DOCKING":
                left_ir, right_ir = self.read_ir_docking()
                
                # Algoritma line-follow IR
                if left_ir and right_ir:
                    self.set_motor_speed(-0.4, -0.4)  # Mundur lurus
                elif left_ir:
                    self.set_motor_speed(-0.2, -0.6)  # Koreksi kanan
                elif right_ir:
                    self.set_motor_speed(-0.6, -0.2)  # Koreksi kiri
                else:
                    self.set_motor_speed(0, 0)  # Berhenti jika sinyal hilang
                    
                # Cek apakah sudah sampai dock
                left_dist, _ = self.read_distance_sensors()
                if left_dist < config.DOCKING_DISTANCE:
                    self.set_motor_speed(0, 0)
                    self.mode = "CHARGING"
                    break
                    
                time.sleep(0.1)
        else:
            # Jika tidak ada sinyal IR, gunakan RTH dengan odometri
            self.return_to_home()
            self.mode = "CHARGING"
    
    def draw_eye(self, device, is_left_eye, state="open", offset_x=0, offset_y=0):
        """Menggambar mata dengan detail pada OLED"""
        with canvas(device) as draw:
            # Koordinat dan ukuran mata
            center_x = device.width // 2
            center_y = device.height // 2
            eye_radius = 25
            
            # Warna (putih untuk OLED monokrom)
            outline_color = "white"
            fill_color = "black"
            
            if state == "sleep":
                # Mata tidur (garis horizontal)
                draw.arc((center_x - eye_radius, center_y - 5, 
                          center_x + eye_radius, center_y + 5), 
                         180, 360, fill=outline_color)
                return
            
            # Gambar outline mata (elips)
            draw.ellipse((center_x - eye_radius, center_y - eye_radius, 
                          center_x + eye_radius, center_y + eye_radius), 
                         outline=outline_color, fill=fill_color)
            
            # Gambar iris (lingkaran lebih kecil)
            iris_radius = 15
            iris_x = center_x + offset_x
            iris_y = center_y + offset_y
            draw.ellipse((iris_x - iris_radius, iris_y - iris_radius, 
                          iris_x + iris_radius, iris_y + iris_radius), 
                         outline=outline_color, fill=outline_color)
            
            # Gambar pupil (lingkaran kecil)
            pupil_radius = 8
            draw.ellipse((iris_x - pupil_radius, iris_y - pupil_radius, 
                          iris_x + pupil_radius, iris_y + pupil_radius), 
                         outline=fill_color, fill=fill_color)
            
            # Gambar highlight kecil (untuk efek 3D)
            highlight_radius = 4
            highlight_x = iris_x - 6 + offset_x/2
            highlight_y = iris_y - 6 + offset_y/2
            draw.ellipse((highlight_x - highlight_radius, 
                          highlight_y - highlight_radius, 
                          highlight_x + highlight_radius, 
                          highlight_y + highlight_radius), 
                         outline="white", fill="white")
            
            # Gambar kelopak mata jika sedang berkedip
            if state == "blink":
                blink_height = 5
                draw.rectangle((center_x - eye_radius, center_y - blink_height, 
                                center_x + eye_radius, center_y + blink_height), 
                               fill=fill_color, outline=fill_color)
    
    def update_eye_animation(self):
        """Mengupdate animasi mata berdasarkan mode dan waktu"""
        current_time = time.time()
        
        # Mode Charging: mata selalu tidur
        if self.mode == "CHARGING":
            self.draw_eye(oled_left, is_left_eye=True, state="sleep")
            self.draw_eye(oled_right, is_left_eye=False, state="sleep")
            return
        
        # Mode Cleaning/Docking: animasi mata aktif
        # Atur pergerakan mata secara acak
        if current_time - self.last_eye_move_time > self.eye_move_interval:
            self.eye_offset_x = random.randint(-10, 10)
            self.eye_offset_y = random.randint(-5, 5)
            self.eye_move_interval = random.uniform(2, 5)
            self.last_eye_move_time = current_time
        
        # Atur animasi berkedip
        if current_time - self.last_blink_time > self.blink_interval:
            self.blink_state = 1  # Mulai berkedip
            self.last_blink_time = current_time
            self.blink_interval = random.uniform(3, 8)  # Interval acak berikutnya
        
        # Proses animasi berkedip
        if self.blink_state > 0:
            if self.blink_state == 1:
                # Fase pertama: menutup mata
                self.draw_eye(oled_left, is_left_eye=True, state="blink", 
                              offset_x=self.eye_offset_x, offset_y=self.eye_offset_y)
                self.draw_eye(oled_right, is_left_eye=False, state="blink", 
                              offset_x=self.eye_offset_x, offset_y=self.eye_offset_y)
                self.blink_state = 2
            elif self.blink_state == 2 and current_time - self.last_blink_time > 0.1:
                # Fase kedua: membuka mata
                self.eye_open = True
                self.blink_state = 0
        else:
            # Mata terbuka normal
            self.draw_eye(oled_left, is_left_eye=True, state="open", 
                          offset_x=self.eye_offset_x, offset_y=self.eye_offset_y)
            self.draw_eye(oled_right, is_left_eye=False, state="open", 
                          offset_x=self.eye_offset_x, offset_y=self.eye_offset_y)

    def emergency_stop(self):
        """Fungsi darurat menghentikan semua motor"""
        self.set_motor_speed(0, 0)
        self.set_vacuum(False)
        self.set_sweeper(False)
        self.set_mop(False)
        GPIO.output(config.RELAY_PUMP, GPIO.LOW)

# ===== MAIN LOOP =====
if __name__ == "__main__":
    robot = DUSKController()
    last_sensor_update = time.time()
    last_display_update = time.time()
    
    try:
        print("Starting DUSK Cleaning Robot...")
        robot.set_vacuum(True)
        robot.set_sweeper(True)
        
        while True:
            current_time = time.time()
            
            # Update status baterai
            voltage = robot.check_battery()
            
            # Update sensor secara periodik
            if current_time - last_sensor_update > config.SENSOR_UPDATE_INTERVAL:
                robot.update_odometry()
                last_sensor_update = current_time
            
            # Mode Operasi
            if robot.mode == "CLEANING":
                robot.zigzag_navigation()
                robot.set_mop(True)
                robot.set_water_pump(config.PUMP_PULSE_DURATION)
                
            elif robot.mode == "DOCKING":
                robot.set_vacuum(False)
                robot.set_sweeper(False)
                robot.set_mop(False)
                robot.docking_procedure()
                
            elif robot.mode == "CHARGING":
                robot.emergency_stop()
                print("Docking success! Charging...")
                while voltage < config.BATTERY_FULL_VOLTAGE:
                    time.sleep(5)
                    voltage = robot.check_battery()
                    print(f"Battery voltage: {voltage:.2f}V")
                    robot.update_eye_animation()
                robot.battery_low = False
                robot.mode = "CLEANING"
                # Reset odometri saat mulai cleaning baru
                robot.x, robot.y, robot.theta = 0.0, 0.0, 0.0
                robot.left_encoder_count = 0
                robot.right_encoder_count = 0
            
            # Update animasi mata
            robot.update_eye_animation()
            
            # Update display status secara periodik
            if current_time - last_display_update > config.DISPLAY_UPDATE_INTERVAL:
                last_display_update = current_time
            
            time.sleep(0.05)  #Loop rate ~20Hz
            
    except KeyboardInterrupt:
        print("Stopping by user request...")
    except Exception as e:
        print(f"Critical error: {str(e)}")
    finally:
        robot.emergency_stop()
        GPIO.cleanup()
        pi.stop()
        print("System shutdown complete.")
