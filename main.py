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

# KONFIGURASI GPIO & PERANGKAT
GPIO.setmode(GPIO.BCM)
pi = pigpio.pi()

# Pin Definitions
VL53L0X_LEFT_XSHUT = 4
VL53L0X_RIGHT_XSHUT = 17
IR_RECEIVER_LEFT = 22
IR_RECEIVER_RIGHT = 23
RELAY_VACUUM = 24
RELAY_SWEEPER = 25
RELAY_MOP = 26
RELAY_PUMP = 27
ESC_PWM = 18
MOTOR_L_IN1 = 12
MOTOR_L_IN2 = 13
MOTOR_R_IN1 = 19
MOTOR_R_IN2 = 16
MOTOR_L_ENA = 10
MOTOR_R_ENB = 9

# Inisialisasi GPIO
GPIO.setup([
    VL53L0X_LEFT_XSHUT, VL53L0X_RIGHT_XSHUT,
    IR_RECEIVER_LEFT, IR_RECEIVER_RIGHT,
    RELAY_VACUUM, RELAY_SWEEPER, RELAY_MOP, RELAY_PUMP
], GPIO.OUT, initial=GPIO.LOW)

# INISIALISASI SENSOR & MODUL

# I2C Bus Setup
i2c_bus0 = busio.I2C(board.SCL, board.SDA)
i2c_bus1 = SMBus(1)  # I2C Bus 1

# OLED Display
serial_oled = i2c(i2c_bus0, address=0x3C)
oled_left = ssd1306(serial_oled, width=128, height=64, rotate=0)
serial_oled2 = i2c(i2c_bus0, address=0x3D)
oled_right = ssd1306(serial_oled2, width=128, height=64, rotate=0)

# IMU Sensor
mpu = mpu6050(0x68, bus=1)

# Voltage Sensor
ina219 = adafruit_ina219.INA219(i2c_bus0, address=0x40)

# LiDAR Sensors
def init_vl53l0x():
    GPIO.output(VL53L0X_LEFT_XSHUT, GPIO.LOW)
    GPIO.output(VL53L0X_RIGHT_XSHUT, GPIO.LOW)
    time.sleep(0.1)
    
    # Aktifkan sensor kiri
    GPIO.output(VL53L0X_LEFT_XSHUT, GPIO.HIGH)
    time.sleep(0.1)
    sensor_left = VL53L0X(i2c_bus1, 0x29)
    sensor_left.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
    
    # Aktifkan sensor kanan
    GPIO.output(VL53L0X_RIGHT_XSHUT, GPIO.HIGH)
    time.sleep(0.1)
    sensor_right = VL53L0X(i2c_bus1, 0x30)
    sensor_right.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
    
    return sensor_left, sensor_right

vl53_left, vl53_right = init_vl53l0x()

# Motor Control Setup
pi.set_mode(MOTOR_L_ENA, pigpio.OUTPUT)
pi.set_mode(MOTOR_R_ENB, pigpio.OUTPUT)
pi.set_PWM_frequency(MOTOR_L_ENA, 1000)  # 1 kHz
pi.set_PWM_frequency(MOTOR_R_ENB, 1000)

# ESC Calibration
pi.set_servo_pulsewidth(ESC_PWM, 1500)  # Netral position
time.sleep(2)

# MAIN FUNCTION
class DUSKController:
    def __init__(self):
        self.mode = "CLEANING"  # CLEANING, DOCKING, CHARGING
        self.battery_low = False
        self.docking_found = False
        self.clean_pattern = "ZIGZAG"
        self.water_level = 100  # mL
        
        # Variabel animasi mata
        self.last_blink_time = time.time()
        self.last_eye_move_time = time.time()
        self.eye_open = True
        self.blink_state = 0
        self.eye_offset_x = 0
        self.eye_offset_y = 0
        self.blink_interval = random.uniform(3, 8)  # Interval kedip acak
        self.eye_move_interval = random.uniform(2, 5)  # Interval gerakan mata acak
        
    def check_battery(self):
        voltage = ina219.bus_voltage
        if voltage < 10.5:  # Threshold 10.5V
            self.battery_low = True
            self.mode = "DOCKING"
        return voltage
    
    def read_distance_sensors(self):
        left_dist = vl53_left.get_distance() / 10.0  # mm to cm
        right_dist = vl53_right.get_distance() / 10.0
        return left_dist, right_dist
    
    def read_ir_docking(self):
        left_ir = GPIO.input(IR_RECEIVER_LEFT)
        right_ir = GPIO.input(IR_RECEIVER_RIGHT)
        return left_ir, right_ir
    
    def read_imu(self):
        accel = mpu.get_accel_data()
        gyro = mpu.get_gyro_data()
        return accel, gyro
    
    def set_motor_speed(self, left_speed, right_speed):
        # Konversi ke duty cycle (0-100%)
        left_duty = int(abs(left_speed) * 10000)
        right_duty = int(abs(right_speed) * 10000)
        
        # Set arah dan kecepatan
        GPIO.output(MOTOR_L_IN1, GPIO.HIGH if left_speed >=0 else GPIO.LOW)
        GPIO.output(MOTOR_L_IN2, GPIO.LOW if left_speed >=0 else GPIO.HIGH)
        GPIO.output(MOTOR_R_IN1, GPIO.HIGH if right_speed >=0 else GPIO.LOW)
        GPIO.output(MOTOR_R_IN2, GPIO.LOW if right_speed >=0 else GPIO.HIGH)
        
        pi.hardware_PWM(MOTOR_L_ENA, 1000, left_duty)
        pi.hardware_PWM(MOTOR_R_ENB, 1000, right_duty)
    
    def set_vacuum(self, state):
        GPIO.output(RELAY_VACUUM, GPIO.HIGH if state else GPIO.LOW)
        if state:
            pi.set_servo_pulsewidth(ESC_PWM, 2000)  # Full speed
        else:
            pi.set_servo_pulsewidth(ESC_PWM, 1500)  # Netral
    
    def set_sweeper(self, state):
        GPIO.output(RELAY_SWEEPER, GPIO.HIGH if state else GPIO.LOW)
    
    def set_mop(self, state):
        GPIO.output(RELAY_MOP, GPIO.HIGH if state else GPIO.LOW)
    
    def set_water_pump(self, duration_ms):
        GPIO.output(RELAY_PUMP, GPIO.HIGH)
        time.sleep(duration_ms / 1000.0)
        GPIO.output(RELAY_PUMP, GPIO.LOW)
        self.water_level -= duration_ms * 0.1  # Estimasi debit
    
    def zigzag_navigation(self):
        """Pola navigasi zigzag dengan koreksi IMU"""
        accel, gyro = self.read_imu()
        left_dist, right_dist = self.read_distance_sensors()
        
        # Hindari rintangan
        if min(left_dist, right_dist) < 20:  # 20cm threshold
            if left_dist > right_dist:
                self.set_motor_speed(0.5, -0.5)  # Putar kanan
            else:
                self.set_motor_speed(-0.5, 0.5)  # Putar kiri
            time.sleep(1)
            return
        
        # Pola zigzag normal
        yaw = gyro['z']
        if yaw > 10:
            self.set_motor_speed(0.7, 0.5)
        elif yaw < -10:
            self.set_motor_speed(0.5, 0.7)
        else:
            self.set_motor_speed(0.6, 0.6)
    
    def docking_procedure(self):
        """Prosedur kembali ke docking station"""
        left_ir, right_ir = self.read_ir_docking()
        
        # Algoritma line-follow IR
        if left_ir and right_ir:
            self.set_motor_speed(0.4, 0.4)  # Maju lurus
        elif left_ir and not right_ir:
            self.set_motor_speed(0.2, 0.6)  # Koreksi kanan
        elif not left_ir and right_ir:
            self.set_motor_speed(0.6, 0.2)  # Koreksi kiri
        else:
            self.set_motor_speed(0, 0)  # Berhenti jika sinyal hilang
            
        # Cek apakah sudah sampai dock
        left_dist, _ = self.read_distance_sensors()
        if left_dist < 10:  # cm dari dock
            self.set_motor_speed(0, 0)
            self.mode = "CHARGING"
    
    def draw_eye(self, device, is_left_eye, state="open", offset_x=0, offset_y=0):
        """Menggambar mata dengan detail pada OLED"""
        with canvas(device) as draw:
            # Koordinat dan ukuran mata
            center_x = 64
            center_y = 32
            eye_radius = 25
            
            # Warna (putih untuk OLED monokrom)
            outline_color = "white"
            fill_color = "black"
            
            if state == "sleep":
                # Mata tidur (garis horizontal)
                draw.arc((center_x - eye_radius, center_y - 5, center_x + eye_radius, center_y + 5), 
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
            draw.ellipse((highlight_x - highlight_radius, highlight_y - highlight_radius, 
                          highlight_x + highlight_radius, highlight_y + highlight_radius), 
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
        
        # Mode Cleaning: animasi mata aktif
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
        GPIO.output(RELAY_PUMP, GPIO.LOW)

# MAIN LOOP
if __name__ == "__main__":
    robot = DUSKController()
    
    try:
        print("Starting DUSK Cleaning Robot...")
        robot.set_vacuum(True)
        robot.set_sweeper(True)
        
        while True:
            # Update status baterai
            voltage = robot.check_battery()
            
            # Mode Operasi
            if robot.mode == "CLEANING":
                robot.zigzag_navigation()
                robot.set_mop(True)
                robot.set_water_pump(300)  # Pompa 0.3mL air setiap iterasi
                
            elif robot.mode == "DOCKING":
                robot.set_vacuum(False)
                robot.set_sweeper(False)
                robot.set_mop(False)
                robot.docking_procedure()
                
            elif robot.mode == "CHARGING":
                robot.emergency_stop()
                print("Docking success! Charging...")
                while voltage < 12.0:  # Tunggu hingga penuh
                    time.sleep(60)
                    voltage = robot.check_battery()
                    robot.update_eye_animation()  # Tetap update animasi saat charging
                    time.sleep(0.1)
                robot.battery_low = False
                robot.mode = "CLEANING"
            
            # Update animasi mata
            robot.update_eye_animation()
            time.sleep(0.05)  # Refresh rate lebih tinggi untuk animasi halus
            
    except KeyboardInterrupt:
        print("Stopping by user request...")
    except Exception as e:
        print(f"Critical error: {str(e)}")
    finally:
        robot.emergency_stop()
        GPIO.cleanup()
        pi.stop()
        print("System shutdown complete.")
