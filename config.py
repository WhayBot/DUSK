#GPIO PIN CONFIGURATION
#Sensor and Actuator Control
VL53L0X_LEFT_XSHUT = 4      #XSHUT pin for left VL53L0X (active LOW for reset)
VL53L0X_RIGHT_XSHUT = 17    #XSHUT pin for right VL53L0X
IR_RECEIVER_LEFT = 22        #Left IR receiver output
IR_RECEIVER_RIGHT = 23       #Right IR receiver output
RELAY_VACUUM = 24            #Relay for vacuum motor (IN1)
RELAY_SWEEPER = 25           #Relay for sweeper motor (IN2)
RELAY_MOP = 26               #Relay for mop motor (IN3)
RELAY_PUMP = 27              #Relay for water pump (IN4)

#Motor Driver
MOTOR_L_IN1 = 12             #L298N IN1 for left motor direction
MOTOR_L_IN2 = 13             #L298N IN2 for left motor direction
MOTOR_R_IN1 = 19             #L298N IN3 for right motor direction
MOTOR_R_IN2 = 16             #L298N IN4 for right motor direction
MOTOR_L_ENA = 10             #L298N ENA (PWM for left motor speed)
MOTOR_R_ENB = 9              #L298N ENB (PWM for right motor speed)

# === WHEEL ENCODER CONFIG ===
ENCODER_LEFT_A = 14    #GPIO14 (TXD)
ENCODER_LEFT_B = 15    #GPIO15 (RXD)
ENCODER_RIGHT_A = 7    #GPIO7 (CE1)
ENCODER_RIGHT_B = 8    #GPIO8 (CE0)
PULSES_PER_REV = 20    #Jumlah pulsa per putaran roda
WHEEL_DIAMETER = 6.5   #cm (diameter roda 65mm)

#Brushless Motor Control
ESC_PWM = 18                 #PWM signal pin for ESC (controls brushless motor)

#I2C Addresses
MPU6050_ADDRESS = 0x68       #IMU sensor address
OLED_LEFT_ADDRESS = 0x3C     #Left OLED display
OLED_RIGHT_ADDRESS = 0x3D    #Right OLED display
VL53L0X_LEFT_ADDRESS = 0x29  #Left distance sensor
VL53L0X_RIGHT_ADDRESS = 0x30 #Right distance sensor
INA219_ADDRESS = 0x40        #Voltage sensor

#SYSTEM PARAMETERS
#Battery Management
BATTERY_LOW_VOLTAGE = 10.5   #Voltage threshold for low battery (switch to docking mode)
BATTERY_FULL_VOLTAGE = 12.0  #Voltage threshold for full battery (stop charging)
BATTERY_NOMINAL_VOLTAGE = 11.1  #Li-Po 3S nominal voltage

#Cleaning System
PUMP_FLOW_RATE = 0.1         #Water pump flow rate in mL per millisecond
WATER_TANK_CAPACITY = 100    #Maximum water capacity in mL
VACUUM_POWER = 2000          #ESC pulse width at full power (μs)

#Navigation
OBSTACLE_DISTANCE = 20       #Minimum obstacle distance in cm (trigger avoidance)
DOCKING_DISTANCE = 10        #Distance to docking station to stop (cm)
ZIGZAG_SPEED_NORMAL = 0.6    #Base speed during cleaning (0.0-1.0)
ZIGZAG_SPEED_CORRECTION = 0.2 #Speed differential for course correction

#OLED Display
OLED_WIDTH = 128             #Display width in pixels
OLED_HEIGHT = 64             #Display height in pixels

#Timing Parameters
SENSOR_UPDATE_INTERVAL = 0.1 #Time between sensor readings (seconds)
DISPLAY_UPDATE_INTERVAL = 0.5 #Time between display updates (seconds)
PUMP_PULSE_DURATION = 300    #Water pump activation time (ms)
