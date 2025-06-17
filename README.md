# DUSK
DUSK - Dust Unification and Sweeping Keeper
![DUSK Logo](assets/images/dusk_logo.png)

Robot pembersih otomatis berbasis Raspberry Pi Zero 2W dengan fitur vacuum, mopping, dan navigasi otonom.

## 🔧 Fitur Utama
- Navigasi pola zigzag dengan koreksi IMU
- Docking otomatis saat baterai rendah
- Sistem mopping presisi dengan pompa peristaltik
- Tampilan OLED ekspresi ganda

## 💻 Hardware Requirements
- Raspberry Pi Zero 2W
- RPi Camera V2
- 2x VL53L0X Time-of-Flight Sensors
- MPU6050 IMU
- 2x OLED 1.3"
- Motor driver L298N
- Brushless Motor 775 + ESC
- Relay 8-channel
- Peristaltic Pump 5V
- Li-Po 11.1V 5000mAh

## 🔌 Wiring Diagram
![Wiring](assets/images/wiring_diagram.png)

## ⚙️ Instalasi
```bash
git clone https://github.com/[username]/DUSK-Robot.git
cd DUSK-Robot

# Install dependencies
sudo apt update
sudo apt install python3-pigpio
pip install -r requirements.txt

# Enable camera interface
sudo raspi-config
# Interface Options > Camera > Enable
```

## 🚀 Menjalankan Robot
```bash
sudo pigpiod  # Start pigpio daemon
python main.py
```

## 📜 Lisensi
Proyek ini dilisensikan di bawah MIT License - lihat [LICENSE](LICENSE) untuk detail.
