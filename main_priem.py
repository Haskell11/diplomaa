import socket
import os
import time
from datetime import datetime
import airsim
import math

# 🔧 Настройки
ESP32_IP = "172.20.10.3"
PORT = 3333

# 📁 Папка для логов
log_dir = "logs"
os.makedirs(log_dir, exist_ok=True)

timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
log_file = os.path.join(log_dir, f"orientation_airsim_{timestamp}.txt")
headers = ["Time(s)", "Yaw(°)", "Pitch(°)", "Roll(°)", "VX", "VY", "VZ"]

with open(log_file, "w", encoding="utf-8") as f:
    f.write("=== Natural Motion Control ===\n")
    f.write("\t".join(headers) + "\n")

# 🔗 Настройка сокета
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", PORT))
sock.settimeout(0.1)

# 🔗 Подключение к AirSim
print("🚁 Connecting to AirSim...")
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

print("🛫 Taking off...")
client.takeoffAsync().join()
print("✅ Takeoff complete")

print("⬆️ Moving to altitude 10m...")
client.moveToZAsync(-10, 3).join()
print("✅ Altitude set")

print(f"📡 Ожидание данных от ESP32 ({ESP32_IP}:{PORT})...")
sock.sendto(b"START", (ESP32_IP, PORT))
print("🚀 Отправлена команда START\n")

start_time = time.time()
last_command_time = time.time()

# РУЧНАЯ КАЛИБРОВКА - установите когда MPU6050 лежит ровно
MANUAL_ZERO_PITCH = -1.0   # ⬅️ Измените на реальные значения
MANUAL_ZERO_ROLL = 14.0    # ⬅️ Измените на реальные значения

print(f"🔧 Калибровочные значения: Pitch={MANUAL_ZERO_PITCH}°, Roll={MANUAL_ZERO_ROLL}°")

# Переменные для сглаживания
prev_yaw = 0
prev_pitch = 0
prev_roll = 0

def degrees_to_radians(deg):
    return deg * math.pi / 180.0

def apply_dead_zone(value, dead_zone=5.0):
    """Применяет мертвую зону - игнорирует маленькие движения"""
    if abs(value) < dead_zone:
        return 0
    return value

try:
    while True:
        current_time = time.time()
        
        try:
            data, addr = sock.recvfrom(1024)
            msg = data.decode().strip()

            # Обработка данных
            if ',' in msg:
                parts = msg.split(',')
                if len(parts) == 3:
                    try:
                        yaw = float(parts[0])
                        pitch = float(parts[1])
                        roll = float(parts[2])
                        
                        # Сглаживание
                        yaw = 0.8 * yaw + 0.2 * prev_yaw
                        pitch = 0.8 * pitch + 0.2 * prev_pitch
                        roll = 0.8 * roll + 0.2 * prev_roll
                        
                        prev_yaw, prev_pitch, prev_roll = yaw, pitch, roll
                        
                        # 🔄 ЕСТЕСТВЕННОЕ УПРАВЛЕНИЕ
                        
                        # 1. Вычисляем ОТНОСИТЕЛЬНЫЕ углы от калибровки
                        rel_pitch = pitch - MANUAL_ZERO_PITCH
                        rel_roll = roll - MANUAL_ZERO_ROLL
                        
                        # 2. Применяем мертвую зону (игнорируем дрожание рук)
                        rel_pitch = apply_dead_zone(rel_pitch, 8.0)   # 8° мертвая зона
                        rel_roll = apply_dead_zone(rel_roll, 8.0)     # 8° мертвая зона
                        
                        # 3. УМЕНЬШЕННЫЕ коэффициенты чувствительности
                        pitch_sensitivity = 0.15  # УМЕНЬШЕНО в 2 раза
                        roll_sensitivity = 0.15   # УМЕНЬШЕНО в 2 раза
                        yaw_sensitivity = 0.2     # УМЕНЬШЕНО в 2.5 раза
                        
                        # 4. Рассчитываем скорости (теперь более плавно)
                        vx = rel_pitch * pitch_sensitivity  # Наклон вперед = движение вперед
                        vy = -rel_roll * roll_sensitivity   # Наклон влево = движение влево
                        
                        # 5. Ограничиваем скорости для безопасности
                        max_speed = 2.0  # УМЕНЬШЕНА максимальная скорость
                        vx = max(min(vx, max_speed), -max_speed)
                        vy = max(min(vy, max_speed), -max_speed)
                        
                        # 6. Рыскание - скорость поворота (теперь медленнее)
                        yaw_rate = yaw * yaw_sensitivity
                        max_yaw_rate = 15.0  # УМЕНЬШЕНА максимальная скорость поворота
                        yaw_rate = max(min(yaw_rate, max_yaw_rate), -max_yaw_rate)
                        
                        # 7. Высота - фиксированная для простоты
                        target_z = -10  # Фиксированная высота
                        
                        print(f"📊 Raw: P={pitch:5.1f}°, R={roll:5.1f}° | Tilt: P={rel_pitch:5.1f}°, R={rel_roll:5.1f}°")
                        print(f"🎮 Speed: vx={vx:4.1f}, vy={vy:4.1f} | Yaw: {yaw_rate:4.1f}°/s")
                        
                        # Управление дроном
                        client.moveByVelocityZAsync(
                            vx, vy, target_z, 0.1,
                            airsim.DrivetrainType.MaxDegreeOfFreedom,
                            airsim.YawMode(True, yaw_rate)
                        )
                        
                        last_command_time = current_time
                        
                        # Логирование
                        log_time = round(current_time - start_time, 2)
                        row = [
                            str(log_time), 
                            str(round(yaw, 2)), 
                            str(round(pitch, 2)), 
                            str(round(roll, 2)),
                            str(round(vx, 2)),
                            str(round(vy, 2)),
                            str(round(target_z, 2))
                        ]
                        with open(log_file, "a", encoding="utf-8") as f:
                            f.write("\t".join(row) + "\n")
                            
                    except ValueError:
                        pass
                        
        except socket.timeout:
            pass
        
        # Keep-alive
        if current_time - last_command_time > 0.15:
            client.moveByVelocityZAsync(0, 0, -10, 0.1,
                                      airsim.DrivetrainType.MaxDegreeOfFreedom,
                                      airsim.YawMode(True, 0))
            last_command_time = current_time

except KeyboardInterrupt:
    print("\n🛑 Остановка...")

finally:
    sock.sendto(b"STOP", (ESP32_IP, PORT))
    sock.close()
    print("🛬 Landing...")
    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)
    print(f"💾 Данные сохранены в {log_file}")