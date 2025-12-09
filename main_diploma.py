import socket
import os
import time
from datetime import datetime
import airsim
import math
import threading
import sys
import msvcrt

# 🔧 Настройки
ESP32_IP = "172.20.10.3"
PORT = 3333

# 📁 Создание папки для логов
LOG_DIR = "logs"
if not os.path.exists(LOG_DIR):
    os.makedirs(LOG_DIR)

# 📝 Создание файла лога
log_filename = f"angles_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
log_filepath = os.path.join(LOG_DIR, log_filename)

def log_angles(timestamp, raw_yaw, raw_pitch, raw_roll, rel_pitch, rel_roll, control_pitch, control_roll, velocity_x, velocity_y):
    """Запись углов в лог файл"""
    data_line = f"{timestamp:.2f}\t{raw_yaw:.2f}\t{raw_pitch:.2f}\t{raw_roll:.2f}\t{rel_pitch:.2f}\t{rel_roll:.2f}\t{control_pitch:.2f}\t{control_roll:.2f}\t{velocity_x:.2f}\t{velocity_y:.2f}"
    with open(log_filepath, 'a', encoding='utf-8') as f:
        f.write(data_line + '\n')

def log_calibration_data():
    """Запись калибровочных данных в лог"""
    with open(log_filepath, 'a', encoding='utf-8') as f:
        f.write(f"# CALIBRATION_DATA: zero_pitch={CALIBRATION['zero_pitch']:.2f}, zero_roll={CALIBRATION['zero_roll']:.2f}, sensitivity={CALIBRATION['sensitivity']:.2f}, dead_zone={CALIBRATION['dead_zone']:.2f}\n")

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

time.sleep(2)

print(f"📡 Ожидание данных от ESP32 ({ESP32_IP}:{PORT})...")
sock.sendto(b"START", (ESP32_IP, PORT))

def get_current_sensor_data():
    try:
        data, addr = sock.recvfrom(1024)
        msg = data.decode().strip()
        parts = msg.split(',')
        if len(parts) == 3:
            return float(parts[0]), float(parts[1]), float(parts[2])
    except:
        pass
    return None

# 🔧 КАЛИБРОВОЧНЫЕ НАСТРОЙКИ
CALIBRATION = {
    'zero_pitch': 0.0,     # Будет установлено при калибровке
    'zero_roll': 0.0,      # Будет установлено при калибровке
    'max_velocity': 8.0,   # Максимальная скорость
    'dead_zone': 8.0,      # Мертвая зона
    'sensitivity': 1.5,    # Общая чувствительность
    'invert_pitch': False,  # ИНВЕРТИРОВАТЬ ТОЛЬКО PITCH
    'invert_roll': True   # НЕ инвертировать ROLL
}

# 🎯 ФУНКЦИЯ КАЛИБРОВКИ
def calibrate_controller():
    """Автоматическая калибровка нулевых положений"""
    print("\n🎯 НАЧАЛО АВТОМАТИЧЕСКОЙ КАЛИБРОВКИ")
    print("=" * 50)
    print("1. Положите контроллер на РОВНУЮ поверхность")
    print("2. Не двигайте контроллер 3 секунды...")
    
    time.sleep(3)
    
    # Сбор данных для калибровки
    pitch_samples = []
    roll_samples = []
    yaw_samples = []
    
    print("📊 Сбор данных калибровки...")
    
    start_time = time.time()
    sample_count = 0
    
    while time.time() - start_time < 3:  # Сбор данных 3 секунды
        data = get_current_sensor_data()
        if data:
            yaw, pitch, roll = data
            yaw_samples.append(yaw)
            pitch_samples.append(pitch)
            roll_samples.append(roll)
            sample_count += 1
            print(f"   Собрано образцов: {sample_count}", end='\r')
        time.sleep(0.05)
    
    # Очищаем прогресс-бар
    print()
    
    if pitch_samples and roll_samples:
        # Вычисляем средние значения
        CALIBRATION['zero_pitch'] = sum(pitch_samples) / len(pitch_samples)
        CALIBRATION['zero_roll'] = sum(roll_samples) / len(roll_samples)
        
        print(f"\n✅ КАЛИБРОВКА ЗАВЕРШЕНА:")
        print(f"   🎯 Zero Pitch: {CALIBRATION['zero_pitch']:.1f}°")
        print(f"   🎯 Zero Roll: {CALIBRATION['zero_roll']:.1f}°")
        print(f"   📊 Образцов собрано: {sample_count}")
        
        return True
    else:
        print("❌ Ошибка калибровки: не удалось собрать данные")
        return False

# 🔧 ВЫБОР РЕЖИМА КАЛИБРОВКИ
print("\n🔧 НАСТРОЙКА КАЛИБРОВКИ")
print("1 - Использовать стандартные настройки")
print("2 - Автоматическая калибровка")

calibration_choice = input("Выберите режим (1 или 2): ").strip()

if calibration_choice == "2":
    if calibrate_controller():
        print("✅ Калибровка успешно завершена")
    else:
        print("❌ Калибровка не удалась, используются стандартные настройки")
        CALIBRATION['zero_pitch'] = 27.0
        CALIBRATION['zero_roll'] = -14.0
else:
    print("⚙️ Используются стандартные калибровочные значения")
    CALIBRATION['zero_pitch'] = 27.0
    CALIBRATION['zero_roll'] = -14.0

print("\n⚙️  Настройки управления:")
print(f"   Нулевые: Pitch={CALIBRATION['zero_pitch']:.1f}°, Roll={CALIBRATION['zero_roll']:.1f}°")
print(f"   Макс. скорость: {CALIBRATION['max_velocity']} m/s")
print(f"   Инвертировать Pitch: {CALIBRATION['invert_pitch']}")
print(f"   Инвертировать Roll: {CALIBRATION['invert_roll']}")

# Создаем заголовок для лог файла и записываем калибровочные данные
with open(log_filepath, 'w', encoding='utf-8') as f:
    f.write("Time(s)\tRaw_Yaw\tRaw_Pitch\tRaw_Roll\tRel_Pitch\tRel_Roll\tControl_Pitch\tControl_Roll\tVelocity_X\tVelocity_Y\n")
    log_calibration_data()

print(f"📝 Логирование начато: {log_filename}")
print(f"💾 Калибровочные данные записаны в лог")

def calculate_oriented_velocity(rel_pitch, rel_roll, drone_yaw):
    """
    Преобразование с исправленным только тангажом
    """
    # 🔧 ИНВЕРТИРУЕМ ОСИ ПРИ НЕОБХОДИМОСТИ
    if CALIBRATION['invert_pitch']:
        rel_pitch = -rel_pitch  # Инвертируем тангаж (влево/вправо)
    
    if CALIBRATION['invert_roll']:
        rel_roll = -rel_roll    # Инвертируем крен (вперед/назад)
    
    # Применяем чувствительность
    control_pitch = rel_pitch * CALIBRATION['sensitivity']
    control_roll = rel_roll * CALIBRATION['sensitivity']
    
    # Ограничиваем углы управления
    max_control = 45.0
    control_pitch = max(-max_control, min(max_control, control_pitch))
    control_roll = max(-max_control, min(max_control, control_roll))
    
    
    # Pitch (тангаж) = влево/вправо
    # Roll (крен) = вперед/назад
    local_forward = -control_roll * CALIBRATION['max_velocity'] / max_control    # Roll → вперед/назад
    local_right = control_pitch * CALIBRATION['max_velocity'] / max_control      # Pitch → влево/вправо
    
    # Минимальная скорость для реакции
    min_speed = 0.5
    if abs(local_forward) < min_speed and abs(local_right) < min_speed:
        return 0, 0, control_pitch, control_roll
    
    # Преобразование в глобальные скорости
    yaw_rad = math.radians(drone_yaw)
    global_x = local_forward * math.cos(yaw_rad) - local_right * math.sin(yaw_rad)
    global_y = local_forward * math.sin(yaw_rad) + local_right * math.cos(yaw_rad)
    
    # Увеличиваем минимальную скорость
    speed = math.sqrt(global_x**2 + global_y**2)
    if 0 < speed < min_speed:
        global_x = global_x * min_speed / speed
        global_y = global_y * min_speed / speed
    
    return global_x, global_y, control_pitch, control_roll

# Основной цикл
prev_yaw, prev_pitch, prev_roll = 0, 0, 0
last_command_time = time.time()
start_time = time.time()

# ТЕКУЩИЙ РЕЖИМ:
# 1 – вывод телеметрии (основной)
# 2 – меню камеры
current_mode = 1
program_running = True
menu_shown = False
camera_menu_shown = False

# Параметры камеры FPV (основной вид)
camera_pitch = 0  # тангаж камеры (вверх/вниз)
camera_yaw = 0    # рыскание камеры (влево/вправо)

def apply_camera_orientation(): 
    """Применяет ориентацию к FPV камере через gimbal"""
    try:
        # Используем gimbal для управления камерой - более надежный способ
        client.simSetCameraPose(
            "0",  # основная камера дрона
            airsim.Pose(
                airsim.Vector3r(0, 0, 0),  # позиция не меняем
                airsim.to_quaternion(
                    math.radians(camera_pitch),  # pitch
                    math.radians(0),             # roll - не используем
                    math.radians(camera_yaw)     # yaw
                )
            )
        )
        return True
    except Exception as e:
        try:
            # Альтернативный способ для FPV камеры
            client.simSetCameraPose(
                "fpv",  # FPV камера
                airsim.Pose(
                    airsim.Vector3r(0, 0, 0),
                    airsim.to_quaternion(
                        math.radians(camera_pitch),
                        math.radians(0),
                        math.radians(camera_yaw)
                    )
                )
            )
            return True
        except Exception as e2:
            print(f"  ❌ Ошибка камеры: {e2}")
            return False

def get_key():
    """Считывание клавиши без ENTER"""
    try:
        if msvcrt.kbhit():
            key = msvcrt.getch()
            if key == b'\xe0':  # стрелки
                key = msvcrt.getch()
                if key == b'H': return 'up'
                if key == b'P': return 'down'
                if key == b'K': return 'left'
                if key == b'M': return 'right'
            else:
                return key.decode('utf-8').lower()
    except:
        pass
    return None

def show_main_menu():
    """Показывает главное меню один раз"""
    global menu_shown
    if not menu_shown:
        print("\n" + "="*50)
        print("🔧 Меню управления:")
        print("   [1] Режим телеметрии")
        print("   [2] Настройка камеры") 
        print("   [q] Посадка и выход")
        print("👉 Нажмите клавишу (1, 2 или q)")
        print("="*50)
        menu_shown = True

def show_camera_menu():
    """Показывает меню камеры один раз"""
    global camera_menu_shown
    if not camera_menu_shown:
        print("\n🎥 РЕЖИМ НАСТРОЙКИ КАМЕРЫ")
        print("  ↑/↓ — наклон камеры (вверх/вниз)")
        print("  ←/→ — поворот камеры (влево/вправо)") 
        print("  [b] — возврат в меню")
        print("  Текущие углы: Pitch=0°, Yaw=0°")
        camera_menu_shown = True

def update_camera_display():
    """Обновляет отображение углов камеры в одной строке"""
    print(f"\r  📷 Камера: Pitch={camera_pitch}°, Yaw={camera_yaw}°", end=' ' * 10, flush=True)
def menu_thread():
    global current_mode, program_running, camera_pitch, camera_yaw, menu_shown, camera_menu_shown

    while program_running:
        key = get_key()
        
        if key:
            if current_mode == 1:
                # Главное меню
                if key == '1':
                    current_mode = 1
                    menu_shown = False
                    camera_menu_shown = False
                    print("\n📡 Режим телеметрии")
                elif key == '2':
                    current_mode = 2
                    menu_shown = False
                    camera_menu_shown = False
                    show_camera_menu()
                elif key == 'q':
                    print("\n🛬 Запуск посадки...")
                    program_running = False
                    break

            elif current_mode == 2:
                # Режим настройки камеры
                step = 5  # шаг в градусах
                changed = False
                
                if key == 'up':
                    camera_pitch += step
                    changed = True
                elif key == 'down':
                    camera_pitch -= step
                    changed = True
                elif key == 'left':
                    camera_yaw -= step  #  лево = уменьшение yaw
                    changed = True
                elif key == 'right':
                    camera_yaw += step  #  право = увеличение yaw
                    changed = True
                elif key == 'b':
                    current_mode = 1
                    menu_shown = False
                    camera_menu_shown = False
                    print("\n↩️ Возврат в меню")
                    continue

                if changed:
                    if apply_camera_orientation():
                        update_camera_display()

        time.sleep(0.1)

# Запускаем меню в отдельном потоке
menu_thread = threading.Thread(target=menu_thread, daemon=True)
menu_thread.start()

print("\n🚀 УПРАВЛЕНИЕ АКТИВНО!")
print("💡 Наклоняйте контроллер для управления:")
print("   ▶️  Наклон ВПЕРЕД = летит ВПЕРЕД")
print("   ◀️  Наклон ВЛЕВО = летит ВЛЕВО") 
print("   ▶️  Наклон ВПРАВО = летит ВПРАВО")
print("   🔽 Наклон НАЗАД = летит НАЗАД")
print("🛑 Для остановки нажмите q в меню")

try:
    while program_running:
        current_time = time.time()
        
        # Показываем меню только когда нужно
        if current_mode == 1 and not menu_shown:
            show_main_menu()
        elif current_mode == 2 and not camera_menu_shown:
            show_camera_menu()
        
        try:
            data, addr = sock.recvfrom(1024)
            msg = data.decode().strip()

            if ',' in msg:
                parts = msg.split(',')
                if len(parts) == 3:
                    yaw_raw, pitch_raw, roll_raw = float(parts[0]), float(parts[1]), float(parts[2])
                    
                    if any(math.isnan(x) for x in [yaw_raw, pitch_raw, roll_raw]):
                        continue

                    # Сглаживание
                    yaw = 0.8 * prev_yaw + 0.2 * yaw_raw
                    pitch = 0.8 * prev_pitch + 0.2 * pitch_raw
                    roll = 0.8 * prev_roll + 0.2 * roll_raw
                    prev_yaw, prev_pitch, prev_roll = yaw, pitch, roll
                    
                    # Относительные углы
                    rel_pitch = pitch - CALIBRATION['zero_pitch']
                    rel_roll = roll - CALIBRATION['zero_roll']
                    
                    # Мертвая зона
                    if abs(rel_pitch) < CALIBRATION['dead_zone']:
                        rel_pitch = 0
                    if abs(rel_roll) < CALIBRATION['dead_zone']:
                        rel_roll = 0
                    
                    # Рассчитываем скорости
                    velocity_x, velocity_y, control_pitch, control_roll = calculate_oriented_velocity(rel_pitch, rel_roll, yaw)
                    
                    # 🔧 ВЫВОДИМ ТЕЛЕМЕТРИЮ ТОЛЬКО В РЕЖИМЕ 1
                    if current_mode == 1:
                        print(f"\r📊 Raw: P={pitch:6.1f}° R={roll:6.1f}° Y={yaw:6.1f}° | Rel: P={rel_pitch:6.1f}° R={rel_roll:6.1f}° | V: X={velocity_x:5.2f} Y={velocity_y:5.2f} m/s", end='', flush=True)
                    
                    # Логируем углы в файл (всегда)
                    timestamp = current_time - start_time
                    log_angles(timestamp, yaw, pitch, roll, rel_pitch, rel_roll, 
                              control_pitch, control_roll, velocity_x, velocity_y)
                    
                    # Управление дроном
                    client.moveByVelocityZAsync(
                        velocity_x, 
                        velocity_y, 
                        -10, 
                        0.1,
                        airsim.DrivetrainType.MaxDegreeOfFreedom,
                        airsim.YawMode(False, yaw)
                    )
                    
                    last_command_time = current_time
                        
        except socket.timeout:
            pass
        
        # Keep-alive
        if current_time - last_command_time > 0.5:
            client.moveByVelocityZAsync(0, 0, -10, 0.2, 
                                      airsim.DrivetrainType.MaxDegreeOfFreedom,
                                      airsim.YawMode(True, 0))
            last_command_time = current_time

        time.sleep(0.05)  # Небольшая задержка для стабильности

except KeyboardInterrupt:
    print("\n🛑 Остановка по Ctrl+C...")
    program_running = False

finally:
    print("\n🛬 Запуск посадки...")
    sock.sendto(b"STOP", (ESP32_IP, PORT))
    sock.close()
    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)
    print("✅ Завершено")
    print(f"💾 Лог сохранен в: {log_filepath}")