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

# Глобальные состояния
ESP32_STATES = {
    'motors_armed': False,
    'stream_active': False,
    'throttle_value': 1100,
    'last_command': None,
    'last_command_time': None
}

# PID коэффициенты по умолчанию
PID_COEFFS = {
    'pitch': {'Kp': 4.0, 'Ki': 0.02, 'Kd': 0.5},
    'roll': {'Kp': 4.0, 'Ki': 0.02, 'Kd': 0.5},
    'yaw': {'Kp': 2.0, 'Ki': 0.005, 'Kd': 0.3}
}

telemetry_enabled = False
program_running = True
print_telemetry_flag = False
show_menu_after_command = False
calibration_done = False
calibration_prepared = False  # Флаг подготовки к калибровке

def clear_telemetry_line():
    """Очищает строку телеметрии"""
    print('\r' + ' ' * 120 + '\r', end='', flush=True)

def log_angles(timestamp, raw_yaw, raw_pitch, raw_roll, rel_pitch, rel_roll, control_pitch, control_roll, velocity_x, velocity_y):
    """Запись углов в лог файл"""
    data_line = f"{timestamp:.2f}\t{raw_yaw:.2f}\t{raw_pitch:.2f}\t{raw_roll:.2f}\t{rel_pitch:.2f}\t{rel_roll:.2f}\t{control_pitch:.2f}\t{control_roll:.2f}\t{velocity_x:.2f}\t{velocity_y:.2f}"
    with open(log_filepath, 'a', encoding='utf-8') as f:
        f.write(data_line + '\n')

def log_calibration_data():
    """Запись калибровочных данных в лог"""
    with open(log_filepath, 'a', encoding='utf-8') as f:
        f.write(f"# CALIBRATION_DATA: zero_pitch={CALIBRATION['zero_pitch']:.2f}, zero_roll={CALIBRATION['zero_roll']:.2f}, sensitivity={CALIBRATION['sensitivity']:.2f}, dead_zone={CALIBRATION['dead_zone']:.2f}\n")

def connect_airsim():
    """Подключение к AirSim"""
    global airsim_client, airsim_connected
    
    try:
        print("🚁 Подключение к AirSim...")
        airsim_client = airsim.MultirotorClient()
        airsim_client.confirmConnection()
        airsim_client.enableApiControl(True)
        airsim_client.armDisarm(False)
        airsim_connected = True
        print("✅ AirSim подключен")
        return True
    except Exception as e:
        print(f"❌ Ошибка подключения к AirSim: {e}")
        airsim_connected = False
        return False

# 🔗 Настройка сокета
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", PORT))
sock.settimeout(0.1)

# Утилиты для отправки команд на ESP32
def send_udp_cmd(cmd_str, silent=False):
    """Отправка команды на ESP32"""
    try:
        sock.sendto(cmd_str.encode('utf-8'), (ESP32_IP, PORT))
        ESP32_STATES['last_command'] = cmd_str
        ESP32_STATES['last_command_time'] = time.time()
        
        if not silent:
            print(f"\n📤 Sent -> {cmd_str}")
        
        # Обновляем состояния
        if cmd_str == "ARM":
            ESP32_STATES['motors_armed'] = True
        elif cmd_str == "DISARM":
            ESP32_STATES['motors_armed'] = False
        elif cmd_str == "START":
            ESP32_STATES['stream_active'] = True
        elif cmd_str == "STOP":
            ESP32_STATES['stream_active'] = False
        elif cmd_str.startswith("SET"):
            parts = cmd_str.split()
            if len(parts) >= 5:
                try:
                    ESP32_STATES['throttle_value'] = int(parts[4])
                except:
                    pass
        
        return True
    except Exception as e:
        if not silent:
            print(f"\n❌ Ошибка отправки команды '{cmd_str}': {e}")
        return False

def set_target_angles_and_throttle(yaw=0, pitch=0, roll=0, throttle=1100):
    """Установка целевых углов и газа на ESP32"""
    cmd = f"SET {yaw} {pitch} {roll} {throttle}"
    return send_udp_cmd(cmd)

# Подключаемся к AirSim
airsim_client = None
airsim_connected = connect_airsim()

print(f"📡 Ожидание данных от ESP32 ({ESP32_IP}:{PORT})...")

# 🔧 КАЛИБРОВОЧНЫЕ НАСТРОЙКИ
CALIBRATION = {
    'zero_pitch': 0.0,
    'zero_roll': 0.0,
    'max_velocity': 8.0,
    'dead_zone': 8.0,
    'sensitivity': 1.5,
    'invert_pitch': False,
    'invert_roll': True
}

# 🔧 ПОМОЩНИК ДЛЯ ЧТЕНИЯ ДАННЫХ ОТ ESP32
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

def prepare_for_calibration():
    """Подготовка к калибровке - настройка стрима"""
    global calibration_prepared
    
    print("\n" + "="*60)
    print("🔧 ПОДГОТОВКА К КАЛИБРОВКЕ")
    print("="*60)
    print("1. Убедитесь, что контроллер подключен к ESP32")
    print("2. Контроллер должен лежать на ровной поверхности")
    print("3. Включите стрим данных для получения углов")
    print("="*60)
    
    # Проверяем подключение к ESP32
    print("🔍 Проверка подключения к ESP32...")
    
    # Попробуем получить данные от ESP32
    data_received = False
    for i in range(10):  # 10 попыток
        data = get_current_sensor_data()
        if data:
            data_received = True
            yaw, pitch, roll = data
            print(f"   ✅ Данные получены: Pitch={pitch:.1f}°, Roll={roll:.1f}°, Yaw={yaw:.1f}°")
            break
        time.sleep(0.5)
        if i == 4:
            print("   ⏳ Включаю стрим данных...")
            send_udp_cmd("START", silent=True)
    
    if not data_received:
        print("   ❌ Не удалось получить данные от ESP32")
        print("   🔄 Пробую включить стрим вручную...")
        send_udp_cmd("START")
        time.sleep(1)
        
        # Еще 5 попыток после включения стрима
        for i in range(5):
            data = get_current_sensor_data()
            if data:
                data_received = True
                yaw, pitch, roll = data
                print(f"   ✅ Данные получены: Pitch={pitch:.1f}°, Roll={roll:.1f}°, Yaw={yaw:.1f}°")
                break
            time.sleep(0.5)
    
    if data_received:
        calibration_prepared = True
        print("\n✅ Подготовка к калибровке завершена!")
        print("   Стрим данных активен")
        return True
    else:
        print("\n❌ Не удалось подготовиться к калибровке")
        print("   Проверьте:")
        print("   1. Подключение ESP32 к WiFi")
        print("   2. Правильность IP адреса ESP32")
        print("   3. Что ESP32 запущен и работает")
        return False

# 🎯 ФУНКЦИЯ КАЛИБРОВКИ
def perform_calibration():
    """Выполнение калибровки после подготовки"""
    global calibration_done
    
    print("\n" + "="*60)
    print("🎯 НАЧАЛО КАЛИБРОВКИ")
    print("="*60)
    print("⚠️  ВНИМАНИЕ:")
    print("1. Положите контроллер на РОВНУЮ поверхность")
    print("2. Не двигайте контроллер во время калибровки")
    print("3. Процесс займет 5 секунд")
    print("="*60)
    
    # Обратный отсчет перед началом
    print("\n⏳ Начинаю через:")
    for i in range(5, 0, -1):
        print(f"   {i}...")
        time.sleep(1)
    
    print("   🚀 НАЧАЛО!")
    
    # Сбор данных для калибровки
    pitch_samples = []
    roll_samples = []
    
    print("\n📊 Сбор данных калибровки...")
    
    start_time = time.time()
    sample_count = 0
    progress_bar_length = 40
    
    while time.time() - start_time < 5:
        elapsed = time.time() - start_time
        progress = int((elapsed / 5) * progress_bar_length)
        
        data = get_current_sensor_data()
        if data:
            yaw, pitch, roll = data
            pitch_samples.append(pitch)
            roll_samples.append(roll)
            sample_count += 1
            
            # Показываем прогресс
            bar = '█' * progress + '░' * (progress_bar_length - progress)
            print(f"\r   [{bar}] {elapsed:.1f}/5.0с | Образцов: {sample_count}", end='', flush=True)
        time.sleep(0.05)
    
    print()  # Новая строка после прогресс-бара
    
    if pitch_samples and roll_samples:
        # Вычисляем средние значения
        CALIBRATION['zero_pitch'] = sum(pitch_samples) / len(pitch_samples)
        CALIBRATION['zero_roll'] = sum(roll_samples) / len(roll_samples)
        
        # Вычисляем стандартное отклонение для проверки стабильности
        if len(pitch_samples) > 1:
            pitch_std = math.sqrt(sum((x - CALIBRATION['zero_pitch'])**2 for x in pitch_samples) / (len(pitch_samples) - 1))
            roll_std = math.sqrt(sum((x - CALIBRATION['zero_roll'])**2 for x in roll_samples) / (len(roll_samples) - 1))
            
            print(f"\n📊 Статистика:")
            print(f"   📈 Pitch std: {pitch_std:.2f}° {'✅' if pitch_std < 2 else '⚠️'}")
            print(f"   📈 Roll std: {roll_std:.2f}° {'✅' if roll_std < 2 else '⚠️'}")
        
        print(f"\n✅ КАЛИБРОВКА ЗАВЕРШЕНА:")
        print(f"   🎯 Zero Pitch: {CALIBRATION['zero_pitch']:.1f}°")
        print(f"   🎯 Zero Roll: {CALIBRATION['zero_roll']:.1f}°")
        print(f"   📊 Образцов собрано: {sample_count}")
        
        # Логируем
        log_calibration_data()
        
        calibration_done = True
        return True
    else:
        print("❌ Ошибка калибровки: не удалось собрать данные")
        return False

def calculate_oriented_velocity(rel_pitch, rel_roll, drone_yaw):
    """Преобразование углов в скорости для управления дроном"""
    # Инвертируем оси при необходимости
    if CALIBRATION['invert_pitch']:
        rel_pitch = -rel_pitch
    
    if CALIBRATION['invert_roll']:
        rel_roll = -rel_roll
    
    # Применяем чувствительность
    control_pitch = rel_pitch * CALIBRATION['sensitivity']
    control_roll = rel_roll * CALIBRATION['sensitivity']
    
    # Ограничиваем углы управления
    max_control = 45.0
    control_pitch = max(-max_control, min(max_control, control_pitch))
    control_roll = max(-max_control, min(max_control, control_roll))
    
    # Pitch (тангаж) = влево/вправо
    # Roll (крен) = вперед/назад
    local_forward = -control_roll * CALIBRATION['max_velocity'] / max_control
    local_right = control_pitch * CALIBRATION['max_velocity'] / max_control
    
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

# ТЕКУЩИЙ РЕЖИМ
current_mode = 1
menu_shown = False
camera_menu_shown = False
drone_in_air = False

# Параметры камеры
camera_pitch = 0
camera_yaw = 0

def apply_camera_orientation(): 
    """Применяет ориентацию камеры"""
    if not airsim_connected:
        return False
    
    try:
        airsim_client.simSetCameraPose(
            "0",
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
    except:
        return False

def get_key():
    """Считывание клавиши без ENTER"""
    try:
        if msvcrt.kbhit():
            key = msvcrt.getch()
            if key == b'\xe0':
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

def show_status():
    """Показывает текущий статус системы"""
    print("\n" + "="*60)
    print("📊 ТЕКУЩИЙ СТАТУС СИСТЕМЫ")
    print("="*60)
    print(f"ESP32:")
    print(f"  Моторы: {'✅ ARM' if ESP32_STATES['motors_armed'] else '❌ DISARM'}")
    print(f"  Стрим: {'✅ ВКЛ' if ESP32_STATES['stream_active'] else '❌ ВЫКЛ'}")
    print(f"  Throttle: {ESP32_STATES['throttle_value']} мкс")
    print("-" * 60)
    print(f"AirSim:")
    print(f"  Подключение: {'✅' if airsim_connected else '❌'}")
    if airsim_connected:
        try:
            state = airsim_client.getMultirotorState()
            drone_state = '✈️ В воздухе' if state.landed_state == 0 else '🛬 На земле'
            print(f"  Дрон: {drone_state}")
        except:
            print(f"  Дрон: ❓ Неизвестно")
    print("-" * 60)
    print(f"Калибровка:")
    print(f"  Статус: {'✅ ВЫПОЛНЕНА' if calibration_done else '❌ НЕ ВЫПОЛНЕНА'}")
    if calibration_done:
        print(f"  Zero Pitch: {CALIBRATION['zero_pitch']:.1f}°")
        print(f"  Zero Roll: {CALIBRATION['zero_roll']:.1f}°")
    print("-" * 60)
    print(f"PID коэффициенты:")
    print(f"  Pitch: Kp={PID_COEFFS['pitch']['Kp']}, Ki={PID_COEFFS['pitch']['Ki']}, Kd={PID_COEFFS['pitch']['Kd']}")
    print(f"  Roll:  Kp={PID_COEFFS['roll']['Kp']}, Ki={PID_COEFFS['roll']['Ki']}, Kd={PID_COEFFS['roll']['Kd']}")
    print(f"  Yaw:   Kp={PID_COEFFS['yaw']['Kp']}, Ki={PID_COEFFS['yaw']['Ki']}, Kd={PID_COEFFS['yaw']['Kd']}")
    print("="*60)

def show_main_menu():
    """Показывает главное меню"""
    global menu_shown, print_telemetry_flag
    
    if not menu_shown:
        clear_telemetry_line()
        print("\n" + "="*70)
        print("🔧 ГЛАВНОЕ МЕНЮ УПРАВЛЕНИЯ:")
        print("="*70)
        print("   [1] Включить телеметрию")
        print("   [2] Настройка камеры")
        print("-" * 70)
        print("   [3] Управление дроном (установка углов)")
        print("        Задает target angles на ESP32")
        print("-" * 70)
        print("   [4] Показать PID коэффициенты")
        print("-" * 70)
        print("   [a] ARM (включить моторы)")
        print("   [z] DISARM (выключить моторы)")
        print("-" * 70)
        print("   [s] START стрим")
        print("   [x] STOP стрим")
        print("-" * 70)
        print("   [t] Takeoff (AirSim)")
        print("   [l] Land (AirSim)")
        print("-" * 70)
        print("   [i] Показать текущий статус")
        print("   [ ] Пробел - выключить телеметрию")
        print("-" * 70)
        print("   [r] Переподключиться к AirSim")
        print("   [q] Посадка и выход")
        print("="*70)
        
        if calibration_done:
            print("✅ Калибровка выполнена. Система готова к работе.")
        else:
            print("❌ Калибровка не выполнена. Сначала выполните калибровку!")
        
        print("="*70)
        print("👉 Нажмите клавишу (цифра/буква)")
        menu_shown = True
        print_telemetry_flag = False

def show_camera_menu():
    """Показывает меню камеры"""
    global camera_menu_shown, print_telemetry_flag
    
    if not camera_menu_shown:
        clear_telemetry_line()
        print("\n🎥 РЕЖИМ НАСТРОЙКИ КАМЕРЫ")
        print("  ↑/↓ — наклон камеры (вверх/вниз)")
        print("  ←/→ — поворот камеры (влево/вправо)") 
        print("  [b] — возврат в меню")
        camera_menu_shown = True
        print_telemetry_flag = False

def update_camera_display():
    """Обновляет отображение углов камеры"""
    print(f"\r  📷 Камера: Pitch={camera_pitch:3}°, Yaw={camera_yaw:3}°", end='', flush=True)

def drone_control_menu():
    """Меню управления дроном - установка целевых углов"""
    global print_telemetry_flag
    
    print_telemetry_flag = False
    clear_telemetry_line()
    
    print("\n" + "="*60)
    print("🎮 УПРАВЛЕНИЕ ДРОНОМ (Target Angles)")
    print("="*60)
    print("Установите желаемые углы для PID контроллера на ESP32")
    print("-" * 60)
    print("Диапазоны углов:")
    print("  Pitch (тангаж): -30° до +30° (наклон вперед/назад)")
    print("  Roll (крен):    -30° до +30° (крен влево/вправо)")
    print("  Yaw (рыскание): -180° до +180° (поворот)")
    print("-" * 60)
    print("Throttle (газ): 1000-2000 мкс")
    print("  1000 - минимальная скорость")
    print("  1100 - рекомендуемая для висения")
    print("  1300+ - увеличение высоты")
    print("-" * 60)
    
    # Текущие значения
    current_throttle = ESP32_STATES['throttle_value']
    
    while True:
        try:
            print(f"\nТекущий throttle: {current_throttle} мкс")
            
            # Ввод углов
            yaw_input = input("Введите Yaw (рыскание) в градусах [0]: ").strip()
            yaw = float(yaw_input) if yaw_input else 0.0
            
            pitch_input = input("Введите Pitch (тангаж) в градусах [0]: ").strip()
            pitch = float(pitch_input) if pitch_input else 0.0
            
            roll_input = input("Введите Roll (крен) в градусах [0]: ").strip()
            roll = float(roll_input) if roll_input else 0.0
            
            throttle_input = input(f"Введите Throttle (1000-2000) [{current_throttle}]: ").strip()
            if throttle_input:
                throttle = int(throttle_input)
                if throttle < 1000 or throttle > 2000:
                    print(f"❌ Ошибка: throttle должен быть от 1000 до 2000")
                    continue
            else:
                throttle = current_throttle
            
            # Проверка диапазонов
            if abs(pitch) > 30:
                print(f"⚠️ Pitch ({pitch}°) выходит за рекомендуемый диапазон ±30°")
            
            if abs(roll) > 30:
                print(f"⚠️ Roll ({roll}°) выходит за рекомендуемый диапазон ±30°")
            
            print(f"\n📢 Будет отправлена команда: SET {yaw:.1f} {pitch:.1f} {roll:.1f} {throttle}")
            print(f"   ESP32 будет пытаться удерживать эти углы")
            
            confirm = input("Подтвердить отправку? (y/n): ").strip().lower()
            if confirm == 'y':
                if set_target_angles_and_throttle(yaw, pitch, roll, throttle):
                    print(f"✅ Команда отправлена: Yaw={yaw:.1f}°, Pitch={pitch:.1f}°, Roll={roll:.1f}°, Throttle={throttle} мкс")
                return
            else:
                print("Отмена.")
                return
                
        except ValueError:
            print("❌ Ошибка: введите числовые значения!")
        except Exception as e:
            print(f"❌ Ошибка: {e}")

def show_pid_info():
    """Показывает информацию о PID коэффициентах"""
    global print_telemetry_flag
    
    print_telemetry_flag = False
    clear_telemetry_line()
    
    print("\n" + "="*60)
    print("⚙️ PID КОЭФФИЦИЕНТЫ (ТОЛЬКО ДЛЯ ПРОСМОТРА)")
    print("="*60)
    print("Эти коэффициенты запрограммированы в ESP32")
    print("и регулируют стабилизацию дрона по углам.")
    print("-" * 60)
    print("📊 Текущие значения:")
    print(f"  Pitch (тангаж):")
    print(f"    Kp = {PID_COEFFS['pitch']['Kp']} - пропорциональный коэффициент")
    print(f"    Ki = {PID_COEFFS['pitch']['Ki']} - интегральный коэффициент")
    print(f"    Kd = {PID_COEFFS['pitch']['Kd']} - дифференциальный коэффициент")
    print()
    print(f"  Roll (крен):")
    print(f"    Kp = {PID_COEFFS['roll']['Kp']}")
    print(f"    Ki = {PID_COEFFS['roll']['Ki']}")
    print(f"    Kd = {PID_COEFFS['roll']['Kd']}")
    print()
    print(f"  Yaw (рыскание):")
    print(f"    Kp = {PID_COEFFS['yaw']['Kp']}")
    print(f"    Ki = {PID_COEFFS['yaw']['Ki']}")
    print(f"    Kd = {PID_COEFFS['yaw']['Kd']}")
    print("-" * 60)
    print("ℹ️  Изменение PID коэффициентов требует перепрошивки ESP32")
    print("="*60)
    
    input("\nНажмите Enter для возврата в меню...")

def menu_thread_func():
    global current_mode, program_running, camera_pitch, camera_yaw, menu_shown, camera_menu_shown, telemetry_enabled, print_telemetry_flag, show_menu_after_command, drone_in_air, calibration_done
    
    while program_running:
        key = get_key()
        
        if key:
            clear_telemetry_line()
            
            if current_mode == 1:
                if key == '1':
                    if not calibration_done:
                        print("\n❌ Сначала выполните калибровку!")
                        show_menu_after_command = True
                        continue
                    
                    print_telemetry_flag = True
                    telemetry_enabled = True
                    print("\n📊 Телеметрия ВКЛЮЧЕНА")
                    show_menu_after_command = True
                elif key == '2':
                    current_mode = 2
                    menu_shown = False
                    camera_menu_shown = False
                    show_camera_menu()
                    update_camera_display()
                elif key == '3':
                    if not calibration_done:
                        print("\n❌ Сначала выполните калибровку!")
                        show_menu_after_command = True
                        continue
                    
                    if not ESP32_STATES['stream_active']:
                        print("\n⚠️  Стрим данных не активен!")
                        print("   Включите стрим командой [s]")
                        show_menu_after_command = True
                        continue
                    
                    drone_control_menu()
                    show_menu_after_command = True
                elif key == '4':
                    show_pid_info()
                    show_menu_after_command = True
                elif key == 'a':
                    if not calibration_done:
                        print("\n❌ Сначала выполните калибровку!")
                        show_menu_after_command = True
                        continue
                    
                    if not ESP32_STATES['stream_active']:
                        print("\n⚠️  Стрим данных не активен!")
                        print("   Включите стрим командой [s]")
                        show_menu_after_command = True
                        continue
                    
                    print("\n⚠️  АРМИРОВАНИЕ МОТОРОВ")
                    confirm = input("Продолжить? (y/n): ").strip().lower()
                    if confirm == 'y':
                        if send_udp_cmd("ARM"):
                            print("✅ Команда ARM отправлена на ESP32")
                    else:
                        print("❌ ARM отменен")
                    show_menu_after_command = True
                elif key == 'z':
                    print("\n⚠️  DISARM МОТОРОВ")
                    confirm = input("Продолжить? (y/n): ").strip().lower()
                    if confirm == 'y':
                        if send_udp_cmd("DISARM"):
                            print("✅ Команда DISARM отправлена на ESP32")
                    else:
                        print("❌ DISARM отменен")
                    show_menu_after_command = True
                elif key == 's':
                    if send_udp_cmd("START"):
                        print("✅ Стрим данных START отправлен")
                    show_menu_after_command = True
                elif key == 'x':
                    if send_udp_cmd("STOP"):
                        print("✅ Стрим данных STOP отправлен")
                    show_menu_after_command = True
                elif key == 't':
                    if not airsim_connected:
                        print("❌ AirSim не подключен! Используйте [r]")
                        show_menu_after_command = True
                        continue
                    
                    if not ESP32_STATES['motors_armed']:
                        print("❌ Сначала взведите моторы [a]!")
                        show_menu_after_command = True
                        continue
                    
                    print("\n🛫 ВЗЛЕТ ДРОНА")
                    confirm = input("Выполнить взлет? (y/n): ").strip().lower()
                    if confirm == 'y':
                        try:
                            print("🛫 Взлетаем...")
                            airsim_client.armDisarm(True)
                            airsim_client.takeoffAsync().join()
                            time.sleep(2)
                            airsim_client.moveToZAsync(-5, 2).join()
                            drone_in_air = True
                            print("✅ Взлет выполнен успешно! Высота: 5 метра")
                        except Exception as e:
                            print(f"❌ Ошибка при взлете: {e}")
                    else:
                        print("❌ Взлет отменен")
                    show_menu_after_command = True
                elif key == 'l':
                    if not airsim_connected:
                        print("❌ AirSim не подключен!")
                        show_menu_after_command = True
                        continue
                    
                    print("\n🛬 Начинаю посадку...")
                    try:
                        airsim_client.landAsync().join()
                        time.sleep(2)
                        airsim_client.armDisarm(False)
                        drone_in_air = False
                        print("✅ Посадка выполнена успешно!")
                    except Exception as e:
                        print(f"❌ Ошибка при посадке: {e}")
                    show_menu_after_command = True
                elif key == 'i':
                    show_status()
                    show_menu_after_command = True
                elif key == ' ':
                    if telemetry_enabled:
                        telemetry_enabled = False
                        print_telemetry_flag = False
                        print("\n📊 Телеметрия ВЫКЛЮЧЕНА")
                    show_menu_after_command = True
                elif key == 'r':
                    print("\n🔄 Переподключение к AirSim...")
                    connect_airsim()
                    show_menu_after_command = True
                elif key == 'q':
                    print("\n🛬 Запуск посадки и выход...")
                    program_running = False
                    break

            elif current_mode == 2:
                step = 5
                changed = False
                
                if key == 'up':
                    camera_pitch += step
                    changed = True
                elif key == 'down':
                    camera_pitch -= step
                    changed = True
                elif key == 'left':
                    camera_yaw -= step
                    changed = True
                elif key == 'right':
                    camera_yaw += step
                    changed = True
                elif key == 'b':
                    current_mode = 1
                    menu_shown = False
                    camera_menu_shown = False
                    print_telemetry_flag = telemetry_enabled
                    print("\n↩️ Возврат в главное меню")
                    continue

                if changed:
                    if apply_camera_orientation():
                        update_camera_display()

        time.sleep(0.1)

# Запускаем меню в отдельном потоке
menu_thread = threading.Thread(target=menu_thread_func, daemon=True)
menu_thread.start()

print("\n" + "="*60)
print("🚀 СИСТЕМА УПРАВЛЕНИЯ ДРОНОМ")
print("="*60)

# Пишем заголовок в лог
with open(log_filepath, 'w', encoding='utf-8') as f:
    f.write("Time(s)\tRaw_Yaw\tRaw_Pitch\tRaw_Roll\tRel_Pitch\tRel_Roll\tControl_Pitch\tControl_Roll\tVelocity_X\tVelocity_Y\n")
print(f"📝 Логирование начато: {log_filename}")

# 🔴 ОБЯЗАТЕЛЬНАЯ КАЛИБРОВКА ПРИ ЗАПУСКЕ
print("\n" + "="*60)
print("🔴 ОБЯЗАТЕЛЬНАЯ КАЛИБРОВКА ПРИ ЗАПУСКЕ")
print("="*60)

# Шаг 1: Подготовка к калибровке
print("\n📋 ШАГ 1: ПОДГОТОВКА К КАЛИБРОВКЕ")
print("-" * 40)

input("⏎ Нажмите Enter, когда будете готовы начать подготовку...")

preparation_success = prepare_for_calibration()

if not preparation_success:
    print("\n❌ ПОДГОТОВКА НЕ УДАЛАСЬ!")
    print("   Программа завершена.")
    print("="*60)
    program_running = False
else:
    # Шаг 2: Выполнение калибровки
    print("\n📋 ШАГ 2: ВЫПОЛНЕНИЕ КАЛИБРОВКИ")
    print("-" * 40)
    
    print("\n⚠️  Подготовьте контроллер:")
    print("   1. Положите на РОВНУЮ поверхность")
    print("   2. Убедитесь, что он неподвижен")
    print("   3. Не прикасайтесь к нему во время калибровки")
    
    ready = input("\n⏎ Нажмите Enter, когда будете готовы начать калибровку...")
    
    calibration_success = perform_calibration()
    
    if not calibration_success:
        print("\n❌ КАЛИБРОВКА НЕ УДАЛАСЬ!")
        print("   Можно повторить позже через меню")
        print("="*60)
    else:
        print("\n✅ КАЛИБРОВКА УСПЕШНО ЗАВЕРШЕНА!")
        print("   Теперь можно управлять дроном")
        print("="*60)

# Переменные для основного цикла
prev_yaw, prev_pitch, prev_roll = 0, 0, 0
last_command_time = time.time()
start_time = time.time()

try:
    while program_running:
        current_time = time.time()
        
        if show_menu_after_command:
            menu_shown = False
            show_menu_after_command = False
        
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
                    
                    # Вывод телеметрии если включена
                    if print_telemetry_flag and telemetry_enabled and current_mode == 1:
                        print(f"\r📊 Raw: P={pitch:6.1f}° R={roll:6.1f}° Y={yaw:6.1f}° | Rel: P={rel_pitch:6.1f}° R={rel_roll:6.1f}° | V: X={velocity_x:5.2f} Y={velocity_y:5.2f} m/s", end='', flush=True)
                    
                    # Логируем углы
                    timestamp = current_time - start_time
                    log_angles(timestamp, yaw, pitch, roll, rel_pitch, rel_roll, 
                              control_pitch, control_roll, velocity_x, velocity_y)
                    
                    # Управление дроном в AirSim если он в воздухе
                    if airsim_connected and drone_in_air:
                        try:
                            airsim_client.moveByVelocityZAsync(
                                velocity_x, 
                                velocity_y, 
                                -3,
                                0.1,
                                airsim.DrivetrainType.MaxDegreeOfFreedom,
                                airsim.YawMode(False, yaw)
                            )
                            last_command_time = current_time
                        except:
                            pass
                        
        except socket.timeout:
            pass
        
        # Keep-alive
        if current_time - last_command_time > 0.5 and airsim_connected and drone_in_air:
            try:
                airsim_client.moveByVelocityZAsync(0, 0, -3, 0.2, 
                                                  airsim.DrivetrainType.MaxDegreeOfFreedom,
                                                  airsim.YawMode(True, 0))
                last_command_time = current_time
            except:
                pass

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n🛑 Остановка по Ctrl+C...")
    program_running = False

except Exception as e:
    print(f"\n❌ Критическая ошибка: {e}")
    program_running = False

finally:
    print("\n🛬 Безопасное завершение...")
    
    # Посадка дрона
    if airsim_connected and drone_in_air:
        try:
            print("🛬 Посадка дрона...")
            airsim_client.landAsync().join()
            time.sleep(1)
            airsim_client.armDisarm(False)
            print("✅ Дрон приземлен")
        except:
            pass
    
    # Отключение ESP32
    try:
        print("🔌 Отключение ESP32...")
        send_udp_cmd("STOP", silent=True)
        time.sleep(0.1)
        send_udp_cmd("DISARM", silent=True)
    except:
        pass
    
    # Закрытие соединений
    try:
        sock.close()
    except:
        pass
    
    print(f"\n💾 Лог сохранен в: {log_filepath}")
    print("✅ Программа завершена")