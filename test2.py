import socket
import os
import time
import math
import threading
import sys
from datetime import datetime
from dataclasses import dataclass, field
from typing import Optional, Tuple, Dict, Any, List
import airsim

try:
    import msvcrt
    WINDOWS = True
except ImportError:
    import select
    import termios
    import tty
    WINDOWS = False

# ============================================================================
# КОНСТАНТЫ И КОНФИГУРАЦИЯ
# ============================================================================

class Config:
    """Конфигурация программы"""
    ESP32_IP = "172.20.10.3"
    PORT = 3333
    LOG_DIR = "logs"
    TARGET_ALTITUDE = -5  # Высота в AirSim (отрицательная = вверх)
    MIN_THROTTLE = 1000
    MAX_THROTTLE = 2000
    DEFAULT_THROTTLE = 1100
    MAX_ANGLE_LIMIT = 30.0  # Максимальный угол наклона контроллера
    MIN_SPEED_THRESHOLD = 0.3  # Минимальная скорость для реакции
    DEAD_ZONE_DEFAULT = 3.0  # Мертвая зона
    SENSITIVITY_DEFAULT = 1.2  # Чувствительность
    MAX_VELOCITY = 8.0  # Максимальная скорость дрона
    SOCKET_TIMEOUT = 0.1
    MAIN_LOOP_SLEEP = 0.05
    KEEP_ALIVE_INTERVAL = 0.5
    CAMERA_UPDATE_INTERVAL = 0.2  # Интервал обновления камеры (секунды)

# ============================================================================
# КЛАССЫ ДАННЫХ
# ============================================================================

@dataclass
class ESP32State:
    """Состояние ESP32"""
    motors_armed: bool = False
    stream_active: bool = False
    throttle_value: int = Config.DEFAULT_THROTTLE
    last_command: Optional[str] = None
    last_command_time: Optional[float] = None

@dataclass
class PIDCoeffs:
    """PID коэффициенты"""
    Kp: float
    Ki: float
    Kd: float

@dataclass
class CalibrationData:
    """Данные калибровки (как во второй программе)"""
    zero_pitch: float = 0.0
    zero_roll: float = 0.0
    max_velocity: float = Config.MAX_VELOCITY
    dead_zone: float = Config.DEAD_ZONE_DEFAULT
    sensitivity: float = Config.SENSITIVITY_DEFAULT
    invert_pitch: bool = False      # НЕ инвертируем по умолчанию
    invert_roll: bool = True       # Инвертируем roll по умолчанию
    calibrated: bool = False
    samples_pitch: List[float] = field(default_factory=list)
    samples_roll: List[float] = field(default_factory=list)

@dataclass
class SensorData:
    """Данные сенсоров"""
    yaw: float
    pitch: float
    roll: float
    timestamp: float

@dataclass
class ControlData:
    """Контрольные данные (как во второй программе)"""
    velocity_x: float
    velocity_y: float
    control_pitch: float
    control_roll: float

@dataclass
class TargetAngles:
    """Целевые углы для ESP32"""
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    throttle: int = Config.DEFAULT_THROTTLE
    active: bool = False  # Флаг активного режима target angles

# ============================================================================
# КЛАСС ДЛЯ КАЛИБРОВКИ
# ============================================================================

class CalibrationManager:
    """Класс для управления калибровкой датчиков"""
    
    def __init__(self, esp32_controller, logger):
        self.esp32 = esp32_controller
        self.logger = logger
        self.calibration = CalibrationData()
        self.sampling = False
        self.sample_count = 0
        self.target_samples = 40
        self.calibration_start_time = 0
        
    def prepare_calibration(self) -> bool:
        """Подготовка к калибровке (как во второй программе)"""
        print("\n" + "="*60)
        print("🔧 ПОДГОТОВКА К КАЛИБРОВКЕ")
        print("="*60)
        
        print("🔍 Проверка подключения к ESP32...")
        
        data_received = False
        for i in range(10):
            data = self.esp32.receive_data()
            if data:
                data_received = True
                yaw, pitch, roll = data
                print(f"✅ Данные получены: Pitch={pitch:.1f}°, Roll={roll:.1f}°, Yaw={yaw:.1f}°")
                break
            time.sleep(0.5)
            
            if i == 4:
                print("⏳ Включаю стрим данных...")
                self.esp32.send_command("START", silent=True)
        
        if not data_received:
            print("❌ Не удалось получить данные от ESP32")
            return False
        
        print("\n✅ Подготовка к калибровке завершена!")
        return True
    
    def perform_calibration(self) -> bool:
        """Выполнение калибровки (как во второй программе)"""
        print("\n" + "="*60)
        print("🎯 ВЫПОЛНЕНИЕ КАЛИБРОВКИ")
        print("="*60)
        
        print("📌 Положите контроллер на ровную поверхность")
        print("⏳ Начинаю через 3 секунды...")
        
        for i in range(3, 0, -1):
            print(f"   {i}...")
            time.sleep(1)
        
        print("   🚀 НАЧАЛО!")
        
        pitch_samples = []
        roll_samples = []
        
        print("\n📊 Сбор данных калибровки...")
        
        start_time = time.time()
        sample_count = 0
        progress_bar_length = 40
        
        while time.time() - start_time < 4:
            elapsed = time.time() - start_time
            progress = int((elapsed / 4) * progress_bar_length)
            
            data = self.esp32.receive_data()
            if data:
                yaw, pitch, roll = data
                pitch_samples.append(pitch)
                roll_samples.append(roll)
                sample_count += 1
                
                bar = '█' * progress + '░' * (progress_bar_length - progress)
                print(f"\r   [{bar}] {elapsed:.1f}/4.0с | Образцов: {sample_count}", 
                      end='', flush=True)
            time.sleep(0.05)
        
        print()
        
        if not pitch_samples or not roll_samples:
            print("❌ Ошибка калибровки: не удалось собрать данные")
            return False
        
        # Вычисляем средние значения (только pitch и roll как во второй программе)
        self.calibration.zero_pitch = sum(pitch_samples) / len(pitch_samples)
        self.calibration.zero_roll = sum(roll_samples) / len(roll_samples)
        
        print(f"\n✅ КАЛИБРОВКА ЗАВЕРШЕНА:")
        print(f"   🎯 Zero Pitch: {self.calibration.zero_pitch:.1f}°")
        print(f"   🎯 Zero Roll: {self.calibration.zero_roll:.1f}°")
        print(f"   📊 Образцов собрано: {sample_count}")
        
        self.calibration.calibrated = True
        return True
    
    def manual_calibration_menu(self) -> bool:
        """Ручная калибровка через меню (упрощенная как во второй программе)"""
        print("\n" + "="*60)
        print("🎯 РУЧНАЯ НАСТРОЙКА УПРАВЛЕНИЯ")
        print("="*60)
        
        input("Нажмите Enter для начала настройки...")
        
        if not self.calibration.calibrated:
            print("⚠️  Сначала выполните автоматическую калибровку!")
            return False
        
        print("\n🔄 НАСТРОЙКА НАПРАВЛЕНИЙ (как во второй программе):")
        print("По умолчанию:")
        print("  - Наклон вперед = движение вперед")
        print("  - Наклон вправо = движение вправо")
        print("  - Roll инвертирован для более интуитивного управления")
        
        pitch_invert = input("Инвертировать ось Pitch (тангаж)? [y/n, по умолчанию n]: ").strip().lower()
        self.calibration.invert_pitch = (pitch_invert == 'y')
        
        roll_invert = input("Инвертировать ось Roll (крен)? [y/n, по умолчанию y]: ").strip().lower()
        self.calibration.invert_roll = (roll_invert != 'n')  # По умолчанию инвертируем
        
        print("\n⚙️  Дополнительные настройки:")
        
        try:
            dead_zone = input(f"Мертвая зона (по умолчанию {self.calibration.dead_zone}): ").strip()
            if dead_zone:
                self.calibration.dead_zone = float(dead_zone)
        except ValueError:
            pass
        
        try:
            sensitivity = input(f"Чувствительность (по умолчанию {self.calibration.sensitivity}): ").strip()
            if sensitivity:
                self.calibration.sensitivity = float(sensitivity)
        except ValueError:
            pass
        
        try:
            max_vel = input(f"Максимальная скорость (по умолчанию {self.calibration.max_velocity}): ").strip()
            if max_vel:
                self.calibration.max_velocity = float(max_vel)
        except ValueError:
            pass
        
        print("\n" + "="*60)
        print("📊 ИТОГОВЫЕ НАСТРОЙКИ:")
        print(f"   Нулевой Pitch: {self.calibration.zero_pitch:.1f}°")
        print(f"   Нулевой Roll:  {self.calibration.zero_roll:.1f}°")
        print(f"   Инверсия Pitch: {'✅ ВКЛ' if self.calibration.invert_pitch else '❌ ВЫКЛ'}")
        print(f"   Инверсия Roll:  {'✅ ВКЛ' if self.calibration.invert_roll else '❌ ВЫКЛ'}")
        print(f"   Мертвая зона:   {self.calibration.dead_zone:.1f}°")
        print(f"   Чувствительность: {self.calibration.sensitivity:.2f}")
        print(f"   Макс. скорость: {self.calibration.max_velocity:.1f} м/с")
        print("="*60)
        
        return True
    
    def apply_calibration(self, raw_pitch: float, raw_roll: float) -> Tuple[float, float]:
        """Применение калибровки к сырым данным (только pitch и roll как во второй программе)"""
        if not self.calibration.calibrated:
            return raw_pitch, raw_roll
        
        # Нулевая точка
        calibrated_pitch = raw_pitch - self.calibration.zero_pitch
        calibrated_roll = raw_roll - self.calibration.zero_roll
        
        # Мертвая зона
        if abs(calibrated_pitch) < self.calibration.dead_zone:
            calibrated_pitch = 0.0
        if abs(calibrated_roll) < self.calibration.dead_zone:
            calibrated_roll = 0.0
        
        # Инверсия осей
        if self.calibration.invert_pitch:
            calibrated_pitch = -calibrated_pitch
        if self.calibration.invert_roll:
            calibrated_roll = -calibrated_roll
        
        return calibrated_pitch, calibrated_roll
    
    def get_calibration_summary(self) -> str:
        """Получение строки с информацией о калибровке"""
        if not self.calibration.calibrated:
            return "❌ Калибровка не выполнена"
        
        return (f"✅ Калибровано | Pitch: {self.calibration.zero_pitch:.1f}° | "
                f"Roll: {self.calibration.zero_roll:.1f}°")

# ============================================================================
# КЛАСС ЛОГГЕРА
# ============================================================================

class Logger:
    """Класс для логирования данных"""
    
    def __init__(self, log_dir: str = "logs"):
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)
        
        self.start_time = time.time()
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filename = f"angles_log_{timestamp}.txt"
        self.filepath = os.path.join(self.log_dir, self.filename)
        
        self.record_count = 0
        
        self._write_header()
        print(f"📝 Логирование начато: {self.filename}")
    
    def _write_header(self):
        """Запись заголовка в лог-файл"""
        header = (
            "Time(s)\tRaw_Yaw\tRaw_Pitch\tRaw_Roll\t"
            "Rel_Pitch\tRel_Roll\t"
            "Control_Pitch\tControl_Roll\t"
            "Velocity_X\tVelocity_Y\n"
        )
        with open(self.filepath, 'w', encoding='utf-8') as f:
            f.write(header)
    
    def log_angles(self, sensor_data: SensorData, control_data: ControlData,
                  rel_pitch: float, rel_roll: float):
        """Логирование данных (как во второй программе)"""
        self.record_count += 1
        
        data_line = (
            f"{sensor_data.timestamp:.3f}\t"
            f"{sensor_data.yaw:.2f}\t"
            f"{sensor_data.pitch:.2f}\t"
            f"{sensor_data.roll:.2f}\t"
            f"{rel_pitch:.2f}\t"
            f"{rel_roll:.2f}\t"
            f"{control_data.control_pitch:.2f}\t"
            f"{control_data.control_roll:.2f}\t"
            f"{control_data.velocity_x:.2f}\t"
            f"{control_data.velocity_y:.2f}"
        )
        
        with open(self.filepath, 'a', encoding='utf-8') as f:
            f.write(data_line + '\n')
    
    def get_log_path(self) -> str:
        """Получение пути к лог-файлу"""
        return self.filepath

# ============================================================================
# КЛАСС ДЛЯ РАБОТЫ С ESP32
# ============================================================================

class ESP32Controller:
    """Класс для управления ESP32"""
    
    def __init__(self, ip: str = Config.ESP32_IP, port: int = Config.PORT):
        self.ip = ip
        self.port = port
        self.state = ESP32State()
        self.target_angles = TargetAngles()
        self.sock = self._create_socket()
    
    def _create_socket(self) -> socket.socket:
        """Создание UDP сокета"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("", self.port))
        sock.settimeout(Config.SOCKET_TIMEOUT)
        return sock
    
    def send_command(self, command: str, silent: bool = False) -> bool:
        """Отправка команды на ESP32"""
        try:
            self.sock.sendto(command.encode('utf-8'), (self.ip, self.port))
            self.state.last_command = command
            self.state.last_command_time = time.time()
            
            self._update_state(command)
            
            if not silent:
                print(f"\n📤 Sent -> {command}")
            return True
            
        except Exception as e:
            if not silent:
                print(f"\n❌ Ошибка отправки команды '{command}': {e}")
            return False
    
    def send_target_angles(self, yaw: float, pitch: float, roll: float, 
                          throttle: int = Config.DEFAULT_THROTTLE) -> bool:
        """Отправка целевых углов на ESP32 (команда SET)"""
        # Форматирование как в Arduino коде: SET yaw pitch roll throttle
        command = f"SET {yaw:.1f} {pitch:.1f} {roll:.1f} {throttle}"
        
        # Сохраняем целевые углы
        self.target_angles.yaw = yaw
        self.target_angles.pitch = pitch
        self.target_angles.roll = roll
        self.target_angles.throttle = throttle
        self.target_angles.active = True
        
        return self.send_command(command)
    
    def reset_target_angles(self):
        """Сброс целевых углов"""
        self.target_angles.active = False
        self.target_angles.yaw = 0.0
        self.target_angles.pitch = 0.0
        self.target_angles.roll = 0.0
        self.target_angles.throttle = Config.DEFAULT_THROTTLE
    
    def _update_state(self, command: str):
        """Обновление состояния на основе команды"""
        if command == "ARM":
            self.state.motors_armed = True
            self.reset_target_angles()  # Сбрасываем при ARM
        elif command == "DISARM":
            self.state.motors_armed = False
            self.reset_target_angles()  # Сбрасываем при DISARM
        elif command == "START":
            self.state.stream_active = True
            self.reset_target_angles()  # Сбрасываем при START
        elif command == "STOP":
            self.state.stream_active = False
            self.reset_target_angles()  # Сбрасываем при STOP
        elif command.startswith("SET"):
            # При команде SET режим target angles становится активным
            self.target_angles.active = True
            parts = command.split()
            if len(parts) >= 5:
                try:
                    self.state.throttle_value = int(parts[4])
                    self.target_angles.throttle = self.state.throttle_value
                except ValueError:
                    pass
    
    def receive_data(self) -> Optional[Tuple[float, float, float]]:
        """Получение данных от ESP32"""
        try:
            data, addr = self.sock.recvfrom(1024)
            msg = data.decode().strip()
            
            if ',' in msg:
                parts = msg.split(',')
                if len(parts) == 3:
                    yaw = float(parts[0])
                    pitch = float(parts[1])
                    roll = float(parts[2])
                    
                    if all(math.isfinite(x) for x in [yaw, pitch, roll]):
                        return (yaw, pitch, roll)
                    
        except socket.timeout:
            pass
        except Exception as e:
            if not hasattr(self, '_error_shown'):
                print(f"⚠️ Ошибка получения данных: {e}")
                self._error_shown = True
        
        return None
    
    def close(self):
        """Закрытие соединения"""
        try:
            self.sock.close()
        except:
            pass

# ============================================================================
# КЛАСС ДЛЯ РАБОТЫ С AirSim
# ============================================================================

class AirSimController:
    """Класс для управления дроном в AirSim"""
    
    def __init__(self):
        self.client = None
        self.connected = False
        self.in_air = False
        self.camera_pitch = 0
        self.camera_yaw = 0
        self.last_camera_update = 0
        self.camera_update_interval = Config.CAMERA_UPDATE_INTERVAL
    
    def connect(self) -> bool:
        """Подключение к AirSim"""
        try:
            print("🚁 Подключение к AirSim...")
            self.client = airsim.MultirotorClient()
            self.client.confirmConnection()
            self.client.enableApiControl(True)
            self.client.armDisarm(False)
            self.connected = True
            print("✅ AirSim подключен")
            return True
            
        except Exception as e:
            print(f"❌ Ошибка подключения к AirSim: {e}")
            self.connected = False
            return False
    
    def get_drone_state(self) -> Optional[str]:
        """Получение состояния дрона"""
        if not self.connected:
            return None
        
        try:
            state = self.client.getMultirotorState()
            return '✈️ В воздухе' if state.landed_state == 0 else '🛬 На земле'
        except:
            return '❓ Неизвестно'
    
    def takeoff(self, target_altitude: float = Config.TARGET_ALTITUDE) -> bool:
        """Взлет дрона"""
        if not self.connected:
            print("❌ AirSim не подключен!")
            return False
        
        try:
            print("🛫 Взлетаем...")
            self.client.armDisarm(True)
            self.client.takeoffAsync().join()
            time.sleep(2)
            self.client.moveToZAsync(target_altitude, 2).join()
            self.in_air = True
            print(f"✅ Взлет выполнен! Высота: {-target_altitude} метров")
            return True
            
        except Exception as e:
            print(f"❌ Ошибка при взлете: {e}")
            return False
    
    def land(self) -> bool:
        """Посадка дрона"""
        if not self.connected:
            print("❌ AirSim не подключен!")
            return False
        
        try:
            print("🛬 Начинаю посадку...")
            self.client.landAsync().join()
            time.sleep(2)
            self.client.armDisarm(False)
            self.in_air = False
            print("✅ Посадка выполнена успешно!")
            return True
            
        except Exception as e:
            print(f"❌ Ошибка при посадке: {e}")
            return False
    
    def move_by_velocity(self, velocity_x: float, velocity_y: float, 
                        yaw: float = 0) -> bool:
        """Движение дрона с заданной скоростью (как во второй программе)"""
        if not self.connected or not self.in_air:
            return False
        
        try:
            self.client.moveByVelocityZAsync(
                velocity_x, 
                velocity_y, 
                Config.TARGET_ALTITUDE,
                0.1,
                airsim.DrivetrainType.MaxDegreeOfFreedom,
                airsim.YawMode(False, yaw)  # Абсолютный yaw как во второй программе
            )
            return True
        except Exception as e:
            print(f"⚠️ Ошибка движения: {e}")
            return False
    
    def set_camera_orientation(self, pitch: float, yaw: float) -> bool:
        """Установка ориентации камеры с защитой от слишком частых обновлений"""
        if not self.connected:
            return False
        
        current_time = time.time()
        # Защита от слишком частых обновлений камеры
        if current_time - self.last_camera_update < self.camera_update_interval:
            return True  # Пропускаем слишком частое обновление
        
        try:
            self.client.simSetCameraPose(
                "0",
                airsim.Pose(
                    airsim.Vector3r(0, 0, 0),
                    airsim.to_quaternion(
                        math.radians(pitch),
                        math.radians(0),
                        math.radians(yaw)
                    )
                )
            )
            self.last_camera_update = current_time
            return True
        except Exception as e:
            print(f"⚠️ Ошибка установки камеры: {e}")
            return False

# ============================================================================
# КЛАСС ДЛЯ ОБРАБОТКИ ДАННЫХ С ДАТЧИКОВ
# ============================================================================

class SensorProcessor:
    """Класс для обработки данных с датчиков (как во второй программе)"""
    
    def __init__(self, calibration: CalibrationData):
        self.calibration = calibration
        self.prev_yaw = 0.0
        self.prev_pitch = 0.0
        self.prev_roll = 0.0
    
    def process_sensor_data(self, raw_yaw: float, raw_pitch: float, 
                           raw_roll: float) -> SensorData:
        """Обработка сырых данных с датчиков (как во второй программе)"""
        # Простое сглаживание
        yaw = 0.8 * self.prev_yaw + 0.2 * raw_yaw
        pitch = 0.8 * self.prev_pitch + 0.2 * raw_pitch
        roll = 0.8 * self.prev_roll + 0.2 * raw_roll
        
        self.prev_yaw, self.prev_pitch, self.prev_roll = yaw, pitch, roll
        
        return SensorData(
            yaw=yaw,
            pitch=pitch,
            roll=roll,
            timestamp=time.time()
        )
    
    def calculate_control_data(self, rel_pitch: float, rel_roll: float, 
                              drone_yaw: float) -> ControlData:
        """
        Расчет контрольных данных для управления дроном
        ТОЧНО как во второй программе:
        - local_forward = -control_roll * max_velocity / MAX_ANGLE_LIMIT
        - local_right = control_pitch * max_velocity / MAX_ANGLE_LIMIT
        - global_x = local_forward * cos(yaw) - local_right * sin(yaw)
        - global_y = local_forward * sin(yaw) + local_right * cos(yaw)
        """
        # Применяем чувствительность
        control_pitch = rel_pitch * self.calibration.sensitivity
        control_roll = rel_roll * self.calibration.sensitivity
        
        # Ограничиваем углы управления
        control_pitch = max(-Config.MAX_ANGLE_LIMIT, 
                           min(Config.MAX_ANGLE_LIMIT, control_pitch))
        control_roll = max(-Config.MAX_ANGLE_LIMIT, 
                          min(Config.MAX_ANGLE_LIMIT, control_roll))
        
        # ТОЧНО как во второй программе:
        # Преобразование в локальные скорости
        local_forward = -control_roll * self.calibration.max_velocity / Config.MAX_ANGLE_LIMIT
        local_right = control_pitch * self.calibration.max_velocity / Config.MAX_ANGLE_LIMIT
        
        # Минимальная скорость для реакции
        if abs(local_forward) < Config.MIN_SPEED_THRESHOLD and \
           abs(local_right) < Config.MIN_SPEED_THRESHOLD:
            return ControlData(0, 0, control_pitch, control_roll)
        
        # Преобразование в глобальные скорости (с учетом Yaw дрона)
        yaw_rad = math.radians(drone_yaw)
        global_x = local_forward * math.cos(yaw_rad) - local_right * math.sin(yaw_rad)
        global_y = local_forward * math.sin(yaw_rad) + local_right * math.cos(yaw_rad)
        
        # Корректировка минимальной скорости
        speed = math.sqrt(global_x**2 + global_y**2)
        if 0 < speed < Config.MIN_SPEED_THRESHOLD:
            scale = Config.MIN_SPEED_THRESHOLD / speed
            global_x *= scale
            global_y *= scale
        
        return ControlData(global_x, global_y, control_pitch, control_roll)

# ============================================================================
# КЛАСС ДЛЯ УПРАВЛЕНИЯ МЕНЮ
# ============================================================================

class MenuManager:
    """Класс для управления меню и вводом с клавиатуры"""
    
    def __init__(self):
        self.current_mode = 1
        self.program_running = True
        self.telemetry_enabled = False
        self.print_telemetry_flag = False
        self.menu_shown = False
        self.camera_menu_shown = False
        self.show_menu_after_command = False
        self.telemetry_line_shown = False
        
        if not WINDOWS:
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
    
    def get_key(self) -> Optional[str]:
        """Считывание клавиши без ENTER"""
        try:
            if WINDOWS:
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
            else:
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    if key == '\x1b':
                        if select.select([sys.stdin], [], [], 0.1)[0]:
                            next1, next2 = sys.stdin.read(2)
                            if next1 == '[':
                                if next2 == 'A': return 'up'
                                if next2 == 'B': return 'down'
                                if next2 == 'D': return 'left'
                                if next2 == 'C': return 'right'
                    else:
                        return key.lower()
        except:
            pass
        return None
    
    def clear_telemetry_line(self):
        """Очищает строку телеметрии"""
        if self.telemetry_line_shown:
            print('\r' + ' ' * 120 + '\r', end='', flush=True)
            self.telemetry_line_shown = False
    
    def cleanup(self):
        """Очистка ресурсов"""
        if not WINDOWS:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

# ============================================================================
# ГЛАВНЫЙ КЛАСС ПРИЛОЖЕНИЯ
# ============================================================================

class DroneControlSystem:
    """Главный класс системы управления дроном"""
    
    def __init__(self):
        self.logger = Logger()
        self.esp32 = ESP32Controller()
        self.airsim = AirSimController()
        self.menu = MenuManager()
        
        self.calibrator = CalibrationManager(self.esp32, self.logger)
        
        # PID коэффициенты
        self.pid_coeffs = {
            'pitch': PIDCoeffs(Kp=4.0, Ki=0.02, Kd=0.5),
            'roll': PIDCoeffs(Kp=4.0, Ki=0.02, Kd=0.5),
            'yaw': PIDCoeffs(Kp=2.0, Ki=0.005, Kd=0.3)
        }
        
        # Обработка данных
        self.sensor_processor = SensorProcessor(self.calibrator.calibration)
        
        # Время
        self.start_time = time.time()
        self.last_command_time = time.time()
        
        # Флаги
        self.last_telemetry_update = 0
        self.telemetry_update_interval = 0.2
        
        # Данные (как во второй программе)
        self.last_sensor_data = None
        self.last_rel_pitch = 0.0
        self.last_rel_roll = 0.0
        self.last_velocity_x = 0.0
        self.last_velocity_y = 0.0
        self.last_control_pitch = 0.0
        self.last_control_roll = 0.0
        
        # Текущий Yaw дрона (из AirSim)
        self.drone_yaw = 0.0
        
        # Флаги режимов
        self.target_angles_mode = False  # Режим управления через SET
        
    def run(self):
        """Запуск основной программы"""
        print("\n" + "="*60)
        print("🚀 СИСТЕМА УПРАВЛЕНИЯ ДРОНОМ")
        print("="*60)
        print(f"📝 Логирование начато: {os.path.basename(self.logger.get_log_path())}")
        
        # Включаем стрим данных с ESP32
        print("\n⏳ Включаю стрим данных ESP32...")
        if self.esp32.send_command("START"):
            print("✅ Стрим данных включен")
        else:
            print("⚠️  Не удалось включить стрим данных")
        
        # Ждем немного для установления соединения
        time.sleep(1)
        
        # Обязательная калибровка при запуске
        print("\n🔴 ОБЯЗАТЕЛЬНАЯ КАЛИБРОВКА ПРИ ЗАПУСКЕ")
        print("="*60)
        
        if not self._run_calibration():
            print("\n❌ Калибровка не выполнена. Программа завершена.")
            return
        
        # Подключение к AirSim
        print("\n🔄 Подключение к AirSim...")
        self.airsim.connect()
        
        # Запуск потока меню
        menu_thread = threading.Thread(target=self.menu_thread_func, daemon=True)
        menu_thread.start()
        
        # Основной цикл
        self._main_loop()
    
    def _run_calibration(self) -> bool:
        """Выполнение калибровки (как во второй программе)"""
        print("\n📋 ШАГ 1: ПОДГОТОВКА К КАЛИБРОВКЕ")
        print("-" * 40)
        input("⏎ Нажмите Enter, когда будете готовы начать подготовку...")
        
        if not self.calibrator.prepare_calibration():
            return False
        
        print("\n📋 ШАГ 2: ВЫПОЛНЕНИЕ КАЛИБРОВКИ")
        print("-" * 40)
        input("\n⏎ Нажмите Enter, когда будете готовы начать калибровку...")
        
        if not self.calibrator.perform_calibration():
            print("\n❌ КАЛИБРОВКА НЕ УДАЛАСЬ!")
            return False
        
        print("\n✅ КАЛИБРОВКА УСПЕШНО ЗАВЕРШЕНА!")
        print("⚠️  Рекомендуется выполнить ручную настройку чувствительности (команда [c])")
        return True
    
    def menu_thread_func(self):
        """Поток обработки меню"""
        while self.menu.program_running:
            key = self.menu.get_key()
            
            if key:
                self.menu.clear_telemetry_line()
                
                if self.menu.current_mode == 1:
                    self._handle_main_menu_key(key)
                elif self.menu.current_mode == 2:
                    self._handle_camera_menu_key(key)
            
            time.sleep(0.1)
    
    def _handle_main_menu_key(self, key: str):
        """Обработка клавиш главного меню"""
        handlers = {
            '1': self._handle_telemetry_enable,
            '2': self._handle_camera_mode,
            '3': self._handle_target_angles_mode,
            'c': self._handle_calibration,
            'a': self._handle_arm,
            'z': self._handle_disarm,
            's': self._handle_start_stream,
            'x': self._handle_stop_stream,
            't': self._handle_takeoff,
            'l': self._handle_land,
            ' ': self._handle_telemetry_disable,
            'r': self._handle_reconnect,
            'q': self._handle_quit
        }
        
        handler = handlers.get(key)
        if handler:
            handler()
        else:
            print(f"\n❌ Неизвестная команда: {key}")
            self.menu.show_menu_after_command = True
    
    def _handle_camera_menu_key(self, key: str):
        """Обработка клавиш меню камеры с защитой от частых нажатий"""
        step = 5
        changed = False
        
        if key == 'up':
            self.airsim.camera_pitch += step
            changed = True
        elif key == 'down':
            self.airsim.camera_pitch -= step
            changed = True
        elif key == 'left':
            self.airsim.camera_yaw -= step
            changed = True
        elif key == 'right':
            self.airsim.camera_yaw += step
            changed = True
        elif key == 'b':
            self.menu.current_mode = 1
            self.menu.menu_shown = False
            self.menu.camera_menu_shown = False
            self.menu.print_telemetry_flag = self.menu.telemetry_enabled
            print("\n↩️ Возврат в главное меню")
            return
        
        if changed:
            # Ограничиваем значения
            self.airsim.camera_pitch = max(-90, min(90, self.airsim.camera_pitch))
            self.airsim.camera_yaw = max(-180, min(180, self.airsim.camera_yaw))
            
            # Обновляем камеру с защитой от ошибок
            try:
                if self.airsim.set_camera_orientation(self.airsim.camera_pitch, 
                                                     self.airsim.camera_yaw):
                    print(f"\r  📷 Камера: Pitch={self.airsim.camera_pitch:3}°, "
                         f"Yaw={self.airsim.camera_yaw:3}°", end='', flush=True)
            except Exception as e:
                print(f"\r⚠️ Ошибка камеры: {e}")
    
    def _handle_target_angles_mode(self):
        """Меню управления дроном через целевые углы (SET команда)"""
        if not self.esp32.state.stream_active:
            print("\n⚠️  Стрим данных не активен!")
            print("   Включите стрим командой [s]")
            self.menu.show_menu_after_command = True
            return
        
        if not self.esp32.state.motors_armed:
            print("\n⚠️  Моторы не взведены!")
            print("   Взведите моторы командой [a]")
            self.menu.show_menu_after_command = True
            return
        
        self.target_angles_menu()
    
    def target_angles_menu(self):
        """Меню управления дроном через целевые углы (как во второй программе)"""
        self.menu.print_telemetry_flag = False
        self.menu.clear_telemetry_line()
        
        print("\n" + "="*60)
        print("🎮 УПРАВЛЕНИЕ ДРОНОМ (Target Angles)")
        print("="*60)
        print("  ESP32 будет пытаться удерживать заданные углы")
        print("  Формат команды: SET <yaw> <pitch> <roll> <throttle>")
        print("-" * 60)
        
        try:
            # Ввод значений как во втором коде
            yaw = self._get_input_float("Введите Yaw (рыскание) в градусах [0]: ", 0.0)
            pitch = self._get_input_float("Введите Pitch (тангаж) в градусах [0]: ", 0.0)
            roll = self._get_input_float("Введите Roll (крен) в градусах [0]: ", 0.0)
            
            throttle = self._get_input_int(
                f"Введите Throttle ({Config.MIN_THROTTLE}-{Config.MAX_THROTTLE}) "
                f"[{self.esp32.state.throttle_value}]: ",
                self.esp32.state.throttle_value,
                Config.MIN_THROTTLE,
                Config.MAX_THROTTLE
            )
            
            # Проверка диапазонов как во втором коде
            if abs(pitch) > 30:
                print(f"⚠️ Pitch ({pitch}°) выходит за рекомендуемый диапазон ±30°")
            if abs(roll) > 30:
                print(f"⚠️ Roll ({roll}°) выходит за рекомендуемый диапазон ±30°")
            
            print(f"\n📢 Будет отправлена команда: SET {yaw:.1f} {pitch:.1f} {roll:.1f} {throttle}")
            print(f"   ESP32 PID-контроллер будет стабилизировать дрон по этим углам")
            
            confirm = input("Подтвердить отправку? (y/n): ").strip().lower()
            if confirm == 'y':
                if self.esp32.send_target_angles(yaw, pitch, roll, throttle):
                    print(f"✅ Команда отправлена на ESP32:")
                    print(f"   Yaw={yaw:.1f}°, Pitch={pitch:.1f}°, Roll={roll:.1f}°, Throttle={throttle} мкс")
                    self.target_angles_mode = True
                else:
                    print("❌ Не удалось отправить команду")
            else:
                print("❌ Отправка отменена")
            
        except ValueError as e:
            print(f"❌ Ошибка ввода: {e}")
        except Exception as e:
            print(f"❌ Ошибка: {e}")
        
        self.menu.show_menu_after_command = True
    
    def _get_input_float(self, prompt: str, default: float) -> float:
        """Получение float ввода с обработкой по умолчанию"""
        value = input(prompt).strip()
        return float(value) if value else default
    
    def _get_input_int(self, prompt: str, default: int, min_val: int, max_val: int) -> int:
        """Получение int ввода с проверкой диапазона"""
        while True:
            value = input(prompt).strip()
            if not value:
                return default
            
            try:
                int_val = int(value)
                if min_val <= int_val <= max_val:
                    return int_val
                else:
                    print(f"❌ Значение должно быть от {min_val} до {max_val}")
            except ValueError:
                print("❌ Введите целое число")
    
    def _handle_telemetry_enable(self):
        if not self.esp32.state.stream_active:
            print("\n⚠️  Стрим данных не активен!")
            print("   Включите стрим командой [s]")
            self.menu.show_menu_after_command = True
            return
        
        if not self.calibrator.calibration.calibrated:
            print("\n⚠️  Калибровка не выполнена!")
            print("   Выполните калибровку командой [c]")
            self.menu.show_menu_after_command = True
            return
        
        # При включении телеметрии сбрасываем режим target angles
        self.target_angles_mode = False
        self.esp32.reset_target_angles()
        
        self.menu.print_telemetry_flag = True
        self.menu.telemetry_enabled = True
        print("\n📊 Телеметрия ВКЛЮЧЕНА (нажмите пробел для отключения)")
        self.menu.menu_shown = False
        self.menu.show_menu_after_command = False
    
    def _handle_camera_mode(self):
        self.menu.current_mode = 2
        self.menu.menu_shown = False
        self.menu.camera_menu_shown = False
        self.menu.clear_telemetry_line()
        print("\n🎥 РЕЖИМ НАСТРОЙКИ КАМЕРЫ")
        print("  ↑/↓ — наклон камеры (вверх/вниз)")
        print("  ←/→ — поворот камеры (влево/вправо)")
        print("  [b] — возврат в меню")
        print(f"\r  📷 Камера: Pitch={self.airsim.camera_pitch:3}°, "
             f"Yaw={self.airsim.camera_yaw:3}°", end='', flush=True)
    
    def _handle_calibration(self):
        if not self.esp32.state.stream_active:
            print("\n⚠️  Стрим данных не активен!")
            print("   Включите стрим командой [s]")
            self.menu.show_menu_after_command = True
            return
        
        if self.calibrator.manual_calibration_menu():
            print("\n✅ Ручная настройка успешно завершена!")
        else:
            print("\n❌ Настройка не удалась")
        self.menu.show_menu_after_command = True
    
    def _handle_arm(self):
        if not self.esp32.state.stream_active:
            print("\n⚠️  Стрим данных не активен!")
            print("   Включите стрим командой [s]")
            self.menu.show_menu_after_command = True
            return
        
        print("\n⚠️  АРМИРОВАНИЕ МОТОРОВ ESP32")
        confirm = input("Продолжить? (y/n): ").strip().lower()
        if confirm == 'y':
            if self.esp32.send_command("ARM"):
                print("✅ Команда ARM отправлена на ESP32")
                # При ARM сбрасываем режим target angles
                self.target_angles_mode = False
                self.esp32.reset_target_angles()
        else:
            print("❌ ARM отменен")
        self.menu.show_menu_after_command = True
    
    def _handle_disarm(self):
        print("\n⚠️  DISARM МОТОРОВ ESP32")
        confirm = input("Продолжить? (y/n): ").strip().lower()
        if confirm == 'y':
            if self.esp32.send_command("DISARM"):
                print("✅ Команда DISARM отправлена на ESP32")
                # При DISARM сбрасываем режим target angles
                self.target_angles_mode = False
                self.esp32.reset_target_angles()
        else:
            print("❌ DISARM отменен")
        self.menu.show_menu_after_command = True
    
    def _handle_start_stream(self):
        if self.esp32.send_command("START"):
            print("✅ Стрим данных START отправлен")
            # При START сбрасываем режим target angles
            self.target_angles_mode = False
            self.esp32.reset_target_angles()
        self.menu.show_menu_after_command = True
    
    def _handle_stop_stream(self):
        if self.esp32.send_command("STOP"):
            print("✅ Стрим данных STOP отправлен")
            # При STOP сбрасываем режим target angles
            self.target_angles_mode = False
            self.esp32.reset_target_angles()
        self.menu.show_menu_after_command = True
    
    def _handle_takeoff(self):
        if not self.airsim.connected:
            print("❌ AirSim не подключен! Используйте [r]")
            self.menu.show_menu_after_command = True
            return
        
        print("\n🛫 ВЗЛЕТ ДРОНА В AirSim")
        confirm = input("Выполнить взлет? (y/n): ").strip().lower()
        if confirm == 'y':
            if self.airsim.takeoff():
                print("✅ Взлет выполнен успешно!")
            else:
                print("❌ Ошибка при взлете")
        else:
            print("❌ Взлет отменен")
        self.menu.show_menu_after_command = True
    
    def _handle_land(self):
        if not self.airsim.connected:
            print("❌ AirSim не подключен!")
            self.menu.show_menu_after_command = True
            return
        
        if not self.airsim.in_air:
            print("❌ Дрон уже на земле!")
            self.menu.show_menu_after_command = True
            return
        
        print("\n🛬 Начинаю посадку...")
        if self.airsim.land():
            print("✅ Посадка выполнена успешно!")
        else:
            print("❌ Ошибка при посадке")
        self.menu.show_menu_after_command = True
    
    def _handle_telemetry_disable(self):
        if self.menu.telemetry_enabled:
            self.menu.telemetry_enabled = False
            self.menu.print_telemetry_flag = False
            self.menu.clear_telemetry_line()
            print("\n📊 Телеметрия ВЫКЛЮЧЕНА")
            self.menu.menu_shown = False
        self.menu.show_menu_after_command = True
    
    def _handle_reconnect(self):
        print("\n🔄 Переподключение к AirSim...")
        if self.airsim.connect():
            print("✅ AirSim переподключен")
        else:
            print("❌ Не удалось переподключиться к AirSim")
        self.menu.show_menu_after_command = True
    
    def _handle_quit(self):
        print("\n🛬 Запуск посадки и выход...")
        self.menu.program_running = False
    
    def _show_main_menu(self):
        """Показывает главное меню"""
        self.menu.clear_telemetry_line()
        
        print("\n" + "="*70)
        print("🔧 ГЛАВНОЕ МЕНЮ УПРАВЛЕНИЯ:")
        print("="*70)
        
        menu_items = [
            "[1] Включить телеметрию",
            "[2] Настройка камеры",
            "[3] Управление дроном (Target Angles)",
            "     Отправляет SET команды на ESP32",
            "[c] Настройка чувствительности",
            "-" * 70,
            "[a] ARM (включить моторы ESP32)",
            "[z] DISARM (выключить моторы ESP32)",
            "-" * 70,
            "[s] START стрим",
            "[x] STOP стрим",
            "-" * 70,
            "[t] Takeoff (AirSim)",
            "[l] Land (AirSim)",
            "-" * 70,
            "[ ] Пробел - выключить телеметрию",
            "-" * 70,
            "[r] Переподключиться к AirSim",
            "[q] Посадка и выход"
        ]
        
        for item in menu_items:
            print(f"   {item}")
        
        print("="*70)
        
        calib_status = self.calibrator.get_calibration_summary()
        print(f"📊 {calib_status}")
        
        if self.esp32.state.stream_active:
            print("✅ ESP32 подключен. Система готова к работе.")
            if self.esp32.state.motors_armed:
                print("✅ Моторы взведены")
            else:
                print("❌ Моторы не взведены (используйте [a])")
        else:
            print("❌ ESP32 стрим не активен. Включите стрим командой [s]")
        
        if self.target_angles_mode:
            print("🎯 РЕЖИМ Target Angles АКТИВЕН")
            print(f"   Текущие углы: Yaw={self.esp32.target_angles.yaw:.1f}°, "
                  f"Pitch={self.esp32.target_angles.pitch:.1f}°, "
                  f"Roll={self.esp32.target_angles.roll:.1f}°")
        
        print("="*70)
        print("👉 Нажмите клавишу (цифра/буква)")
        
        self.menu.menu_shown = True
        self.menu.print_telemetry_flag = False
    
    def _main_loop(self):
        """Основной цикл обработки данных"""
        try:
            while self.menu.program_running:
                current_time = time.time()
                
                if self.menu.show_menu_after_command:
                    self.menu.menu_shown = False
                    self.menu.show_menu_after_command = False
                
                if self.menu.current_mode == 1 and not self.menu.menu_shown:
                    self._show_main_menu()
                
                # Получение данных от ESP32
                data = self.esp32.receive_data()
                
                # Если режим target angles активен, не обрабатываем данные для управления
                if data and not self.target_angles_mode:
                    self._process_sensor_data(data, current_time)
                
                # Обновление телеметрии
                if (self.menu.print_telemetry_flag and self.menu.telemetry_enabled and 
                    self.menu.current_mode == 1 and 
                    current_time - self.last_telemetry_update > self.telemetry_update_interval):
                    
                    if self.last_sensor_data:
                        self._print_telemetry_line()
                        self.last_telemetry_update = current_time
                
                # Keep-alive для AirSim
                if (current_time - self.last_command_time > Config.KEEP_ALIVE_INTERVAL and 
                    self.airsim.connected and self.airsim.in_air and 
                    not self.target_angles_mode):  # Не отправляем в режиме target angles
                    
                    # Периодически отправляем нулевую команду для поддержания высоты
                    self.airsim.move_by_velocity(0, 0, 0)
                    self.last_command_time = current_time
                
                time.sleep(Config.MAIN_LOOP_SLEEP)
                
        except KeyboardInterrupt:
            print("\n🛑 Остановка по Ctrl+C...")
        except Exception as e:
            print(f"\n❌ Критическая ошибка: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.shutdown()
    
    def _process_sensor_data(self, raw_data: Tuple[float, float, float], current_time: float):
        """Обработка данных с датчиков (ТОЧНО как во второй программе)"""
        raw_yaw, raw_pitch, raw_roll = raw_data
        
        if any(math.isnan(x) for x in [raw_yaw, raw_pitch, raw_roll]):
            return
        
        # Обработка данных
        sensor_data = self.sensor_processor.process_sensor_data(raw_yaw, raw_pitch, raw_roll)
        
        # Применение калибровки (только pitch и roll)
        rel_pitch, rel_roll = self.calibrator.apply_calibration(
            sensor_data.pitch, sensor_data.roll
        )
        
        # Получаем текущий Yaw дрона из AirSim
        if self.airsim.connected and self.airsim.in_air:
            try:
                kinematics = self.airsim.client.getMultirotorState().kinematics_estimated
                current_yaw = airsim.to_eularian_angles(kinematics.orientation)[2]
                self.drone_yaw = math.degrees(current_yaw)
            except:
                pass
        
        # Расчет управляющих данных для AirSim (ТОЧНО как во второй программе)
        control_data = self.sensor_processor.calculate_control_data(
            rel_pitch, rel_roll, self.drone_yaw
        )
        
        # Сохраняем последние данные для телеметрии
        self.last_sensor_data = sensor_data
        self.last_rel_pitch = rel_pitch
        self.last_rel_roll = rel_roll
        self.last_velocity_x = control_data.velocity_x
        self.last_velocity_y = control_data.velocity_y
        self.last_control_pitch = control_data.control_pitch
        self.last_control_roll = control_data.control_roll
        
        # Логирование (как во второй программе)
        self.logger.log_angles(sensor_data, control_data, rel_pitch, rel_roll)
        
        # Управление дроном в AirSim (ТОЧНО как во второй программе)
        if self.airsim.connected and self.airsim.in_air:
            try:
                self.airsim.move_by_velocity(
                    control_data.velocity_x, 
                    control_data.velocity_y,
                    sensor_data.yaw  # Абсолютный yaw как во второй программе
                )
                self.last_command_time = current_time
            except Exception as e:
                print(f"\r⚠️ Ошибка движения дрона: {e}", end='', flush=True)
    
    def _print_telemetry_line(self):
        """Вывод одной строки телеметрии (как во второй программе)"""
        if not self.last_sensor_data:
            return
        
        telemetry_str = (
            f"📊 Raw: P={self.last_sensor_data.pitch:6.1f}° "
            f"R={self.last_sensor_data.roll:6.1f}° "
            f"Y={self.last_sensor_data.yaw:6.1f}° | "
            f"Rel: P={self.last_rel_pitch:6.1f}° "
            f"R={self.last_rel_roll:6.1f}° | "
            f"V: X={self.last_velocity_x:5.2f} "
            f"Y={self.last_velocity_y:5.2f} m/s | "
            f"Drone Yaw={self.drone_yaw:6.1f}°"
        )
        
        if self.target_angles_mode:
            telemetry_str += " | 🎯 Target Angles MODE"
        
        if self.menu.telemetry_line_shown:
            print('\r' + ' ' * len(telemetry_str) + '\r', end='', flush=True)
        
        print(f"\r{telemetry_str}", end='', flush=True)
        self.menu.telemetry_line_shown = True
    
    def shutdown(self):
        """Безопасное завершение работы"""
        print("\n🛬 Безопасное завершение...")
        
        if self.airsim.connected and self.airsim.in_air:
            self.airsim.land()
        
        try:
            print("🔌 Отключение ESP32...")
            self.esp32.send_command("STOP", silent=True)
            time.sleep(0.1)
            self.esp32.send_command("DISARM", silent=True)
        except:
            pass
        
        self.esp32.close()
        self.menu.cleanup()
        
        print(f"\n💾 Лог сохранен в: {self.logger.get_log_path()}")
        print("✅ Программа завершена")

# ============================================================================
# ЗАПУСК ПРОГРАММЫ
# ============================================================================

if __name__ == "__main__":
    app = DroneControlSystem()
    app.run()