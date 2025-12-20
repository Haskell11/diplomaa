import socket
import os
import time
import math
import threading
import sys
from datetime import datetime
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, Any
import airsim

# Для Windows используем msvcrt, для Linux - select
try:
    import msvcrt
    WINDOWS = True
except ImportError:
    import select
    import termios
    import tty
    WINDOWS = False

# Импорты для YOLOv8
import cv2
import numpy as np
import torch
from ultralytics import YOLO
from collections import defaultdict
from threading import Thread, Lock

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
    MAX_ANGLE_LIMIT = 60.0
    MIN_SPEED_THRESHOLD = 0.5
    DEAD_ZONE_DEFAULT = 8.0
    SENSITIVITY_DEFAULT = 1.5
    SOCKET_TIMEOUT = 0.1
    MAIN_LOOP_SLEEP = 0.05
    KEEP_ALIVE_INTERVAL = 0.5
    
    # YOLOv8 настройки
    YOLO_MODEL_PATH = r"C:\Users\mkravtsov\Desktop\diplomaa\diplomaa\Best.pt"
    YOLO_CONFIDENCE = 0.5
    YOLO_FRAME_WIDTH = 640
    YOLO_FRAME_HEIGHT = 360

# ============================================================================
# КЛАСС ДЛЯ YOLOv8 ДЕТЕКЦИИ
# ============================================================================

class YOLOv8Detector:
    """Класс для детекции объектов с помощью YOLOv8"""
    
    def __init__(self, model_path: str = Config.YOLO_MODEL_PATH, 
                 conf_threshold: float = Config.YOLO_CONFIDENCE):
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.detection_active = False
        self.current_frame = None
        self.detection_results = []
        self.lock = Lock()
        self.detection_thread = None
        self.camera_client = None
        
        # Проверка устройства
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print(f'🔧 YOLOv8 использует устройство: {self.device}')
        self.model.to(self.device)
        
        # Цвета для аннотаций
        self.colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255),
            (255, 0, 255), (192, 192, 192), (128, 128, 128), (128, 0, 0), (128, 128, 0),
            (0, 128, 0), (128, 0, 128), (0, 128, 128), (0, 0, 128), (72, 61, 139),
            (47, 79, 79), (47, 79, 47), (0, 206, 209), (148, 0, 211), (255, 20, 147)
        ]
        
    def connect_to_airsim(self, airsim_client):
        """Подключение к камере AirSim"""
        self.camera_client = airsim_client
        
    def start_detection(self):
        """Запуск детекции в отдельном потоке"""
        if self.detection_active:
            return
            
        self.detection_active = True
        self.detection_thread = Thread(target=self._detection_loop, daemon=True)
        self.detection_thread.start()
        print("🚀 YOLOv8 детекция запущена")
        
    def stop_detection(self):
        """Остановка детекции"""
        self.detection_active = False
        if self.detection_thread:
            self.detection_thread.join(timeout=2)
        print("🛑 YOLOv8 детекция остановлена")
        
    def _detection_loop(self):
        """Основной цикл детекции"""
        while self.detection_active:
            try:
                # Получение кадра из AirSim
                if self.camera_client:
                    responses = self.camera_client.simGetImages([
                        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
                    ])
                    
                    if responses and responses[0]:
                        # Конвертация изображения
                        img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
                        img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)
                        
                        # Изменение размера для ускорения обработки
                        frame_resized = cv2.resize(img_rgb, 
                                                  (Config.YOLO_FRAME_WIDTH, 
                                                   Config.YOLO_FRAME_HEIGHT))
                        
                        # Детекция
                        results = self.model(frame_resized, 
                                           conf=self.conf_threshold, 
                                           device=self.device,
                                           verbose=False)[0]
                        
                        with self.lock:
                            self.current_frame = img_rgb  # Сохраняем оригинальный кадр
                            self.detection_results = results
                            
                time.sleep(0.033)  # ~30 FPS
                
            except Exception as e:
                print(f"❌ Ошибка детекции: {e}")
                time.sleep(1)
                
    def get_detection_display(self):
        """Получение кадра с bounding boxes"""
        with self.lock:
            if self.current_frame is None or self.detection_results is None:
                return None
                
            # Создание копии кадра для отрисовки
            display_frame = self.current_frame.copy()
            
            # Отрисовка bounding boxes
            if hasattr(self.detection_results, 'boxes'):
                boxes = self.detection_results.boxes
                for box in boxes:
                    # Координаты (масштабированные)
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = box.conf[0].cpu().numpy()
                    cls = int(box.cls[0].cpu().numpy())
                    
                    # Масштабирование координат до оригинального размера
                    original_height, original_width = self.current_frame.shape[:2]
                    x1 = int(x1 * original_width / Config.YOLO_FRAME_WIDTH)
                    y1 = int(y1 * original_height / Config.YOLO_FRAME_HEIGHT)
                    x2 = int(x2 * original_width / Config.YOLO_FRAME_WIDTH)
                    y2 = int(y2 * original_height / Config.YOLO_FRAME_HEIGHT)
                    
                    # Отрисовка прямоугольника
                    color = self.colors[cls % len(self.colors)]
                    cv2.rectangle(display_frame, 
                                (int(x1), int(y1)), 
                                (int(x2), int(y2)), 
                                color, 2)
                    
                    # Текст с классом и уверенностью
                    class_name = self.detection_results.names[cls]
                    label = f'{class_name} ({conf:.2%})'
                    cv2.putText(display_frame, label,
                              (int(x1), int(y1) - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            return display_frame
            
    def get_detected_objects_count(self):
        """Получение количества обнаруженных объектов"""
        with self.lock:
            if self.detection_results and hasattr(self.detection_results, 'boxes'):
                return len(self.detection_results.boxes)
        return 0

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
    """Данные калибровки"""
    zero_pitch: float = 0.0
    zero_roll: float = 0.0
    max_velocity: float = 8.0
    dead_zone: float = Config.DEAD_ZONE_DEFAULT
    sensitivity: float = Config.SENSITIVITY_DEFAULT
    invert_pitch: bool = False
    invert_roll: bool = True

@dataclass
class SensorData:
    """Данные сенсоров"""
    yaw: float
    pitch: float
    roll: float
    timestamp: float

@dataclass
class ControlData:
    """Контрольные данные"""
    velocity_x: float
    velocity_y: float
    control_pitch: float
    control_roll: float

# ============================================================================
# КЛАСС ЛОГГЕРА 
# ============================================================================

class Logger:
    """Класс для логирования данных с корректным временем"""
    
    def __init__(self, log_dir: str = Config.LOG_DIR):
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Запоминаем время старта программы
        self.program_start_time = time.time()
        self.start_datetime = datetime.now()
        
        timestamp = self.start_datetime.strftime('%Y%m%d_%H%M%S')
        self.filename = f"angles_log_{timestamp}.txt"
        self.filepath = os.path.join(self.log_dir, self.filename)
        
        self._write_header()
    
    def _write_header(self):
        """Запись заголовка в лог-файл"""
        header = ("#" * 80 + "\n"
                 f"# Flight Log Session Started: {self.start_datetime.strftime('%Y-%m-%d %H:%M:%S')}\n"
                 f"# Log File: {self.filename}\n"
                 "#" * 80 + "\n\n"
                 "Absolute_Time\tTime_From_Start(s)\tRaw_Yaw\tRaw_Pitch\tRaw_Roll\t"
                 "Rel_Pitch\tRel_Roll\tControl_Pitch\tControl_Roll\tVelocity_X\tVelocity_Y\n")
        
        with open(self.filepath, 'w', encoding='utf-8') as f:
            f.write(header)
    
    def log_angles(self, sensor_data: SensorData, control_data: ControlData,
                  rel_pitch: float, rel_roll: float):
        """Логирование углов и контрольных данных с корректным временем"""
        current_time = time.time()
        elapsed_time = current_time - self.program_start_time
        
        # Форматируем абсолютное время
        abs_time = datetime.fromtimestamp(current_time).strftime('%H:%M:%S.%f')[:-3]
        
        data_line = (f"{abs_time}\t"
                    f"{elapsed_time:.3f}\t"
                    f"{sensor_data.yaw:.2f}\t"
                    f"{sensor_data.pitch:.2f}\t"
                    f"{sensor_data.roll:.2f}\t"
                    f"{rel_pitch:.2f}\t"
                    f"{rel_roll:.2f}\t"
                    f"{control_data.control_pitch:.2f}\t"
                    f"{control_data.control_roll:.2f}\t"
                    f"{control_data.velocity_x:.2f}\t"
                    f"{control_data.velocity_y:.2f}")
        
        with open(self.filepath, 'a', encoding='utf-8') as f:
            f.write(data_line + '\n')
    
    def log_calibration(self, calibration: CalibrationData):
        """Логирование калибровочных данных"""
        current_time = datetime.now().strftime('%H:%M:%S')
        elapsed_time = time.time() - self.program_start_time
        
        log_line = (f"# [{current_time}] [{elapsed_time:.2f}s] CALIBRATION_DATA:\n"
                   f"#   zero_pitch={calibration.zero_pitch:.2f}\n"
                   f"#   zero_roll={calibration.zero_roll:.2f}\n"
                   f"#   sensitivity={calibration.sensitivity:.2f}\n"
                   f"#   dead_zone={calibration.dead_zone:.2f}\n")
        
        with open(self.filepath, 'a', encoding='utf-8') as f:
            f.write(log_line)
    
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
    
    def _update_state(self, command: str):
        """Обновление состояния на основе команды"""
        if command == "ARM":
            self.state.motors_armed = True
        elif command == "DISARM":
            self.state.motors_armed = False
        elif command == "START":
            self.state.stream_active = True
        elif command == "STOP":
            self.state.stream_active = False
        elif command.startswith("SET"):
            parts = command.split()
            if len(parts) >= 5:
                try:
                    self.state.throttle_value = int(parts[4])
                except ValueError:
                    pass
    
    def set_target_angles(self, yaw: float = 0, pitch: float = 0, 
                         roll: float = 0, throttle: int = Config.DEFAULT_THROTTLE) -> bool:
        """Установка целевых углов и газа"""
        if throttle < Config.MIN_THROTTLE or throttle > Config.MAX_THROTTLE:
            print(f"❌ Throttle должен быть между {Config.MIN_THROTTLE} и {Config.MAX_THROTTLE}")
            return False
        
        command = f"SET {yaw:.1f} {pitch:.1f} {roll:.1f} {throttle}"
        return self.send_command(command)
    
    def receive_data(self) -> Optional[Tuple[float, float, float]]:
        """Получение данных от ESP32"""
        try:
            data, addr = self.sock.recvfrom(1024)
            msg = data.decode().strip()
            
            if ',' in msg:
                parts = msg.split(',')
                if len(parts) == 3:
                    return tuple(float(part) for part in parts)
                    
        except socket.timeout:
            pass
        except Exception as e:
            print(f"⚠️ Ошибка получения данных: {e}")
        
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
        """Движение дрона с заданной скоростью"""
        if not self.connected or not self.in_air:
            return False
        
        try:
            self.client.moveByVelocityZAsync(
                velocity_x, 
                velocity_y, 
                Config.TARGET_ALTITUDE,
                0.1,
                airsim.DrivetrainType.MaxDegreeOfFreedom,
                airsim.YawMode(False, yaw)
            )
            return True
        except:
            return False
    
    def set_camera_orientation(self, pitch: float, yaw: float) -> bool:
        """Установка ориентации камеры"""
        if not self.connected:
            return False
        
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
            return True
        except:
            return False

# ============================================================================
# КЛАСС ДЛЯ ОБРАБОТКИ ДАННЫХ С ДАТЧИКОВ
# ============================================================================

class SensorProcessor:
    """Класс для обработки данных с датчиков"""
    
    def __init__(self, calibration: CalibrationData):
        self.calibration = calibration
        self.prev_yaw = 0.0
        self.prev_pitch = 0.0
        self.prev_roll = 0.0
    
    def process_sensor_data(self, raw_yaw: float, raw_pitch: float, 
                           raw_roll: float) -> SensorData:
        """Обработка сырых данных с датчиков"""
        # Сглаживание данных
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
    
    def calculate_relative_angles(self, sensor_data: SensorData) -> Tuple[float, float]:
        """Расчет относительных углов с учетом калибровки"""
        rel_pitch = sensor_data.pitch - self.calibration.zero_pitch
        rel_roll = sensor_data.roll - self.calibration.zero_roll
        
        # Применение мертвой зоны
        if abs(rel_pitch) < self.calibration.dead_zone:
            rel_pitch = 0
        if abs(rel_roll) < self.calibration.dead_zone:
            rel_roll = 0
        
        return rel_pitch, rel_roll
    
    def calculate_control_data(self, rel_pitch: float, rel_roll: float, 
                              drone_yaw: float) -> ControlData:
        """Расчет контрольных данных для управления дроном"""
        # Инвертируем оси при необходимости
        if self.calibration.invert_pitch:
            rel_pitch = -rel_pitch
        if self.calibration.invert_roll:
            rel_roll = -rel_roll
        
        # Применяем чувствительность
        control_pitch = rel_pitch * self.calibration.sensitivity
        control_roll = rel_roll * self.calibration.sensitivity
        
        # Ограничиваем углы управления
        control_pitch = max(-Config.MAX_ANGLE_LIMIT, 
                           min(Config.MAX_ANGLE_LIMIT, control_pitch))
        control_roll = max(-Config.MAX_ANGLE_LIMIT, 
                          min(Config.MAX_ANGLE_LIMIT, control_roll))
        
        # Преобразование в локальные скорости
        local_forward = control_pitch * self.calibration.max_velocity / Config.MAX_ANGLE_LIMIT
        local_right = control_roll * self.calibration.max_velocity / Config.MAX_ANGLE_LIMIT
        
        # Минимальная скорость для реакции
        if abs(local_forward) < Config.MIN_SPEED_THRESHOLD and \
           abs(local_right) < Config.MIN_SPEED_THRESHOLD:
            return ControlData(0, 0, control_pitch, control_roll)
        
        # Преобразование в глобальные скорости
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
# КЛАСС ДЛЯ КАЛИБРОВКИ
# ============================================================================

class CalibrationManager:
    """Класс для управления калибровкой"""
    
    def __init__(self, esp32_controller: ESP32Controller):
        self.esp32 = esp32_controller
        self.calibration = CalibrationData()
        self.calibration_done = False
    
    def prepare_calibration(self) -> bool:
        """Подготовка к калибровке"""
        print("\n" + "="*60)
        print("🔧 ПОДГОТОВКА К КАЛИБРОВКЕ")
        print("="*60)
        
        print("🔍 Проверка подключения к ESP32...")
        
        # Попробуем получить данные от ESP32
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
        """Выполнение калибровки"""
        print("\n" + "="*60)
        print("🎯 НАЧАЛО КАЛИБРОВКИ")
        print("="*60)
        
        # Обратный отсчет
        print("\n⏳ Начинаю через:")
        for i in range(5, 0, -1):
            print(f"   {i}...")
            time.sleep(1)
        
        print("   🚀 НАЧАЛО!")
        
        # Сбор данных
        pitch_samples = []
        roll_samples = []
        
        print("\n📊 Сбор данных калибровки...")
        
        start_time = time.time()
        sample_count = 0
        progress_bar_length = 40
        
        while time.time() - start_time < 5:
            elapsed = time.time() - start_time
            progress = int((elapsed / 5) * progress_bar_length)
            
            data = self.esp32.receive_data()
            if data:
                yaw, pitch, roll = data
                pitch_samples.append(pitch)
                roll_samples.append(roll)
                sample_count += 1
                
                bar = '█' * progress + '░' * (progress_bar_length - progress)
                print(f"\r   [{bar}] {elapsed:.1f}/5.0с | Образцов: {sample_count}", 
                      end='', flush=True)
            time.sleep(0.05)
        
        print()
        
        if not pitch_samples or not roll_samples:
            print("❌ Ошибка калибровки: не удалось собрать данные")
            return False
        
        # Вычисляем средние значения
        self.calibration.zero_pitch = sum(pitch_samples) / len(pitch_samples)
        self.calibration.zero_roll = sum(roll_samples) / len(roll_samples)
        
        # Вычисляем стандартное отклонение
        if len(pitch_samples) > 1:
            pitch_std = math.sqrt(sum((x - self.calibration.zero_pitch)**2 
                                    for x in pitch_samples) / (len(pitch_samples) - 1))
            roll_std = math.sqrt(sum((x - self.calibration.zero_roll)**2 
                                   for x in roll_samples) / (len(roll_samples) - 1))
            
            print(f"\n📊 Статистика:")
            print(f"   📈 Pitch std: {pitch_std:.2f}° {'✅' if pitch_std < 2 else '⚠️'}")
            print(f"   📈 Roll std: {roll_std:.2f}° {'✅' if roll_std < 2 else '⚠️'}")
        
        print(f"\n✅ КАЛИБРОВКА ЗАВЕРШЕНА:")
        print(f"   🎯 Zero Pitch: {self.calibration.zero_pitch:.1f}°")
        print(f"   🎯 Zero Roll: {self.calibration.zero_roll:.1f}°")
        print(f"   📊 Образцов собрано: {sample_count}")
        
        self.calibration_done = True
        return True

# ============================================================================
# КЛАСС ДЛЯ УПРАВЛЕНИЯ МЕНЮ
# ============================================================================

class MenuManager:
    """Класс для управления меню и вводом с клавиатуры"""
    
    def __init__(self):
        self.current_mode = 1  # 1 - главное меню, 2 - меню камеры
        self.program_running = True
        self.telemetry_enabled = False
        self.print_telemetry_flag = False
        self.menu_shown = False
        self.camera_menu_shown = False
        self.show_menu_after_command = False
        self.telemetry_line_shown = False
        self.object_detection_flag = False
        
        if not WINDOWS:
            # Сохраняем настройки терминала для Linux
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
        
        # PID коэффициенты
        self.pid_coeffs = {
            'pitch': PIDCoeffs(Kp=4.0, Ki=0.02, Kd=0.5),
            'roll': PIDCoeffs(Kp=4.0, Ki=0.02, Kd=0.5),
            'yaw': PIDCoeffs(Kp=2.0, Ki=0.005, Kd=0.3)
        }
        
        # Калибровка
        self.calibration_manager = CalibrationManager(self.esp32)
        self.calibration = self.calibration_manager.calibration
        
        # Обработка данных
        self.sensor_processor = SensorProcessor(self.calibration)
        
        # YOLOv8 детектор
        self.yolo_detector = YOLOv8Detector()
        self.object_detection_active = False
        self.detection_display_thread = None
        
        # Время
        self.start_time = time.time()
        self.last_command_time = time.time()
        
        # Флаг для обновления телеметрии
        self.last_telemetry_update = 0
        self.telemetry_update_interval = 0.2  # Обновлять телеметрию каждые 200 мс
    
    def show_status(self):
        """Показывает текущий статус системы"""
        print("\n" + "="*60)
        print("📊 ТЕКУЩИЙ СТАТУС СИСТЕМЫ")
        print("="*60)
        
        print(f"ESP32:")
        print(f"  Моторы: {'✅ ARM' if self.esp32.state.motors_armed else '❌ DISARM'}")
        print(f"  Стрим: {'✅ ВКЛ' if self.esp32.state.stream_active else '❌ ВЫКЛ'}")
        print(f"  Throttle: {self.esp32.state.throttle_value} мкс")
        
        print("-" * 60)
        print(f"AirSim:")
        print(f"  Подключение: {'✅' if self.airsim.connected else '❌'}")
        if self.airsim.connected:
            drone_state = self.airsim.get_drone_state()
            if drone_state:
                print(f"  Дрон: {drone_state}")
            else:
                print(f"  Дрон: ❓ Неизвестно")
        
        print("-" * 60)
        print(f"Калибровка:")
        status = '✅ ВЫПОЛНЕНА' if self.calibration_manager.calibration_done else '❌ НЕ ВЫПОЛНЕНА'
        print(f"  Статус: {status}")
        if self.calibration_manager.calibration_done:
            print(f"  Zero Pitch: {self.calibration.zero_pitch:.1f}°")
            print(f"  Zero Roll: {self.calibration.zero_roll:.1f}°")
        
        print("-" * 60)
        print(f"YOLOv8 Детекция:")
        status = '✅ АКТИВНА' if self.object_detection_active else '❌ НЕАКТИВНА'
        print(f"  Статус: {status}")
        if self.object_detection_active:
            objects_count = self.yolo_detector.get_detected_objects_count()
            print(f"  Обнаружено объектов: {objects_count}")
        
        print("="*60)
    
    def show_main_menu(self):
        """Показывает главное меню"""
        if not self.menu.menu_shown:
            self.menu.clear_telemetry_line()
            print("\n" + "="*70)
            print("🔧 ГЛАВНОЕ МЕНЮ УПРАВЛЕНИЯ:")
            print("="*70)
            
            menu_items = [
                "[1] Включить телеметрию",
                "[2] Настройка камеры",
                "[5] YOLOv8 Детекция объектов",  
                "-" * 70,
                "[3] Управление дроном (установка углов)",
                "     Задает target angles на ESP32",
                "-" * 70,
                "[4] Показать PID коэффициенты",
                "-" * 70,
                "[a] ARM (включить моторы)",
                "[z] DISARM (выключить моторы)",
                "-" * 70,
                "[s] START стрим",
                "[x] STOP стрим",
                "-" * 70,
                "[t] Takeoff (AirSim)",
                "[l] Land (AirSim)",
                "-" * 70,
                "[i] Показать текущий статус",
                "[ ] Пробел - выключить телеметрию",
                "-" * 70,
                "[r] Переподключиться к AirSim",
                "[q] Посадка и выход"
            ]
            
            for item in menu_items:
                print(f"   {item}")
            
            print("="*70)
            
            if self.calibration_manager.calibration_done:
                print("✅ Калибровка выполнена. Система готова к работе.")
            else:
                print("❌ Калибровка не выполнена. Сначала выполните калибровку!")
            
            print("="*70)
            print("👉 Нажмите клавишу (цифра/буква)")
            
            self.menu.menu_shown = True
    
    def drone_control_menu(self):
        """Меню управления дроном"""
        self.menu.print_telemetry_flag = False
        self.menu.clear_telemetry_line()
        
        print("\n" + "="*60)
        print("🎮 УПРАВЛЕНИЕ ДРОНОМ (Target Angles)")
        print("="*60)
        
        # Ввод значений
        try:
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
            
            # Проверка диапазонов
            if abs(pitch) > 30:
                print(f"⚠️ Pitch ({pitch}°) выходит за рекомендуемый диапазон ±30°")
            if abs(roll) > 30:
                print(f"⚠️ Roll ({roll}°) выходит за рекомендуемый диапазон ±30°")
            
            print(f"\n📢 Будет отправлена команда: SET {yaw:.1f} {pitch:.1f} {roll:.1f} {throttle}")
            print(f"   ESP32 будет пытаться удерживать эти углы")
            
            confirm = input("Подтвердить отправку? (y/n): ").strip().lower()
            if confirm == 'y':
                if self.esp32.set_target_angles(yaw, pitch, roll, throttle):
                    print(f"✅ Команда отправлена: Yaw={yaw:.1f}°, "
                         f"Pitch={pitch:.1f}°, Roll={roll:.1f}°, "
                         f"Throttle={throttle} мкс")
            
        except ValueError as e:
            print(f"❌ Ошибка ввода: {e}")
        except Exception as e:
            print(f"❌ Ошибка: {e}")
    
    def show_pid_info(self):
        """Показывает информацию о PID коэффициентах"""
        self.menu.print_telemetry_flag = False
        self.menu.clear_telemetry_line()
        
        print("\n" + "="*60)
        print("⚙️ PID КОЭФФИЦИЕНТЫ (ТОЛЬКО ДЛЯ ПРОСМОТРА)")
        print("="*60)
        print("Эти коэффициенты запрограммированы в ESP32")
        print("и регулируют стабилизацию дрона по углам.")
        print("-" * 60)
        print("📊 Текущие значения:")
        print(f"  Pitch (тангаж):")
        print(f"    Kp = {self.pid_coeffs['pitch'].Kp} - пропорциональный коэффициент")
        print(f"    Ki = {self.pid_coeffs['pitch'].Ki} - интегральный коэффициент")
        print(f"    Kd = {self.pid_coeffs['pitch'].Kd} - дифференциальный коэффициент")
        print()
        print(f"  Roll (крен):")
        print(f"    Kp = {self.pid_coeffs['roll'].Kp}")
        print(f"    Ki = {self.pid_coeffs['roll'].Ki}")
        print(f"    Kd = {self.pid_coeffs['roll'].Kd}")
        print()
        print(f"  Yaw (рыскание):")
        print(f"    Kp = {self.pid_coeffs['yaw'].Kp}")
        print(f"    Ki = {self.pid_coeffs['yaw'].Ki}")
        print(f"    Kd = {self.pid_coeffs['yaw'].Kd}")
        print("-" * 60)
        print("ℹ️  Изменение PID коэффициентов требует перепрошивки ESP32")
        print("="*60)
        
        input("\nНажмите Enter для возврата в меню...")
    
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
            '3': self._handle_drone_control,
            '4': self._handle_pid_info,
            '5': self._handle_object_detection,  
            'a': self._handle_arm,
            'z': self._handle_disarm,
            's': self._handle_start_stream,
            'x': self._handle_stop_stream,
            't': self._handle_takeoff,
            'l': self._handle_land,
            'i': self._handle_status,
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
        """Обработка клавиш меню камеры"""
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
            if self.airsim.set_camera_orientation(self.airsim.camera_pitch, 
                                                 self.airsim.camera_yaw):
                print(f"\r  📷 Камера: Pitch={self.airsim.camera_pitch:3}°, "
                     f"Yaw={self.airsim.camera_yaw:3}°", end='', flush=True)
    
    # ============================================================================
    # ОБРАБОТЧИКИ КОМАНД
    # ============================================================================
    
    def _handle_telemetry_enable(self):
        if not self.calibration_manager.calibration_done:
            print("\n❌ Сначала выполните калибровку!")
            self.menu.show_menu_after_command = True
            return
        
        self.menu.print_telemetry_flag = True
        self.menu.telemetry_enabled = True
        print("\n📊 Телеметрия ВКЛЮЧЕНА (нажмите пробел для отключения)")
        self.menu.show_menu_after_command = True
    
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
    
    def _handle_drone_control(self):
        if not self.calibration_manager.calibration_done:
            print("\n❌ Сначала выполните калибровку!")
            self.menu.show_menu_after_command = True
            return
        
        if not self.esp32.state.stream_active:
            print("\n⚠️  Стрим данных не активен!")
            print("   Включите стрим командой [s]")
            self.menu.show_menu_after_command = True
            return
        
        self.drone_control_menu()
        self.menu.show_menu_after_command = True
    
    def _handle_pid_info(self):
        self.show_pid_info()
        self.menu.show_menu_after_command = True
    
    def _handle_object_detection(self):
        """Обработка запуска/остановки детекции объектов"""
        if not self.airsim.connected:
            print("\n❌ Сначала подключитесь к AirSim!")
            self.menu.show_menu_after_command = True
            return
        
        if not self.object_detection_active:
            # ЗАПУСК ДЕТЕКЦИИ
            print("\n🎯 ЗАПУСК YOLOv8 ДЕТЕКЦИИ")
            print("-" * 40)
            print("• Нажмите [5] повторно для остановки")
            print("• Нажмите ESC в окне детекции для выхода")
            print("• Обнаруженные объекты отображаются в bounding boxes")
            print("-" * 40)
            
            # Подключаем детектор к AirSim
            self.yolo_detector.connect_to_airsim(self.airsim.client)
            self.object_detection_active = True
            
            # Запускаем детекцию
            self.yolo_detector.start_detection()
            
            # Запускаем отображение в отдельном потоке
            self.detection_display_thread = Thread(target=self._display_detection, 
                                                  daemon=True)
            self.detection_display_thread.start()
            
            print("✅ Детекция запущена. Окно OpenCV должно открыться.")
            
        else:
            # ОСТАНОВКА ДЕТЕКЦИИ
            print("\n🛑 Останавливаю детекцию...")
            self.object_detection_active = False
            self.yolo_detector.stop_detection()
            cv2.destroyAllWindows()
            print("✅ Детекция остановлена")
        
        self.menu.show_menu_after_command = True
    
    def _display_detection(self):
        """Отображение детекции в отдельном окне"""
        window_name = 'YOLOv8 Object Detection - AirSim'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        
        last_objects_count = 0
        last_update_time = time.time()
        
        while self.object_detection_active and self.yolo_detector.detection_active:
            try:
                frame = self.yolo_detector.get_detection_display()
                if frame is not None:
                    # Добавляем информацию о количестве объектов
                    objects_count = self.yolo_detector.get_detected_objects_count()
                    
                    # Обновляем счетчик в консоли каждые 2 секунды
                    current_time = time.time()
                    if objects_count != last_objects_count or current_time - last_update_time > 2:
                        print(f"\r📦 Обнаружено объектов: {objects_count}", end='', flush=True)
                        last_objects_count = objects_count
                        last_update_time = current_time
                    
                    # Добавляем текст на кадр
                    info_text = f"Objects: {objects_count}"
                    cv2.putText(frame, info_text,
                              (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                              1, (0, 255, 0), 2)
                    
                    # Показываем кадр
                    cv2.imshow(window_name, frame)
                
                # Выход по ESC или кнопке 5
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    self.object_detection_active = False
                    break
                    
            except Exception as e:
                print(f"❌ Ошибка отображения: {e}")
                break
        
        cv2.destroyAllWindows()
    
    def _handle_arm(self):
        if not self.calibration_manager.calibration_done:
            print("\n❌ Сначала выполните калибровку!")
            self.menu.show_menu_after_command = True
            return
        
        if not self.esp32.state.stream_active:
            print("\n⚠️  Стрим данных не активен!")
            print("   Включите стрим командой [s]")
            self.menu.show_menu_after_command = True
            return
        
        print("\n⚠️  АРМИРОВАНИЕ МОТОРОВ")
        confirm = input("Продолжить? (y/n): ").strip().lower()
        if confirm == 'y':
            if self.esp32.send_command("ARM"):
                print("✅ Команда ARM отправлена на ESP32")
        else:
            print("❌ ARM отменен")
        self.menu.show_menu_after_command = True
    
    def _handle_disarm(self):
        print("\n⚠️  DISARM МОТОРОВ")
        confirm = input("Продолжить? (y/n): ").strip().lower()
        if confirm == 'y':
            if self.esp32.send_command("DISARM"):
                print("✅ Команда DISARM отправлена на ESP32")
        else:
            print("❌ DISARM отменен")
        self.menu.show_menu_after_command = True
    
    def _handle_start_stream(self):
        if self.esp32.send_command("START"):
            print("✅ Стрим данных START отправлен")
        self.menu.show_menu_after_command = True
    
    def _handle_stop_stream(self):
        if self.esp32.send_command("STOP"):
            print("✅ Стрим данных STOP отправлен")
        self.menu.show_menu_after_command = True
    
    def _handle_takeoff(self):
        if not self.airsim.connected:
            print("❌ AirSim не подключен! Используйте [r]")
            self.menu.show_menu_after_command = True
            return
        
        if not self.esp32.state.motors_armed:
            print("❌ Сначала взведите моторы [a]!")
            self.menu.show_menu_after_command = True
            return
        
        print("\n🛫 ВЗЛЕТ ДРОНА")
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
    
    def _handle_status(self):
        self.show_status()
        self.menu.show_menu_after_command = True
    
    def _handle_telemetry_disable(self):
        if self.menu.telemetry_enabled:
            self.menu.telemetry_enabled = False
            self.menu.print_telemetry_flag = False
            self.menu.clear_telemetry_line()
            print("\n📊 Телеметрия ВЫКЛЮЧЕНА")
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
    
    def run(self):
        """Запуск основной программы"""
        print("\n" + "="*60)
        print("🚀 СИСТЕМА УПРАВЛЕНИЯ ДРОНОМ С YOLOv8")
        print("="*60)
        print(f"📝 Логирование начато: {os.path.basename(self.logger.get_log_path())}")
        print(f"🤖 YOLOv8 модель: {Config.YOLO_MODEL_PATH}")
        
        # Запуск калибровки
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
        """Выполнение калибровки"""
        print("\n" + "="*60)
        print("🔴 ОБЯЗАТЕЛЬНАЯ КАЛИБРОВКА ПРИ ЗАПУСКЕ")
        print("="*60)
        
        print("\n📋 ШАГ 1: ПОДГОТОВКА К КАЛИБРОВКЕ")
        print("-" * 40)
        input("⏎ Нажмите Enter, когда будете готовы начать подготовку...")
        
        if not self.calibration_manager.prepare_calibration():
            return False
        
        print("\n📋 ШАГ 2: ВЫПОЛНЕНИЕ КАЛИБРОВКИ")
        print("-" * 40)
        input("\n⏎ Нажмите Enter, когда будете готовы начать калибровку...")
        
        if not self.calibration_manager.perform_calibration():
            print("\n❌ КАЛИБРОВКА НЕ УДАЛАСЬ!")
            return False
        
        print("\n✅ КАЛИБРОВКА УСПЕШНО ЗАВЕРШЕНА!")
        return True
    
    def _main_loop(self):
        """Основной цикл обработки данных"""
        try:
            while self.menu.program_running:
                current_time = time.time()
                
                if self.menu.show_menu_after_command:
                    self.menu.menu_shown = False
                    self.menu.show_menu_after_command = False
                
                # Показ меню
                if self.menu.current_mode == 1 and not self.menu.menu_shown:
                    self.show_main_menu()
                
                # Получение данных от ESP32
                data = self.esp32.receive_data()
                if data:
                    self._process_sensor_data(data, current_time)
                
                # Обновление телеметрии (с ограничением частоты)
                if (self.menu.print_telemetry_flag and self.menu.telemetry_enabled and 
                    self.menu.current_mode == 1 and 
                    current_time - self.last_telemetry_update > self.telemetry_update_interval):
                    
                    # Показываем телеметрию только если есть данные
                    if hasattr(self, 'last_sensor_data'):
                        self._print_telemetry_line()
                        self.last_telemetry_update = current_time
                
                # Keep-alive для AirSim
                if (current_time - self.last_command_time > Config.KEEP_ALIVE_INTERVAL and 
                    self.airsim.connected and self.airsim.in_air):
                    self.airsim.move_by_velocity(0, 0, 0)
                    self.last_command_time = current_time
                
                time.sleep(Config.MAIN_LOOP_SLEEP)
                
        except KeyboardInterrupt:
            print("\n🛑 Остановка по Ctrl+C...")
        except Exception as e:
            print(f"\n❌ Критическая ошибка: {e}")
        finally:
            self.shutdown()
    
    def _process_sensor_data(self, raw_data: Tuple[float, float, float], current_time: float):
        """Обработка данных с датчиков"""
        raw_yaw, raw_pitch, raw_roll = raw_data
        
        # Проверка на NaN
        if any(math.isnan(x) for x in [raw_yaw, raw_pitch, raw_roll]):
            return
        
        # Обработка данных
        sensor_data = self.sensor_processor.process_sensor_data(raw_yaw, raw_pitch, raw_roll)
        rel_pitch, rel_roll = self.sensor_processor.calculate_relative_angles(sensor_data)
        control_data = self.sensor_processor.calculate_control_data(
            rel_pitch, rel_roll, sensor_data.yaw
        )
        
        # Сохраняем последние данные для телеметрии
        self.last_sensor_data = sensor_data
        self.last_control_data = control_data
        self.last_rel_pitch = rel_pitch
        self.last_rel_roll = rel_roll
        
        # Логирование с корректным временем
        self.logger.log_angles(sensor_data, control_data, rel_pitch, rel_roll)
        
        # Управление дроном в AirSim
        if self.airsim.connected and self.airsim.in_air:
            self.airsim.move_by_velocity(
                control_data.velocity_x,
                control_data.velocity_y,
                sensor_data.yaw
            )
            self.last_command_time = current_time
    
    def _print_telemetry_line(self):
        """Вывод одной строки телеметрии"""
        if not hasattr(self, 'last_sensor_data'):
            return
        
        # Очищаем предыдущую строку
        if self.menu.telemetry_line_shown:
            print('\r' + ' ' * 120 + '\r', end='', flush=True)
        
        # Получаем количество объектов, если детекция активна
        objects_info = ""
        if self.object_detection_active:
            objects_count = self.yolo_detector.get_detected_objects_count()
            objects_info = f" | Objects: {objects_count}"
        
        # Выводим новую строку
        print(f"\r📊 Raw: P={self.last_sensor_data.pitch:6.1f}° R={self.last_sensor_data.roll:6.1f}° "
              f"Y={self.last_sensor_data.yaw:6.1f}° | Rel: P={self.last_rel_pitch:6.1f}° "
              f"R={self.last_rel_roll:6.1f}° | V: X={self.last_control_data.velocity_x:5.2f} "
              f"Y={self.last_control_data.velocity_y:5.2f} m/s{objects_info}", end='', flush=True)
        
        self.menu.telemetry_line_shown = True
    
    def shutdown(self):
        """Безопасное завершение работы"""
        print("\n🛬 Безопасное завершение...")
        
        # Остановка детекции, если активна
        if self.object_detection_active:
            print("🛑 Остановка YOLOv8 детекции...")
            self.object_detection_active = False
            self.yolo_detector.stop_detection()
            cv2.destroyAllWindows()
        
        # Посадка дрона
        if self.airsim.connected and self.airsim.in_air:
            self.airsim.land()
        
        # Отключение ESP32
        try:
            print("🔌 Отключение ESP32...")
            self.esp32.send_command("STOP", silent=True)
            time.sleep(0.1)
            self.esp32.send_command("DISARM", silent=True)
        except:
            pass
        
        # Закрытие соединений 
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