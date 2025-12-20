import cv2
import numpy as np
import time
import threading
from ultralytics import YOLO
import airsim
import msvcrt  # Для Windows
import sys

# ============================================================================
# КЛАСС ДЕТЕКЦИИ YOLOv8
# ============================================================================

class YOLOv8Detector:
    """Простой детектор объектов YOLOv8"""
    
    def __init__(self, model_path='Best.pt', conf_threshold=0.5):
        print(f"🔧 Загрузка модели YOLOv8 из: {model_path}")
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.detection_active = False
        self.current_frame = None
        self.lock = threading.Lock()
        self.detection_thread = None
        self.airsim_client = None
        
        # Проверка доступности GPU
        print(f"   Устройство: {'CUDA (GPU)' if self.model.device.type == 'cuda' else 'CPU'}")
        
    def connect_to_airsim(self, client):
        """Подключение к AirSim"""
        self.airsim_client = client
        print("📡 Детектор подключен к AirSim")
        
    def start_detection(self):
        """Запуск детекции"""
        if self.detection_active:
            return
            
        print("🚀 Запуск детекции объектов...")
        self.detection_active = True
        self.detection_thread = threading.Thread(target=self._detection_loop, daemon=True)
        self.detection_thread.start()
        
    def stop_detection(self):
        """Остановка детекции"""
        if not self.detection_active:
            return
            
        print("🛑 Остановка детекции...")
        self.detection_active = False
        if self.detection_thread:
            self.detection_thread.join(timeout=2)
        cv2.destroyAllWindows()
        
    def _detection_loop(self):
        """Оптимизированный цикл детекции для CPU"""
        print("🎥 Начинаю захват кадров (режим оптимизации для CPU)...")
        cv2.namedWindow('YOLOv8 Detection', cv2.WINDOW_NORMAL)
        
        frame_skip = 3  # Обрабатываем каждый 3-й кадр
        frame_counter = 0
        last_fps_time = time.time()
        fps = 0
        
        # Используем маленькую модель для CPU
        print("   Использую облегченный режим для CPU")
        
        while self.detection_active:
            try:
                frame_counter += 1
                
                # Пропускаем кадры для экономии ресурсов
                if frame_counter % frame_skip != 0:
                    cv2.waitKey(1)
                    continue
                    
                # Получение кадра
                responses = self.airsim_client.simGetImages([
                    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
                ])
                
                if not responses or not responses[0]:
                    time.sleep(0.1)
                    continue
                
                # Уменьшаем разрешение кадра ВДВОЕ для ускорения
                img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
                img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)
                
                # Уменьшаем изображение
                small_height = responses[0].height // 2
                small_width = responses[0].width // 2
                img_small = cv2.resize(img_rgb, (small_width, small_height))
                
                # 📌 КЛЮЧЕВАЯ ОПТИМИЗАЦИЯ: используем самую маленькую модель YOLO
                results = self.model(img_small, 
                                conf=self.conf_threshold,
                                imgsz=320,        # Маленький размер изображения
                                half=False,       # Не использовать половинную точность (только для GPU)
                                device='cpu',     # Явно указываем CPU
                                verbose=False,
                                max_det=10,       # Максимум 10 объектов
                                agnostic_nms=True)[0]
                
                # Отрисовка на УМЕНЬШЕННОМ изображении
                frame_with_boxes = img_small.copy()
                objects_detected = 0
                
                if hasattr(results, 'boxes') and results.boxes is not None:
                    boxes = results.boxes
                    objects_detected = len(boxes)
                    
                    for box in boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                        conf = box.conf[0].cpu().numpy()
                        cls = int(box.cls[0].cpu().numpy())
                        
                        color = (0, 255, 0)
                        cv2.rectangle(frame_with_boxes, (x1, y1), (x2, y2), color, 1)
                        
                        class_name = results.names[cls]
                        label = f'{class_name} {conf:.2f}'
                        cv2.putText(frame_with_boxes, label,
                                (x1, max(y1-5, 10)),  # Защита от выхода за границы
                                cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)
                
                # Вычисляем FPS
                current_time = time.time()
                if current_time - last_fps_time >= 1.0:
                    fps = frame_skip / (current_time - last_fps_time)
                    last_fps_time = current_time
                
                # Информация на экране
                info_text = f"CPU Mode | FPS: {fps:.1f} | Objects: {objects_detected}"
                cv2.putText(frame_with_boxes, info_text,
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                
                # Масштабируем обратно для показа (чтобы не было слишком мелко)
                display_frame = cv2.resize(frame_with_boxes, 
                                        (small_width * 2, small_height * 2))
                
                cv2.imshow('YOLOv8 Detection', display_frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    self.detection_active = False
                    break
                    
            except Exception as e:
                print(f"Ошибка: {e}")
                time.sleep(0.5)
        
        cv2.destroyAllWindows()
        print("📹 Детекция остановлена")
# ============================================================================
# ОСНОВНАЯ ПРОГРАММА
# ============================================================================

class SimpleDroneTest:
    """Простой тест дрона с детекцией"""
    
    def __init__(self):
        self.airsim_client = None
        self.detector = None
        self.drone_in_air = False
        self.running = True
        self.altitude = -5  # Высота в AirSim (отрицательная = вверх)
        
    def connect_to_airsim(self):
        """Подключение к AirSim"""
        print("\n" + "="*50)
        print("🚁 ПОДКЛЮЧЕНИЕ К AirSim")
        print("="*50)
        
        try:
            print("🔌 Подключаюсь к AirSim...")
            self.airsim_client = airsim.MultirotorClient()
            self.airsim_client.confirmConnection()
            self.airsim_client.enableApiControl(True)
            self.airsim_client.armDisarm(False)
            
            # Удалите или закомментируйте проблемные строки:
            # print(f"   Адрес: {self.airsim_client.client._host}:{self.airsim_client.client._port}")
            
            print("✅ AirSim подключен успешно!")
            return True
            
        except Exception as e:
            print(f"❌ Ошибка подключения к AirSim: {e}")
            print("\nУбедитесь, что:")
            print("1. AirSim запущен")
            print("2. Выбран симулятор дрона")
            print("3. Карта загружена")
            return False
        
    def takeoff(self):
        """Взлет дрона"""
        if not self.airsim_client:
            print("❌ AirSim не подключен!")
            return False
            
        try:
            print("\n✈️  ВЗЛЕТ ДРОНА")
            print("-" * 30)
            
            # Взведение моторов
            print("🔧 Взвожу моторы...")
            self.airsim_client.armDisarm(True)
            time.sleep(1)
            
            # Взлет
            print("🛫 Взлетаю...")
            self.airsim_client.takeoffAsync().join()
            time.sleep(2)
            
            # Подъем на заданную высоту
            print(f"📈 Поднимаюсь на высоту {-self.altitude} метров...")
            self.airsim_client.moveToZAsync(self.altitude, 2).join()
            
            self.drone_in_air = True
            print("✅ Дрон в воздухе!")
            return True
            
        except Exception as e:
            print(f"❌ Ошибка при взлете: {e}")
            return False
    
    def land(self):
        """Посадка дрона"""
        if not self.drone_in_air:
            return
            
        try:
            print("\n🛬 ПОСАДКА ДРОНА")
            print("-" * 30)
            
            print("📉 Начинаю посадку...")
            self.airsim_client.landAsync().join()
            time.sleep(2)
            self.airsim_client.armDisarm(False)
            
            self.drone_in_air = False
            print("✅ Дрон приземлился")
            
        except Exception as e:
            print(f"❌ Ошибка при посадке: {e}")
    
    def initialize_detector(self):
        """Инициализация детектора"""
        print("\n" + "="*50)
        print("🤖 ИНИЦИАЛИЗАЦИЯ YOLOv8")
        print("="*50)
        
        try:
            # Укажите правильный путь к вашим весам
            model_path = r'C:\Users\mkravtsov\Desktop\diplomaa\diplomaa\Best.pt'
            self.detector = YOLOv8Detector(model_path)
            self.detector.connect_to_airsim(self.airsim_client)
            print("✅ Детектор инициализирован")
            return True
            
        except Exception as e:
            print(f"❌ Ошибка инициализации детектора: {e}")
            print("\nВозможные причины:")
            print("1. Файл Best.pt не найден")
            print("2. Проблемы с PyTorch/Ultralytics")
            print("3. Недостаточно памяти")
            return False
    
    def print_menu(self):
        """Показ меню управления"""
        print("\n" + "="*50)
        print("🎮 УПРАВЛЕНИЕ ДРОНОМ")
        print("="*50)
        print("[t] - Взлет дрона")
        print("[l] - Посадка дрона")
        print("[5] - Включить/выключить детекцию объектов")
        print("[q] - Посадка и выход")
        print("[i] - Информация о системе")
        print("="*50)
        print("💡 Нажмите цифру 5 для теста детекции объектов!")
        print("="*50)
    
    def print_system_info(self):
        """Информация о системе"""
        print("\n📊 ИНФОРМАЦИЯ О СИСТЕМЕ")
        print("-" * 30)
        print(f"Дрон в воздухе: {'✅ Да' if self.drone_in_air else '❌ Нет'}")
        print(f"Детекция активна: {'✅ Да' if self.detector and self.detector.detection_active else '❌ Нет'}")
        print(f"Высота: {-self.altitude} метров")
        print(f"AirSim подключен: {'✅ Да' if self.airsim_client else '❌ Нет'}")
    
    def get_key(self):
        """Получение нажатой клавиши (без ENTER)"""
        try:
            if msvcrt.kbhit():
                key = msvcrt.getch()
                return key.decode('utf-8').lower()
        except:
            pass
        return None
    
    def run(self):
        """Основной цикл программы"""
        print("\n" + "="*50)
        print("🚀 ПРОГРАММА ТЕСТА ДРОНА С YOLOv8")
        print("="*50)
        print("Версия: 1.0")
        print("Автор: Дипломный проект")
        print("="*50)
        
        # Подключение к AirSim
        if not self.connect_to_airsim():
            print("❌ Не удалось подключиться к AirSim. Завершение работы.")
            return
        
        # Инициализация детектора
        if not self.initialize_detector():
            print("⚠️  Детектор не инициализирован, но продолжим без него...")
        
        # Показ меню
        self.print_menu()
        
        # Основной цикл обработки команд
        try:
            while self.running:
                key = self.get_key()
                
                if key:
                    if key == 't':
                        # Взлет
                        self.takeoff()
                        self.print_menu()
                        
                    elif key == 'l':
                        # Посадка
                        self.land()
                        self.print_menu()
                        
                    elif key == '5':
                        # Включение/выключение детекции
                        if not self.detector:
                            print("❌ Детектор не инициализирован!")
                            continue
                            
                        if not self.detector.detection_active:
                            if not self.drone_in_air:
                                print("⚠️  Дрон не в воздухе! Сначала взлетаем...")
                                if self.takeoff():
                                    time.sleep(1)
                                else:
                                    continue
                            
                            print("\n🎯 ЗАПУСК ДЕТЕКЦИИ ОБЪЕКТОВ")
                            print("="*30)
                            print("• Нажмите ESC в окне детекции для выхода")
                            print("• Или нажмите 5 снова для остановки")
                            print("• Bounding boxes показывают найденные объекты")
                            print("="*30)
                            
                            self.detector.start_detection()
                            
                        else:
                            print("\n🛑 ОСТАНОВКА ДЕТЕКЦИИ")
                            self.detector.stop_detection()
                            
                        self.print_menu()
                        
                    elif key == 'i':
                        # Информация о системе
                        self.print_system_info()
                        self.print_menu()
                        
                    elif key == 'q':
                        # Выход
                        print("\n🛑 ЗАВЕРШЕНИЕ РАБОТЫ")
                        self.running = False
                        break
                
                # Небольшая пауза для снижения нагрузки на CPU
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n🛑 Программа прервана пользователем")
        except Exception as e:
            print(f"\n❌ Критическая ошибка: {e}")
        finally:
            # Завершение работы
            self.shutdown()
    
    def shutdown(self):
        """Безопасное завершение работы"""
        print("\n" + "="*50)
        print("🔴 ЗАВЕРШЕНИЕ РАБОТЫ")
        print("="*50)
        
        # Остановка детекции
        if self.detector and self.detector.detection_active:
            print("🛑 Останавливаю детекцию...")
            self.detector.stop_detection()
        
        # Посадка дрона
        if self.drone_in_air:
            print("🛬 Сажаю дрон...")
            self.land()
        
        # Отключение от AirSim
        if self.airsim_client:
            print("🔌 Отключаюсь от AirSim...")
            try:
                self.airsim_client.enableApiControl(False)
            except:
                pass
        
        print("✅ Программа завершена")
        print("="*50)

# ============================================================================
# ЗАПУСК ПРОГРАММЫ
# ============================================================================

if __name__ == "__main__":
    # Проверка наличия необходимых библиотек
    try:
        import ultralytics
        print(f"✅ Ultralytics version: {ultralytics.__version__}")
    except ImportError:
        print("❌ Ultralytics не установлен!")
        print("Установите: pip install ultralytics")
        sys.exit(1)
    
    try:
        import airsim
        print(f"✅ AirSim импортируется успешно")
    except ImportError:
        print("❌ AirSim не установлен!")
        print("Установите: pip install airsim")
        sys.exit(1)
    
    # Запуск программы
    app = SimpleDroneTest()
    app.run()