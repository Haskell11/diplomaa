import socket
import os
import time
from datetime import datetime

# 🔧 Настройки
ESP32_IP = "192.168.1.26"
PORT = 3333

# 📁 Папка для логов
log_dir = "logs"
os.makedirs(log_dir, exist_ok=True)

# 🕓 Создание нового лог-файла
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
log_file = os.path.join(log_dir, f"orientation_{timestamp}.txt")
headers = ["Time(s)", "Yaw(°)", "Pitch(°)", "Roll(°)"]

with open(log_file, "w", encoding="utf-8") as f:
    f.write("=== Orientation Log ===\n")
    f.write("\t".join(headers) + "\n")

# 🔗 Настройка сокета
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", PORT))

print(f"📡 Ожидание данных от ESP32 ({ESP32_IP}:{PORT})...")
sock.sendto(b"START", (ESP32_IP, PORT))
print("🚀 Отправлена команда START\n")

start_time = time.time()

try:
    while True:
        data, addr = sock.recvfrom(1024)
        msg = data.decode().strip()

        print(f"📨 {msg}")

        # Обработка формата UDP: "45.12,-3.45,2.89" (только числа через запятую)
        if ',' in msg:
            parts = msg.split(',')
            if len(parts) == 3:
                yaw, pitch, roll = parts
            else:
                yaw, pitch, roll = "0", "0", "0"
        else:
            yaw, pitch, roll = "0", "0", "0"

        # Время с начала сеанса
        current_time = round(time.time() - start_time, 2)

        # Формируем строку лога
        row = [str(current_time), yaw, pitch, roll]

        with open(log_file, "a", encoding="utf-8") as f:
            f.write("\t".join(row) + "\n")

except KeyboardInterrupt:
    print("\n🛑 Остановка...")
    sock.sendto(b"STOP", (ESP32_IP, PORT))
    sock.close()
    print(f"💾 Данные сохранены в {log_file}")