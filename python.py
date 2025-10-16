import socket
import os
import csv
import time
from datetime import datetime

ESP32_IP = "192.168.1.26"
PORT = 3333

# === Создаём папку для логов ===
log_dir = "logs"
os.makedirs(log_dir, exist_ok=True)

# === Имя файла с текущей датой и временем ===
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
log_file = os.path.join(log_dir, f"flight_{timestamp}.csv")

# === Создаём CSV-файл и заголовки ===
headers = ["Time(s)", "AX", "AY", "AZ", "GX", "GY", "GZ", "AngleX", "AngleY", "AngleZ"]

with open(log_file, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(headers)

# === Настройка UDP ===
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", PORT))

print(f"📡 Connecting to ESP32 ({ESP32_IP}) ...")

# отправляем команду START
sock.sendto(b"START", (ESP32_IP, PORT))
print("🚀 Sent START command to ESP32")

start_time = time.time()

try:
    while True:
        data, addr = sock.recvfrom(1024)
        msg = data.decode().strip()
        print(f"📨 {msg}")

        # Разбираем строку в формат "ключ:значение"
        values = {}
        for part in msg.split():
            if ":" in part:
                k, v = part.split(":")
                values[k] = v

        # Записываем строку в CSV
        current_time = time.time() - start_time
        row = [
            round(current_time, 2),
            values.get("AX"), values.get("AY"), values.get("AZ"),
            values.get("GX"), values.get("GY"), values.get("GZ"),
            values.get("AngleX"), values.get("AngleY"), values.get("AngleZ")
        ]

        with open(log_file, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(row)

except KeyboardInterrupt:
    print("\n🛑 Stopping...")
    sock.sendto(b"STOP", (ESP32_IP, PORT))
    sock.close()
    print(f"💾 Data saved to {log_file}")
