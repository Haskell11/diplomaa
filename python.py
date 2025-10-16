import socket
import time
import os
from datetime import datetime

ESP32_IP = "192.168.1.26"  # IP ESP32 
PORT = 3333

# логи
log_dir = "log"
os.makedirs(log_dir, exist_ok=True)  # создается папку

# создарние уникальное имя файла по времени 
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
log_path = os.path.join(log_dir, f"log_{timestamp}.csv")

# открытие файл для записи
log_file = open(log_path, "w", encoding="utf-8")
log_file.write("time,data\n")

print(f"🧾 Логирование в файл: {log_path}")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", PORT))

print(f" Connecting to ESP32 ({ESP32_IP}) ...")

# Отправляем команду на запуск
sock.sendto(b"START", (ESP32_IP, PORT))
print("🚀 Sent START command to ESP32\n")

try:
    while True:
        data, addr = sock.recvfrom(1024)
        msg = data.decode().strip()
        current_time = time.strftime("%H:%M:%S")

        print(f"📨 {current_time} | {msg}")

        # сохраняем данные в лог
        log_file.write(f"{current_time},{msg}\n")
        log_file.flush()  # сбрасываем буфер (на случай внезапного завершения)

except KeyboardInterrupt:
    print("\n🛑 Stopping...")
    sock.sendto(b"STOP", (ESP32_IP, PORT))

finally:
    log_file.close()
    sock.close()
    print("📁 Лог сохранён и соединение закрыто.")
