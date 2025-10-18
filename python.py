import socket
import os
import time
from datetime import datetime

ESP32_IP = "192.168.1.26"
PORT = 3333

# Папка для логов
log_dir = "logs"
os.makedirs(log_dir, exist_ok=True)


timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
log_file = os.path.join(log_dir, f"flight_{timestamp}.txt")
headers = ["Time(s)", "AX", "AY", "AZ", "GX", "GY", "GZ", "AngleX", "AngleY", "AngleZ"]

with open(log_file, "w", encoding="utf-8") as f:
    f.write("=== Flight Session ===\n")
    f.write("\t".join(headers) + "\n")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", PORT))

print(f"📡 Connecting to ESP32 ({ESP32_IP}) ...")

# команда START
sock.sendto(b"START", (ESP32_IP, PORT))
print("🚀 Sent START command to ESP32")

start_time = time.time()

try:
    while True:
        data, addr = sock.recvfrom(1024)
        msg = data.decode().strip()
        print(f"📨 {msg}")
        values = {}
        for part in msg.split():
            if ":" in part:
                k, v = part.split(":")
                values[k] = v

        current_time = round(time.time() - start_time, 2)
        row = [
            str(current_time),
            values.get("AX", ""), values.get("AY", ""), values.get("AZ", ""),
            values.get("GX", ""), values.get("GY", ""), values.get("GZ", ""),
            values.get("AngleX", ""), values.get("AngleY", ""), values.get("AngleZ", "")
        ]

        # запись данных в файл 
        with open(log_file, "a", encoding="utf-8") as f:
            f.write("\t".join(row) + "\n")

except KeyboardInterrupt:
    print("\n🛑 Stopping...")
    sock.sendto(b"STOP", (ESP32_IP, PORT))
    sock.close()
    print(f"💾 Data saved to {log_file}")
