import socket

# --- Настройки ---
ESP32_IP = "192.168.1.26"  # IP ESP32
PORT = 3333     # Порт (должен совпадать)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", PORT))

print(f"🎯 Listening for data on {PORT} ...")

try:
    while True:
        data, addr = sock.recvfrom(1024)  # 1024 байта буфера
        message = data.decode("utf-8").strip()
        print(f"[{addr[0]}] {message}")

        # --- (опционально) отправка ответа ESP32 ---
        response = "PC received data ✅"
        sock.sendto(response.encode(), addr)

except KeyboardInterrupt:
    print("\n🛑 Программа остановлена пользователем.")
finally:
    sock.close()
