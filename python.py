import socket

ESP32_IP = "192.168.1.26"  # IP ESP32
PORT = 3333

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", PORT))  # слушаем входящие пакеты

while True:
    # --- Получаем данные с ESP32 ---
    data, addr = sock.recvfrom(1024)
    print(f"Received from {addr}: {data.decode()}")

    # --- Отправляем команду на ESP32 ---
    sock.sendto(b"Hello ESP32!", (ESP32_IP, PORT))
