import socket
import time

ESP32_IP = "192.168.1.26"  # IP ESP32 
PORT = 3333

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(( "", PORT))

print(f"📡 Connecting to ESP32 ({ESP32_IP}) ...")

# команда на запуск
sock.sendto(b"START", (ESP32_IP, PORT))
print("🚀 Sent START command to ESP32")


while True:
    try:
        data, addr = sock.recvfrom(1024)
        print(f"📨 {addr}: {data.decode()}")
    except KeyboardInterrupt:
        print("🛑 Stopping...")
        sock.sendto(b"STOP", (ESP32_IP, PORT))
        break

sock.close()