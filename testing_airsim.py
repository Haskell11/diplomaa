import airsim
import time

# Подключение к симулятору
client = airsim.MultirotorClient()
client.confirmConnection()

# Разблокировка и взлёт
client.enableApiControl(True)
client.armDisarm(True)

print("Взлетаем...")
client.takeoffAsync().join()
time.sleep(3)

print("Двигаемся вперёд...")
client.moveByVelocityAsync(1, 0, 0, 3).join()

print("Посадка...")
client.landAsync().join()

client.armDisarm(False)
client.enableApiControl(False)
print("✅ Завершено")
