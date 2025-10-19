import sys
import os
sys.path.insert(0, os.path.abspath("airsim_client"))

try:
    import airsim
    print("✅ airsim импортирован")
    print("Путь:", airsim.__file__)
    print("Есть MultirotorClient:", hasattr(airsim, 'MultirotorClient'))
except Exception as e:
    print("❌ Ошибка:", e)