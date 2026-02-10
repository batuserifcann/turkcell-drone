import time
import numpy as np
from src.algorithms.swarm_intelligence import BoidSwarm
from src.algorithms.signal_filter import RSSIKalmanFilter
from src.algorithms.trilateration import GeoLocator

def run_simulation():
    print("🚀 Swarm-Link: Otonom Sürü ve İstihbarat Simülasyonu Başlatılıyor...\n")
    
    # 1. Modülleri Başlat
    swarm = BoidSwarm(num_drones=3)
    kf = RSSIKalmanFilter()
    locator = GeoLocator()
    
    # Sanal Hedef Konumu (Gizli)
    target_true_pos = np.array([55.0, 45.0])
    print(f"📍 Gerçek Hedef Konumu (Simüle): {target_true_pos}")
    print("-" * 50)

    # 10 Adımlık Simülasyon Döngüsü
    for step in range(1, 11):
        # A. Sürüyü Hareket Ettir (Boid Algorithm)
        swarm.update()
        drone_positions = swarm.positions
        
        # B. Sensör Verisi Simüle Et (RSSI + Gürültü)
        sensor_data = []
        raw_rssi_log = []
        
        for i, pos in enumerate(drone_positions):
            # Gerçek mesafe
            true_dist = np.linalg.norm(pos - target_true_pos)
            
            # RSSI Hesapla (Gürültü ekle)
            # Mesafe arttıkça sinyal düşer + Random Noise
            raw_rssi = -50 - (10 * 2 * np.log10(true_dist)) + np.random.normal(0, 2)
            
            # C. Kalman Filtresi ile Temizle (Sadece Drone 0 için örnek)
            filtered_rssi = raw_rssi
            if i == 0:
                filtered_rssi = kf.update(raw_rssi)
                raw_rssi_log.append(f"Raw: {raw_rssi:.2f} -> Filtered: {filtered_rssi:.2f}")

            sensor_data.append({'pos': pos, 'rssi': filtered_rssi})

        # D. Konum Hesapla (Trilateration)
        estimated_pos = locator.locate_target(sensor_data)
        error = np.linalg.norm(estimated_pos - target_true_pos)

        print(f"⏱️ Adım {step}:")
        print(f"   🚁 Drone Konumları: {np.round(drone_positions[0], 1)} ...")
        print(f"   📶 Sinyal (Drone 1): {raw_rssi_log[0]}")
        print(f"   🎯 Tahmini Hedef: {np.round(estimated_pos, 2)}")
        print(f"   ⚠️ Sapma Payı: {error:.2f} metre")
        print("-" * 30)
        
        time.sleep(0.5)

    print("\n✅ Simülasyon Başarıyla Tamamlandı.")

if __name__ == "__main__":
    run_simulation()