# Autonomous Disaster Response Drone Swarm System
 
 **Türkçe** | [English](#english-version)
 
 Afet bölgelerinde otonom sürü drone sistemi ile arama-kurtarma operasyonları için geliştirilmiş kapsamlı Python projesi.
 
 ## Proje Özellikleri
 
 ### Temel Yetenekler
 - **Dinamik Sürü Kontrolü**: 3-20+ drone ile ölçeklenebilir
 - **Otonom Formasyonlar**: V-shape, Grid, Circle, Line, Search Pattern
 - **Akıllı Görev Dağılımı**: Leader-Follower hiyerarşisi ile görev atama
 - **Engel Algılama & Kaçınma**: LiDAR/Kamera tabanlı otonom navigasyon
 - **WiFi/RF Sinyal Tespiti**: Enkaz altındaki cep telefonu sinyallerini bulma
 - **Trilaterasyon**: 3+ drone ile 3-5 metre hassasiyetinde konum belirleme
 - **Kalman Filtering**: Gürültülü sensör verilerini temizleme
 - **Gerçek Drone Desteği**: DroneKit ile Pixhawk/ArduPilot entegrasyonu
 
 ### Mimari
 
 ```
 ┌─────────────────────────────────────────────────────────────┐
 │                    Mission Commander                         │
 │              (Görev Planlama & Koordinasyon)                 │
 └─────────────┬───────────────────────────────┬───────────────┘
               │                               │
     ┌─────────▼─────────┐         ┌──────────▼──────────┐
     │  Swarm Controller │         │  Mission Planner    │
     │  (Sürü Kontrolü)  │◄────────┤  (Görev Yönetimi)   │
     └─────────┬─────────┘         └──────────┬──────────┘
               │                               │
     ┌─────────▼─────────────────────────┬────▼──────────┐
     │   Obstacle Avoidance              │  Signal Intel │
     │   (Engel Algılama & Kaçınma)      │  (SIGINT)     │
     └───────────────────────────────────┴───────────────┘
                           │
               ┌───────────▼───────────┐
               │   Real Drone Interface │
               │   (DroneKit / SITL)    │
               └───────────────────────┘
 ```
 
 ## Kurulum
 
 ### Sistem Gereksinimleri
 - Python 3.8+
 - Ubuntu 20.04+ / Raspberry Pi OS (gerçek drone için)
 - 4GB+ RAM (simülasyon için)
 
 ### 1. Temel Kurulum
 
 ```bash
 # Repository'yi clone'la
 git clone https://github.com/batuserifcann/turkcell-drone.git
 cd turkcell-drone
 
 # Virtual environment oluştur
 python3 -m venv venv
 source venv/bin/activate  # Linux/Mac
 # venv\Scripts\activate   # Windows
 
 # Bağımlılıkları kur
 pip install -r requirements.txt
 
 # Yeni sürü modüllerini kur
 pip install matplotlib scipy filterpy
 ```
 
 ### 2. DroneKit Kurulumu (Gerçek Drone için)
 
 ```bash
 pip install dronekit dronekit-sitl pymavlink
 ```
 
 ### 3. Dizin Yapısı
 
 ```
 turkcell-drone/
 ├── src/
 │   └── algorithms/
 │       ├── trilateration.py
 │       ├── signal_filter.py
 │       ├── swarm_intelligence.py
 │       └── demo_simulation.py
 ├── swarm_controller.py
 ├── mission_planner.py
 ├── obstacle_avoidance.py
 ├── integrated_simulation.py
 ├── real_drone_interface.py
 ├── requirements.txt
 └── README.md
 ```
 
 ## Kullanım
 
 ### Örnek 1: Basit Simülasyon
 
 ```python
 from swarm_controller import SwarmController, FormationType
 
 # 5 drone'lu sürü oluştur
 swarm = SwarmController(num_drones=5)
 
 # V formasyonu ayarla
 swarm.set_formation(FormationType.V_SHAPE)
 
 # Lider dronu hedefe yönlendir
 swarm.update_leader_target(
     target_position=[150, 150, 20],
     target_velocity=[2, 0, 0]
 )
 
 # Simülasyon döngüsü
 for i in range(100):
     swarm.update_swarm(dt=0.1)
     status = swarm.get_swarm_status()
     print(f"Adım {i}: {status['active_drones']} drone aktif")
 ```
 
 ### Örnek 2: Görev Planlama
 
 ```python
 from mission_planner import MissionPlanner
 import numpy as np
 
 # Alan tanımla
 search_area = (
     np.array([0, 0, 0]),     # Min
     np.array([200, 200, 50])  # Max
 )
 
 planner = MissionPlanner(search_area)
 
 # Grid arama deseni oluştur
 planner.generate_search_pattern_tasks(grid_size=5)
 
 # Sinyal tespiti görevi ekle
 planner.create_task(
     task_type="signal_detection",
     target_location=[80, 90, 15],
     signal_strength=-65,  # dBm
     required_drones=3
 )
 
 # Drone'lara görev ata
 drones_info = [
     {'id': 0, 'position': [10, 10, 15], 'battery': 85, 'is_free': True},
     {'id': 1, 'position': [15, 12, 15], 'battery': 90, 'is_free': True},
     # ...
 ]
 
 assignments = planner.assign_tasks_to_drones(drones_info)
 ```
 
 ### Örnek 3: Tam Afet Senaryosu
 
 ```python
 from integrated_simulation import IntegratedSwarmSystem
 
 # 8 drone'lu sistem oluştur
 system = IntegratedSwarmSystem(
     num_drones=8,
     scenario_type="earthquake"
 )
 
 # 5 dakikalık simülasyon (3D görselleştirme ile)
 system.run_simulation(duration=300, visualize=True)
 
 # Sonuçları göster
 ```
 
 ### Örnek 4: Gerçek Drone Kontrolü (SITL)
 
 ```bash
 # Terminal 1: SITL başlat
 dronekit-sitl copter --home=40.0,-90.0,0,180
 ```
 
 ```python
 # Terminal 2: Python script
 from real_drone_interface import RealDroneController
 
 drone = RealDroneController('127.0.0.1:14550')
 
 if drone.connect_vehicle():
     # 15 metre yüksekliğe kalk
     drone.arm_and_takeoff(15)
     
     # Kuzey-doğuya 20m git
     drone.goto_position_ned(north=20, east=20, down=-15)
     
     # İniş yap
     drone.land()
     drone.disconnect()
 ```
 
 ### Örnek 5: Sürü SITL Testi
 
 ```bash
 # 3 SITL instance başlat (3 ayrı terminal)
 dronekit-sitl copter --instance 0 --home=40.0,-90.0,0,180
 dronekit-sitl copter --instance 1 --home=40.0,-90.01,0,180
 dronekit-sitl copter --instance 2 --home=40.0,-90.02,0,180
 ```
 
 ```python
 from real_drone_interface import SwarmDroneInterface
 
 swarm = SwarmDroneInterface()
 
 # 3 drone ekle
 swarm.add_drone(0, '127.0.0.1:14550')
 swarm.add_drone(1, '127.0.0.1:14560')
 swarm.add_drone(2, '127.0.0.1:14570')
 
 # Kalkış
 swarm.takeoff_swarm(altitude=15)
 
 # V formasyonunda uç
 velocities = {
     0: [1, 0, 0],      # Leader
     1: [0.8, -0.5, 0], # Sol
     2: [0.8, 0.5, 0],  # Sağ
 }
 swarm.send_velocity_commands(velocities)
 
 # Eve dön
 swarm.return_all_to_home()
 swarm.disconnect_all()
 ```
 
 ## Modül Detayları
 
 ### 1. SwarmController (swarm_controller.py)
 
 **Sürü davranışını yöneten ana kontrolcü.**
 
 #### Özellikler:
 - **Boid Algoritması**: Cohesion, Separation, Alignment
 - **Formasyon Kontrolü**: 5 farklı formasyon tipi
 - **Leader-Follower**: Dinamik lider ataması
 - **Batarya Yönetimi**: Otomatik düşük batarya uyarıları
 - **Acil Durum**: Emergency landing ve lider değişimi
 
 #### Kullanım:
 ```python
 controller = SwarmController(num_drones=10)
 controller.set_formation(FormationType.GRID)
 controller.assign_search_zones(area_bounds)
 controller.update_swarm(dt=0.1)
 ```
 
 ### 2. MissionPlanner (mission_planner.py)
 
 **Görevleri planlar ve drone'lara atar.**
 
 #### Özellikler:
 - **Önceliklendirme**: CRITICAL, HIGH, MEDIUM, LOW
 - **Görev Tipleri**: Sinyal tespiti, alan tarama, relay
 - **Otomatik Atama**: Hungarian Algorithm benzeri
 - **Grid Üretimi**: Otomatik arama deseni oluşturma
 
 #### Kullanım:
 ```python
 planner = MissionPlanner(search_area_bounds)
 planner.create_task(task_type="signal_detection", ...)
 assignments = planner.assign_tasks_to_drones(drones_info)
 ```
 
 ### 3. ObstacleAvoidance (obstacle_avoidance.py)
 
 **Engel algılama ve kaçınma algoritmaları.**
 
 #### Özellikler:
 - **Algılama**: LiDAR nokta bulutu, Kamera (YOLO)
 - **Takip**: DBSCAN clustering, dinamik engel tespiti
 - **Kaçınma**: Artificial Potential Field (APF)
 - **Yol Bulma**: RRT (Rapidly-exploring Random Tree)
 
 #### Kullanım:
 ```python
 detector = ObstacleDetector(detection_range=30)
 avoidance = ObstacleAvoidance()
 
 obstacles = detector.detect_obstacles_lidar(position, heading, point_cloud)
 force = avoidance.calculate_avoidance_vector(position, velocity, obstacles)
 path = avoidance.find_safe_path(start, goal, obstacles)
 ```
 
 ### 4. IntegratedSimulation (integrated_simulation.py)
 
 **Tüm modüllerin bir araya geldiği ana simülasyon.**
 
 #### Özellikler:
 - **Afet Senaryoları**: Deprem, yangın vb.
 - **Sinyal Simülasyonu**: Gerçekçi RSSI değerleri
 - **Trilaterasyon**: Gerçek zamanlı konum hesaplama
 - **3D Görselleştirme**: Matplotlib ile canlı takip
 
 #### Kullanım:
 ```python
 system = IntegratedSwarmSystem(num_drones=8)
 system.run_simulation(duration=300, visualize=True)
 ```
 
 ## Algoritmalar
 
 ### 1. Boid Sürü Algoritması
 
 ```
 Cohesion:   Sürünün merkezine doğru hareket et
 Separation: Yakındaki drone'lardan uzaklaş (çarpışma önleme)
 Alignment:  Komşuların ortalama yönüne dön
 Formation:  Belirlenen offseti koru
 ```
 
 ### 2. Trilaterasyon (RSSI → Konum)
 
 ```
 3 drone'dan RSSI ölçümü:
   Drone A: (-65 dBm) → 12.5m mesafe
   Drone B: (-72 dBm) → 25.0m mesafe
   Drone C: (-68 dBm) → 18.2m mesafe
 
 Least Squares optimizasyonu ile kesişim noktası bulunur
 → Hedef pozisyonu: [x, y]
 ```
 
 ### 3. Kalman Filtering
 
 ```
 Prediction:  x̂ₖ = x̂ₖ₋₁
              Pₖ = Pₖ₋₁ + Q
 
 Update:      K = Pₖ / (Pₖ + R)
              x̂ₖ = x̂ₖ + K(zₖ - x̂ₖ)
              Pₖ = (1 - K)Pₖ
 
 Ham RSSI: -72, -68, -75, -70 dBm
 Filtre çıktı: -71.2 dBm (düzgün)
 ```
 
 ### 4. Artificial Potential Field
 
 ```
 Hedefe Çekim:   F_attract = k_att × (goal - position)
 Engelden İtme:  F_repel = k_rep × (position - obstacle) / distance²
 
 Toplam Kuvvet:  F = F_attract + Σ F_repel
 Yeni Hız:       v = v + F × dt
 ```
 
 ## Gerçek Drone Kurulumu
 
 ### Donanım
 - **Flight Controller**: Pixhawk 2.4.8 / Cube Orange
 - **Onboard Computer**: Raspberry Pi 4B (4GB+)
 - **Telemetry**: 915MHz / 433MHz radio
 - **GPS**: u-blox NEO-M8N
 - **Power**: 4S LiPo, BEC/UBEC
 
 ### Yazılım Stack
 ```
 ┌──────────────────────────┐
 │   Python Swarm Code      │
 ├──────────────────────────┤
 │   DroneKit (MAVLink)     │
 ├──────────────────────────┤
 │   ArduPilot / PX4        │
 ├──────────────────────────┤
 │   Pixhawk Firmware       │
 └──────────────────────────┘
 ```
 
 ### Bağlantı Şemaları
 
 #### USB Seri Bağlantı (Raspberry Pi ↔ Pixhawk)
 ```python
 connection_string = '/dev/ttyACM0'  # Pixhawk USB
 baud_rate = 921600
 ```
 
 #### Telemetry Radio Bağlantı
 ```python
 connection_string = '/dev/ttyUSB0'  # Telemetry
 baud_rate = 57600
 ```
 
 #### UDP WiFi Bağlantı (Ground Station)
 ```python
 connection_string = 'udp:192.168.1.100:14550'
 ```
 
 ### Raspberry Pi Üzerinde Çalıştırma
 
 ```bash
 # 1. SSH ile Raspberry Pi'ye bağlan
 ssh pi@raspberrypi.local
 
 # 2. Projeyi klonla
 git clone https://github.com/yourusername/disaster-drone-swarm
 cd disaster-drone-swarm
 
 # 3. Virtual environment
 python3 -m venv venv
 source venv/bin/activate
 
 # 4. Kurulum
 pip install -r requirements.txt
 
 # 5. Çalıştır
 sudo python3 real_drone_interface.py
 ```
 
 ## Mesh Network Entegrasyonu
 
 ### Batman-adv Setup (Raspberry Pi)
 
 ```bash
 # 1. Batman-adv modülünü yükle
 sudo modprobe batman-adv
 
 # 2. Mesh interface oluştur
 sudo batctl if add wlan0
 sudo ifconfig bat0 up
 
 # 3. IP ata
 sudo ifconfig bat0 192.168.199.10  # Her drone farklı IP
 ```
 
 ### Python'dan Mesh İletişimi
 
 ```python
 import socket
 
 # Broadcast mesaj gönder
 sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
 sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
 sock.sendto(b"SWARM_STATUS:OK", ('192.168.199.255', 5000))
 
 # Dinle
 sock.bind(('0.0.0.0', 5000))
 data, addr = sock.recvfrom(1024)
 ```
 
 ## Test Senaryoları
 
 ### Test 1: Formasyon Kontrolü
 ```bash
 python3 -c "
 from swarm_controller import SwarmController, FormationType
 swarm = SwarmController(num_drones=5)
 for formation in FormationType:
     swarm.set_formation(formation)
     print(f'Test: {formation.value}')
 "
 ```
 
 ### Test 2: Görev Atama
 ```bash
 python3 mission_planner.py
 ```
 
 ### Test 3: Engel Kaçınma
 ```bash
 python3 obstacle_avoidance.py
 ```
 
 ### Test 4: Full Stack Simülasyon
 ```bash
 python3 integrated_simulation.py
 ```
 
 ### Test 5: SITL Drone
 ```bash
 # Terminal 1
 dronekit-sitl copter
 
 # Terminal 2
 python3 real_drone_interface.py
 ```
 
 ## Performans Metrikleri
 
 ### Tipik Değerler (8 Drone, 200x200m Alan)
 
 | Metrik | Değer |
 |--------|-------|
 | Kurban Tespit Süresi | 45-90 saniye |
 | Konum Hassasiyeti | 3-5 metre |
 | Algılama Mesafesi | 30-50 metre |
 | Formasyon Hatası | <2 metre |
 | CPU Kullanımı | %30-50 (Raspberry Pi 4) |
 | Batarya Ömrü | 15-20 dakika (hovering) |
 
 ## Katkıda Bulunma
 
 1. Fork yapın
 2. Feature branch oluşturun (`git checkout -b feature/amazing`)
 3. Commit yapın (`git commit -m 'Add amazing feature'`)
 4. Push yapın (`git push origin feature/amazing`)
 5. Pull Request açın
 
 ## Lisans
 
 Bu proje MIT lisansı altında lisanslanmıştır.
 
 ## Ekip
 
 - **Orijinal Proje**: [Turkcell Drone](https://github.com/batuserifcann/turkcell-drone)
 - **Sürü Sistemi Geliştirici**: [İsminiz]
 
 ## Teşekkürler
 
 - ArduPilot ve DroneKit topluluğu
 - Turkcell Geleceği Yazanlar programı
 - Açık kaynak drone geliştirme topluluğu
 
 ---
 
 ## Sorun Giderme
 
 ### "DroneKit bulunamadı" hatası
 ```bash
 pip install dronekit dronekit-sitl
 ```
 
 ### "GPS kilidi alınamıyor" (SITL)
 ```bash
 # SITL'i GPS parametreleriyle başlat
 dronekit-sitl copter --home=40.0,-90.0,0,180
 ```
 
 ### "Bağlantı zaman aşımı"
 - Pixhawk'ın USB bağlantısını kontrol edin
 - `/dev/ttyACM0` veya `/dev/ttyUSB0` olmalı
 - Kullanıcıyı dialout grubuna ekleyin: `sudo usermod -a -G dialout $USER`
 
 ### "Permission denied" hatası
 ```bash
 sudo chmod 666 /dev/ttyACM0
 ```
 
 ## Ek Kaynaklar
 
 - [ArduPilot Docs](https://ardupilot.org/copter/)
 - [DroneKit Python](https://dronekit-python.readthedocs.io/)
 - [MAVLink Protocol](https://mavlink.io/)
 - [Raspberry Pi GPIO](https://pinout.xyz/)
 
 ---
 
 **İyi Uçuşlar!**
