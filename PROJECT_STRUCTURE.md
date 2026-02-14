# Proje Dosya Yapısı

## Ana Dosyalar (Yeni Geliştirilen)

### 1. `swarm_controller.py` (17 KB)
**Sürü kontrolü ve koordinasyon sistemi**

**İçindekiler:**
- `SwarmController`: Ana sürü kontrolcü sınıfı
- `DroneStatus`: Her drone'un durumu (pozisyon, batarya, rol vb.)
- `FormationType`: 5 formasyon tipi (V-shape, Grid, Circle, Line, Search)
- `DroneRole`: Leader, Follower, Scout, Relay rolleri
- `MissionType`: Görev tipleri

**Özellikler:**
- Boid algoritması (Cohesion, Separation, Alignment)
- Dinamik formasyon kontrolü
- Leader-Follower hiyerarşisi
- Batarya yönetimi
- Acil durum protokolleri
- Alan sınır kontrolü

**Kullanım:**
```python
from swarm_controller import SwarmController, FormationType
swarm = SwarmController(num_drones=8)
swarm.set_formation(FormationType.V_SHAPE)
swarm.update_swarm(dt=0.1)
```

---

### 2. `mission_planner.py` (15 KB)
**Görev planlama ve atama sistemi**

**İçindekiler:**
- `MissionPlanner`: Görev oluşturma ve atama
- `MissionTask`: Görev tanımı
- `TaskPriority`: CRITICAL, HIGH, MEDIUM, LOW
- `AdaptiveTaskScheduler`: Adaptif zamanlama

**Özellikler:**
- Öncelik bazlı görev sıralama
- Otomatik grid arama deseni
- Sinyal tespiti görevleri
- Hungarian algorithm benzeri atama
- Performans bazlı tahminleme
- Görev optimizasyonu (TSP)

**Kullanım:**
```python
from mission_planner import MissionPlanner
planner = MissionPlanner(search_area_bounds)
planner.create_task(task_type="signal_detection", ...)
assignments = planner.assign_tasks_to_drones(drones)
```

---

### 3. `obstacle_avoidance.py` (19 KB)
**Engel algılama ve kaçınma algoritmaları**

**İçindekiler:**
- `ObstacleDetector`: LiDAR/Kamera ile engel algılama
- `ObstacleAvoidance`: Kaçınma algoritmaları
- `Obstacle`: Engel tanımı
- `ObstacleType`: Bina, ağaç, enerji hattı vb.

**Özellikler:**
- DBSCAN clustering (nokta bulutu)
- Dinamik engel tespiti
- Artificial Potential Field (APF)
- Velocity Obstacle (VO) yöntemi
- RRT yol bulma
- Çarpışma tahmini

**Kullanım:**
```python
from obstacle_avoidance import ObstacleDetector, ObstacleAvoidance
detector = ObstacleDetector(detection_range=30)
avoidance = ObstacleAvoidance()
force = avoidance.calculate_avoidance_vector(pos, vel, obstacles)
```

---

### 4. `integrated_simulation.py` (17 KB)
**Tam entegre afet senaryosu simülatörü**

**İçindekiler:**
- `IntegratedSwarmSystem`: Ana entegrasyon sınıfı
- `DisasterScenario`: Afet senaryosu üretici
- 3D görselleştirme (Matplotlib)
- Gerçek zamanlı metrikler

**Özellikler:**
- Deprem senaryosu simülasyonu
- Rastgele kurban ve engel üretimi
- RSSI bazlı sinyal simülasyonu
- Trilaterasyon entegrasyonu
- Kalman filtering entegrasyonu
- Canlı 3D animasyon
- Performans raporlama

**Kullanım:**
```python
from integrated_simulation import IntegratedSwarmSystem
system = IntegratedSwarmSystem(num_drones=8)
system.run_simulation(duration=300, visualize=True)
```

---

### 5. `real_drone_interface.py` (13 KB)
**Gerçek drone kontrolü (DroneKit entegrasyonu)**

**İçindekiler:**
- `RealDroneController`: Tek drone kontrolü
- `SwarmDroneInterface`: Çoklu drone interface
- MAVLink komutları
- SITL test fonksiyonları

**Özellikler:**
- DroneKit/MAVLink entegrasyonu
- Arm/Takeoff/Land komutları
- NED koordinat sistemi
- GPS global koordinatlar
- Hız kontrolü
- Telemetri okuma
- SITL desteği

**Kullanım:**
```python
from real_drone_interface import RealDroneController
drone = RealDroneController('/dev/ttyACM0')
drone.connect_vehicle()
drone.arm_and_takeoff(15)
```

---

### 6. `examples.py` (10 KB)
**Hızlı başlangıç örnekleri**

**5 Hazır Örnek:**
1. Basit sürü kontrolü (formasyonlar)
2. Görev planlama ve atama
3. Engel algılama ve kaçınma
4. Tam afet senaryosu (60s)
5. SITL drone testi

**Kullanım:**
```bash
# İnteraktif menü
python examples.py

# Direkt örnek çalıştırma
python examples.py 1  # Örnek 1
python examples.py 4  # Örnek 4
```

---

### 7. `README.md` (15 KB)
**Kapsamlı dokümantasyon**

**İçerik:**
- Proje özellikleri
- Mimari açıklaması
- Kurulum adımları
- Kullanım örnekleri
- Algoritma detayları
- Gerçek drone setup
- Mesh network kurulumu
- Test senaryoları
- Sorun giderme

---

### 8. `requirements.txt` (2 KB)
**Python bağımlılıkları**

**Ana Paketler:**
- dronekit, pymavlink (drone kontrolü)
- numpy, scipy (matematik)
- matplotlib (görselleştirme)
- filterpy (Kalman filtresi)
- scapy (network sniffer)

**Opsiyonel:**
- opencv-python (görüntü işleme)
- ultralytics (YOLO)
- RPi.GPIO (Raspberry Pi)

---

## Dosya Boyutları ve Satır Sayıları

| Dosya | Boyut | Satır | Açıklama |
|-------|-------|-------|----------|
| swarm_controller.py | 17 KB | ~450 | Ana sürü kontrolü |
| obstacle_avoidance.py | 19 KB | ~500 | Engel algılama/kaçınma |
| integrated_simulation.py | 17 KB | ~450 | Tam simülasyon |
| mission_planner.py | 15 KB | ~400 | Görev planlama |
| real_drone_interface.py | 13 KB | ~350 | Drone interface |
| examples.py | 10 KB | ~280 | Örnek scriptler |
| README.md | 15 KB | ~500 | Dokümantasyon |
| requirements.txt | 2 KB | ~50 | Bağımlılıklar |
| **TOPLAM** | **~108 KB** | **~3000** | **8 dosya** |

---

## Dosya Bağımlılıkları

```
integrated_simulation.py
    ├── swarm_controller.py
    ├── mission_planner.py
    ├── obstacle_avoidance.py
    └── src/algorithms/
        ├── trilateration.py
        └── signal_filter.py

real_drone_interface.py
    └── swarm_controller.py (DroneStatus)

examples.py
    ├── swarm_controller.py
    ├── mission_planner.py
    ├── obstacle_avoidance.py
    ├── integrated_simulation.py
    └── real_drone_interface.py
```

---

## Hangi Dosyayı Ne Zaman Kullanmalı?

### Simülasyon Yapmak İstiyorum
- `examples.py` (Örnek 1-4)
- `integrated_simulation.py`

### Gerçek Drone Test Etmek İstiyorum
- `real_drone_interface.py`
- `examples.py` (Örnek 5)

### Sadece Formasyon Algoritması
- `swarm_controller.py`

### Sadece Görev Atama
- `mission_planner.py`

### Sadece Engel Kaçınma
- `obstacle_avoidance.py`

### Kendi Projemi Geliştirmek İstiyorum
- Tüm modülleri import et ve kullan

---

## Kod Kalitesi ve Standartlar

**Tüm kodlar:**
- PEP 8 standartlarına uygun
- Type hints kullanımı
- Docstring'ler (Google style)
- Logging sistemi
- Error handling
- Test edilebilir yapı
- Modüler tasarım

**Örnek Docstring:**
```python
def calculate_avoidance_vector(self,
                               drone_position: np.ndarray,
                               drone_velocity: np.ndarray,
                               obstacles: List[Obstacle]) -> np.ndarray:
    """
    Artificial Potential Field (APF) yöntemi ile kaçınma vektörü
    
    Args:
        drone_position: Drone pozisyonu [x, y, z]
        drone_velocity: Drone hızı [vx, vy, vz]
        obstacles: Engeller listesi
        
    Returns:
        Kaçınma kuvvet vektörü
    """
```

---

## Test Coverage

Her modül kendi test fonksiyonlarına sahip:

```bash
# Modül testleri
python swarm_controller.py
python mission_planner.py
python obstacle_avoidance.py

# Entegre testler
python examples.py

# Gerçek drone testi
python real_drone_interface.py
```

---

## Öğrenme Sırası (Önerilen)

1. **Başlangıç** → `README.md` oku
2. **Örnekler** → `examples.py` çalıştır (Örnek 1-3)
3. **Temel** → `swarm_controller.py` incele
4. **Görevler** → `mission_planner.py` incele
5. **Engeller** → `obstacle_avoidance.py` incele
6. **Entegrasyon** → `integrated_simulation.py` çalıştır
7. **Gerçek Dünya** → `real_drone_interface.py` ile SITL test

---

## En İyi Pratikler

### Geliştirme
```bash
# Virtual environment kullan
python3 -m venv venv
source venv/bin/activate

# Bağımlılıkları kur
pip install -r requirements.txt

# Önce örnekleri dene
python examples.py
```

### Test
```bash
# Simülasyon testi (hızlı)
python examples.py 1

# Tam senaryo testi (uzun)
python examples.py 4

# SITL testi (gerçek drone simülasyonu)
dronekit-sitl copter
python examples.py 5
```

### Production
```bash
# Raspberry Pi'de çalıştırma
sudo python3 real_drone_interface.py

# Logging'i dosyaya yönlendirme
python3 integrated_simulation.py > mission.log 2>&1
```

---

**Tüm dosyalar hazır ve kullanıma hazır!**
