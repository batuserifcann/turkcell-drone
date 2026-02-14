"""
Swarm Controller - Ana Sürü Yönetim Sistemi
Dinamik sayıda drone ile afet bölgesi operasyonları için tasarlanmış
"""

import numpy as np
import time
from enum import Enum
from typing import List, Dict, Tuple, Optional
import logging

# Logging ayarları
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class MissionType(Enum):
    """Görev Tipleri"""
    SEARCH_AND_RESCUE = "search_and_rescue"  # Arama kurtarma
    SIGNAL_DETECTION = "signal_detection"    # Sinyal tespiti
    AREA_MAPPING = "area_mapping"            # Alan haritalama
    FORMATION_FLIGHT = "formation_flight"    # Formasyon uçuşu


class FormationType(Enum):
    """Formasyon Tipleri"""
    V_SHAPE = "v_shape"
    GRID = "grid"
    CIRCLE = "circle"
    LINE = "line"
    SEARCH_PATTERN = "search_pattern"


class DroneRole(Enum):
    """Drone Rolleri"""
    LEADER = "leader"
    FOLLOWER = "follower"
    SCOUT = "scout"      # Keşif dronu
    RELAY = "relay"      # İletişim aktarıcı


class DroneStatus:
    """Tek bir drone'un durumu"""
    
    def __init__(self, drone_id: int, initial_position: np.ndarray):
        self.id = drone_id
        self.position = np.array(initial_position, dtype=float)  # [x, y, z]
        self.velocity = np.zeros(3)
        self.role = DroneRole.FOLLOWER
        self.battery_level = 100.0
        self.is_active = True
        self.formation_offset = np.zeros(3)
        self.target_position = self.position.copy()
        self.obstacles_detected = []
        self.last_update_time = time.time()
        
    def update_position(self, new_position: np.ndarray, dt: float = 0.1):
        """Pozisyon güncelle ve hız hesapla"""
        if dt > 0:
            self.velocity = (new_position - self.position) / dt
        self.position = np.array(new_position, dtype=float)
        self.last_update_time = time.time()
        
    def get_distance_to(self, target: np.ndarray) -> float:
        """Hedef noktaya olan mesafe"""
        return np.linalg.norm(self.position - target)
    
    def update_battery(self, consumption_rate: float = 0.1):
        """Batarya seviyesini güncelle"""
        self.battery_level = max(0, self.battery_level - consumption_rate)
        if self.battery_level < 20:
            logger.warning(f"Drone {self.id}: Düşük batarya! (%{self.battery_level:.1f})")


class SwarmController:
    """Ana Sürü Kontrolcüsü"""
    
    def __init__(self, num_drones: int = 5, mission_type: MissionType = MissionType.SEARCH_AND_RESCUE):
        self.num_drones = num_drones
        self.mission_type = mission_type
        self.drones: List[DroneStatus] = []
        self.formation_type = FormationType.V_SHAPE
        
        # Sürü parametreleri
        self.separation_distance = 5.0  # Minimum mesafe (metre)
        self.formation_spacing = 10.0   # Formasyon aralığı
        self.max_speed = 15.0           # Maksimum hız (m/s)
        self.perception_radius = 50.0   # Algılama mesafesi
        
        # Mission alanı
        self.search_area = {
            'min': np.array([0, 0, 0]),
            'max': np.array([200, 200, 50])
        }
        
        # Leader pozisyonu
        self.leader_position = np.array([100.0, 100.0, 20.0])
        self.leader_velocity = np.zeros(3)
        
        # İstatistikler
        self.mission_start_time = time.time()
        self.area_covered = 0.0
        self.signals_detected = []
        
        self._initialize_swarm()
        
    def _initialize_swarm(self):
        """Sürüyü başlat"""
        logger.info(f"{self.num_drones} drone'lu sürü başlatılıyor...")
        
        # İlk drone'u lider yap
        initial_pos = self.leader_position.copy()
        leader = DroneStatus(0, initial_pos)
        leader.role = DroneRole.LEADER
        self.drones.append(leader)
        
        # Diğer drone'ları başlat
        for i in range(1, self.num_drones):
            # Rastgele ama yakın pozisyonlarda başlat
            offset = np.random.randn(3) * 5
            offset[2] = abs(offset[2])  # Z pozitif
            pos = initial_pos + offset
            
            drone = DroneStatus(i, pos)
            
            # Her 5 drone'da bir keşif dronu
            if i % 5 == 0:
                drone.role = DroneRole.SCOUT
            
            self.drones.append(drone)
            
        logger.info(f"Sürü hazır: {len(self.drones)} drone aktif")
        
    def set_formation(self, formation_type: FormationType):
        """Formasyon tipini değiştir"""
        self.formation_type = formation_type
        logger.info(f"Formasyon değiştirildi: {formation_type.value}")
        self._update_formation_offsets()
        
    def _update_formation_offsets(self):
        """Her drone için formasyon offsetlerini hesapla"""
        spacing = self.formation_spacing
        
        if self.formation_type == FormationType.V_SHAPE:
            # V formasyonu
            for i, drone in enumerate(self.drones[1:], 1):
                side = 1 if i % 2 == 0 else -1
                row = (i + 1) // 2
                drone.formation_offset = np.array([
                    -row * spacing,
                    side * row * spacing * 0.7,
                    -row * 2  # Hafif alçalma
                ])
                
        elif self.formation_type == FormationType.GRID:
            # Grid formasyonu (3x3, 4x4 vs)
            cols = int(np.ceil(np.sqrt(self.num_drones)))
            for i, drone in enumerate(self.drones[1:], 1):
                row = i // cols
                col = i % cols
                drone.formation_offset = np.array([
                    -row * spacing,
                    (col - cols/2) * spacing,
                    0
                ])
                
        elif self.formation_type == FormationType.CIRCLE:
            # Dairesel formasyon
            n = len(self.drones) - 1
            for i, drone in enumerate(self.drones[1:], 1):
                angle = 2 * np.pi * i / n
                radius = spacing * 2
                drone.formation_offset = np.array([
                    radius * np.cos(angle),
                    radius * np.sin(angle),
                    0
                ])
                
        elif self.formation_type == FormationType.LINE:
            # Düz sıra
            for i, drone in enumerate(self.drones[1:], 1):
                drone.formation_offset = np.array([
                    0,
                    i * spacing,
                    0
                ])
                
        elif self.formation_type == FormationType.SEARCH_PATTERN:
            # Arama deseni - geniş alan taraması
            cols = max(3, int(np.sqrt(self.num_drones)))
            for i, drone in enumerate(self.drones[1:], 1):
                row = i // cols
                col = i % cols
                drone.formation_offset = np.array([
                    -row * spacing * 1.5,
                    (col - cols/2) * spacing * 2,
                    0
                ])
                
    def update_leader_target(self, target_position: np.ndarray, target_velocity: np.ndarray = None):
        """Lider drone'un hedef pozisyonunu güncelle"""
        self.leader_position = np.array(target_position, dtype=float)
        if target_velocity is not None:
            self.leader_velocity = np.array(target_velocity, dtype=float)
            
    def calculate_swarm_forces(self, drone: DroneStatus) -> np.ndarray:
        """
        Sürü kuvvetlerini hesapla (Boid algoritması)
        Returns: Birleşik kuvvet vektörü
        """
        
        # 1. Cohesion (Bütünleşme) - Sürünün merkezine çekim
        cohesion_force = self._cohesion_force(drone)
        
        # 2. Separation (Ayrılma) - Çarpışma önleme
        separation_force = self._separation_force(drone)
        
        # 3. Alignment (Hizalanma) - Aynı yöne gitme
        alignment_force = self._alignment_force(drone)
        
        # 4. Formation (Formasyon) - Belirlenen formasyonu koruma
        formation_force = self._formation_force(drone)
        
        # 5. Obstacle Avoidance (Engel kaçınma)
        obstacle_force = self._obstacle_avoidance_force(drone)
        
        # Ağırlıklı birleştirme
        total_force = (
            cohesion_force * 0.3 +
            separation_force * 2.0 +
            alignment_force * 0.5 +
            formation_force * 1.5 +
            obstacle_force * 3.0
        )
        
        return total_force
        
    def _cohesion_force(self, drone: DroneStatus) -> np.ndarray:
        """Sürünün kütle merkezine doğru çekim"""
        if drone.role == DroneRole.LEADER:
            return np.zeros(3)
            
        center_of_mass = np.zeros(3)
        count = 0
        
        for other in self.drones:
            if other.id != drone.id and other.is_active:
                dist = drone.get_distance_to(other.position)
                if dist < self.perception_radius:
                    center_of_mass += other.position
                    count += 1
                    
        if count > 0:
            center_of_mass /= count
            desired_direction = center_of_mass - drone.position
            return desired_direction * 0.01
            
        return np.zeros(3)
        
    def _separation_force(self, drone: DroneStatus) -> np.ndarray:
        """Çarpışma önleme - yakındaki drone'lardan uzaklaş"""
        repulsion = np.zeros(3)
        
        for other in self.drones:
            if other.id != drone.id and other.is_active:
                dist = drone.get_distance_to(other.position)
                if dist < self.separation_distance:
                    # Çok yakın! Uzaklaş
                    direction = drone.position - other.position
                    if dist > 0.1:
                        # Mesafe azaldıkça kuvvet artar
                        repulsion += direction / (dist ** 2)
                        
        return repulsion
        
    def _alignment_force(self, drone: DroneStatus) -> np.ndarray:
        """Komşu drone'larla aynı yöne git"""
        if drone.role == DroneRole.LEADER:
            return np.zeros(3)
            
        avg_velocity = np.zeros(3)
        count = 0
        
        for other in self.drones:
            if other.id != drone.id and other.is_active:
                dist = drone.get_distance_to(other.position)
                if dist < self.perception_radius:
                    avg_velocity += other.velocity
                    count += 1
                    
        if count > 0:
            avg_velocity /= count
            desired_velocity = avg_velocity - drone.velocity
            return desired_velocity * 0.1
            
        return np.zeros(3)
        
    def _formation_force(self, drone: DroneStatus) -> np.ndarray:
        """Belirlenen formasyonu koruma kuvveti"""
        if drone.role == DroneRole.LEADER:
            # Leader kendi hedefine gider
            desired_pos = self.leader_position
        else:
            # Follower'lar leader'a göre offset konumda kalır
            leader = self.drones[0]
            desired_pos = leader.position + drone.formation_offset
            
        drone.target_position = desired_pos
        direction = desired_pos - drone.position
        distance = np.linalg.norm(direction)
        
        if distance > 0.5:
            return direction / distance * min(distance, 2.0)
        return np.zeros(3)
        
    def _obstacle_avoidance_force(self, drone: DroneStatus) -> np.ndarray:
        """Engel algılama ve kaçınma"""
        avoidance = np.zeros(3)
        
        # Basit engel simülasyonu (gerçek implementasyonda sensör verisi kullanılır)
        for obstacle in drone.obstacles_detected:
            dist = drone.get_distance_to(obstacle)
            if dist < 10.0:  # 10 metre içindeki engeller
                direction = drone.position - obstacle
                if dist > 0.1:
                    avoidance += direction / (dist ** 2) * 5.0
                    
        return avoidance
        
    def update_swarm(self, dt: float = 0.1):
        """
        Tüm sürüyü güncelle
        Args:
            dt: Zaman adımı (saniye)
        """
        
        # 1. Lider pozisyonunu güncelle
        leader = self.drones[0]
        leader.update_position(self.leader_position, dt)
        leader.velocity = self.leader_velocity
        
        # 2. Her drone için kuvvetleri hesapla ve uygula
        for drone in self.drones[1:]:
            if not drone.is_active:
                continue
                
            # Sürü kuvvetlerini hesapla
            force = self.calculate_swarm_forces(drone)
            
            # Hızı güncelle (F = ma, a = F/m, m=1 kabul)
            drone.velocity += force * dt
            
            # Hız limitini uygula
            speed = np.linalg.norm(drone.velocity)
            if speed > self.max_speed:
                drone.velocity = (drone.velocity / speed) * self.max_speed
                
            # Pozisyonu güncelle
            new_position = drone.position + drone.velocity * dt
            
            # Alan sınırlarını kontrol et
            new_position = np.clip(
                new_position,
                self.search_area['min'],
                self.search_area['max']
            )
            
            drone.update_position(new_position, dt)
            drone.update_battery(0.05 * dt)
            
    def get_swarm_status(self) -> Dict:
        """Sürü durumunu döndür"""
        active_drones = sum(1 for d in self.drones if d.is_active)
        avg_battery = np.mean([d.battery_level for d in self.drones if d.is_active])
        
        return {
            'total_drones': len(self.drones),
            'active_drones': active_drones,
            'average_battery': avg_battery,
            'formation': self.formation_type.value,
            'mission_duration': time.time() - self.mission_start_time,
            'leader_position': self.drones[0].position.tolist(),
            'swarm_center': np.mean([d.position for d in self.drones if d.is_active], axis=0).tolist()
        }
        
    def assign_search_zones(self, area_bounds: Tuple[np.ndarray, np.ndarray]):
        """
        Arama bölgelerini drone'lara dağıt
        Args:
            area_bounds: (min_point, max_point) tuple
        """
        min_point, max_point = area_bounds
        area_size = max_point - min_point
        
        # Her drone için alt bölge oluştur
        active_drones = [d for d in self.drones if d.is_active and d.role != DroneRole.RELAY]
        n = len(active_drones)
        
        cols = int(np.ceil(np.sqrt(n)))
        rows = int(np.ceil(n / cols))
        
        zone_width = area_size[0] / cols
        zone_height = area_size[1] / rows
        
        logger.info(f"Alan {cols}x{rows} grid'e bölünüyor, {n} drone'a dağıtılıyor")
        
        for i, drone in enumerate(active_drones):
            row = i // cols
            col = i % cols
            
            zone_min = min_point + np.array([
                col * zone_width,
                row * zone_height,
                0
            ])
            
            zone_max = zone_min + np.array([
                zone_width,
                zone_height,
                area_size[2]
            ])
            
            logger.info(f"  Drone {drone.id}: Bölge ({zone_min[:2]}) - ({zone_max[:2]})")
            
    def emergency_landing_protocol(self, drone_id: int):
        """Acil iniş protokolü"""
        drone = self.drones[drone_id]
        logger.warning(f"Drone {drone_id} acil iniş yapıyor!")
        drone.is_active = False
        
        # Görevleri yeniden dağıt
        if drone.role == DroneRole.LEADER:
            logger.warning("Lider drone devre dışı! Yeni lider atanıyor...")
            self._assign_new_leader()
            
    def _assign_new_leader(self):
        """Yeni lider ata"""
        active_drones = [d for d in self.drones if d.is_active and d.role != DroneRole.LEADER]
        if active_drones:
            new_leader = max(active_drones, key=lambda d: d.battery_level)
            new_leader.role = DroneRole.LEADER
            # İlk pozisyona taşı
            self.drones[0], self.drones[new_leader.id] = self.drones[new_leader.id], self.drones[0]
            self.drones[0].id = 0
            logger.info(f"Yeni lider: Drone {new_leader.id}")


if __name__ == "__main__":
    # Test
    controller = SwarmController(num_drones=7)
    controller.set_formation(FormationType.V_SHAPE)
    
    for i in range(10):
        controller.update_swarm(dt=0.1)
        status = controller.get_swarm_status()
        print(f"\nAdım {i+1}: {status['active_drones']} drone aktif, "
              f"Ortalama batarya: %{status['average_battery']:.1f}")
        time.sleep(0.5)
