"""
Integrated Disaster Response Drone Swarm Simulation
Tüm modüllerin entegrasyonu ve afet bölgesi simülasyonu
"""

import numpy as np
import time
import logging
from typing import List, Dict, Tuple
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

from swarm_controller import SwarmController, FormationType, DroneRole, MissionType
from mission_planner import MissionPlanner, TaskPriority, TaskStatus
from obstacle_avoidance import ObstacleDetector, ObstacleAvoidance, Obstacle, ObstacleType

# Mevcut algoritmalar (projedeki)
import sys
sys.path.append('/home/claude/turkcell-drone-main')
from src.algorithms.trilateration import GeoLocator
from src.algorithms.signal_filter import RSSIKalmanFilter

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class DisasterScenario:
    """Afet Senaryosu Simülatörü"""
    
    def __init__(self, scenario_type: str = "earthquake"):
        self.scenario_type = scenario_type
        self.victims: List[Dict] = []
        self.obstacles: List[Obstacle] = []
        self.search_area = {
            'min': np.array([0, 0, 0]),
            'max': np.array([200, 200, 50])
        }
        
        self._generate_scenario()
        
    def _generate_scenario(self):
        """Afet senaryosu oluştur"""
        
        if self.scenario_type == "earthquake":
            logger.info("Deprem senaryosu oluşturuluyor...")
            
            # Enkaz altındaki kurbanlar (cep telefonu sinyalleri)
            num_victims = np.random.randint(5, 10)
            for i in range(num_victims):
                victim = {
                    'id': i,
                    'position': np.random.rand(3) * [180, 180, 0] + [10, 10, 0],
                    'signal_strength': np.random.uniform(-85, -55),  # dBm
                    'is_alive': np.random.random() > 0.2,  # %80 hayatta
                    'time_trapped': np.random.uniform(0, 24),  # saat
                }
                # Z koordinatı 0 (enkaz altında)
                victim['position'][2] = 0
                self.victims.append(victim)
                
            logger.info(f"  {num_victims} kurban konumu oluşturuldu")
            
            # Yıkılmış binalar (engeller)
            num_buildings = np.random.randint(10, 20)
            for i in range(num_buildings):
                obstacle = Obstacle(
                    position=np.random.rand(3) * [180, 180, 15] + [10, 10, 0],
                    size=np.random.rand(3) * [8, 8, 10] + [3, 3, 5],
                    obstacle_type=ObstacleType.BUILDING
                )
                self.obstacles.append(obstacle)
                
            logger.info(f"  {num_buildings} bina enkazı oluşturuldu")
            
    def get_victim_signals(self, drone_positions: List[np.ndarray]) -> List[Dict]:
        """
        Drone'ların tespit edebileceği kurban sinyallerini döndür
        Args:
            drone_positions: Drone pozisyonları
        Returns:
            [{'victim_id': 0, 'rssi': -65, 'drone_id': 1}, ...]
        """
        detections = []
        
        for drone_id, drone_pos in enumerate(drone_positions):
            for victim in self.victims:
                if not victim['is_alive']:
                    continue
                    
                # Mesafe hesapla
                distance = np.linalg.norm(drone_pos - victim['position'])
                
                # RSSI hesapla (mesafe arttıkça azalır)
                base_rssi = victim['signal_strength']
                path_loss = 20 * np.log10(distance + 1)  # Free space path loss
                rssi = base_rssi - path_loss + np.random.normal(0, 3)  # Gürültü ekle
                
                # Algılama eşiği (çok zayıf sinyaller alınamaz)
                if rssi > -95 and distance < 50:
                    detections.append({
                        'victim_id': victim['id'],
                        'rssi': rssi,
                        'drone_id': drone_id,
                        'distance': distance
                    })
                    
        return detections


class IntegratedSwarmSystem:
    """Entegre Sürü Drone Sistemi"""
    
    def __init__(self, 
                 num_drones: int = 6,
                 scenario_type: str = "earthquake"):
        
        # Senaryo
        self.scenario = DisasterScenario(scenario_type)
        
        # Ana modüller
        self.swarm = SwarmController(
            num_drones=num_drones,
            mission_type=MissionType.SEARCH_AND_RESCUE
        )
        
        self.planner = MissionPlanner(
            search_area_bounds=(
                self.scenario.search_area['min'],
                self.scenario.search_area['max']
            )
        )
        
        self.obstacle_detector = ObstacleDetector(detection_range=30.0)
        self.obstacle_avoidance = ObstacleAvoidance()
        
        # Orijinal algoritmalar
        self.geo_locator = GeoLocator()
        self.kalman_filters = {i: RSSIKalmanFilter() for i in range(num_drones)}
        
        # İstatistikler
        self.victims_found = []
        self.mission_time = 0
        self.total_distance_traveled = 0
        
        logger.info("Entegre sürü sistemi başlatıldı")
        
    def initialize_mission(self):
        """Görevi başlat"""
        logger.info("=" * 60)
        logger.info("AFET BÖLGESI ARAMA KURTARMA GÖREVI BAŞLATILDI")
        logger.info("=" * 60)
        
        # İlk tarama deseni oluştur
        self.planner.generate_search_pattern_tasks(grid_size=4)
        
        # Formasyonu ayarla
        self.swarm.set_formation(FormationType.SEARCH_PATTERN)
        
        # Engelleri sisteme yükle
        self.obstacle_detector.detected_obstacles = self.scenario.obstacles.copy()
        
        logger.info(f"📋 {len(self.planner.tasks)} arama görevi oluşturuldu")
        logger.info(f"🏚️ {len(self.scenario.obstacles)} engel tespit edildi")
        logger.info(f"👥 {len(self.scenario.victims)} kurban konumu (bilinmeyen)")
        
    def simulation_step(self, dt: float = 0.1):
        """Bir simülasyon adımı"""
        
        # 1. Sürüyü güncelle
        self.swarm.update_swarm(dt)
        
        # 2. Sinyal taraması yap
        drone_positions = [d.position for d in self.swarm.drones if d.is_active]
        signal_detections = self.scenario.get_victim_signals(drone_positions)
        
        # 3. Sinyalleri filtrele ve analiz et
        if signal_detections:
            self._process_signal_detections(signal_detections)
            
        # 4. Engel kaçınma
        for drone in self.swarm.drones:
            if not drone.is_active:
                continue
                
            # Engellere yakın mı?
            avoidance_force = self.obstacle_avoidance.calculate_avoidance_vector(
                drone.position,
                drone.velocity,
                self.obstacle_detector.detected_obstacles
            )
            
            # Kuvveti uygula
            if np.linalg.norm(avoidance_force) > 0.1:
                drone.velocity += avoidance_force * dt * 0.5
                
        # 5. Görev atama
        if self.mission_time % 5 < dt:  # Her 5 saniyede bir
            self._update_task_assignments()
            
        # 6. İstatistikleri güncelle
        self.mission_time += dt
        
    def _process_signal_detections(self, detections: List[Dict]):
        """Sinyal tespitlerini işle"""
        
        # Aynı kurbandan gelen sinyalleri grupla
        victim_signals = {}
        for det in detections:
            vid = det['victim_id']
            if vid not in victim_signals:
                victim_signals[vid] = []
            victim_signals[vid].append(det)
            
        # Her kurban için trilaterasyon yap
        for victim_id, signals in victim_signals.items():
            if len(signals) < 3:
                continue  # En az 3 drone gerekli
                
            # Filtrelenmiş RSSI değerleri
            drone_data = []
            for sig in signals[:3]:  # İlk 3 drone
                # Kalman filtresi uygula
                filtered_rssi = self.kalman_filters[sig['drone_id']].update(sig['rssi'])
                
                drone_data.append({
                    'pos': self.swarm.drones[sig['drone_id']].position[:2],  # 2D
                    'rssi': filtered_rssi
                })
                
            # Konum tahmini
            try:
                estimated_pos_2d = self.geo_locator.locate_target(drone_data)
                estimated_pos = np.array([estimated_pos_2d[0], estimated_pos_2d[1], 0])
                
                # Gerçek pozisyonla karşılaştır
                true_pos = self.scenario.victims[victim_id]['position']
                error = np.linalg.norm(estimated_pos - true_pos)
                
                if error < 5.0:  # 5 metre tolerans
                    if victim_id not in [v['id'] for v in self.victims_found]:
                        self.victims_found.append({
                            'id': victim_id,
                            'position': estimated_pos,
                            'error': error,
                            'time': self.mission_time
                        })
                        logger.info(f"KURBAN BULUNDU! ID: {victim_id}, "
                                  f"Konum: {estimated_pos[:2]}, Hata: {error:.2f}m")
                        
                        # Yüksek öncelikli görev oluştur
                        self.planner.create_task(
                            task_type="signal_detection",
                            target_location=estimated_pos,
                            signal_strength=signals[0]['rssi'],
                            required_drones=2
                        )
            except Exception as e:
                logger.debug(f"Trilaterasyon hatası: {e}")
                
    def _update_task_assignments(self):
        """Görevleri drone'lara ata"""
        
        # Aktif drone'ları listele
        drones_info = []
        for drone in self.swarm.drones:
            if drone.is_active:
                # Drone serbest mi? (hedefe çok yakınsa serbest sayılır)
                is_free = drone.get_distance_to(drone.target_position) < 3.0
                
                drones_info.append({
                    'id': drone.id,
                    'position': drone.position,
                    'battery': drone.battery_level,
                    'is_free': is_free
                })
                
        # Görevleri ata
        assignments = self.planner.assign_tasks_to_drones(drones_info)
        
        # Atanan görevleri drone'lara uygula
        for drone_id, task in assignments.items():
            drone = self.swarm.drones[drone_id]
            drone.target_position = task.target_location
            task.status = TaskStatus.IN_PROGRESS
            
    def run_simulation(self, duration: float = 300, visualize: bool = True):
        """
        Simülasyonu çalıştır
        Args:
            duration: Simülasyon süresi (saniye)
            visualize: Görselleştirme açık mı?
        """
        
        self.initialize_mission()
        
        if visualize:
            self._run_with_visualization(duration)
        else:
            self._run_headless(duration)
            
    def _run_headless(self, duration: float):
        """Görselleştirme olmadan çalıştır"""
        
        dt = 0.1
        steps = int(duration / dt)
        
        for step in range(steps):
            self.simulation_step(dt)
            
            # Her 10 saniyede rapor
            if step % 100 == 0:
                self._print_status_report()
                
        self._print_final_report()
        
    def _print_status_report(self):
        """Durum raporu yazdır"""
        status = self.swarm.get_swarm_status()
        mission_stats = self.planner.get_mission_statistics()
        
        logger.info(f"\n{'='*60}")
        logger.info(f"Geçen Süre: {self.mission_time:.1f}s")
        logger.info(f"Aktif Drone: {status['active_drones']}/{status['total_drones']}")
        logger.info(f"Ortalama Batarya: %{status['average_battery']:.1f}")
        logger.info(f"Görevler: {mission_stats['status_breakdown']}")
        logger.info(f"Bulunan Kurban: {len(self.victims_found)}/{len(self.scenario.victims)}")
        logger.info(f"{'='*60}\n")
        
    def _print_final_report(self):
        """Final raporu"""
        logger.info("\n" + "="*60)
        logger.info("GÖREV TAMAMLANDI - FINAL RAPOR")
        logger.info("="*60)
        
        logger.info(f"Toplam Süre: {self.mission_time:.1f} saniye")
        logger.info(f"Bulunan Kurbanlar: {len(self.victims_found)}/{len(self.scenario.victims)}")
        logger.info(f"Başarı Oranı: %{100 * len(self.victims_found) / len(self.scenario.victims):.1f}")
        
        if self.victims_found:
            avg_error = np.mean([v['error'] for v in self.victims_found])
            logger.info(f"Ortalama Konum Hatası: {avg_error:.2f} metre")
            
            avg_time = np.mean([v['time'] for v in self.victims_found])
            logger.info(f"Ortalama Bulma Süresi: {avg_time:.1f} saniye")
            
        mission_stats = self.planner.get_mission_statistics()
        logger.info(f"Toplam Görev: {mission_stats['total_tasks']}")
        logger.info(f"Tamamlanan: {mission_stats['status_breakdown']['completed']}")
        
        logger.info("="*60 + "\n")
        
    def _run_with_visualization(self, duration: float):
        """3D görselleştirme ile çalıştır"""
        
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        dt = 0.1
        frame_count = [0]
        
        def update(frame):
            self.simulation_step(dt)
            frame_count[0] += 1
            
            ax.clear()
            
            # Sınırları çiz
            ax.set_xlim(0, 200)
            ax.set_ylim(0, 200)
            ax.set_zlim(0, 50)
            ax.set_xlabel('X (metre)')
            ax.set_ylabel('Y (metre)')
            ax.set_zlabel('Z (metre)')
            ax.set_title(f'Afet Bölgesi Drone Sürüsü - Zaman: {self.mission_time:.1f}s')
            
            # Drone'ları çiz
            for drone in self.swarm.drones:
                if drone.is_active:
                    color = 'red' if drone.role == DroneRole.LEADER else 'blue'
                    marker = 'o' if drone.role != DroneRole.SCOUT else '^'
                    ax.scatter(*drone.position, c=color, marker=marker, s=100, 
                             label=f'Drone {drone.id}' if frame_count[0] == 1 else '')
                    
                    # Hedef çizgisi
                    ax.plot([drone.position[0], drone.target_position[0]],
                           [drone.position[1], drone.target_position[1]],
                           [drone.position[2], drone.target_position[2]],
                           'g--', alpha=0.3, linewidth=0.5)
                    
            # Engelleri çiz
            for obs in self.scenario.obstacles[:20]:  # İlk 20 engel
                ax.scatter(*obs.position, c='gray', marker='s', s=50, alpha=0.3)
                
            # Kurbanları çiz
            for victim in self.scenario.victims:
                if victim['is_alive']:
                    ax.scatter(*victim['position'], c='yellow', marker='*', 
                             s=200, edgecolors='red', linewidths=2)
                    
            # Bulunan kurbanları çiz
            for found in self.victims_found:
                ax.scatter(*found['position'], c='green', marker='o', 
                         s=150, alpha=0.7)
                
            # İstatistikler
            status_text = (f"Aktif: {self.swarm.get_swarm_status()['active_drones']}, "
                          f"Bulunan: {len(self.victims_found)}/{len(self.scenario.victims)}")
            ax.text2D(0.05, 0.95, status_text, transform=ax.transAxes)
            
            # Rapor
            if frame_count[0] % 100 == 0:
                self._print_status_report()
                
        # Animasyon
        frames = int(duration / dt)
        anim = FuncAnimation(fig, update, frames=frames, interval=int(dt*1000), repeat=False)
        
        plt.show()
        
        self._print_final_report()


if __name__ == "__main__":
    # Ana simülasyon
    system = IntegratedSwarmSystem(num_drones=8, scenario_type="earthquake")
    
    # Simülasyonu çalıştır (5 dakika)
    system.run_simulation(duration=300, visualize=False)  # True yaparak 3D görselleştirme
