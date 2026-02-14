"""
Mission Planner - Afet Bölgesi Görev Planlayıcı
Dinamik görev ataması ve önceliklendirme sistemi
"""

import numpy as np
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
from enum import Enum
import heapq
import logging

logger = logging.getLogger(__name__)


class TaskPriority(Enum):
    """Görev Öncelik Seviyeleri"""
    CRITICAL = 1    # Hayati tehlike (enkaz altında sinyal)
    HIGH = 2        # Yüksek (güçlü sinyal)
    MEDIUM = 3      # Orta (zayıf sinyal)
    LOW = 4         # Düşük (alan taraması)


class TaskStatus(Enum):
    """Görev Durumları"""
    PENDING = "pending"
    ASSIGNED = "assigned"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass
class MissionTask:
    """Tek bir görev tanımı"""
    task_id: int
    task_type: str
    priority: TaskPriority
    target_location: np.ndarray
    search_radius: float
    required_drones: int = 1
    assigned_drones: List[int] = None
    status: TaskStatus = TaskStatus.PENDING
    signal_strength: float = 0.0
    estimated_duration: float = 60.0  # saniye
    creation_time: float = 0.0
    completion_time: float = None
    
    def __post_init__(self):
        if self.assigned_drones is None:
            self.assigned_drones = []
            
    def __lt__(self, other):
        """Öncelik sıralaması için"""
        return self.priority.value < other.priority.value


class MissionPlanner:
    """Görev Planlama ve Atama Sistemi"""
    
    def __init__(self, search_area_bounds: Tuple[np.ndarray, np.ndarray]):
        self.search_area_min, self.search_area_max = search_area_bounds
        self.tasks: List[MissionTask] = []
        self.completed_tasks: List[MissionTask] = []
        self.task_counter = 0
        
        # Görev önceliklendirme parametreleri
        self.signal_threshold_critical = -60  # dBm - güçlü sinyal
        self.signal_threshold_high = -75
        self.signal_threshold_medium = -85
        
        logger.info("📋 Görev planlayıcı başlatıldı")
        
    def create_task(self, 
                   task_type: str,
                   target_location: np.ndarray,
                   signal_strength: float = 0.0,
                   search_radius: float = 10.0,
                   required_drones: int = 1) -> MissionTask:
        """
        Yeni görev oluştur
        Args:
            task_type: "signal_detection", "area_search", "relay_position" vb.
            target_location: [x, y, z] hedef konum
            signal_strength: Tespit edilen sinyal gücü (dBm)
            search_radius: Arama yarıçapı (metre)
            required_drones: Gerekli drone sayısı
        """
        
        # Sinyale göre öncelik belirle
        if signal_strength >= self.signal_threshold_critical:
            priority = TaskPriority.CRITICAL
        elif signal_strength >= self.signal_threshold_high:
            priority = TaskPriority.HIGH
        elif signal_strength >= self.signal_threshold_medium:
            priority = TaskPriority.MEDIUM
        else:
            priority = TaskPriority.LOW
            
        task = MissionTask(
            task_id=self.task_counter,
            task_type=task_type,
            priority=priority,
            target_location=np.array(target_location),
            search_radius=search_radius,
            required_drones=required_drones,
            signal_strength=signal_strength,
            creation_time=np.random.random()  # Simülasyon için
        )
        
        self.tasks.append(task)
        self.task_counter += 1
        
        logger.info(f"✨ Yeni görev: #{task.task_id} - {task.task_type} "
                   f"Öncelik: {priority.name}, Konum: {target_location}")
        
        return task
        
    def assign_tasks_to_drones(self, drones_info: List[Dict]) -> Dict[int, MissionTask]:
        """
        Görevleri drone'lara ata (Hungarian Algorithm benzeri)
        Args:
            drones_info: [{'id': 0, 'position': [x,y,z], 'battery': 80, 'is_free': True}, ...]
        Returns:
            {drone_id: task} mapping
        """
        
        assignments = {}
        
        # Öncelikli görevleri al
        pending_tasks = sorted(
            [t for t in self.tasks if t.status == TaskStatus.PENDING],
            key=lambda t: (t.priority.value, -t.signal_strength)
        )
        
        # Uygun drone'ları bul
        available_drones = [d for d in drones_info if d['is_free'] and d['battery'] > 20]
        
        for task in pending_tasks:
            if len(available_drones) < task.required_drones:
                continue
                
            # Bu görev için en uygun drone'ları seç
            best_drones = self._select_best_drones_for_task(
                task, 
                available_drones, 
                task.required_drones
            )
            
            if best_drones:
                # Görevi ata
                for drone in best_drones:
                    task.assigned_drones.append(drone['id'])
                    assignments[drone['id']] = task
                    available_drones.remove(drone)
                    
                task.status = TaskStatus.ASSIGNED
                logger.info(f"✅ Görev #{task.task_id} → Drone'lar {task.assigned_drones}")
                
        return assignments
        
    def _select_best_drones_for_task(self, 
                                    task: MissionTask, 
                                    available_drones: List[Dict],
                                    count: int) -> List[Dict]:
        """Görev için en uygun drone'ları seç"""
        
        # Her drone için maliyet hesapla
        drone_costs = []
        for drone in available_drones:
            # Mesafe maliyeti
            distance = np.linalg.norm(
                np.array(drone['position']) - task.target_location
            )
            
            # Batarya maliyeti (düşük batarya = yüksek maliyet)
            battery_cost = (100 - drone['battery']) / 100
            
            # Toplam maliyet
            total_cost = distance * 0.7 + battery_cost * 0.3 * 100
            
            drone_costs.append((total_cost, drone))
            
        # En düşük maliyetli drone'ları seç
        drone_costs.sort(key=lambda x: x[0])
        return [d for _, d in drone_costs[:count]]
        
    def update_task_status(self, task_id: int, new_status: TaskStatus):
        """Görev durumunu güncelle"""
        for task in self.tasks:
            if task.task_id == task_id:
                task.status = new_status
                
                if new_status == TaskStatus.COMPLETED:
                    task.completion_time = np.random.random()
                    self.completed_tasks.append(task)
                    self.tasks.remove(task)
                    logger.info(f"🎉 Görev #{task_id} tamamlandı!")
                    
                break
                
    def generate_search_pattern_tasks(self, grid_size: int = 5) -> List[MissionTask]:
        """
        Arama deseni görevleri oluştur (Grid pattern)
        Args:
            grid_size: Grid boyutu (NxN)
        """
        area_size = self.search_area_max - self.search_area_min
        cell_width = area_size[0] / grid_size
        cell_height = area_size[1] / grid_size
        
        tasks = []
        for i in range(grid_size):
            for j in range(grid_size):
                center = self.search_area_min + np.array([
                    (i + 0.5) * cell_width,
                    (j + 0.5) * cell_height,
                    15.0  # Sabit irtifa
                ])
                
                task = self.create_task(
                    task_type="area_search",
                    target_location=center,
                    search_radius=min(cell_width, cell_height) / 2,
                    required_drones=1
                )
                tasks.append(task)
                
        logger.info(f"🗺️ {len(tasks)} arama görevi oluşturuldu ({grid_size}x{grid_size} grid)")
        return tasks
        
    def detect_signal_and_create_task(self, 
                                     drone_positions: List[np.ndarray],
                                     drone_rssi_readings: List[float]) -> Optional[MissionTask]:
        """
        Drone'lardan gelen RSSI verilerini analiz et ve sinyal kaynağı için görev oluştur
        Args:
            drone_positions: Drone pozisyonları listesi
            drone_rssi_readings: Her drone'dan ölçülen RSSI değerleri
        """
        
        if len(drone_positions) < 3:
            return None
            
        # Trilaterasyon ile sinyal kaynağını hesapla (basitleştirilmiş)
        avg_position = np.mean(drone_positions, axis=0)
        max_rssi = max(drone_rssi_readings)
        
        if max_rssi > self.signal_threshold_medium:
            # Yeni görev oluştur
            task = self.create_task(
                task_type="signal_detection",
                target_location=avg_position,
                signal_strength=max_rssi,
                search_radius=5.0,
                required_drones=3  # Üçgenleme için 3 drone
            )
            return task
            
        return None
        
    def get_mission_statistics(self) -> Dict:
        """Görev istatistiklerini döndür"""
        total_tasks = len(self.tasks) + len(self.completed_tasks)
        
        status_counts = {
            'pending': len([t for t in self.tasks if t.status == TaskStatus.PENDING]),
            'assigned': len([t for t in self.tasks if t.status == TaskStatus.ASSIGNED]),
            'in_progress': len([t for t in self.tasks if t.status == TaskStatus.IN_PROGRESS]),
            'completed': len(self.completed_tasks),
            'failed': len([t for t in self.tasks if t.status == TaskStatus.FAILED])
        }
        
        priority_counts = {
            'critical': len([t for t in self.tasks if t.priority == TaskPriority.CRITICAL]),
            'high': len([t for t in self.tasks if t.priority == TaskPriority.HIGH]),
            'medium': len([t for t in self.tasks if t.priority == TaskPriority.MEDIUM]),
            'low': len([t for t in self.tasks if t.priority == TaskPriority.LOW])
        }
        
        return {
            'total_tasks': total_tasks,
            'active_tasks': len(self.tasks),
            'status_breakdown': status_counts,
            'priority_breakdown': priority_counts,
            'completion_rate': len(self.completed_tasks) / total_tasks if total_tasks > 0 else 0
        }
        
    def optimize_task_order(self) -> List[MissionTask]:
        """
        Travelling Salesman Problem benzeri optimizasyon
        Görevleri en kısa toplam mesafe olacak şekilde sırala
        """
        if len(self.tasks) <= 1:
            return self.tasks
            
        # Basitleştirilmiş greedy yaklaşım
        optimized = []
        remaining = self.tasks.copy()
        current_pos = np.array([0, 0, 0])  # Başlangıç noktası
        
        while remaining:
            # En yakın görevi bul
            nearest = min(
                remaining,
                key=lambda t: np.linalg.norm(t.target_location - current_pos)
            )
            optimized.append(nearest)
            current_pos = nearest.target_location
            remaining.remove(nearest)
            
        logger.info(f"🎯 {len(optimized)} görev optimize edildi")
        return optimized


class AdaptiveTaskScheduler:
    """Adaptif Görev Zamanlayıcı - Batarya ve performans bazlı"""
    
    def __init__(self):
        self.task_queue = []  # Priority queue
        self.performance_history = {}  # {drone_id: [completion_times]}
        
    def add_task(self, task: MissionTask):
        """Görevi öncelik sırasına ekle"""
        heapq.heappush(self.task_queue, task)
        
    def get_next_task(self) -> Optional[MissionTask]:
        """En yüksek öncelikli görevi al"""
        if self.task_queue:
            return heapq.heappop(self.task_queue)
        return None
        
    def estimate_task_duration(self, 
                              task: MissionTask, 
                              drone_id: int,
                              current_position: np.ndarray) -> float:
        """
        Görev süresini tahmin et
        Args:
            task: Görev
            drone_id: Drone ID
            current_position: Drone'un şu anki konumu
        Returns:
            Tahmini süre (saniye)
        """
        
        # Mesafe bazlı süre
        distance = np.linalg.norm(task.target_location - current_position)
        travel_time = distance / 10.0  # 10 m/s ortalama hız
        
        # Görev bazlı süre
        task_time = task.estimated_duration
        
        # Drone performans geçmişi
        if drone_id in self.performance_history:
            avg_perf = np.mean(self.performance_history[drone_id])
            performance_factor = avg_perf / 60.0  # Normalize
        else:
            performance_factor = 1.0
            
        total_time = (travel_time + task_time) * performance_factor
        return total_time
        
    def record_completion(self, drone_id: int, duration: float):
        """Görev tamamlanma süresini kaydet"""
        if drone_id not in self.performance_history:
            self.performance_history[drone_id] = []
        self.performance_history[drone_id].append(duration)
        
        # Son 10 görevi tut
        if len(self.performance_history[drone_id]) > 10:
            self.performance_history[drone_id].pop(0)


if __name__ == "__main__":
    # Test
    import time
    
    # Alan tanımla
    search_area = (
        np.array([0, 0, 0]),
        np.array([200, 200, 50])
    )
    
    planner = MissionPlanner(search_area)
    
    # Simüle sinyal tespitleri
    signals = [
        {'pos': [50, 60, 15], 'rssi': -62},   # Kritik
        {'pos': [120, 80, 15], 'rssi': -78},  # Yüksek
        {'pos': [150, 150, 15], 'rssi': -88}, # Orta
    ]
    
    for signal in signals:
        planner.create_task(
            task_type="signal_detection",
            target_location=signal['pos'],
            signal_strength=signal['rssi'],
            required_drones=2
        )
        
    # Grid arama deseni oluştur
    planner.generate_search_pattern_tasks(grid_size=3)
    
    # Drone'ları simüle et
    drones = [
        {'id': i, 'position': [10*i, 10*i, 15], 'battery': 85, 'is_free': True}
        for i in range(6)
    ]
    
    # Görevleri ata
    assignments = planner.assign_tasks_to_drones(drones)
    print(f"\n📊 {len(assignments)} görev atandı")
    
    # İstatistikleri göster
    stats = planner.get_mission_statistics()
    print(f"\n📈 Görev İstatistikleri:")
    print(f"  Toplam: {stats['total_tasks']}")
    print(f"  Aktif: {stats['active_tasks']}")
    print(f"  Durum: {stats['status_breakdown']}")
    print(f"  Öncelik: {stats['priority_breakdown']}")
