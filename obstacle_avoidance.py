"""
Obstacle Detection and Avoidance System
Engel algılama ve otonom kaçınma algoritmaları
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class ObstacleType(Enum):
    """Engel Tipleri"""
    BUILDING = "building"
    TREE = "tree"
    POWER_LINE = "power_line"
    TERRAIN = "terrain"
    OTHER_DRONE = "other_drone"
    UNKNOWN = "unknown"


@dataclass
class Obstacle:
    """Engel Tanımı"""
    position: np.ndarray  # [x, y, z]
    size: np.ndarray      # [width, height, depth] veya radius
    obstacle_type: ObstacleType
    confidence: float = 1.0  # 0-1 arası güven skoru
    is_dynamic: bool = False  # Hareket eden engel mi?
    velocity: np.ndarray = None  # Dinamik engeller için
    
    def __post_init__(self):
        if self.velocity is None:
            self.velocity = np.zeros(3)
            
    def get_bounding_sphere_radius(self) -> float:
        """Engeli çevreleyen kürenin yarıçapı"""
        if len(self.size) == 1:
            return self.size[0]  # Zaten yarıçap
        return np.linalg.norm(self.size) / 2  # Kutu için


class ObstacleDetector:
    """Engel Algılama Sistemi"""
    
    def __init__(self, detection_range: float = 30.0):
        self.detection_range = detection_range
        self.detected_obstacles: List[Obstacle] = []
        
        # Sensör parametreleri
        self.sensor_fov = np.radians(120)  # Field of view (derece)
        self.sensor_accuracy = 0.95
        self.min_obstacle_size = 0.5  # metre
        
        logger.info(f"🔍 Engel algılama sistemi başlatıldı (Menzil: {detection_range}m)")
        
    def detect_obstacles_lidar(self, 
                              drone_position: np.ndarray,
                              drone_heading: np.ndarray,
                              point_cloud: np.ndarray) -> List[Obstacle]:
        """
        LiDAR nokta bulutundan engel algıla
        Args:
            drone_position: Drone pozisyonu [x, y, z]
            drone_heading: Yönelim vektörü
            point_cloud: Nx3 nokta bulutu verisi
        Returns:
            Algılanan engeller listesi
        """
        obstacles = []
        
        # Nokta bulutunu kümelere ayır (DBSCAN benzeri)
        clusters = self._cluster_points(point_cloud, eps=1.0, min_points=5)
        
        for cluster in clusters:
            if len(cluster) < 5:
                continue
                
            # Kümenin merkezini ve boyutunu hesapla
            center = np.mean(cluster, axis=0)
            size = np.max(cluster, axis=0) - np.min(cluster, axis=0)
            
            # Drone'a olan mesafe
            distance = np.linalg.norm(center - drone_position)
            
            if distance <= self.detection_range:
                # Engel tipini tahmin et (basitleştirilmiş)
                obstacle_type = self._classify_obstacle(size, center)
                
                obstacle = Obstacle(
                    position=center,
                    size=size,
                    obstacle_type=obstacle_type,
                    confidence=min(0.95, len(cluster) / 100.0)
                )
                obstacles.append(obstacle)
                
        return obstacles
        
    def detect_obstacles_camera(self,
                               drone_position: np.ndarray,
                               image_detections: List[Dict]) -> List[Obstacle]:
        """
        Kamera görüntüsünden engel algıla (YOLO vb.)
        Args:
            drone_position: Drone pozisyonu
            image_detections: [{'class': 'tree', 'bbox': [x,y,w,h], 'depth': 10.5}, ...]
        """
        obstacles = []
        
        for detection in image_detections:
            # Bounding box'tan 3D pozisyon tahmin et
            depth = detection.get('depth', 10.0)
            bbox = detection['bbox']
            
            # Kamera koordinatlarından dünya koordinatlarına dönüşüm
            # (Basitleştirilmiş - gerçekte kamera kalibrasyonu gerekir)
            relative_pos = np.array([
                (bbox[0] - 0.5) * depth * 0.5,
                (bbox[1] - 0.5) * depth * 0.3,
                depth
            ])
            
            world_pos = drone_position + relative_pos
            
            # Engel boyutunu bbox'tan tahmin et
            size = np.array([bbox[2] * depth * 0.5, bbox[3] * depth * 0.5, 2.0])
            
            obstacle = Obstacle(
                position=world_pos,
                size=size,
                obstacle_type=ObstacleType[detection['class'].upper()] 
                    if detection['class'].upper() in ObstacleType.__members__ 
                    else ObstacleType.UNKNOWN,
                confidence=detection.get('confidence', 0.7)
            )
            obstacles.append(obstacle)
            
        return obstacles
        
    def _cluster_points(self, points: np.ndarray, eps: float, min_points: int) -> List[np.ndarray]:
        """Basit DBSCAN benzeri nokta kümeleme"""
        if len(points) == 0:
            return []
            
        clusters = []
        visited = set()
        
        for i, point in enumerate(points):
            if i in visited:
                continue
                
            # Komşuları bul
            neighbors = self._find_neighbors(points, point, eps)
            
            if len(neighbors) < min_points:
                visited.add(i)
                continue
                
            # Yeni küme oluştur
            cluster = [point]
            visited.add(i)
            
            # Komşuları kümeye ekle
            j = 0
            while j < len(neighbors):
                neighbor_idx = neighbors[j]
                if neighbor_idx not in visited:
                    visited.add(neighbor_idx)
                    cluster.append(points[neighbor_idx])
                    
                    # Komşunun komşularını bul
                    new_neighbors = self._find_neighbors(points, points[neighbor_idx], eps)
                    if len(new_neighbors) >= min_points:
                        neighbors.extend(new_neighbors)
                j += 1
                        
            clusters.append(np.array(cluster))
            
        return clusters
        
    def _find_neighbors(self, points: np.ndarray, center: np.ndarray, radius: float) -> List[int]:
        """Yarıçap içindeki noktaları bul"""
        distances = np.linalg.norm(points - center, axis=1)
        return [i for i, d in enumerate(distances) if d <= radius]
        
    def _classify_obstacle(self, size: np.ndarray, position: np.ndarray) -> ObstacleType:
        """Engel tipini boyut ve pozisyondan tahmin et"""
        height = size[2]
        width = max(size[0], size[1])
        
        if height > 10 and width < 5:
            return ObstacleType.TREE
        elif height > 15:
            return ObstacleType.BUILDING
        elif position[2] > 10 and height < 2:
            return ObstacleType.POWER_LINE
        else:
            return ObstacleType.UNKNOWN
            
    def update_obstacle_tracking(self, 
                                new_detections: List[Obstacle],
                                time_delta: float):
        """
        Engel takibi - önceki ve yeni tespitleri eşleştir
        Args:
            new_detections: Yeni algılanan engeller
            time_delta: Zaman farkı (saniye)
        """
        # Önceki engellerle eşleştir
        matched = set()
        
        for new_obs in new_detections:
            best_match = None
            best_distance = float('inf')
            
            for i, old_obs in enumerate(self.detected_obstacles):
                if i in matched:
                    continue
                    
                # Mesafeye bak
                dist = np.linalg.norm(new_obs.position - old_obs.position)
                
                if dist < 2.0 and dist < best_distance:  # 2 metre eşik
                    best_match = i
                    best_distance = dist
                    
            if best_match is not None:
                # Engeli güncelle
                old_obs = self.detected_obstacles[best_match]
                
                # Hızı hesapla (dinamik engel tespiti)
                if time_delta > 0:
                    velocity = (new_obs.position - old_obs.position) / time_delta
                    speed = np.linalg.norm(velocity)
                    
                    if speed > 0.5:  # 0.5 m/s üstü hareket
                        old_obs.is_dynamic = True
                        old_obs.velocity = velocity * 0.7 + old_obs.velocity * 0.3  # Smoothing
                        
                # Pozisyonu güncelle
                old_obs.position = new_obs.position * 0.7 + old_obs.position * 0.3
                old_obs.confidence = min(1.0, old_obs.confidence + 0.1)
                matched.add(best_match)
            else:
                # Yeni engel ekle
                self.detected_obstacles.append(new_obs)
                
        # Eşleşmeyen eski engellerin güvenini azalt
        self.detected_obstacles = [
            obs for i, obs in enumerate(self.detected_obstacles)
            if i in matched or obs.confidence > 0.3
        ]


class ObstacleAvoidance:
    """Engel Kaçınma Algoritmaları"""
    
    def __init__(self):
        self.safety_margin = 3.0  # Minimum engel mesafesi (metre)
        self.look_ahead_time = 2.0  # Kaç saniye ileriyi kontrol et
        
    def calculate_avoidance_vector(self,
                                  drone_position: np.ndarray,
                                  drone_velocity: np.ndarray,
                                  obstacles: List[Obstacle]) -> np.ndarray:
        """
        Artificial Potential Field (APF) yöntemi ile kaçınma vektörü
        Args:
            drone_position: Drone pozisyonu
            drone_velocity: Drone hızı
            obstacles: Engeller listesi
        Returns:
            Kaçınma kuvvet vektörü
        """
        total_repulsion = np.zeros(3)
        
        for obstacle in obstacles:
            # Engele olan vektör
            obstacle_vector = drone_position - obstacle.position
            distance = np.linalg.norm(obstacle_vector)
            
            # Minimum mesafe kontrolü
            safety_distance = self.safety_margin + obstacle.get_bounding_sphere_radius()
            
            if distance < safety_distance * 3:  # Etki alanı
                # Normalize edilmiş yön
                if distance > 0.1:
                    direction = obstacle_vector / distance
                else:
                    direction = np.random.randn(3)
                    direction /= np.linalg.norm(direction)
                    
                # İtme kuvveti (mesafe azaldıkça artar)
                repulsion_strength = ((safety_distance * 3) - distance) / distance
                
                # Dinamik engellerden daha güçlü kaç
                if obstacle.is_dynamic:
                    # Çarpışma tahmini
                    relative_velocity = drone_velocity - obstacle.velocity
                    time_to_collision = distance / (np.linalg.norm(relative_velocity) + 0.1)
                    
                    if time_to_collision < self.look_ahead_time:
                        repulsion_strength *= 2.0
                        
                total_repulsion += direction * repulsion_strength
                
        return total_repulsion
        
    def velocity_obstacle_method(self,
                                drone_position: np.ndarray,
                                drone_velocity: np.ndarray,
                                target_position: np.ndarray,
                                obstacles: List[Obstacle],
                                max_speed: float = 10.0) -> np.ndarray:
        """
        Velocity Obstacle (VO) yöntemi - dinamik engeller için
        Args:
            drone_position: Drone pozisyonu
            drone_velocity: Mevcut hız
            target_position: Hedef pozisyon
            obstacles: Engeller
            max_speed: Maksimum hız
        Returns:
            Güvenli hız vektörü
        """
        
        # İstenilen hız (hedefe doğru)
        desired_velocity = target_position - drone_position
        desired_speed = np.linalg.norm(desired_velocity)
        
        if desired_speed > 0:
            desired_velocity = (desired_velocity / desired_speed) * min(desired_speed, max_speed)
        else:
            return np.zeros(3)
            
        # Her engel için yasaklı hız konilerini kontrol et
        safe_velocity = desired_velocity.copy()
        
        for obstacle in obstacles:
            if not obstacle.is_dynamic:
                continue
                
            # Göreceli pozisyon ve hız
            rel_pos = obstacle.position - drone_position
            rel_vel = obstacle.velocity - drone_velocity
            
            distance = np.linalg.norm(rel_pos)
            safety_radius = self.safety_margin + obstacle.get_bounding_sphere_radius()
            
            # Çarpışma riski var mı?
            if distance < safety_radius * 5:
                # Hız vektörünü ayarla
                if np.dot(rel_vel, rel_pos) < 0:  # Yaklaşıyor
                    # Perpendicular yöne kaç
                    perpendicular = np.cross(rel_pos, np.array([0, 0, 1]))
                    if np.linalg.norm(perpendicular) > 0.1:
                        perpendicular /= np.linalg.norm(perpendicular)
                        safe_velocity += perpendicular * max_speed * 0.5
                        
        # Hız limitini uygula
        speed = np.linalg.norm(safe_velocity)
        if speed > max_speed:
            safe_velocity = (safe_velocity / speed) * max_speed
            
        return safe_velocity
        
    def find_safe_path(self,
                      start: np.ndarray,
                      goal: np.ndarray,
                      obstacles: List[Obstacle],
                      resolution: float = 1.0) -> Optional[List[np.ndarray]]:
        """
        A* benzeri yol bulma algoritması
        Args:
            start: Başlangıç noktası
            goal: Hedef nokta
            obstacles: Engeller
            resolution: Grid çözünürlüğü
        Returns:
            Waypoint'ler listesi veya None
        """
        
        # Basit Rapidly-exploring Random Tree (RRT) yaklaşımı
        max_iterations = 500
        step_size = 2.0
        goal_threshold = 3.0
        
        # Tree başlatma
        tree = [start]
        parent = {0: None}
        
        for i in range(max_iterations):
            # Rastgele nokta seç (%10 ihtimalle hedef)
            if np.random.random() < 0.1:
                random_point = goal
            else:
                random_point = np.random.rand(3) * 100  # Rastgele alan
                
            # En yakın tree node'unu bul
            nearest_idx = min(
                range(len(tree)),
                key=lambda idx: np.linalg.norm(tree[idx] - random_point)
            )
            nearest = tree[nearest_idx]
            
            # Yeni nokta oluştur (step_size kadar ilerle)
            direction = random_point - nearest
            distance = np.linalg.norm(direction)
            
            if distance > 0:
                direction /= distance
                new_point = nearest + direction * min(step_size, distance)
                
                # Engel kontrolü
                if self._is_path_collision_free(nearest, new_point, obstacles):
                    tree.append(new_point)
                    parent[len(tree) - 1] = nearest_idx
                    
                    # Hedefe ulaştık mı?
                    if np.linalg.norm(new_point - goal) < goal_threshold:
                        # Yolu geri izle
                        path = [goal, new_point]
                        current_idx = len(tree) - 1
                        
                        while parent[current_idx] is not None:
                            current_idx = parent[current_idx]
                            path.append(tree[current_idx])
                            
                        path.reverse()
                        logger.info(f"✅ Güvenli yol bulundu: {len(path)} waypoint")
                        return path
                        
        logger.warning(" Güvenli yol bulunamadı!")
        return None
        
    def _is_path_collision_free(self,
                                start: np.ndarray,
                                end: np.ndarray,
                                obstacles: List[Obstacle]) -> bool:
        """İki nokta arasındaki yolda engel var mı?"""
        
        # Yolu küçük adımlara böl
        direction = end - start
        distance = np.linalg.norm(direction)
        
        if distance < 0.1:
            return True
            
        num_steps = int(distance / 0.5) + 1
        
        for i in range(num_steps):
            point = start + direction * (i / num_steps)
            
            # Her engeli kontrol et
            for obstacle in obstacles:
                dist_to_obstacle = np.linalg.norm(point - obstacle.position)
                safety_dist = self.safety_margin + obstacle.get_bounding_sphere_radius()
                
                if dist_to_obstacle < safety_dist:
                    return False
                    
        return True


if __name__ == "__main__":
    # Test
    detector = ObstacleDetector(detection_range=30.0)
    avoidance = ObstacleAvoidance()
    
    # Simüle engeller
    obstacles = [
        Obstacle(
            position=np.array([15, 10, 10]),
            size=np.array([3, 3, 12]),
            obstacle_type=ObstacleType.BUILDING
        ),
        Obstacle(
            position=np.array([25, 15, 8]),
            size=np.array([1.5]),
            obstacle_type=ObstacleType.TREE
        ),
        Obstacle(
            position=np.array([20, 20, 15]),
            size=np.array([2, 2, 2]),
            obstacle_type=ObstacleType.OTHER_DRONE,
            is_dynamic=True,
            velocity=np.array([1, 0, 0])
        )
    ]
    
    # Drone durumu
    drone_pos = np.array([10, 10, 15])
    drone_vel = np.array([2, 1, 0])
    target = np.array([30, 25, 15])
    
    # Kaçınma vektörü hesapla
    avoidance_force = avoidance.calculate_avoidance_vector(
        drone_pos, drone_vel, obstacles
    )
    print(f"Kaçınma kuvveti: {avoidance_force}")
    
    # Güvenli yol bul
    path = avoidance.find_safe_path(drone_pos, target, obstacles)
    if path:
        print(f"\nGüvenli yol bulundu ({len(path)} waypoint):")
        for i, wp in enumerate(path):
            print(f"  {i}: {wp}")
