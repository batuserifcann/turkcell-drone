import numpy as np

class BoidSwarm:
    """
    Kurallar:
    1. Separation (Ayrılma): Diğer drone'lara çarpmamak için mesafe koy.
    2. Alignment (Hizalanma): Sürünün ortalama yönüne dön.
    3. Cohesion (Bütünleşme): Sürünün merkezine doğru uç.
    """

    def __init__(self, num_drones=3, width=100, height=100):
        self.num_drones = num_drones
        self.width = width
        self.height = height
        
        # Konumlar (x, y) ve Hızlar (vx, vy) rastgele başlatılır
        self.positions = np.random.rand(num_drones, 2) * width
        self.velocities = np.random.rand(num_drones, 2) * 10 - 5
        
        # Fizik Limitleri
        self.max_speed = 5.0
        self.perception_radius = 50.0 # Drone'un diğerlerini görme mesafesi

    def update(self):
        """
        Her adımda sürü davranışını günceller.
        """
        for i in range(self.num_drones):
            # 3 Temel Kural Vektörleri
            v1 = self.rule1_cohesion(i)
            v2 = self.rule2_separation(i)
            v3 = self.rule3_alignment(i)

            # Hız güncelleme (Rules + Inertia)
            self.velocities[i] += v1 + v2 + v3
            
            # Hız Limiti Uygula (Limit Speed)
            speed = np.linalg.norm(self.velocities[i])
            if speed > self.max_speed:
                self.velocities[i] = (self.velocities[i] / speed) * self.max_speed

            # Konum güncelleme
            self.positions[i] += self.velocities[i]
            
            # Sınır kontrolü (Sonsuz döngü / Torus World)
            self.positions[i] %= [self.width, self.height]

    def rule1_cohesion(self, boid_index):
        """Kural 1: Sürünün kütle merkezine git."""
        center_of_mass = np.zeros(2)
        count = 0
        
        for i in range(self.num_drones):
            if i != boid_index:
                dist = np.linalg.norm(self.positions[i] - self.positions[boid_index])
                if dist < self.perception_radius:
                    center_of_mass += self.positions[i]
                    count += 1
        
        if count > 0:
            center_of_mass /= count
            return (center_of_mass - self.positions[boid_index]) / 100  # %1 Güçle çek
        return np.zeros(2)

    def rule2_separation(self, boid_index):
        """Kural 2: Diğer drone'lara çok yaklaşma (Çarpışma Önleyici)."""
        move_vector = np.zeros(2)
        
        for i in range(self.num_drones):
            if i != boid_index:
                dist = np.linalg.norm(self.positions[i] - self.positions[boid_index])
                if dist < 10:  # 10 metreden fazla yaklaşırsa it
                    move_vector -= (self.positions[i] - self.positions[boid_index])
        
        return move_vector

    def rule3_alignment(self, boid_index):
        """Kural 3: Sürünün gittiği yöne dön."""
        avg_velocity = np.zeros(2)
        count = 0
        
        for i in range(self.num_drones):
            if i != boid_index:
                dist = np.linalg.norm(self.positions[i] - self.positions[boid_index])
                if dist < self.perception_radius:
                    avg_velocity += self.velocities[i]
                    count += 1
                    
        if count > 0:
            avg_velocity /= count
            return (avg_velocity - self.velocities[boid_index]) / 8
        return np.zeros(2)