import numpy as np
from scipy.optimize import least_squares

class GeoLocator:
    """
    Multilaterasyon (Trilaterasyon) Algoritması.
    Birden fazla referans noktasından (Drone) hedefe olan uzaklıkları
    kullanarak hedefin 2D koordinatını (x, y) hesaplar.
    """
    
    def __init__(self):
        # FSPL (Free Space Path Loss) Katsayıları
        self.tx_power = -50 # 1 metredeki referans sinyal gücü (dBm)
        self.n_factor = 2.0 # Ortam katsayısı (2.0 = Açık alan, 3.0 = Şehir)

    def rssi_to_distance(self, rssi):
        """
        RSSI (dBm) değerini Metreye çevirir.
        Formula: d = 10 ^ ((TxPower - RSSI) / (10 * n))
        """
        return 10 ** ((self.tx_power - rssi) / (10 * self.n_factor))

    def _error_function(self, target_pos, drones_pos, distances):
        """
        Optimizasyon için hata fonksiyonu.
        Hedef ile drone arasındaki hesaplanan mesafe ile ölçülen mesafe
        arasındaki farkı (Residual) döndürür.
        """
        return np.linalg.norm(drones_pos - target_pos, axis=1) - distances

    def locate_target(self, drone_data):
        """
        Args:
            drone_data: List of dicts [{'pos': [x, y], 'rssi': -65}, ...]
        Returns:
            np.array: Hedefin tahmini [x, y] koordinatı.
        """
        positions = []
        distances = []

        # Veriyi hazırla
        for data in drone_data:
            positions.append(data['pos'])
            dist = self.rssi_to_distance(data['rssi'])
            distances.append(dist)
        
        positions = np.array(positions)
        distances = np.array(distances)

        # Başlangıç tahmini (Drone'ların orta noktası)
        initial_guess = np.mean(positions, axis=0)

        # Non-linear Least Squares optimizasyonu
        # Hata fonksiyonunu minimize eden x,y noktasını bulur
        result = least_squares(
            self._error_function, 
            initial_guess, 
            args=(positions, distances)
        )

        return result.x  # Optimize edilmiş koordinat