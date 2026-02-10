
import numpy as np

class RSSIKalmanFilter:
    """
    1D Kalman Filtresi: Wi-Fi RSSI sinyallerindeki gürültüyü (Noise) 
    temizlemek için kullanılır.
    
    Model:
    X_k = A * X_k-1 + B * u + w (Process)
    Z_k = H * X_k + v (Measurement)
    """
    def __init__(self, process_noise=1e-5, measurement_noise=0.1, estimated_error=1.0):
        # Process Noise (Q): Sistemin kendi içindeki belirsizlik
        self.Q = process_noise
        
        # Measurement Noise (R): Sensör/Ortam gürültüsü (Hava durumu, binalar)
        self.R = measurement_noise
        
        # Estimation Error (P): Tahmin hatası kovaryansı
        self.P = estimated_error
        
        # Initial State (X): Başlangıç RSSI değeri tahmini
        self.x = -60.0 # Varsayılan -60dBm ile başlatıyoruz

    def update(self, measurement):
        """
        Yeni ölçüm geldiğinde tahminle birleştirip gerçek değeri bulur.
        Args:
            measurement (float): Ham RSSI değeri (Örn: -72)
        Returns:
            float: Filtrelenmiş RSSI değeri
        """
        # 1. Prediction Step (Tahmin)
        # Sinyal gücünün durağan olduğunu varsayıyoruz (A=1, u=0)
        x_pred = self.x
        P_pred = self.P + self.Q

        # 2. Update Step (Düzeltme)
        # Kalman Kazancı (Gain) Hesabı
        K = P_pred / (P_pred + self.R)
        
        # Tahmini, ölçümle güncelle
        self.x = x_pred + K * (measurement - x_pred)
        
        # Hata kovaryansını güncelle
        self.P = (1 - K) * P_pred

        return self.x