# Swarm-Link: Autonomous Drone Swarm & Intelligence Network

![Status](https://img.shields.io/badge/Status-In%20Development-yellow)
![Python](https://img.shields.io/badge/Python-3.9+-blue)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%20%7C%20Pixhawk-green)

**Swarm-Link**, GPS olmayan ve iletişimin kesildiği afet bölgelerinde veya operasyonel sahalarda, otonom drone sürüsü kullanarak **Mesh Network** kuran ve **Wi-Fi Sinyal İstihbaratı (SIGINT)** ile hedef tespiti yapan çift kullanımlı (Dual-Use) bir sistemdir.

## Proje Vizyonu

Sistem iki ana modda çalışır:

1.  **Sivil Mod (Hayat Hattı):** Deprem enkazı altında kalan kazazedelerin cep telefonu sinyallerini tespit eder ve konumlarını 3 metre hassasiyetle belirler.
2.  **Askeri Mod (Sessiz Avcı):** Sınır güvenliğinde, pasif dinleme yaparak tehdit unsurlarını radyo sessizliğinde tespit eder.

## Teknik Mimari

### 1. Donanım Bağımsız Sürü Zekası

- **Flight Controller:** Pixhawk 2.4.8 (ArduPilot/PX4)
- **Onboard Computer:** Raspberry Pi 4 & Zero 2 W
- **Communication:** Ad-Hoc Mesh Network (Batman-adv / UDP)

### 2. Algoritmik Çekirdek (The Brain)

- **RSSI Trilateration:** 3 farklı drone'dan alınan sinyal gücü ile üçgenleme.
- **Kalman Filtering:** Gürültülü RF verisinin (Noise) temizlenmesi ve konum tahmini.
- **Swarm Consensus:** Lider-Takipçi (Leader-Follower) hiyerarşisi ile otonom görev paylaşımı.

## Kurulum ve Kullanım

Gerekli kütüphanelerin kurulumu:

```bash
pip install -r requirements.txt
```
