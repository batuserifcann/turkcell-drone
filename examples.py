"""
Quick Start Examples - Hızlı Başlangıç Örnekleri
Her senaryoya hazır örnek scriptler
"""

import numpy as np
import sys
import logging

logging.basicConfig(level=logging.INFO, format='%(message)s')


def example_1_basic_swarm():
    """Örnek 1: Basit Sürü Kontrolü"""
    print("\n" + "="*60)
    print("ÖRNEK 1: BASİT SÜRÜ KONTROLÜ")
    print("="*60 + "\n")
    
    from swarm_controller import SwarmController, FormationType
    
    # 5 drone'lu sürü oluştur
    swarm = SwarmController(num_drones=5)
    
    # Farklı formasyonları dene
    formations = [
        FormationType.V_SHAPE,
        FormationType.GRID,
        FormationType.CIRCLE
    ]
    
    for formation in formations:
        print(f"\nFormasyon: {formation.value}")
        swarm.set_formation(formation)
        
        # Hedefe git
        swarm.update_leader_target(
            target_position=np.array([100, 100, 20]),
            target_velocity=np.array([2, 0, 0])
        )
        
        # 20 adım simüle et
        for i in range(20):
            swarm.update_swarm(dt=0.1)
            
        status = swarm.get_swarm_status()
        print(f"  Aktif drone: {status['active_drones']}")
        print(f"  Ortalama batarya: %{status['average_battery']:.1f}")
        print(f"  Sürü merkezi: {status['swarm_center']}")


def example_2_mission_planning():
    """Örnek 2: Görev Planlama"""
    print("\n" + "="*60)
    print("ÖRNEK 2: GÖREV PLANLAMA VE ATAMA")
    print("="*60 + "\n")
    
    from mission_planner import MissionPlanner
    
    # Alan tanımla
    search_area = (
        np.array([0, 0, 0]),
        np.array([200, 200, 50])
    )
    
    planner = MissionPlanner(search_area)
    
    # Grid arama deseni oluştur
    print("5x5 Grid arama deseni oluşturuluyor...")
    planner.generate_search_pattern_tasks(grid_size=5)
    
    # Kritik sinyal görevleri ekle
    signals = [
        {'pos': [50, 60, 15], 'rssi': -62, 'desc': 'Güçlü sinyal (CRITICAL)'},
        {'pos': [120, 80, 15], 'rssi': -78, 'desc': 'Orta sinyal (HIGH)'},
        {'pos': [150, 150, 15], 'rssi': -88, 'desc': 'Zayıf sinyal (MEDIUM)'},
    ]
    
    print("\nSinyal tespiti görevleri ekleniyor...")
    for signal in signals:
        task = planner.create_task(
            task_type="signal_detection",
            target_location=signal['pos'],
            signal_strength=signal['rssi'],
            required_drones=2
        )
        print(f"  • {signal['desc']}: {signal['pos']}")
    
    # Drone'ları simüle et
    drones = [
        {'id': i, 'position': [10*i, 10*i, 15], 'battery': 85, 'is_free': True}
        for i in range(6)
    ]
    
    # Görevleri ata
    print("\nGörevler drone'lara atanıyor...")
    assignments = planner.assign_tasks_to_drones(drones)
    
    print(f"\n{len(assignments)} görev atandı:")
    for drone_id, task in assignments.items():
        print(f"  • Drone {drone_id} -> Görev #{task.task_id} "
              f"({task.priority.name})")
    
    # İstatistikler
    stats = planner.get_mission_statistics()
    print("\nGörev İstatistikleri:")
    print(f"  Toplam görev: {stats['total_tasks']}")
    print(f"  Durum dağılımı: {stats['status_breakdown']}")
    print(f"  Öncelik dağılımı: {stats['priority_breakdown']}")


def example_3_obstacle_avoidance():
    """Örnek 3: Engel Algılama ve Kaçınma"""
    print("\n" + "="*60)
    print("ÖRNEK 3: ENGEL ALGILAMA VE KAÇINMA")
    print("="*60 + "\n")
    
    from obstacle_avoidance import (
        ObstacleDetector, ObstacleAvoidance, 
        Obstacle, ObstacleType
    )
    
    detector = ObstacleDetector(detection_range=30.0)
    avoidance = ObstacleAvoidance()
    
    # Engeller oluştur
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
    
    print(f"{len(obstacles)} engel algılandı:")
    for i, obs in enumerate(obstacles):
        print(f"  {i+1}. {obs.obstacle_type.value}: {obs.position}")
    
    # Drone durumu
    drone_pos = np.array([10, 10, 15])
    drone_vel = np.array([2, 1, 0])
    target = np.array([30, 25, 15])
    
    print(f"\nDrone Durumu:")
    print(f"  Pozisyon: {drone_pos}")
    print(f"  Hız: {drone_vel}")
    print(f"  Hedef: {target}")
    
    # Kaçınma vektörü hesapla
    print("\nKaçınma kuvveti hesaplanıyor...")
    avoidance_force = avoidance.calculate_avoidance_vector(
        drone_pos, drone_vel, obstacles
    )
    print(f"  Kaçınma vektörü: {avoidance_force}")
    print(f"  Kuvvet büyüklüğü: {np.linalg.norm(avoidance_force):.2f}")
    
    # Güvenli yol bul
    print("\nGüvenli yol aranıyor...")
    path = avoidance.find_safe_path(drone_pos, target, obstacles)
    
    if path:
        print(f"  Yol bulundu! ({len(path)} waypoint)")
        for i, wp in enumerate(path[:5]):  # İlk 5 waypoint
            print(f"    {i}: {wp}")
        if len(path) > 5:
            print(f"    ... ve {len(path)-5} waypoint daha")
    else:
        print("  Güvenli yol bulunamadı!")


def example_4_full_simulation():
    """Örnek 4: Tam Entegre Simülasyon"""
    print("\n" + "="*60)
    print("ÖRNEK 4: TAM AFET SENARYOSU SİMÜLASYONU")
    print("="*60 + "\n")
    
    from integrated_simulation import IntegratedSwarmSystem
    
    print("8 drone'lu afet kurtarma sistemi başlatılıyor...\n")
    
    system = IntegratedSwarmSystem(
        num_drones=8,
        scenario_type="earthquake"
    )
    
    print("\n60 saniyelik simülasyon başlıyor...")
    print("   (visualize=True yaparak 3D animasyon görebilirsiniz)\n")
    
    # Kısa simülasyon (görselleştirmesiz)
    system.run_simulation(duration=60, visualize=False)


def example_5_sitl_test():
    """Örnek 5: SITL (Simülatör) Testi"""
    print("\n" + "="*60)
    print("ÖRNEK 5: SITL (SOFTWARE IN THE LOOP) TESTİ")
    print("="*60 + "\n")
    
    print("Bu örnek için önce SITL başlatmalısınız:")
    print("\n  Terminal'de çalıştırın:")
    print("  $ dronekit-sitl copter --home=40.0,-90.0,0,180\n")
    
    try:
        from real_drone_interface import RealDroneController
        
        connection_string = '127.0.0.1:14550'
        print(f"SITL'e bağlanılıyor ({connection_string})...")
        
        drone = RealDroneController(connection_string)
        
        if drone.connect_vehicle():
            print("Bağlantı başarılı!\n")
            
            # Basit test
            print("Kalkış testi yapılıyor...")
            drone.arm_and_takeoff(10)
            
            print("\nKuzey-Doğuya hareket...")
            drone.goto_position_ned(north=10, east=10, down=-10)
            
            import time
            time.sleep(5)
            
            print("\nİniş yapılıyor...")
            drone.land()
            
            time.sleep(5)
            drone.disconnect()
            
            print("\nTest tamamlandı!")
        else:
            print("\nBağlantı başarısız!")
            print("   SITL'in çalıştığından emin olun.")
            
    except ImportError:
        print("\nDroneKit kurulu değil!")
        print("   Kurulum: pip install dronekit dronekit-sitl")


def show_menu():
    """Ana menü"""
    print("\n" + "="*60)
    print("   AUTONOMOUS DISASTER DRONE SWARM - ÖRNEKLER")
    print("="*60)
    print("\nHangi örneği çalıştırmak istersiniz?\n")
    print("  1. Basit Sürü Kontrolü (Formasyonlar)")
    print("  2. Görev Planlama ve Atama")
    print("  3. Engel Algılama ve Kaçınma")
    print("  4. Tam Afet Senaryosu (60 saniye)")
    print("  5. SITL Drone Testi (Gerçek simülasyon)")
    print("\n  0. Çıkış")
    print("\n" + "="*60)
    
    choice = input("\nSeçiminiz (0-5): ")
    return choice


def main():
    """Ana program"""
    examples = {
        '1': example_1_basic_swarm,
        '2': example_2_mission_planning,
        '3': example_3_obstacle_avoidance,
        '4': example_4_full_simulation,
        '5': example_5_sitl_test,
    }
    
    # Komut satırı argümanı varsa direkt çalıştır
    if len(sys.argv) > 1:
        choice = sys.argv[1]
        if choice in examples:
            examples[choice]()
        else:
            print(f"Geçersiz örnek numarası: {choice}")
            print("   Kullanım: python examples.py [1-5]")
        return
    
    # Menü ile çalıştır
    while True:
        choice = show_menu()
        
        if choice == '0':
            print("\nGörüşmek üzere!\n")
            break
        elif choice in examples:
            try:
                examples[choice]()
                input("\nDevam etmek için Enter'a basın...")
            except KeyboardInterrupt:
                print("\n\nİşlem kullanıcı tarafından durduruldu.")
                input("Devam etmek için Enter'a basın...")
            except Exception as e:
                print(f"\nHata oluştu: {e}")
                import traceback
                traceback.print_exc()
                input("Devam etmek için Enter'a basın...")
        else:
            print("\nGeçersiz seçim! Lütfen 0-5 arası bir sayı girin.")
            input("Devam etmek için Enter'a basın...")


if __name__ == "__main__":
    main()
