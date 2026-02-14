"""
Real Drone Interface - DroneKit Integration
Simülasyondan gerçek drone'lara geçiş için DroneKit entegrasyonu
"""

import time
import numpy as np
from typing import List, Dict, Optional
import logging

try:
    from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
    from pymavlink import mavutil
    DRONEKIT_AVAILABLE = True
except ImportError:
    DRONEKIT_AVAILABLE = False
    logging.warning("DroneKit bulunamadı. Simülasyon modunda çalışılacak.")
    
from swarm_controller import DroneStatus, DroneRole

logger = logging.getLogger(__name__)


class RealDroneController:
    """Gerçek Drone Kontrolcüsü (DroneKit)"""
    
    def __init__(self, connection_string: str, baud_rate: int = 57600):
        """
        Args:
            connection_string: 
                - Serial: '/dev/ttyUSB0' veya 'COM3'
                - UDP: 'udp:127.0.0.1:14550'
                - TCP: 'tcp:127.0.0.1:5760'
                - SITL: '127.0.0.1:14550'
        """
        self.connection_string = connection_string
        self.baud_rate = baud_rate
        self.vehicle = None
        self.is_connected = False
        self.home_location = None
        
    def connect_vehicle(self, timeout: int = 60) -> bool:
        """Drone'a bağlan"""
        
        if not DRONEKIT_AVAILABLE:
            logger.error("DroneKit kurulu değil!")
            return False
            
        try:
            logger.info(f"Drone'a bağlanılıyor: {self.connection_string}")
            
            self.vehicle = connect(
                self.connection_string,
                baud=self.baud_rate,
                wait_ready=True,
                timeout=timeout
            )
            
            self.is_connected = True
            logger.info("Bağlantı başarılı!")
            self._print_vehicle_info()
            
            return True
            
        except Exception as e:
            logger.error(f"Bağlantı hatası: {e}")
            return False
            
    def _print_vehicle_info(self):
        """Drone bilgilerini yazdır"""
        logger.info(f"  Mod: {self.vehicle.mode.name}")
        logger.info(f"  GPS: {self.vehicle.gps_0}")
        logger.info(f"  Batarya: %{self.vehicle.battery.level}")
        logger.info(f"  Silahlandırılmış: {self.vehicle.armed}")
        
    def arm_and_takeoff(self, target_altitude: float) -> bool:
        """
        Drone'u arm et ve kaldır
        Args:
            target_altitude: Hedef irtifa (metre)
        """
        
        if not self.is_connected:
            logger.error("Drone bağlı değil!")
            return False
            
        logger.info(f"{target_altitude}m yüksekliğe kalkılıyor...")
        
        # GPS kilidi kontrol et
        while not self.vehicle.is_armable:
            logger.info("  GPS kilidi bekleniyor...")
            time.sleep(1)
            
        # GUIDED moda geç
        self.vehicle.mode = VehicleMode("GUIDED")
        while self.vehicle.mode.name != "GUIDED":
            time.sleep(0.5)
            
        # Arm et
        logger.info("  Arm ediliyor...")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            time.sleep(0.5)
            
        # Kalkış
        self.vehicle.simple_takeoff(target_altitude)
        
        # Hedefe ulaşana kadar bekle
        while True:
            current_alt = self.vehicle.location.global_relative_frame.alt
            logger.info(f"  İrtifa: {current_alt:.1f}m")
            
            if current_alt >= target_altitude * 0.95:
                logger.info("Hedef irtifaya ulaşıldı!")
                break
                
            time.sleep(1)
            
        return True
        
    def goto_position_ned(self, north: float, east: float, down: float, 
                         velocity: float = 5.0):
        """
        NED koordinatlarında hedefe git (North-East-Down)
        Args:
            north: Kuzey (metre, pozitif=kuzey)
            east: Doğu (metre, pozitif=doğu)
            down: Aşağı (metre, pozitif=aşağı)
            velocity: Hız (m/s)
        """
        
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # type_mask (position only)
            north, east, down,
            0, 0, 0,  # velocity
            0, 0, 0,  # acceleration
            0, 0
        )
        self.vehicle.send_mavlink(msg)
        
    def goto_position_global(self, lat: float, lon: float, alt: float):
        """
        Global GPS koordinatlarına git
        Args:
            lat: Latitude
            lon: Longitude
            alt: Altitude (metre, home'dan göreceli)
        """
        location = LocationGlobalRelative(lat, lon, alt)
        self.vehicle.simple_goto(location)
        
    def set_velocity(self, vx: float, vy: float, vz: float):
        """
        Hız vektörünü ayarla (NED frame)
        Args:
            vx: Kuzey hızı (m/s)
            vy: Doğu hızı (m/s)
            vz: Aşağı hızı (m/s)
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # type_mask (velocity only)
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0
        )
        self.vehicle.send_mavlink(msg)
        
    def get_position_ned(self) -> np.ndarray:
        """
        Mevcut pozisyonu NED olarak al (home'dan göreceli)
        Returns:
            [north, east, down]
        """
        if self.home_location is None:
            self.home_location = self.vehicle.location.global_frame
            
        # Global frame'den NED'e dönüşüm (basitleştirilmiş)
        current = self.vehicle.location.global_frame
        
        # Latitude/Longitude farkını metreye çevir (yaklaşık)
        dlat = current.lat - self.home_location.lat
        dlon = current.lon - self.home_location.lon
        
        north = dlat * 111320.0  # 1 derece ~ 111.32 km
        east = dlon * 111320.0 * np.cos(np.radians(current.lat))
        down = -(current.alt - self.home_location.alt)
        
        return np.array([north, east, down])
        
    def get_velocity(self) -> np.ndarray:
        """
        Mevcut hız vektörünü al
        Returns:
            [vx, vy, vz] NED frame
        """
        vel = self.vehicle.velocity
        return np.array([vel[0], vel[1], vel[2]])
        
    def get_battery_level(self) -> float:
        """Batarya seviyesi (%)"""
        return self.vehicle.battery.level if self.vehicle.battery.level else 100.0
        
    def land(self):
        """İniş yap"""
        logger.info(" İniş yapılıyor...")
        self.vehicle.mode = VehicleMode("LAND")
        
    def return_to_launch(self):
        """Başlangıç noktasına dön"""
        logger.info("Başlangıç noktasına dönülüyor...")
        self.vehicle.mode = VehicleMode("RTL")
        
    def disconnect(self):
        """Bağlantıyı kes"""
        if self.vehicle:
            self.vehicle.close()
            self.is_connected = False
            logger.info("Bağlantı kapatıldı")


class SwarmDroneInterface:
    """Sürü drone'ları için birleşik interface"""
    
    def __init__(self):
        self.drones: Dict[int, RealDroneController] = {}
        self.drone_status: Dict[int, DroneStatus] = {}
        
    def add_drone(self, 
                  drone_id: int, 
                  connection_string: str,
                  initial_position: np.ndarray = None) -> bool:
        """
        Sürüye drone ekle
        Args:
            drone_id: Drone ID
            connection_string: Bağlantı string'i
            initial_position: Başlangıç pozisyonu [x, y, z]
        """
        
        controller = RealDroneController(connection_string)
        
        if controller.connect_vehicle():
            self.drones[drone_id] = controller
            
            # Durum nesnesi oluştur
            if initial_position is None:
                initial_position = controller.get_position_ned()
                
            status = DroneStatus(drone_id, initial_position)
            status.battery_level = controller.get_battery_level()
            self.drone_status[drone_id] = status
            
            logger.info(f"✅ Drone {drone_id} sürüye eklendi")
            return True
        else:
            logger.error(f"❌ Drone {drone_id} eklenemedi")
            return False
            
    def takeoff_swarm(self, altitude: float = 10.0):
        """Tüm drone'ları kaldır"""
        logger.info(f"🚀 {len(self.drones)} drone kalkıyor...")
        
        for drone_id, controller in self.drones.items():
            success = controller.arm_and_takeoff(altitude)
            if success:
                self.drone_status[drone_id].position[2] = altitude
                
    def update_swarm_positions(self):
        """Tüm drone pozisyonlarını güncelle"""
        for drone_id, controller in self.drones.items():
            if controller.is_connected:
                position = controller.get_position_ned()
                velocity = controller.get_velocity()
                battery = controller.get_battery_level()
                
                status = self.drone_status[drone_id]
                status.update_position(position, dt=0.1)
                status.velocity = velocity
                status.battery_level = battery
                
    def send_velocity_commands(self, velocity_commands: Dict[int, np.ndarray]):
        """
        Hız komutları gönder
        Args:
            velocity_commands: {drone_id: [vx, vy, vz], ...}
        """
        for drone_id, velocity in velocity_commands.items():
            if drone_id in self.drones:
                self.drones[drone_id].set_velocity(*velocity)
                
    def emergency_land_all(self):
        """Tüm drone'ları acil iniş yap"""
        logger.warning(" TÜM DRONE'LAR ACİL İNİŞ YAPIYOR!")
        for controller in self.drones.values():
            controller.land()
                
    def return_all_to_home(self):
        """Tüm drone'ları eve gönder"""
        logger.info("🏠 Tüm drone'lar eve dönüyor...")
        for controller in self.drones.values():
            controller.return_to_launch()
            
    def disconnect_all(self):
        """Tüm bağlantıları kes"""
        for controller in self.drones.values():
            controller.disconnect()
                

# SITL (Software In The Loop) Test Fonksiyonları
def test_sitl_single_drone():
    """Tek drone SITL testi"""
    
    logger.info(" SITL Test başlatılıyor...")
    
    # SITL başlatma komutu (terminal'de çalıştırılmalı):
    # dronekit-sitl copter --home=40.0,-90.0,0,180
    
    connection_string = '127.0.0.1:14550'
    drone = RealDroneController(connection_string)
    
    if drone.connect_vehicle():
        # Kalkış
        drone.arm_and_takeoff(10)
        
        # Kare çiz
        points = [
            (10, 0, -10),   # Kuzey
            (10, 10, -10),  # Kuzeydoğu
            (0, 10, -10),   # Doğu
            (0, 0, -10),    # Başlangıç
        ]
        
        for north, east, down in points:
            logger.info(f"  Hedefe gidiliyor: ({north}, {east}, {down})")
            drone.goto_position_ned(north, east, down)
            time.sleep(5)
            
        # İniş
        drone.land()
        time.sleep(10)
        drone.disconnect()
        

def test_sitl_swarm():
    """Sürü SITL testi (3 drone)"""
    
    logger.info("🧪 Sürü SITL Test başlatılıyor...")
    
    # SITL başlatma (3 instance):
    # dronekit-sitl copter --instance 0 --home=40.0,-90.0,0,180
    # dronekit-sitl copter --instance 1 --home=40.0,-90.01,0,180
    # dronekit-sitl copter --instance 2 --home=40.0,-90.02,0,180
    
    swarm = SwarmDroneInterface()
    
    # Drone'ları ekle
    connections = [
        '127.0.0.1:14550',
        '127.0.0.1:14560',
        '127.0.0.1:14570',
    ]
    
    for i, conn in enumerate(connections):
        swarm.add_drone(i, conn)
        time.sleep(2)
        
    # Kalkış
    swarm.takeoff_swarm(altitude=15)
    time.sleep(10)
    
    # Formasyon oluştur (V-shape)
    for i in range(30):
        swarm.update_swarm_positions()
        
        # V formasyonu hız komutları
        velocities = {
            0: np.array([1, 0, 0]),      # Leader - düz
            1: np.array([0.8, -0.5, 0]), # Sol kanat
            2: np.array([0.8, 0.5, 0]),  # Sağ kanat
        }
        
        swarm.send_velocity_commands(velocities)
        time.sleep(1)
        
    # Eve dön
    swarm.return_all_to_home()
    time.sleep(20)
    swarm.disconnect_all()


if __name__ == "__main__":
    # Test seçenekleri
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "swarm":
        test_sitl_swarm()
    else:
        test_sitl_single_drone()
