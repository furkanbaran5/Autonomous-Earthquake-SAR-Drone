#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Rescue Data Logger Node
Furkan'ın YOLO marker'larını dinler ve rescue verilerini otomatik kayıt eder
"""

import rospy
import cv2
import json
import os
from datetime import datetime
import math
import shutil
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry


class RescueDataLogger:
    def __init__(self):
        rospy.init_node('rescue_data_logger', anonymous=True)
        rospy.loginfo("🗃️ Rescue Data Logger başlatılıyor...")

        self.cv_bridge = CvBridge()
        self.current_image = None
        self.current_position = None

        # Bina konumları (smart_building_selector'dan)
        self.buildings = {
            'building_1': {
                'center': (14.0, 5.0),
                'corners': [(10.0, 10.0), (10.0, 0.0), (17.0, 10.0), (17.0, 0.0)],
                'type': 'saglam'
            },
            'building_2': {
                'center': (-14.0, -5.0),
                'corners': [(-17.0, 0.0), (-17.0, -10.0), (-10.0, 0.0), (-10.0, -10.0)],
                'type': 'saglam'
            }
        }

        # Veri kayıt dizinleri
        self.data_dir = os.path.expanduser("~/rescue_archive")
        self.database_file = os.path.join(self.data_dir, "rescue_database.json")

        # Önceki verileri temizle ve ana dizini oluştur
        if os.path.exists(self.data_dir):
            shutil.rmtree(self.data_dir)
            rospy.loginfo(f"🗑️ Önceki veriler temizlendi: {self.data_dir}")
        os.makedirs(self.data_dir, exist_ok=True)

        # Bina bazında görüntü dizinleri oluştur
        self.building_image_dirs = {}
        for building_name in self.buildings.keys():
            building_dir = os.path.join(self.data_dir, "images", building_name)
            os.makedirs(building_dir, exist_ok=True)
            self.building_image_dirs[building_name] = building_dir

        # Bilinmeyen alan için de dizin
        unknown_dir = os.path.join(self.data_dir, "images", "bilinmeyen_alan")
        os.makedirs(unknown_dir, exist_ok=True)
        self.building_image_dirs["bilinmeyen_alan"] = unknown_dir

        # Rescue veritabanı
        self.rescue_database = self.load_database()
        self.rescue_counter = len(self.rescue_database) + 1

        # ROS Subscribers
        self.marker_sub = rospy.Subscriber('/yolov5/human_markers', MarkerArray, self.marker_callback)
        self.image_sub = rospy.Subscriber('/yolov5/image_out', Image, self.image_callback)
        self.odom_sub = rospy.Subscriber('/firefly/odometry_sensor1/odometry', Odometry, self.odometry_callback)

        # Kayıtlı marker'ları takip et (duplikasyon kontrolü için)
        self.recorded_positions = []

        rospy.loginfo("🎯 Rescue Data Logger hazır - Furkan'ın marker'larını dinliyorum...")
        rospy.loginfo(f"📁 Veri kayıt dizini: {self.data_dir}")

    def load_database(self):
        """Mevcut veritabanını yükle"""
        try:
            if os.path.exists(self.database_file):
                with open(self.database_file, 'r', encoding='utf-8') as f:
                    return json.load(f)
        except Exception as e:
            rospy.logwarn(f"Database yükleme hatası: {e}")
        return []

    def save_database(self):
        """Veritabanını kaydet"""
        try:
            with open(self.database_file, 'w', encoding='utf-8') as f:
                json.dump(self.rescue_database, f, indent=2, ensure_ascii=False)
        except Exception as e:
            rospy.logwarn(f"Database kaydetme hatası: {e}")

    def image_callback(self, msg):
        """Güncel kamera görüntüsünü tut"""
        try:
            self.current_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn(f"Image dönüşüm hatası: {e}")

    def odometry_callback(self, msg):
        """Drone'un güncel pozisyonunu tut"""
        self.current_position = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }

    def marker_callback(self, msg):
        """Furkan'ın marker'larını dinle ve otomatik kaydet"""
        rospy.loginfo(f"📥 MarkerArray alındı - {len(msg.markers)} marker")

        if not msg.markers:
            rospy.loginfo("⚠️ Boş marker array")
            return

        for marker in msg.markers:
            if marker.ns == "person_detections":
                # Odometry'den drone pozisyonunu al
                if self.current_position is None:
                    rospy.logwarn("⚠️ Odometry verisi henüz alınmadı, marker göz ardı ediliyor")
                    continue

                x, y, z = self.current_position['x'], self.current_position['y'], self.current_position['z']

                # Duplikasyon kontrolü - daha önce kaydedilmiş mi?
                if self.is_already_recorded(x, y, z):
                    continue

                rospy.loginfo(f"🚨 Yeni insan marker tespit edildi: ({x:.2f}, {y:.2f}, {z:.2f})")

                # Bina bilgisini al
                building_info = self.find_building(x, y)
                building_name = building_info.split(' (')[0]  # "building_1 (mesafe: 5.2m)" -> "building_1"

                # Kat bilgisini hesapla (z yüksekliği 2.5'e bölünür)
                floor = int(z / 2.5) + 1  # Zemin kat = 1

                # Otomatik kayıt et
                rescue_data = {
                    'id': f"insan_{self.rescue_counter}",
                    'position': {'x': x, 'y': y, 'z': z},
                    'floor': floor,
                    'building': building_info,
                    'building_name': building_name,
                    'timestamp': datetime.now().isoformat(),
                    'image_saved': False,
                    'confidence': getattr(marker.scale, 'x', 0.5)
                }

                # Binaya göre görüntü dizini seç
                building_dir = self.building_image_dirs.get(building_name, self.building_image_dirs["bilinmeyen_alan"])

                # Görüntüyü binaya özel dizinde kaydet
                image_filename = f"{rescue_data['id']}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
                image_path = os.path.join(building_dir, image_filename)

                if self.current_image is not None:
                    cv2.imwrite(image_path, self.current_image)
                    rescue_data['image_path'] = image_path
                    rescue_data['image_saved'] = True
                    rospy.loginfo(f"📷 Görüntü kaydedildi: {image_filename}")
                else:
                    rospy.logwarn("⚠️ Mevcut görüntü yok, sadece pozisyon kaydedildi")

                # Veritabanına ekle
                self.rescue_database.append(rescue_data)
                self.recorded_positions.append((x, y, z))
                self.save_database()

                rospy.loginfo(f"✅ {rescue_data['id']} otomatik kaydedildi - {rescue_data['building']}")
                self.rescue_counter += 1

    def is_already_recorded(self, x, y, z, threshold=2.0):
        """Bu pozisyon daha önce kaydedilmiş mi kontrol et"""
        for rx, ry, rz in self.recorded_positions:
            distance = math.sqrt((x - rx)**2 + (y - ry)**2 + (z - rz)**2)
            if distance < threshold:
                return True
        return False

    def find_building(self, x, y):
        """Koordinata göre bina bul"""
        closest_building = "bilinmeyen_alan"
        min_distance = float('inf')

        for building_name, building_data in self.buildings.items():
            center_x, center_y = building_data['center']
            distance = math.sqrt((x - center_x)**2 + (y - center_y)**2)

            if distance < min_distance:
                min_distance = distance
                closest_building = building_name

        return f"{closest_building} (mesafe: {min_distance:.1f}m)"

    def print_statistics(self):
        """İstatistikleri yazdır"""
        total_count = len(self.rescue_database)
        images_saved = len([r for r in self.rescue_database if r['image_saved']])

        rospy.loginfo(f"📊 Toplam kayıt: {total_count}, Görüntülü: {images_saved}")

        # Bina bazında istatistik
        building_stats = {}
        for rescue in self.rescue_database:
            building = rescue['building'].split(' (')[0]  # Mesafeyi çıkar
            building_stats[building] = building_stats.get(building, 0) + 1

        for building, count in building_stats.items():
            rospy.loginfo(f"🏢 {building}: {count} kişi")

    def get_database_summary(self):
        """Veritabanı özetini döndür"""
        # Bina bazında istatistik
        building_stats = {}
        for rescue in self.rescue_database:
            building_name = rescue.get('building_name', rescue['building'].split(' (')[0])
            if building_name not in building_stats:
                building_stats[building_name] = {'count': 0, 'with_images': 0}
            building_stats[building_name]['count'] += 1
            if rescue['image_saved']:
                building_stats[building_name]['with_images'] += 1

        return {
            'total_rescues': len(self.rescue_database),
            'images_saved': len([r for r in self.rescue_database if r['image_saved']]),
            'database_file': self.database_file,
            'building_stats': building_stats,
            'building_image_dirs': self.building_image_dirs
        }


def main():
    try:
        logger = RescueDataLogger()

        # Başlangıç istatistikleri
        summary = logger.get_database_summary()
        rospy.loginfo(f"📈 Başlangıç: {summary['total_rescues']} kayıt mevcut")

        # Her 30 saniyede istatistik yazdır
        rospy.Timer(rospy.Duration(30), lambda event: logger.print_statistics())

        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("🛑 Rescue Data Logger durduruluyor...")
        if 'logger' in locals():
            logger.print_statistics()
            summary = logger.get_database_summary()
            rospy.loginfo(f"📁 Toplam {summary['total_rescues']} kayıt: {summary['database_file']}")
            rospy.loginfo("📷 Bina bazında görüntüler:")
            for building, dir_path in summary['building_image_dirs'].items():
                stats = summary['building_stats'].get(building, {'count': 0, 'with_images': 0})
                rospy.loginfo(f"   🏢 {building}: {stats['with_images']} görüntü ({dir_path})")


if __name__ == '__main__':
    main()