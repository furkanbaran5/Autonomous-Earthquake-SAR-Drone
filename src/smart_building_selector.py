#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import yaml
import os
import re
import math

class SmartBuildingSelector:
    def __init__(self):
        rospy.init_node('smart_building_selector', anonymous=True)
        rospy.loginfo("Smart Building Selector başlatılıyor...")

        # Dosya yolları
        self.corners_file = '/home/furkan/catkin_ws_staj/src/firefly_control/data/detected_corners.txt'
        self.output_yaml = '/home/furkan/catkin_ws_staj/src/firefly_control/data/detected_buildings.yaml'

        # Önceden tanımlı bina koordinatları (keskin koordinatlar)
        self.predefined_buildings = {
            'building_1': {
                'center': (14.0, 5.0),
                'corners': [
                    {'x': 10.0, 'y': 0.0},
                    {'x': 10.0, 'y': 10.0},
                    {'x': 17.0, 'y': 0.0},
                    {'x': 17.0, 'y': 10.0}
                ],
                'type': 'saglam',
                'size': (10.0, 8.0)  # genişlik x yükseklik
            },
            'building_2': {
                'center': (-14.0, -5.0),
                'corners': [
                    {'x': -17.0, 'y': -10.0},
                    {'x': -17.0, 'y': 0.0},
                    {'x': -10.0, 'y': -10.0},
                    {'x': -10.0, 'y': 0.0}
                ],
                'type': 'saglam',
                'size': (10.0, 8.0)
            }
        }

        # Publisher
        self.result_pub = rospy.Publisher('/smart_building_selector/result', String, queue_size=1)

        # Mission coordinator'dan gelen komutları dinle
        self.command_sub = rospy.Subscriber('/mission/command', String, self.command_callback)

        rospy.loginfo(f"Smart Building Selector hazır. {len(self.predefined_buildings)} adet tanımlı bina mevcut.")

    def calculate_distance(self, point1, point2):
        """İki nokta arasındaki Euclidean mesafesini hesapla"""
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def parse_txt_buildings(self):
        """detected_corners.txt dosyasından bina bilgilerini çıkar"""
        if not os.path.exists(self.corners_file):
            rospy.logwarn(f"Köşe dosyası bulunamadı: {self.corners_file}")
            return []

        detected_buildings = []

        try:
            with open(self.corners_file, 'r') as f:
                content = f.read()

            # CENTER satırlarını bul
            center_pattern = r'\[([\d.]+)\] (\w+)_CENTER \(([\d.]+)\) - X: ([-\d.]+), Y: ([-\d.]+)'
            center_matches = re.findall(center_pattern, content)

            for timestamp, building_type, confidence, x, y in center_matches:
                center = (float(x), float(y))

                # Bu CENTER'dan önce gelen 4 köşeyi bul
                corners = []
                lines = content.split('\n')

                for i, line in enumerate(lines):
                    if f'{building_type}_CENTER' in line and f'[{timestamp}]' in line:
                        # Bu CENTER satırından önceki köşe satırlarını kontrol et
                        for j in range(max(0, i-10), i):  # 10 satır geriye git
                            corner_line = lines[j]
                            if f'[{timestamp}]' in corner_line and f'{building_type} (' in corner_line and '_CENTER' not in corner_line:
                                # Köşe koordinatını çıkar
                                corner_match = re.search(r'X: ([-\d.]+), Y: ([-\d.]+)', corner_line)
                                if corner_match:
                                    corner_x, corner_y = float(corner_match.group(1)), float(corner_match.group(2))
                                    corners.append((corner_x, corner_y))
                        break

                if len(corners) >= 2:  # En az 2 köşe var ise ekle
                    detected_buildings.append({
                        'center': center,
                        'corners': corners,
                        'type': building_type,
                        'confidence': float(confidence),
                        'timestamp': float(timestamp)
                    })

        except Exception as e:
            rospy.logerr(f"Txt dosyası parse edilirken hata: {e}")

        # Aynı pozisyondaki binaları gruplandır (5m yakınlık)
        unique_buildings = self.group_similar_buildings(detected_buildings)

        rospy.loginfo(f"📊 Txt'den {len(detected_buildings)} bina merkezi okundu")
        rospy.loginfo(f"🔗 Gruplandırma sonrası {len(unique_buildings)} unique bina")
        return unique_buildings

    def group_similar_buildings(self, detected_buildings):
        """Yakın pozisyondaki binaları gruplandır"""
        unique_buildings = []

        for detected in detected_buildings:
            # Bu bina mevcut unique listede var mı kontrol et
            merged = False
            for existing in unique_buildings:
                distance = self.calculate_distance(detected['center'], existing['center'])

                # 5m yakınlığındaki binalar aynı kabul edilir
                if distance <= 5.0 and detected['type'] == existing['type']:
                    # En yüksek confidence'a sahip olanı kullan
                    if detected['confidence'] > existing['confidence']:
                        existing.update(detected)
                    merged = True
                    break

            if not merged:
                unique_buildings.append(detected)

        return unique_buildings

    def find_best_matches(self, detected_buildings):
        """Tespit edilen her binayı en uygun tanımlı bina ile eşleştir"""
        if not detected_buildings:
            rospy.logwarn("Tespit edilen bina yok!")
            return []

        matches = []
        used_predefined = set()  # Aynı tanımlı binayı 2 kere kullanma

        # Her tespit edilen bina için en iyi eşleşmeyi bul
        for detected in detected_buildings:
            detected_center = detected['center']
            best_match = None
            best_score = float('inf')

            rospy.loginfo(f"🔍 Tespit edilen bina: {detected_center} ({detected['type']})")

            for building_id, predefined in self.predefined_buildings.items():
                if building_id in used_predefined:
                    continue  # Bu tanımlı bina zaten kullanıldı

                predefined_center = predefined['center']

                # Merkez mesafesi hesapla
                center_distance = self.calculate_distance(detected_center, predefined_center)

                # Tür uyumu (bonus/ceza)
                type_bonus = 0
                if detected['type'] == predefined['type']:
                    type_bonus = -5.0  # Tür eşleşirse bonus ver

                # Toplam skor (mesafe + tür bonusu)
                total_score = center_distance + type_bonus

                rospy.loginfo(f"   → {building_id}: mesafe={center_distance:.2f}m, skor={total_score:.2f}")

                if total_score < best_score:
                    best_score = total_score
                    best_match = {
                        'building_id': building_id,
                        'predefined': predefined,
                        'detected': detected,
                        'distance': center_distance,
                        'score': total_score
                    }

            if best_match:
                matches.append(best_match)
                used_predefined.add(best_match['building_id'])
                rospy.loginfo(f"✅ Eşleştirme: {best_match['building_id']} (skor: {best_match['score']:.2f})")

        rospy.loginfo(f"📊 Toplam {len(matches)} bina eşleştirildi")
        return matches

    def create_yaml_output(self, matches):
        """Eşleştirilen binaların koordinatlarını YAML formatında oluştur"""
        if not matches:
            return None

        yaml_data = {}

        # Her eşleştirilen bina için YAML entry oluştur
        for i, match in enumerate(matches, 1):
            predefined = match['predefined']

            # Bina koordinatları - 2 uzak köşe (diagonal)
            yaml_data[i] = {
                'x0': int(predefined['corners'][0]['x']),  # Sol üst
                'y0': int(predefined['corners'][0]['y']),
                'x1': int(predefined['corners'][3]['x']),  # Sağ alt
                'y1': int(predefined['corners'][3]['y'])
            }

        return yaml_data

    def save_yaml(self, yaml_data):
        """YAML dosyasını kaydet"""
        try:
            with open(self.output_yaml, 'w') as f:
                yaml.dump(yaml_data, f, default_flow_style=False, allow_unicode=True)

            rospy.loginfo(f"✅ Smart building YAML kaydedildi: {self.output_yaml}")
            return True

        except Exception as e:
            rospy.logerr(f"❌ YAML kaydetme hatası: {e}")
            return False

    def process_buildings(self):
        """Ana işlem: txt okuma, eşleştirme, YAML yazma"""
        rospy.loginfo("🏗️ Smart building selection başlıyor...")

        # 1. Txt dosyasından tespit edilen binaları oku
        detected_buildings = self.parse_txt_buildings()

        if not detected_buildings:
            rospy.logwarn("❌ Hiç bina tespit edilmemiş!")
            return False

        # 2. Tüm binaları eşleştir
        matches = self.find_best_matches(detected_buildings)

        if not matches:
            rospy.logwarn("❌ Hiçbir tanımlı bina ile eşleşme bulunamadı!")
            return False

        # 3. YAML dosyası oluştur
        yaml_data = self.create_yaml_output(matches)

        if not yaml_data:
            rospy.logerr("❌ YAML data oluşturulamadı!")
            return False

        # 4. YAML dosyasını kaydet
        success = self.save_yaml(yaml_data)

        if success:
            # Sonucu yayınla
            building_ids = [match['building_id'] for match in matches]
            result_msg = f"smart_buildings_selected:{','.join(building_ids)}"
            self.result_pub.publish(result_msg)

            rospy.loginfo(f"📡 Sonuç yayınlandı: {result_msg}")
            rospy.loginfo("🎯 İşlem tamamlandı, node kapanıyor...")
            
            rospy.signal_shutdown("Smart building selection tamamlandı")

        return False

    def command_callback(self, msg):
        """Mission coordinator'dan gelen komutları dinle"""
        if msg.data == "start_smart_building_selection":
            rospy.loginfo("📥 Smart building selection komutu alındı")
            self.process_buildings()
        elif msg.data == "detection_completed":
            rospy.loginfo("📥 Detection completed - smart selection başlatılıyor")
            # Kısa bir gecikme ekle (dosya yazım işleminin tamamlanması için)
            rospy.sleep(1.0)
            self.process_buildings()

if __name__ == '__main__':
    try:
        selector = SmartBuildingSelector()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Smart Building Selector kapatılıyor...")