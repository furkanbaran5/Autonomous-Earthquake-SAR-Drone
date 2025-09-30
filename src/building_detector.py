#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry # YENİ: Dronun pozisyonu için
from std_msgs.msg import String, ColorRGBA, Float64MultiArray
from geometry_msgs.msg import Point, PointStamped # YENİ: 3D nokta yayınlamak için
from visualization_msgs.msg import Marker, MarkerArray
import cv2
from ultralytics import YOLO
import os
from cv_bridge import CvBridge, CvBridgeError
import numpy as np # YENİ: Matematiksel hesaplamalar için
import tf.transformations as tft # YENİ: Rotasyon işlemleri için
import yaml

class BuildingDetector:
    def __init__(self):
        rospy.init_node('building_detector', anonymous=True)
        rospy.loginfo("Bina Tespit Düğümü başlatılıyor...")

        # Modelinizi yükleyin
        try:
            model_path = '/home/furkan/catkin_ws_staj/src/yolov5_ros/weights/best.pt'
            self.model = YOLO(model_path)
            rospy.loginfo(f"YOLO modeli '{model_path}' adresinden başarıyla yüklendi.")
        except Exception as e:
            rospy.logerr(f"Model yüklenirken kritik hata: {e}")
            return

        self.bridge = CvBridge()

        self.received_images = []
        self.images_to_process = 5

        # Marker takip değişkenleri
        self.marker_id_counter = 0
        self.text_markers = []

        # Dosya yolları
        self.corners_file = '/home/furkan/catkin_ws_staj/src/firefly_control/data/detected_corners.txt'
        self.buildings_yaml = '/home/furkan/catkin_ws_staj/src/firefly_control/data/detected_buildings.yaml'

        # Building summary için değişkenler
        self.detected_buildings = {}  # Tespit edilen binaları toplamak için

        # Köşe dosyasını başlangıçta temizle
        with open(self.corners_file, 'w') as f:
            f.write("Tespit Edilen Bina Köşeleri\n")
            f.write("=" * 40 + "\n")
            f.write("Format: [Timestamp] Class (Confidence) - X, Y\n\n")

        # YAML dosyasını başlangıçta temizle
        try:
            if os.path.exists(self.buildings_yaml):
                os.remove(self.buildings_yaml)
                rospy.loginfo(f"🗑️ Önceki YAML dosyası silindi: {self.buildings_yaml}")
        except Exception as e:
            rospy.logwarn(f"YAML dosyası silinirken hata: {e}")

        
        # --- YENİ: Projeksiyon için gerekli veriler ---
        self.camera_intrinsics = None
        self.latest_odom = None
        
        # Kamera bilgilerini almak için bekle. Bu genellikle tek sefer yayınlanır.
        try:
            rospy.loginfo("Kamera bilgisi bekleniyor...")
            camera_info_msg = rospy.wait_for_message('/firefly/new_extra_camera/camera_info', CameraInfo, timeout=10)
            # Kamera matrisini (K) alıyoruz. [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.camera_intrinsics = np.array(camera_info_msg.K).reshape(3, 3)
            rospy.loginfo("Kamera bilgisi başarıyla alındı.")
        except rospy.ROSException as e:
            rospy.logerr(f"Kamera bilgisi alınamadı: {e}")
            return

        # --- YENİ ve GÜNCELLENMİŞ PUBLISHER/SUBSCRIBER'lar ---
        self.status_pub = rospy.Publisher('/building_detector/status', String, queue_size=1)
        # YENİ: GeofenceGenerator'ın beklediği 3D köşe noktalarını yayınlamak için
        self.corner_pub = rospy.Publisher('/building_detector/corners', PointStamped, queue_size=20)
        # YENİ: Bounding box'lı görüntüleri yayınlamak için
        self.detection_image_pub = rospy.Publisher('/building_detector/detection_image', Image, queue_size=1)
        # YENİ: RViz'de text marker'ları göstermek için
        self.text_marker_pub = rospy.Publisher('/building_detector/text_markers', MarkerArray, queue_size=10)
        # YENİ: Tespit edilen binaların koordinatlarını yayınlamak için
        self.buildings_summary_pub = rospy.Publisher('/building_detector/buildings_summary', String, queue_size=1)
        
        self.command_sub = rospy.Subscriber('/mission/command', String, self.command_callback)
        # YENİ: Dronun anlık pozisyonunu ve yönelimini almak için
        self.odom_sub = rospy.Subscriber('/firefly/ground_truth/odometry', Odometry, self.odom_callback)
        
        # Subscriber'ı baştan oluştur - active flag ile kontrol et
        self.image_sub = rospy.Subscriber('/captured_images', Image, self.image_callback, queue_size=50)
        self.active = False
        
        rospy.loginfo("Bina Tespit Düğümü hazır. Komut bekleniyor...")

    def odom_callback(self, msg):
        # YENİ: Dronun en son pozisyonunu sakla
        self.latest_odom = msg

    def command_callback(self, msg):
        if msg.data == "start_detection" and not self.active:
            self.active = True
            self.received_images = []  # Listeyi temizle
            self.text_markers = []  # Marker'ları temizle
            self.marker_id_counter = 0  # ID counter'ı sıfırla
            rospy.loginfo("Tespit komutu alındı. Görüntüler dinleniyor...")

    def quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        """Quaternion'ı 3x3 rotation matrix'e çevirir"""
        # Normalize quaternion (safety check)
        norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm

        # Rotation matrix calculation
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
        ])
        return R

    def project_pixel_to_world_simple(self, u, v):
        """Basit görüntü koordinatından dünya koordinatına dönüşüm - kamerayı (0,0,0) varsayarak"""
        rospy.loginfo(f"🔍 BASİT PROJEKSIYON: u={u}, v={v}")

        if self.latest_odom is None:
            rospy.logwarn("Drone pozisyonu eksik.")
            return None

        # Drone pozisyonu
        pose = self.latest_odom.pose.pose
        drone_x, drone_y, drone_z = pose.position.x, pose.position.y, pose.position.z

        # Görüntü parametreleri
        img_center_x, img_center_y = 320.0, 240.0  # 640x480 görüntünün merkezi
        img_width, img_height = 640.0, 480.0

        # Görüntüden merkeze göre offset hesapla
        pixel_offset_x = u - img_center_x  # Sol(-) / Sağ(+)
        pixel_offset_y = v - img_center_y  # Üst(-) / Alt(+)

        # Drone yüksekliğine göre ground footprint scaling
        # Yükseklik ne kadar fazla olursa görüş alanı o kadar geniş
        scale_factor = (drone_z / 200.0) / 2.5  # 2.5 kat küçültülmüş scaling

        # Piksel offsetini dünya koordinatlarına çevir
        # X: görüntü X ekseni = dünya X ekseni
        # Y: görüntü Y ekseni ters = dünya Y ekseni (görüntü Y aşağı, dünya Y yukarı)
        world_offset_x = pixel_offset_x * scale_factor
        world_offset_y = -pixel_offset_y * scale_factor  # Y'yi ters çevir

        # Final dünya koordinatları
        temp_x = drone_x + world_offset_x
        temp_y = drone_y + world_offset_y

        # 90° sağa döndürme (saat yönünde - kamera yönelim düzeltmesi)
        world_x = temp_y   # Yeni X = Eski Y
        world_y = -temp_x  # Yeni Y = -Eski X
        world_z = 0.0

        rospy.loginfo(f"📍 Drone: ({drone_x:.2f}, {drone_y:.2f}, {drone_z:.2f})")
        rospy.loginfo(f"📐 Piksel offset: ({pixel_offset_x:.1f}, {pixel_offset_y:.1f})")
        rospy.loginfo(f"🌍 Dünya offset: ({world_offset_x:.2f}, {world_offset_y:.2f})")
        rospy.loginfo(f"🔄 90° sağa döndürme: ({temp_x:.2f}, {temp_y:.2f}) → ({world_x:.2f}, {world_y:.2f})")

        return np.array([world_x, world_y, world_z])

    def save_corner_to_file(self, corner, class_name, confidence):
        """Köşe koordinatını dosyaya kaydet"""
        try:
            with open(self.corners_file, 'a') as f:
                timestamp = rospy.Time.now().to_sec()
                f.write(f"[{timestamp:.1f}] {class_name} ({confidence:.3f}) - X: {corner[0]:.2f}, Y: {corner[1]:.2f}\n")
        except Exception as e:
            rospy.logerr(f"Köşe dosyaya kaydedilirken hata: {e}")

    def create_text_marker(self, center_point, class_name, confidence, marker_id):
        """Bina merkezi üzerinde text marker oluştur"""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "building_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Pozisyon - bina merkezinin üstünde
        marker.pose.position.x = center_point[0]
        marker.pose.position.y = center_point[1]
        marker.pose.position.z = 5.0  # 5m yükseklikte göster

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Boyut
        marker.scale.z = 8.0  # Text yüksekliği

        # Renk - saglam: yeşil, yikilmis: kırmızı
        if class_name == 'saglam':
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:  # yikilmis
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        marker.color.a = 1.0

        # Text içeriği
        marker.text = f"{class_name.upper()}\n{confidence:.2f}"
        marker.lifetime = rospy.Duration(300)  # 5 dakika görünür kalacak

        return marker

    def add_detection_summary(self):
        """Tespit işlemi sonunda özet bilgi ekle"""
        try:
            with open(self.corners_file, 'a') as f:
                f.write(f"\n{'='*50}\n")
                f.write(f"Tespit Tamamlandı - {rospy.Time.now()}\n")
                f.write(f"İşlenen Görüntü Sayısı: {len(self.received_images)}\n")
                f.write(f"{'='*50}\n\n")
        except Exception as e:
            rospy.logerr(f"Özet kaydedilirken hata: {e}")

    def create_and_publish_building_summary(self):
        """Txt dosyasından CENTER ve köşeleri okuyup gruplandır"""
        try:
            # CENTER satırlarını ve köşeleri oku
            centers = []
            all_corners = {}  # timestamp -> corner data

            with open(self.corners_file, 'r') as f:
                lines = f.readlines()

            # Önce tüm köşeleri topla - her CENTER öncesi 4 köşeyi grupla
            current_corners = []
            current_type = None
            current_confidence = None

            for line in lines:
                line = line.strip()
                if line.startswith('[') and ') - X:' in line:
                    if '_CENTER' in line:
                        # CENTER satırı - önceki 4 köşeyi kaydet
                        if len(current_corners) == 4 and current_type:
                            timestamp = float(line.split(']')[0][1:])
                            all_corners[timestamp] = {
                                'type': current_type,
                                'confidence': current_confidence,
                                'corners': current_corners.copy()
                            }
                        current_corners = []
                        current_type = None
                    else:
                        # Normal köşe satırı
                        try:
                            parts = line.split('] ')
                            if len(parts) >= 2:
                                info_part = parts[1]
                                class_conf = info_part.split(' - X:')[0]
                                building_type = class_conf.split(' (')[0]
                                confidence = float(class_conf.split('(')[1].split(')')[0])
                                coords = info_part.split(' - X:')[1]
                                x = float(coords.split(', Y:')[0])
                                y = float(coords.split(', Y:')[1])

                                current_corners.append([x, y])
                                current_type = building_type
                                current_confidence = confidence
                        except Exception as e:
                            continue

            # Şimdi CENTER satırlarını oku
            for line in lines:
                line = line.strip()
                if line.startswith('[') and '_CENTER' in line and ') - X:' in line:
                    try:
                        timestamp = float(line.split(']')[0][1:])
                        parts = line.split('] ')
                        if len(parts) >= 2:
                            info_part = parts[1]
                            class_conf = info_part.split(' - X:')[0]
                            building_type = class_conf.split('_CENTER')[0]
                            confidence = float(class_conf.split('(')[1].split(')')[0])
                            coords = info_part.split(' - X:')[1]
                            x = float(coords.split(', Y:')[0])
                            y = float(coords.split(', Y:')[1])

                            # Bu CENTER'a ait köşeleri bul (en yakın timestamp)
                            closest_corners = None
                            min_time_diff = float('inf')
                            for t, corner_data in all_corners.items():
                                time_diff = abs(t - timestamp)
                                if time_diff < min_time_diff and corner_data['type'] == building_type:
                                    min_time_diff = time_diff
                                    closest_corners = corner_data['corners']

                            centers.append({
                                'type': building_type.upper(),
                                'confidence': confidence,
                                'x': x,
                                'y': y,
                                'corners': closest_corners if closest_corners else []
                            })
                    except Exception as e:
                        rospy.logwarn(f"CENTER parse hatası: {line} - {e}")

            rospy.loginfo(f"🏢 {len(centers)} merkez bulundu")

            # 10 metre yakınındaki merkezleri gruplandır
            unique_buildings = []

            for center in centers:
                is_close = False
                for existing in unique_buildings:
                    distance = ((center['x'] - existing['center_x'])**2 + (center['y'] - existing['center_y'])**2)**0.5
                    if distance <= 10.0:
                        if center['confidence'] > existing['confidence']:
                            # Köşeleri daha okunabilir formata çevir
                            formatted_corners = []
                            for corner in center['corners']:
                                formatted_corners.append({
                                    'x': float(corner[0]),
                                    'y': float(corner[1])
                                })

                            existing.update({
                                'type': center['type'],
                                'confidence': center['confidence'],
                                'center_x': center['x'],
                                'center_y': center['y'],
                                'corners': formatted_corners
                            })
                        is_close = True
                        break

                if not is_close:
                    # Köşeleri daha okunabilir formata çevir
                    formatted_corners = []
                    for corner in center['corners']:
                        formatted_corners.append({
                            'x': float(corner[0]),
                            'y': float(corner[1])
                        })

                    unique_buildings.append({
                        'type': center['type'],
                        'confidence': center['confidence'],
                        'center_x': center['x'],
                        'center_y': center['y'],
                        'corners': formatted_corners
                    })

            rospy.loginfo(f"🏗️ Gruplandırma sonrası {len(unique_buildings)} unique bina")
            buildings_list = unique_buildings

            # Sayaçları hesapla
            saglam_count = sum(1 for b in buildings_list if b['type'] == 'SAGLAM')
            yikilmis_count = sum(1 for b in buildings_list if b['type'] == 'YIKILMIS')

            # Sadece SAGLAM binaları yeni formatta organize et
            saglam_buildings = [b for b in buildings_list if b['type'] == 'SAGLAM']

            # Yeni format: liste olarak düzenle
            formatted_buildings = []
            building_counter = 1

            for building in saglam_buildings:
                building_data = {building_counter: []}

                # Köşeler arasında en uzak iki köşeyi bul
                corners = building['corners']
                if len(corners) >= 2:
                    max_distance = 0
                    farthest_pair = (corners[0], corners[1])

                    # Tüm köşe çiftlerini kontrol et
                    for i in range(len(corners)):
                        for j in range(i+1, len(corners)):
                            corner1 = corners[i]
                            corner2 = corners[j]
                            distance = ((corner1['x'] - corner2['x'])**2 + (corner1['y'] - corner2['y'])**2)**0.5

                            if distance > max_distance:
                                max_distance = distance
                                farthest_pair = (corner1, corner2)

                    # En uzak iki köşeyi ekle
                    building_data[building_counter].append({
                        'x': float(farthest_pair[0]['x']),
                        'y': float(farthest_pair[0]['y'])
                    })
                    building_data[building_counter].append({
                        'x': float(farthest_pair[1]['x']),
                        'y': float(farthest_pair[1]['y'])
                    })

                formatted_buildings.append(building_data)
                building_counter += 1

            buildings_data = {
                'buildings': formatted_buildings
            }

            # YAML dosyasına kaydet
            with open(self.buildings_yaml, 'w') as f:
                yaml.dump(buildings_data, f, default_flow_style=False, allow_unicode=True)

            rospy.loginfo(f"📋 YAML dosyasına kaydedildi: {self.buildings_yaml}")

            # Topic'e JSON string olarak yayınla
            import json
            summary_json = json.dumps(buildings_data, indent=2)
            self.buildings_summary_pub.publish(summary_json)

            rospy.loginfo(f"📡 Building summary yayınlandı: {saglam_count} SAGLAM, {yikilmis_count} YIKILMIş")

        except Exception as e:
            rospy.logerr(f"Building summary oluşturulurken hata: {e}")
            
    def image_callback(self, msg):
        # Debug için hemen başta log
        rospy.loginfo(f"📥 Image callback çağrıldı - Active: {self.active}, Mevcut: {len(self.received_images)}/{self.images_to_process}")
        
        # Active kontrolü
        if not self.active:
            rospy.loginfo("⚠️ Detector aktif değil, görüntü atlandı")
            return
            
        # Maksimum görüntü kontrolü
        if len(self.received_images) >= self.images_to_process:
            rospy.loginfo(f"⚠️ Maksimum görüntü sayısına ulaşıldı: {len(self.received_images)}/{self.images_to_process}")
            return

        self.received_images.append(msg)
        rospy.loginfo(f"Görüntü alındı ({len(self.received_images)}/{self.images_to_process}). İşleniyor...")
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            results = self.model(cv_image)

            # Görselleştirme için görüntünün kopyasını al
            vis_image = cv_image.copy()

            # YOLOv8 yeni API kullan
            rospy.loginfo(f"🔍 YOLO sonuçları işleniyor...")
            detection_found = False

            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    rospy.loginfo(f"📦 {len(boxes)} detection bulundu")
                    for i, box in enumerate(boxes):
                        # Confidence ve class kontrolü
                        conf = float(box.conf[0])
                        cls = int(box.cls[0])

                        # Class name'i al (model.names[cls] ile)
                        class_name = self.model.names[cls]

                        rospy.loginfo(f"   Detection {i+1}: class='{class_name}', confidence={conf:.3f}")

                        # Model 'saglam' ve 'yikilmis' sınıflarını biliyor
                        if (class_name == 'saglam' or class_name == 'yikilmis') and conf > 0.3:
                            detection_found = True
                            rospy.loginfo(f"✅ Bina tespit edildi: '{class_name}' (confidence: {conf:.3f})")
                            # Bounding box koordinatları
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            xmin, ymin, xmax, ymax = int(x1), int(y1), int(x2), int(y2)

                            # Bounding box'ı görüntüye çiz
                            color = (0, 255, 0) if class_name == 'saglam' else (0, 0, 255)  # Yeşil/Kırmızı
                            cv2.rectangle(vis_image, (xmin, ymin), (xmax, ymax), color, 2)

                            # Label ekle
                            label = f"{class_name}: {conf:.2f}"
                            cv2.putText(vis_image, label, (xmin, ymin-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

                            # 4 köşeyi de al: sol üst, sağ üst, sol alt, sağ alt
                            corners_2d = [
                                (xmin, ymin),  # Sol üst
                                (xmax, ymin),  # Sağ üst
                                (xmin, ymax),  # Sol alt
                                (xmax, ymax)   # Sağ alt
                            ]

                            # Tüm köşeleri 3D dünyaya projekte et
                            valid_corners = []
                            for i, (u, v) in enumerate(corners_2d):
                                world_corner = self.project_pixel_to_world_simple(u, v)
                                if world_corner is not None:
                                    valid_corners.append(world_corner)

                            if len(valid_corners) >= 2:
                                # Köşeleri direkt yayınla ve dosyaya kaydet
                                for corner in valid_corners:
                                    ps = PointStamped()
                                    ps.header.stamp = rospy.Time.now()
                                    ps.header.frame_id = "world"
                                    ps.point = Point(corner[0], corner[1], corner[2])
                                    self.corner_pub.publish(ps)

                                    # Köşeyi dosyaya da kaydet
                                    self.save_corner_to_file(corner, class_name, conf)

                                # Bina merkezini hesapla (köşelerin ortalaması)
                                center_x = sum(corner[0] for corner in valid_corners) / len(valid_corners)
                                center_y = sum(corner[1] for corner in valid_corners) / len(valid_corners)
                                center_point = [center_x, center_y, 0.0]

                                # Merkez koordinatını da txt dosyasına yaz
                                try:
                                    with open(self.corners_file, 'a') as f:
                                        timestamp = rospy.Time.now().to_sec()
                                        f.write(f"[{timestamp:.1f}] {class_name}_CENTER ({conf:.3f}) - X: {center_x:.2f}, Y: {center_y:.2f}\n")
                                except Exception as e:
                                    rospy.logerr(f"Merkez dosyaya kaydedilirken hata: {e}")

                                # Text marker oluştur ve listeye ekle
                                text_marker = self.create_text_marker(center_point, class_name, conf, self.marker_id_counter)
                                self.text_markers.append(text_marker)
                                self.marker_id_counter += 1

                                rospy.loginfo(f"✅ {len(valid_corners)} köşe yayınlandı ve dosyaya kaydedildi: {class_name} conf={conf:.3f}")
                                rospy.loginfo(f"🏷️ Text marker oluşturuldu: {class_name.upper()} at ({center_x:.2f}, {center_y:.2f})")
                                rospy.loginfo(f"📍 Merkez koordinatı txt'ye kaydedildi: ({center_x:.2f}, {center_y:.2f})")
                                for i, corner in enumerate(valid_corners):
                                    rospy.loginfo(f"   K{i+1}({corner[0]:.2f}, {corner[1]:.2f})")
                            else:
                                rospy.logwarn(f"⚠️ Projeksiyon başarısız: class='{class_name}', conf={conf:.3f}")
                        else:
                            rospy.logwarn(f"⚠️ Tespit edildi ama confidence düşük: class='{class_name}', conf={conf:.3f}")
                else:
                    rospy.loginfo("📦 Bu result'ta box bulunamadı")

            if not detection_found:
                rospy.logwarn("❌ Hiç 'saglam' veya 'yikilmis' bina detection'ı bulunamadı!")
                rospy.logwarn(f"💡 Model class isimleri: {list(self.model.names.values())}")

            # Görselleştirilmiş görüntüyü yayınla
            try:
                detection_img_msg = self.bridge.cv2_to_imgmsg(vis_image, "bgr8")
                detection_img_msg.header = msg.header
                self.detection_image_pub.publish(detection_img_msg)
                rospy.loginfo("📸 Bounding box'lı görüntü yayınlandı")
            except CvBridgeError as e:
                rospy.logerr(f"Görüntü yayınlanırken hata: {e}")


##########################################################################################
# DAHA GELİŞMİTİRMEK İÇİN
# 5 FOTOĞRAFTAN GELEN TESPİTLERİ KÜMELEYİP ORTALAMASINI ALABİLİRİZ.
###########################################################################################
        except CvBridgeError as e:
            rospy.logerr(e)
        
        # Tüm görüntüler işlendi mi kontrol et
        rospy.loginfo(f"🔍 İşlenen görüntü sayısı: {len(self.received_images)}/{self.images_to_process}")
        if len(self.received_images) == self.images_to_process:
            rospy.loginfo(f"✅ Tüm görüntüler işlendi ({len(self.received_images)}/{self.images_to_process}). Görev tamamlandı.")

            # Dosyaya özet bilgi ekle
            self.add_detection_summary()

            # Text marker'ları yayınla
            if self.text_markers:
                marker_array = MarkerArray()
                marker_array.markers = self.text_markers
                self.text_marker_pub.publish(marker_array)
                rospy.loginfo(f"🏷️ {len(self.text_markers)} text marker RViz'e gönderildi!")
            else:
                rospy.logwarn("⚠️ Hiç text marker oluşturulmadı")

            # Building summary oluştur ve yayınla
            self.create_and_publish_building_summary()

            rospy.loginfo("📤 'detection_completed' mesajı yayınlanıyor...")
            self.status_pub.publish("detection_completed")
            rospy.loginfo("✅ 'detection_completed' mesajı yayınlandı!")
            rospy.loginfo(f"📄 Köşe koordinatları dosyaya kaydedildi: {self.corners_file}")
            # SUBSCRIBER'I KAPATMA - 5. görüntüyü kaçırmasın
            # if self.image_sub:
            #     self.image_sub.unregister()
            #     rospy.loginfo("🔌 Image subscriber kapatıldı")
            rospy.loginfo("📌 Subscriber açık bırakıldı - gelecek görevler için hazır")
            self.active = False

if __name__ == '__main__':
    try:
        detector = BuildingDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Bina Tespit Düğümü kapatılıyor...")