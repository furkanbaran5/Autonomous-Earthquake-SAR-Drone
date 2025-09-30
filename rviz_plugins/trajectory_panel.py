#!/usr/bin/env python3
"""
RViz Panel for Trajectory Playback and Rescue System
"""

import rospy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import sys
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *
import numpy as np
from collections import deque
import threading
import time
import cv2
from cv_bridge import CvBridge
import json
import os

try:
    from rviz import bindings as rviz
except ImportError:
    import rviz


class RescueDetectionSystem:
    """İnsan tespiti ve kurtarma sistemi"""

    def __init__(self):
        self.cv_bridge = CvBridge()
        self.current_image = None
        self.rescue_positions = []
        self.detection_callback = None

        # Create rescue log file
        self.rescue_log_file = os.path.expanduser("~/rescue_positions.json")
        self.load_rescue_positions()

        # ROS Subscribers
        self.detection_sub = rospy.Subscriber('/human_detection_flag', Bool, self.detection_callback_handler)
        self.image_sub = rospy.Subscriber('/firefly/vi_sensor/left/image_raw', Image, self.image_callback)

    def detection_callback_handler(self, msg):
        """İnsan tespiti flag'ı geldiğinde çağrılır"""
        if msg.data and self.detection_callback:
            self.detection_callback(self.current_image)

    def image_callback(self, msg):
        """Kamera görüntüsü geldiğinde güncelle"""
        try:
            self.current_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn(f"Image conversion failed: {e}")

    def add_rescue_position(self, x, y, z, image_path=None):
        """Kurtarma pozisyonu kaydet"""
        position_data = {
            'x': float(x),
            'y': float(y),
            'z': float(z),
            'timestamp': rospy.Time.now().to_sec(),
            'image_path': image_path
        }
        self.rescue_positions.append(position_data)
        self.save_rescue_positions()
        rospy.loginfo(f"Rescue position saved: ({x:.2f}, {y:.2f}, {z:.2f})")

    def get_rescue_positions(self):
        """Kaydedilen kurtarma pozisyonlarını döndür"""
        return self.rescue_positions

    def save_rescue_positions(self):
        """Kurtarma pozisyonlarını dosyaya kaydet"""
        try:
            with open(self.rescue_log_file, 'w') as f:
                json.dump(self.rescue_positions, f, indent=2)
        except Exception as e:
            rospy.logwarn(f"Failed to save rescue positions: {e}")

    def load_rescue_positions(self):
        """Kaydedilen kurtarma pozisyonlarını yükle"""
        try:
            if os.path.exists(self.rescue_log_file):
                with open(self.rescue_log_file, 'r') as f:
                    self.rescue_positions = json.load(f)
        except Exception as e:
            rospy.logwarn(f"Failed to load rescue positions: {e}")
            self.rescue_positions = []


class CameraView(QLabel):
    """Kamera görüntüsü widget'ı"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 200)
        self.setMaximumSize(400, 300)
        self.setScaledContents(True)
        self.setStyleSheet("QLabel { border: 2px solid gray; background-color: #f0f0f0; }")
        self.setText("📷 Camera View\n\nWaiting for detection...")
        self.setAlignment(Qt.AlignCenter)

    def update_image(self, cv_image):
        """OpenCV görüntüsünü QLabel'de göster"""
        if cv_image is not None:
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(q_image)
            self.setPixmap(pixmap)
        else:
            self.setText("📷 Camera View\n\nNo image available")


class RescueDecisionDialog(QDialog):
    """Kurtarma kararı vereceğin dialog penceresi"""

    def __init__(self, image, drone_position, operator_confirm_pub, parent=None):
        super().__init__(parent)
        self.image = image
        self.drone_position = drone_position
        self.operator_confirm_pub = operator_confirm_pub
        self.result = None
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("🚨 Human Detection Alert")
        self.setModal(True)
        self.resize(600, 500)

        layout = QVBoxLayout()

        # Başlık
        title = QLabel("🚨 Human Detected!")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("QLabel { color: #e74c3c; margin: 10px; }")
        layout.addWidget(title)

        # Kamera görüntüsü
        self.camera_view = CameraView()
        if self.image is not None:
            self.camera_view.update_image(self.image)
        layout.addWidget(self.camera_view)

        # Pozisyon bilgisi
        pos_info = QLabel(f"📍 Drone Position: X:{self.drone_position[0]:.2f}m, Y:{self.drone_position[1]:.2f}m, Z:{self.drone_position[2]:.2f}m")
        pos_info.setAlignment(Qt.AlignCenter)
        pos_info.setStyleSheet("QLabel { font-size: 11px; color: #2c3e50; margin: 10px; }")
        layout.addWidget(pos_info)

        # Karar butonları
        button_layout = QHBoxLayout()

        self.rescue_button = QPushButton("✅ Kurtarılması Gerekiyor")
        self.rescue_button.setStyleSheet("""
            QPushButton {
                background-color: #27ae60;
                color: white;
                font-size: 12px;
                font-weight: bold;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #229954; }
        """)
        self.rescue_button.clicked.connect(self.rescue_needed)

        self.no_rescue_button = QPushButton("❌ Kurtarılmasına Gerek Yok")
        self.no_rescue_button.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                font-size: 12px;
                font-weight: bold;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #c0392b; }
        """)
        self.no_rescue_button.clicked.connect(self.no_rescue_needed)

        button_layout.addWidget(self.rescue_button)
        button_layout.addWidget(self.no_rescue_button)
        layout.addLayout(button_layout)

        self.setLayout(layout)

    def rescue_needed(self):
        self.result = True
        # Furkan'a onay gönder
        confirm_msg = Bool()
        confirm_msg.data = True
        self.operator_confirm_pub.publish(confirm_msg)
        self.accept()

    def no_rescue_needed(self):
        self.result = False
        # Furkan'a ret gönder
        confirm_msg = Bool()
        confirm_msg.data = False
        self.operator_confirm_pub.publish(confirm_msg)
        self.accept()


class TrajectoryRecorder:
    """Drone pozisyonlarını kaydeder ve Path mesajı oluşturur"""

    def __init__(self, max_points=None):  # None = unlimited
        self.trajectory_points = deque(maxlen=max_points)
        self.timestamps = deque(maxlen=max_points)
        self.start_time = None
        self.lock = threading.Lock()
        self.is_recording = True

        # Session management
        self.sessions = {}  # session_id -> {points, timestamps, start_time, end_time}
        self.current_session_id = None
        self.session_counter = 0
        self.sessions_dir = os.path.expanduser("~/trajectory_sessions")
        if not os.path.exists(self.sessions_dir):
            os.makedirs(self.sessions_dir)
        self.load_saved_sessions()

        # ROS Publishers - RViz'de görünecek
        self.path_pub = rospy.Publisher('/drone/trajectory_path', Path, queue_size=10)
        self.marker_pub = rospy.Publisher('/drone/trajectory_marker', Marker, queue_size=10)
        self.current_pos_pub = rospy.Publisher('/drone/current_position_marker', Marker, queue_size=10)
        self.rescue_markers_pub = rospy.Publisher('/rescue_positions_markers', MarkerArray, queue_size=10)
        self.operator_confirm_pub = rospy.Publisher('/operator_rescue_confirm', Bool, queue_size=10)

        # ROS Subscriber
        self.pose_sub = rospy.Subscriber('/firefly/ground_truth/odometry', Odometry, self.pose_callback)

    def pose_callback(self, msg):
        """Drone pozisyonu geldiğinde kaydet"""
        if self.is_recording:
            with self.lock:
                if self.start_time is None:
                    self.start_time = rospy.Time.now().to_sec()

                current_time = rospy.Time.now().to_sec() - self.start_time

                point = {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z,
                    'time': current_time
                }

                self.trajectory_points.append(point)
                self.timestamps.append(current_time)

    def get_trajectory_at_time(self, target_time):
        """Belirli bir zamana kadarki trajectory'yi döndür"""
        with self.lock:
            if not self.trajectory_points:
                return []

            # Binary search for efficiency
            filtered_points = []
            for point in self.trajectory_points:
                if point['time'] <= target_time:
                    filtered_points.append(point)
                else:
                    break  # Points are chronologically ordered

            return filtered_points

    def publish_trajectory(self, points, current_point=None):
        """Trajectory'yi RViz'de göstermek için yayınla"""
        # Path mesajı oluştur
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "world"

        for point in points:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "world"
            pose.pose.position.x = point['x']
            pose.pose.position.y = point['y']
            pose.pose.position.z = point['z']
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.path_pub.publish(path)

        # Mevcut pozisyon marker'ı
        if current_point:
            pos_marker = Marker()
            pos_marker.header.frame_id = "world"
            pos_marker.header.stamp = rospy.Time.now()
            pos_marker.ns = "current_position"
            pos_marker.id = 0
            pos_marker.type = Marker.SPHERE
            pos_marker.action = Marker.ADD

            pos_marker.pose.position.x = current_point['x']
            pos_marker.pose.position.y = current_point['y']
            pos_marker.pose.position.z = current_point['z']
            pos_marker.pose.orientation.w = 1.0

            pos_marker.scale.x = 0.5
            pos_marker.scale.y = 0.5
            pos_marker.scale.z = 0.5

            pos_marker.color.r = 1.0
            pos_marker.color.g = 0.0
            pos_marker.color.b = 0.0
            pos_marker.color.a = 1.0

            self.current_pos_pub.publish(pos_marker)

    def publish_rescue_markers(self, rescue_positions):
        """Kurtarma pozisyonlarını RViz'de göster"""
        marker_array = MarkerArray()

        for i, pos in enumerate(rescue_positions):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "rescue_positions"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = pos['x']
            marker.pose.position.y = pos['y']
            marker.pose.position.z = pos['z']
            marker.pose.orientation.w = 1.0

            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 2.0

            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

        self.rescue_markers_pub.publish(marker_array)

    def get_max_time(self):
        """Kaydedilen maksimum zamanı döndür"""
        with self.lock:
            if self.timestamps:
                return max(self.timestamps)
            return 0

    def get_start_time(self):
        """İlk timestamp'i döndür"""
        with self.lock:
            if self.timestamps:
                return min(self.timestamps)
            return 0.0

    def get_valid_time_range(self):
        """Valid time range döndür (start, end)"""
        with self.lock:
            if self.timestamps:
                return (min(self.timestamps), max(self.timestamps))
            return (0.0, 0.0)

    def clear_trajectory(self):
        """Tüm trajectory verilerini temizle"""
        with self.lock:
            self.trajectory_points.clear()
            self.timestamps.clear()
            self.start_time = None

    def start_new_session(self):
        """Yeni bir recording session başlat"""
        with self.lock:
            # Mevcut session'ı kaydet
            if self.current_session_id is not None and self.trajectory_points:
                self.save_current_session()

            # Yeni session başlat
            self.session_counter += 1
            self.current_session_id = f"session_{self.session_counter}_{int(time.time())}"

            # Trajectory'yi temizle
            self.trajectory_points.clear()
            self.timestamps.clear()
            self.start_time = None

            rospy.loginfo(f"🎬 New recording session started: {self.current_session_id}")
            return self.current_session_id

    def save_current_session(self):
        """Mevcut session'ı kaydet"""
        if self.current_session_id is None or not self.trajectory_points:
            return

        session_data = {
            'session_id': self.current_session_id,
            'points': list(self.trajectory_points),
            'timestamps': list(self.timestamps),
            'start_time': self.start_time,
            'end_time': rospy.Time.now().to_sec(),
            'point_count': len(self.trajectory_points)
        }

        # Memory'de sakla
        self.sessions[self.current_session_id] = session_data

        # Dosyaya kaydet
        session_file = os.path.join(self.sessions_dir, f"{self.current_session_id}.json")
        try:
            with open(session_file, 'w') as f:
                json.dump(session_data, f, indent=2)
            rospy.loginfo(f"💾 Session saved: {session_file}")
        except Exception as e:
            rospy.logwarn(f"Failed to save session: {e}")

    def load_saved_sessions(self):
        """Kaydedilmiş session'ları yükle"""
        try:
            for filename in os.listdir(self.sessions_dir):
                if filename.endswith('.json'):
                    filepath = os.path.join(self.sessions_dir, filename)
                    with open(filepath, 'r') as f:
                        session_data = json.load(f)
                        session_id = session_data['session_id']
                        self.sessions[session_id] = session_data

                        # Session counter'ı güncelle
                        try:
                            session_num = int(session_id.split('_')[1])
                            self.session_counter = max(self.session_counter, session_num)
                        except:
                            pass

            rospy.loginfo(f"📂 Loaded {len(self.sessions)} saved sessions")
        except Exception as e:
            rospy.logwarn(f"Failed to load sessions: {e}")

    def get_sessions_list(self):
        """Session listesini döndür"""
        sessions_info = []
        for session_id, data in self.sessions.items():
            info = {
                'id': session_id,
                'point_count': data['point_count'],
                'duration': data.get('end_time', 0) - data.get('start_time', 0),
                'start_time': data.get('start_time', 0)
            }
            sessions_info.append(info)

        # Başlangıç zamanına göre sırala (en yeni önce)
        sessions_info.sort(key=lambda x: x['start_time'], reverse=True)
        return sessions_info

    def load_session(self, session_id):
        """Belirli bir session'ı yükle ve görüntüle"""
        if session_id not in self.sessions:
            rospy.logwarn(f"Session not found: {session_id}")
            return False

        with self.lock:
            session_data = self.sessions[session_id]

            # Trajectory verilerini yükle
            self.trajectory_points = deque(session_data['points'], maxlen=None)
            self.timestamps = deque(session_data['timestamps'], maxlen=None)
            self.start_time = session_data['start_time']

            rospy.loginfo(f"📖 Loaded session: {session_id} ({len(self.trajectory_points)} points)")
            return True

    def delete_session(self, session_id):
        """Session'ı sil"""
        if session_id in self.sessions:
            # Memory'den sil
            del self.sessions[session_id]

            # Dosyadan sil
            session_file = os.path.join(self.sessions_dir, f"{session_id}.json")
            if os.path.exists(session_file):
                os.remove(session_file)

            rospy.loginfo(f"🗑️ Deleted session: {session_id}")
            return True
        return False

    def set_recording(self, recording):
        """Kayıt durumunu ayarla"""
        self.is_recording = recording

    def is_recording_active(self):
        """Kayıt durumunu döndür"""
        return self.is_recording


class TrajectoryRVizPanel(QWidget):
    """RViz Panel for Trajectory Control"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.recorder = TrajectoryRecorder()
        self.is_playing = False
        self.play_speed = 1.0
        self.current_time = 0.0

        # Slider optimization variables
        self.last_slider_value = 0
        self.slider_threshold = 5  # Minimum change to trigger update
        self.last_update_time = 0
        self.update_cooldown = 0.1  # 100ms cooldown

        # Rescue detection sistemi
        self.rescue_system = RescueDetectionSystem()
        self.rescue_system.detection_callback = self.on_human_detected

        self.init_ui()

        # Timer for updating timeline
        self.update_timer = QTimer()

        # Timer for debouncing slider
        self.slider_timer = QTimer()
        self.slider_timer.setSingleShot(True)
        self.slider_timer.timeout.connect(self.process_slider_change)
        self.pending_slider_value = 0
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(50)  # 20 FPS güncelleme

        # Timer for playback
        self.play_timer = QTimer()
        self.play_timer.timeout.connect(self.playback_step)

        # Initialize sessions after UI
        QTimer.singleShot(100, self.init_sessions)  # Delay to ensure UI is ready

    def init_ui(self):
        """RViz Panel UI'sini oluştur"""
        layout = QVBoxLayout()

        # Başlık
        title = QLabel("🚁 Firefly Trajectory & Rescue Control")
        title.setFont(QFont("Arial", 12, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("QLabel { color: #2c3e50; margin: 5px; }")
        layout.addWidget(title)

        # Flight Information
        info_group = QGroupBox("📊 Flight Info")
        info_layout = QGridLayout()

        self.status_label = QLabel("Recording")
        self.status_label.setStyleSheet("QLabel { color: green; font-weight: bold; }")
        info_layout.addWidget(QLabel("Status:"), 0, 0)
        info_layout.addWidget(self.status_label, 0, 1)

        self.time_label = QLabel("0.0s / 0.0s")
        info_layout.addWidget(QLabel("Time:"), 1, 0)
        info_layout.addWidget(self.time_label, 1, 1)

        self.point_count_label = QLabel("0")
        info_layout.addWidget(QLabel("Points:"), 2, 0)
        info_layout.addWidget(self.point_count_label, 2, 1)

        info_group.setLayout(info_layout)
        layout.addWidget(info_group)

        # Timeline Control
        timeline_group = QGroupBox("⏯️ Timeline")
        timeline_layout = QVBoxLayout()

        self.timeline_slider = QSlider(Qt.Horizontal)
        self.timeline_slider.setMinimum(0)
        self.timeline_slider.setMaximum(1000)
        self.timeline_slider.setValue(0)
        self.timeline_slider.valueChanged.connect(self.on_timeline_changed)
        timeline_layout.addWidget(self.timeline_slider)

        # Control buttons
        control_layout = QHBoxLayout()

        self.play_button = QPushButton("▶")
        self.play_button.clicked.connect(self.toggle_play)
        self.play_button.setMaximumWidth(40)
        control_layout.addWidget(self.play_button)

        self.pause_button = QPushButton("⏸")
        self.pause_button.clicked.connect(self.pause_playback)
        self.pause_button.setEnabled(False)
        self.pause_button.setMaximumWidth(40)
        control_layout.addWidget(self.pause_button)

        self.reset_button = QPushButton("⏮")
        self.reset_button.clicked.connect(self.reset_timeline)
        self.reset_button.setMaximumWidth(40)
        control_layout.addWidget(self.reset_button)

        # Speed control
        self.speed_spinbox = QDoubleSpinBox()
        self.speed_spinbox.setMinimum(0.1)
        self.speed_spinbox.setMaximum(5.0)
        self.speed_spinbox.setValue(1.0)
        self.speed_spinbox.setSingleStep(0.1)
        self.speed_spinbox.setSuffix("x")
        self.speed_spinbox.valueChanged.connect(self.on_speed_changed)
        self.speed_spinbox.setMaximumWidth(80)
        control_layout.addWidget(self.speed_spinbox)

        control_layout.addStretch()
        timeline_layout.addLayout(control_layout)
        timeline_group.setLayout(timeline_layout)
        layout.addWidget(timeline_group)

        # Recording Control
        record_group = QGroupBox("⏺️ Recording")
        record_layout = QHBoxLayout()

        self.record_button = QPushButton("⏹ Stop")
        self.record_button.setCheckable(True)
        self.record_button.setChecked(True)
        self.record_button.clicked.connect(self.toggle_recording)
        self.record_button.setStyleSheet("QPushButton:checked { background-color: #e74c3c; }")
        record_layout.addWidget(self.record_button)

        self.clear_button = QPushButton("🗑 Clear")
        self.clear_button.clicked.connect(self.clear_trajectory)
        record_layout.addWidget(self.clear_button)

        record_group.setLayout(record_layout)
        layout.addWidget(record_group)

        # Session Management
        session_group = QGroupBox("📁 Sessions")
        session_layout = QVBoxLayout()

        # Session selector
        session_selector_layout = QHBoxLayout()
        session_selector_layout.addWidget(QLabel("Session:"))

        self.session_combo = QComboBox()
        self.session_combo.addItem("Current Session")
        self.session_combo.currentTextChanged.connect(self.on_session_changed)
        session_selector_layout.addWidget(self.session_combo)

        self.refresh_sessions_button = QPushButton("🔄")
        self.refresh_sessions_button.clicked.connect(self.refresh_sessions_list)
        self.refresh_sessions_button.setMaximumWidth(30)
        session_selector_layout.addWidget(self.refresh_sessions_button)

        session_layout.addLayout(session_selector_layout)

        # Session controls
        session_controls_layout = QHBoxLayout()

        self.new_session_button = QPushButton("🎬 New")
        self.new_session_button.clicked.connect(self.start_new_session)
        session_controls_layout.addWidget(self.new_session_button)

        self.save_session_button = QPushButton("💾 Save")
        self.save_session_button.clicked.connect(self.save_current_session)
        session_controls_layout.addWidget(self.save_session_button)

        self.delete_session_button = QPushButton("🗑️ Del")
        self.delete_session_button.clicked.connect(self.delete_selected_session)
        self.delete_session_button.setEnabled(False)
        session_controls_layout.addWidget(self.delete_session_button)

        session_layout.addLayout(session_controls_layout)

        session_group.setLayout(session_layout)
        layout.addWidget(session_group)

        # Rescue System
        rescue_group = QGroupBox("🚨 Rescue System")
        rescue_layout = QVBoxLayout()

        self.detection_status_label = QLabel("Waiting for detection...")
        self.detection_status_label.setStyleSheet("QLabel { color: #7f8c8d; font-style: italic; font-size: 10px; }")
        rescue_layout.addWidget(self.detection_status_label)

        rescue_info_layout = QHBoxLayout()
        rescue_info_layout.addWidget(QLabel("Saved:"))
        self.rescue_count_label = QLabel("0")
        self.rescue_count_label.setStyleSheet("QLabel { color: #e74c3c; font-weight: bold; }")
        rescue_info_layout.addWidget(self.rescue_count_label)

        self.view_positions_button = QPushButton("📋")
        self.view_positions_button.clicked.connect(self.show_rescue_positions)
        self.view_positions_button.setEnabled(False)
        self.view_positions_button.setMaximumWidth(40)
        rescue_info_layout.addWidget(self.view_positions_button)

        self.test_detection_button = QPushButton("🧪 Test")
        self.test_detection_button.clicked.connect(self.test_human_detection)
        self.test_detection_button.setStyleSheet("QPushButton { background-color: #f39c12; }")
        rescue_info_layout.addWidget(self.test_detection_button)

        rescue_info_layout.addStretch()
        rescue_layout.addLayout(rescue_info_layout)
        rescue_group.setLayout(rescue_layout)
        layout.addWidget(rescue_group)

        layout.addStretch()
        self.setLayout(layout)

    def on_timeline_changed(self, value):
        """Timeline slider değiştiğinde - debounced"""
        current_time = time.time()

        # Threshold kontrolü - küçük değişiklikleri ignore et
        if abs(value - self.last_slider_value) < self.slider_threshold:
            return

        # Cooldown kontrolü - çok hızlı güncellemeleri önle
        if current_time - self.last_update_time < self.update_cooldown:
            # Pending value'yu güncelle ve timer'ı restart et
            self.pending_slider_value = value
            self.slider_timer.stop()
            self.slider_timer.start(100)  # 100ms debounce
            return

        # Immediate update
        self.process_slider_change_immediate(value)

    def process_slider_change(self):
        """Debounced slider change handler"""
        self.process_slider_change_immediate(self.pending_slider_value)

    def process_slider_change_immediate(self, value):
        """Actual slider processing - 0s'den max_time'a kadar"""
        max_time = self.recorder.get_max_time()

        if max_time > 0:
            # Map slider value to full timeline (0s to max_time)
            normalized_value = value / 1000.0  # 0.0 to 1.0
            self.current_time = normalized_value * max_time

            # Sadece büyük değişikliklerde log
            if abs(value - self.last_slider_value) > 20:
                print(f"🎛️ Timeline: value={value}, time={self.current_time:.2f}s/{max_time:.2f}s")

            self.update_trajectory_display()
            self.last_slider_value = value
            self.last_update_time = time.time()
        else:
            # Sadece ilk hata mesajı
            if self.last_slider_value == 0:
                print(f"⚠️ No trajectory data available")

    def publish_test_slider_marker(self, slider_value):
        """Slider değişikliğini görmek için test marker'ı"""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "slider_test"
        marker.id = 999
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Slider değerine göre pozisyon değiştir (Z ekseninde)
        marker.pose.position.x = 5.0
        marker.pose.position.y = 5.0
        marker.pose.position.z = 1.0 + (slider_value / 100.0)  # 1.0 - 11.0 arası

        marker.pose.orientation.w = 1.0

        # Slider değerine göre renk değiştir
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        # Renk: slider değerine göre kırmızıdan yeşile
        marker.color.r = (1000 - slider_value) / 1000.0
        marker.color.g = slider_value / 1000.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.current_pos_pub.publish(marker)
        print(f"🔴➡️🟢 Test marker: z={marker.pose.position.z:.2f}, color=r{marker.color.r:.2f}g{marker.color.g:.2f}")

    def toggle_play(self):
        """Play/Stop durumunu değiştir"""
        if not self.is_playing:
            self.is_playing = True
            self.play_button.setText("⏹")
            self.pause_button.setEnabled(True)
            self.record_button.setEnabled(False)
            self.play_timer.start(50)
        else:
            self.stop_playback()

    def pause_playback(self):
        """Playback'i duraklat"""
        if self.is_playing:
            self.is_playing = False
            self.play_timer.stop()
            self.play_button.setText("▶")
            self.pause_button.setEnabled(False)

    def stop_playback(self):
        """Playback'i durdur"""
        self.is_playing = False
        self.play_timer.stop()
        self.play_button.setText("▶")
        self.pause_button.setEnabled(False)
        self.record_button.setEnabled(True)

    def reset_timeline(self):
        """Timeline'ı başa al"""
        self.current_time = 0.0
        self.timeline_slider.setValue(0)
        self.update_trajectory_display()

    def on_speed_changed(self, value):
        """Playback hızını değiştir"""
        self.play_speed = value

    def toggle_recording(self):
        """Kayıt durumunu değiştir"""
        if self.record_button.isChecked():
            # Start recording - yeni session başlat
            self.start_new_session()
            self.recorder.set_recording(True)
            self.record_button.setText("⏹ Stop")
            self.status_label.setText("Recording")
            self.status_label.setStyleSheet("QLabel { color: green; font-weight: bold; }")
        else:
            # Stop recording - mevcut session'ı kaydet
            self.recorder.set_recording(False)
            self.save_current_session()
            self.record_button.setText("⏺ Start")
            self.status_label.setText("Stopped")
            self.status_label.setStyleSheet("QLabel { color: red; font-weight: bold; }")

    def clear_trajectory(self):
        """Trajectory'yi temizle"""
        reply = QMessageBox.question(self, 'Clear Trajectory',
                                    'Clear all trajectory data?',
                                    QMessageBox.Yes | QMessageBox.No,
                                    QMessageBox.No)

        if reply == QMessageBox.Yes:
            self.recorder.clear_trajectory()
            self.current_time = 0.0
            self.timeline_slider.setValue(0)
            self.update_display()

    def playback_step(self):
        """Playback için bir adım"""
        if self.is_playing:
            max_time = self.recorder.get_max_time()
            if max_time > 0:
                self.current_time += 0.05 * self.play_speed

                if self.current_time >= max_time:
                    self.current_time = max_time
                    self.stop_playback()

                slider_value = int((self.current_time / max_time) * 1000)
                self.timeline_slider.setValue(slider_value)
                self.update_trajectory_display()

    def update_trajectory_display(self):
        """Trajectory'yi RViz'de göster"""
        points = self.recorder.get_trajectory_at_time(self.current_time)

        current_point = None
        if points:
            current_point = points[-1]

        # Sadece önemli durumlarda log
        point_count = len(points) if points else 0
        if hasattr(self, 'last_logged_count'):
            # Sadece büyük değişikliklerde log
            if abs(point_count - self.last_logged_count) > 50 or point_count == 0:
                print(f"📡 Publishing: {point_count} trajectory points")
                self.last_logged_count = point_count
        else:
            self.last_logged_count = point_count

        # Current marker sadece debug modunda göster (disable for now)
        # if current_point:
        #     print(f"🎯 Current marker: x={current_point['x']:.2f}, y={current_point['y']:.2f}, z={current_point['z']:.2f}")

        # RViz'de göster
        self.recorder.publish_trajectory(points, current_point)

        # Rescue pozisyonlarını da göster
        rescue_positions = self.rescue_system.get_rescue_positions()
        self.recorder.publish_rescue_markers(rescue_positions)

    def update_display(self):
        """GUI bilgilerini güncelle"""
        max_time = self.recorder.get_max_time()

        # Full timeline göster
        self.time_label.setText(f"{self.current_time:.1f}s / {max_time:.1f}s")

        point_count = len(self.recorder.trajectory_points)
        self.point_count_label.setText(f"{point_count}")

        if self.record_button.isChecked() and not self.is_playing:
            self.current_time = max_time
            if max_time > 0:
                slider_value = 1000
                self.timeline_slider.setValue(slider_value)
            self.update_trajectory_display()

        self.update_rescue_display()

    def update_rescue_display(self):
        """Rescue sayısını güncelle"""
        count = len(self.rescue_system.get_rescue_positions())
        self.rescue_count_label.setText(str(count))
        self.view_positions_button.setEnabled(count > 0)

    def on_human_detected(self, image):
        """İnsan tespit edildiğinde çağrılır"""
        try:
            if self.recorder.trajectory_points:
                latest_point = self.recorder.trajectory_points[-1]
                drone_pos = (latest_point['x'], latest_point['y'], latest_point['z'])
            else:
                drone_pos = (0, 0, 0)

            self.detection_status_label.setText("🚨 Human Detected!")
            self.detection_status_label.setStyleSheet("QLabel { color: #e74c3c; font-weight: bold; font-size: 10px; }")

            dialog = RescueDecisionDialog(image, drone_pos, self.recorder.operator_confirm_pub, self)
            result = dialog.exec_()

            if result == QDialog.Accepted and dialog.result:
                self.rescue_system.add_rescue_position(drone_pos[0], drone_pos[1], drone_pos[2])
                self.detection_status_label.setText("✅ Rescue confirmed!")
                self.detection_status_label.setStyleSheet("QLabel { color: #27ae60; font-weight: bold; font-size: 10px; }")
            else:
                self.detection_status_label.setText("❌ No rescue needed")
                self.detection_status_label.setStyleSheet("QLabel { color: #e74c3c; font-weight: bold; font-size: 10px; }")

            QTimer.singleShot(3000, self.reset_detection_status)

        except Exception as e:
            rospy.logwarn(f"Human detection error: {e}")

    def reset_detection_status(self):
        """Detection status'u sıfırla"""
        self.detection_status_label.setText("Waiting for detection...")
        self.detection_status_label.setStyleSheet("QLabel { color: #7f8c8d; font-style: italic; font-size: 10px; }")

    def show_rescue_positions(self):
        """Kaydedilen rescue pozisyonlarını göster"""
        positions = self.rescue_system.get_rescue_positions()

        dialog = QDialog(self)
        dialog.setWindowTitle("📋 Rescue Positions")
        dialog.resize(500, 300)

        layout = QVBoxLayout()
        list_widget = QListWidget()

        for i, pos in enumerate(positions):
            timestamp = pos.get('timestamp', 0)
            item_text = f"Pos {i+1}: ({pos['x']:.2f}, {pos['y']:.2f}, {pos['z']:.2f}) - {timestamp:.1f}s"
            list_widget.addItem(item_text)

        layout.addWidget(list_widget)

        close_button = QPushButton("Close")
        close_button.clicked.connect(dialog.accept)
        layout.addWidget(close_button)

        dialog.setLayout(layout)
        dialog.exec_()

    def test_human_detection(self):
        """Test için manuel insan tespiti tetikle"""
        test_image = self.rescue_system.current_image if self.rescue_system.current_image is not None else self.create_test_image()
        self.on_human_detected(test_image)

    def create_test_image(self):
        """Test görüntüsü oluştur"""
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(test_image, "TEST DETECTION", (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        return test_image

    # Session Management Methods
    def start_new_session(self):
        """Yeni session başlat"""
        session_id = self.recorder.start_new_session()
        self.refresh_sessions_list()
        self.session_combo.setCurrentText("Current Session")
        rospy.loginfo(f"🎬 Started new session: {session_id}")

    def save_current_session(self):
        """Mevcut session'ı kaydet"""
        self.recorder.save_current_session()
        self.refresh_sessions_list()
        rospy.loginfo("💾 Current session saved")

    def refresh_sessions_list(self):
        """Session listesini güncelle"""
        current_text = self.session_combo.currentText()
        self.session_combo.clear()
        self.session_combo.addItem("Current Session")

        sessions = self.recorder.get_sessions_list()
        for session in sessions:
            duration = session['duration']
            session_text = f"{session['id']} ({session['point_count']} pts, {duration:.1f}s)"
            self.session_combo.addItem(session_text)

        # Restore selection if possible
        index = self.session_combo.findText(current_text)
        if index >= 0:
            self.session_combo.setCurrentIndex(index)

    def on_session_changed(self, session_text):
        """Session değiştirildiğinde"""
        if session_text == "Current Session" or not session_text:
            self.delete_session_button.setEnabled(False)
            return

        # Extract session ID from text
        session_id = session_text.split(' (')[0]

        # Load selected session
        if self.recorder.load_session(session_id):
            self.delete_session_button.setEnabled(True)
            # Update display
            self.update_trajectory_display()
            rospy.loginfo(f"📖 Loaded session: {session_id}")
        else:
            self.delete_session_button.setEnabled(False)

    def delete_selected_session(self):
        """Seçili session'ı sil"""
        current_text = self.session_combo.currentText()
        if current_text == "Current Session" or not current_text:
            return

        session_id = current_text.split(' (')[0]

        # Confirm deletion
        reply = QMessageBox.question(self, 'Delete Session',
                                   f'Are you sure you want to delete session {session_id}?',
                                   QMessageBox.Yes | QMessageBox.No,
                                   QMessageBox.No)

        if reply == QMessageBox.Yes:
            if self.recorder.delete_session(session_id):
                self.refresh_sessions_list()
                self.session_combo.setCurrentText("Current Session")
                rospy.loginfo(f"🗑️ Deleted session: {session_id}")
            else:
                QMessageBox.warning(self, 'Error', f'Failed to delete session {session_id}')

    def init_sessions(self):
        """Initialize sessions on startup"""
        self.refresh_sessions_list()


def main():
    """Standalone çalıştırma için"""
    # ROS Node başlat
    rospy.init_node('trajectory_panel', anonymous=True)

    # Qt Application
    import sys
    try:
        from python_qt_binding.QtWidgets import QApplication
    except ImportError:
        from PyQt5.QtWidgets import QApplication

    app = QApplication(sys.argv)

    # Ana panel'i oluştur
    panel = TrajectoryRVizPanel()
    panel.setWindowTitle("🚁 Firefly Trajectory Control Panel")
    panel.resize(400, 600)
    panel.show()

    # Qt event loop'u başlat
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()