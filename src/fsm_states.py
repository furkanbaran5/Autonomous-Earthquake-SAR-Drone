#!/usr/bin/env python3
import rospy
import smach
import smach_ros
import subprocess
import os
from std_msgs.msg import String


# ------------------- GLOBAL -------------------
fsm_command = None
building_coords = None  

def command_callback(msg):
    global fsm_command
    fsm_command = msg.data
    rospy.loginfo(f"[FSM] Komut alındı: {msg.data}")

# ------------------- STATES -------------------

class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['sfs'])

    def execute(self, userdata):
        rospy.loginfo("[FSM] START - 15 saniye bekleniyor...")
        rospy.sleep(15)
        rospy.loginfo("[FSM] START tamamlandı, SFS başlıyor")
        return 'sfs'

class SFS(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ascend'])
        self.command_pub = rospy.Publisher('/mission/command', String, queue_size=1, latch=True)
        self.altitude_proc = None
        self.detector_proc = None

    def execute(self, userdata):
        rospy.loginfo("[FSM] SFS - altitude_scanner node başlatılıyor...")
        # altitude_scanner node başlat
        self.altitude_proc = subprocess.Popen(
            ["rosrun", "firefly_control", "altitude_scanner.py"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        rospy.sleep(5)

        rospy.loginfo("[FSM] SFS - building_detector node başlatılıyor...")
        # building_detector node başlat
        self.detector_proc = subprocess.Popen(
            ["rosrun", "firefly_control", "building_detector.py"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        # 5 saniye bekle
        rospy.loginfo("[FSM] SFS - node’lar açıldı, 5 saniye bekleniyor...")

        rospy.loginfo("[FSM] SFS - start_ascent komutu gönderiliyor")
        self.command_pub.publish("start_ascent")
        return 'ascend'


class Ascending(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['scanning'])
        self.altitude_reached = False
        self.command_pub = rospy.Publisher('/mission/command', String, queue_size=1, latch=True)
        rospy.Subscriber('/altitude_scanner/status', String, self.altitude_cb)

    def altitude_cb(self, msg):
        if msg.data == "altitude_reached":
            self.altitude_reached = True

    def execute(self, userdata):
        rospy.loginfo("[FSM] ASCENDING - Yükseklik bekleniyor...")
        rate = rospy.Rate(10)
        while not self.altitude_reached and not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo("[FSM] ASCENDING tamamlandı, start_scan komutu gönderiliyor")
        self.command_pub.publish("start_scan")
        return 'scanning'


class Scanning(smach.State):
    def __init__(self, start_first_topics):
        smach.State.__init__(self, outcomes=['detecting'])
        self.scan_completed = False
        self.command_pub = rospy.Publisher('/mission/command', String, queue_size=1, latch=True)
        self.start_first_topics = start_first_topics  # SFS state referansı
        rospy.Subscriber('/altitude_scanner/status', String, self.scan_cb)

    def scan_cb(self, msg):
        if msg.data == "scan_completed":
            self.scan_completed = True

    def execute(self, userdata):
        rospy.loginfo("[FSM] SCANNING - Tarama bekleniyor...")
        rate = rospy.Rate(10)
        while not self.scan_completed and not rospy.is_shutdown():
            rate.sleep()

        rospy.loginfo("[FSM] SCANNING tamamlandı, start_detection komutu gönderiliyor")
        self.command_pub.publish("start_detection")

        return 'detecting'


class Detecting(smach.State):
    def __init__(self, start_first_topics):
        smach.State.__init__(self, outcomes=['done'])
        self.detection_completed = False
        self.start_first_topics = start_first_topics
        self.command_pub = rospy.Publisher('/mission/command', String, queue_size=1, latch=True)
        rospy.Subscriber('/building_detector/status', String, self.detect_cb)

    def detect_cb(self, msg):
        if msg.data == "detection_completed":
            self.detection_completed = True

    def execute(self, userdata):
        rospy.loginfo("[FSM] DETECTING - Bina tespiti bekleniyor...")
        rate = rospy.Rate(10)
        while not self.detection_completed and not rospy.is_shutdown():
            rate.sleep()

        # building_detector node kapat
        if self.start_first_topics.detector_proc:
            self.start_first_topics.detector_proc.terminate()
            rospy.loginfo("[FSM] building_detector node kapatıldı.")

           # altitude_scanner node’u kapat
        if self.start_first_topics.altitude_proc:
            self.start_first_topics.altitude_proc.terminate()
            rospy.loginfo("[FSM] altitude_scanner node kapatıldı.")

        rospy.loginfo("[FSM] DETECTING tamamlandı, EXPLORE başlıyor")
        return 'done'

class SmartBuildingSelectorState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'failed'])
        self.command_pub = rospy.Publisher('/mission/command', String, queue_size=1, latch=True)

    def execute(self, userdata):
        rospy.loginfo("[FSM] SMART_BUILDING_SELECTOR state started.")

        global rescue_logger_proc
        try:
            # smart_building_selector node'u başlat
            proc = subprocess.Popen(
                ["rosrun", "firefly_control", "smart_building_selector.py"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            rospy.loginfo("[FSM] smart_building_selector node başlatıldı.")

            # rescue_data_logger node'u başlat
            rescue_logger_proc = subprocess.Popen(
                ["rosrun", "firefly_control", "rescue_data_logger.py"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            rospy.loginfo("[FSM] rescue_data_logger node başlatıldı.")

            rospy.sleep(1)

            # Komutu gönder
            rospy.loginfo("[FSM] start_smart_building_selection komutu gönderiliyor")
            self.command_pub.publish("start_smart_building_selection")

            # Node kapanana kadar bekle
            rate = rospy.Rate(10)
            while proc.poll() is None and not rospy.is_shutdown():
                rate.sleep()

            rospy.loginfo("[FSM] smart_building_selector node tamamlandı.")
            return 'done'

        except Exception as e:
            rospy.logerr(f"[FSM] smart_building_selector node başlatılırken hata: {e}")
            return 'failed'

class PopCoordinate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'failed'])

    def execute(self, userdata):
        rospy.loginfo("[FSM] PopCoordinate state started.")

        # pop_cordinate.py çalıştır
        try:
            subprocess.run(
                ["rosrun", "firefly_control", "pop_cordinate.py"],
                check=True
            )
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"[FSM] pop_cordinate.py hata ile kapandı: {e}")
            return 'failed'

        # Param server’dan oku
        if rospy.has_param("/building_coords"):
            coords = rospy.get_param("/building_coords")
            rospy.loginfo(f"[FSM] Koordinatlar alındı: {coords}")

            # 🔹 Global değişkene yaz
            global building_coords
            building_coords = coords

            return 'done'
        else:
            rospy.logwarn("[FSM] Param server’da /building_coords yok.")
            return 'failed'

class MoveCornerBuild(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'failed'])

    def execute(self, userdata):
        global building_coords
        rospy.loginfo("[FSM] MoveCornerBuild state started.")

        if building_coords is None:
            rospy.logwarn("[FSM] building_coords alınmamış, state başarısız.")
            return 'failed'

        try:
            x0 = building_coords['x1']
            y0 = building_coords['y1']
            rospy.loginfo(f"[FSM] move_corner_build node başlatılıyor: x0={x0}, y0={y0}")

            # Node'u başlat
            proc = subprocess.Popen(
                ["rosrun", "firefly_control", "move_corner_build", "_x:=" + str(x0), "_y:=" + str(y0)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

            # Node kapanana kadar bekle
            rate = rospy.Rate(10)
            while proc.poll() is None and not rospy.is_shutdown():
                rate.sleep()

            rospy.loginfo("[FSM] move_corner_build node kapandı, hedefe ulaşıldı.")
            return 'done'

        except KeyError as e:
            rospy.logerr(f"[FSM] building_coords eksik key: {e}")
            return 'failed'

class Turn360(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'failed'])

    def execute(self, userdata):
        global building_coords
        rospy.loginfo("[FSM] Turn360 state started.")

        if building_coords is None:
            rospy.logwarn("[FSM] building_coords alınmamış, Turn360 state başarısız.")
            return 'failed'

        x0 = building_coords.get('x1', 0.0)
        y0 = building_coords.get('y1', 0.0)
        x1 = building_coords.get('x0', 0.0)
        y1 = building_coords.get('y0', 0.0)

        try:
            rospy.loginfo(f"[FSM] turn_360 node başlatılıyor: x0={x0}, y0={y0}, x1={x1}, y1={y1}")
            proc = subprocess.Popen(
                ["rosrun", "firefly_control", "turn_360", "_x0:=" + str(x0), "_y0:=" + str(y0),
                 "_x1:=" + str(x1), "_y1:=" + str(y1)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

            rate = rospy.Rate(10)
            while proc.poll() is None and not rospy.is_shutdown():
                rate.sleep()

            rospy.loginfo("[FSM] turn_360 node tamamlandı.")
            return 'done'

        except Exception as e:
            rospy.logerr(f"[FSM] turn_360 node başlatılırken hata: {e}")
            return 'failed'

class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'failed'])

    def execute(self, userdata):
        rospy.loginfo("[FSM] EXPLORE state started.")

        # Parametreleri al
        xmin_allowed = building_coords.get('x0', 0.0)
        xmax_allowed = building_coords.get('x1', 0.0)
        ymin_allowed = building_coords.get('y0', 0.0)
        ymax_allowed = building_coords.get('y1', 0.0)
        rospy.loginfo(f"[FSM] Params: xmin={xmin_allowed}, xmax={xmax_allowed}, ymin={ymin_allowed}, ymax={ymax_allowed}")

        fd_proc = None
        fp_proc = None
        cp_proc = None
        yolo_proc = None

        try:
            # frontier_detection başlat
            fd_proc = subprocess.Popen(
                [
                    "rosrun", "firefly_control", "frontier_detection",
                    "_xmin_allowed:=" + str(xmin_allowed),
                    "_xmax_allowed:=" + str(xmax_allowed),
                    "_ymin_allowed:=" + str(ymin_allowed),
                    "_ymax_allowed:=" + str(ymax_allowed)
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

            # follow_path başlat
            fp_proc = subprocess.Popen(
                ["rosrun", "firefly_control", "follow_path"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

            # yolov5.launch başlat
            yolo_proc = subprocess.Popen(
                ["roslaunch", "yolov5_ros", "yolov5.launch"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=os.environ.copy()
            )

            # 10 saniye bekle
            rospy.sleep(10)

            # create_path çalıştır
            cp_proc = subprocess.Popen(
                ["rosrun", "firefly_control", "create_path"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

            # create_path bitene kadar bekle
            rate = rospy.Rate(10)
            while cp_proc.poll() is None and not rospy.is_shutdown():
                rate.sleep()

            rospy.loginfo("[FSM] EXPLORE state tamamlandı, node'lar kapatılıyor")

            # Açılan tüm node’ları kapat
            for proc, name in [(fd_proc, "frontier_detection"), (fp_proc, "follow_path"), (yolo_proc, "yolov5")]:
                if proc:
                    proc.terminate()
                    rospy.loginfo(f"[FSM] {name} node kapatıldı.")

            return 'done'

        except Exception as e:
            rospy.logerr(f"[FSM] EXPLORE node başlatılırken hata: {e}")
            # Hata durumunda da node’ları kapat
            for proc in [fd_proc, fp_proc, yolo_proc]:
                if proc:
                    proc.terminate()
            return 'failed'


class AscendAgain(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo("[FSM] ASCEND_AGAIN - up komutu çalıştırılıyor...")
        try:
            subprocess.run(
                ["rosrun", "firefly_control", "up"],
                check=True
            )
            rospy.loginfo("[FSM] ASCEND_AGAIN tamamlandı")
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"[FSM] up komutu hata ile kapandı: {e}")
        return 'done'

        
class ReturnHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['land'])

    def execute(self, userdata):
        rospy.loginfo("[FSM] State: RETURN_HOME")
        rospy.sleep(2)
        return 'land'


class Land(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo("[FSM] State: LAND")

        global rescue_logger_proc
        if rescue_logger_proc:
            rescue_logger_proc.terminate()
            rospy.loginfo("[FSM] rescue_data_logger node kapatıldı.")

        # rescue_web node'u başlat
        try:
            subprocess.Popen(
                ["rosrun", "firefly_control", "rescue_web.py"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            rospy.loginfo("[FSM] rescue_web node başlatıldı.")
        except Exception as e:
            rospy.logerr(f"[FSM] rescue_web node başlatılırken hata: {e}")

        rospy.sleep(2)
        return 'done'


# ------------------- MAIN -------------------
if __name__ == '__main__':
    rospy.init_node('fsm_full_example')

    # FSM komutları dinle
    rospy.Subscriber('/fsm_command', String, command_callback)

    # --- Launch dosyası FSM başlamadan sessiz açılıyor ---
    launch_proc = subprocess.Popen(
        ["roslaunch", "rotors_gazebo", "mav_hovering_example_with_vi_sensor.launch"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env=os.environ.copy()
    )
    rospy.loginfo("[FSM] Global launch dosyası sessiz şekilde başlatıldı")

    # FSM setup
    sm = smach.StateMachine(outcomes=['done'])
    start_first_topics = SFS()
    with sm:
        smach.StateMachine.add('START', Start(), transitions={'sfs': 'SFS'})
        smach.StateMachine.add('SFS', start_first_topics, transitions={'ascend': 'ASCENDING'})
        smach.StateMachine.add('ASCENDING', Ascending(), transitions={'scanning': 'SCANNING'})
        smach.StateMachine.add('SCANNING', Scanning(start_first_topics), transitions={'detecting': 'DETECTING'})
        smach.StateMachine.add('DETECTING', Detecting(start_first_topics), transitions={'done': 'SMART_BUILDING_SELECTOR'})
        smach.StateMachine.add('SMART_BUILDING_SELECTOR', SmartBuildingSelectorState(),transitions={'done': 'POP_COORDINATE', 'failed':'RETURN_HOME'})
        smach.StateMachine.add('POP_COORDINATE', PopCoordinate(), transitions={'done':'MOVE_CORNER_BUILD', 'failed':'RETURN_HOME'})
        smach.StateMachine.add('MOVE_CORNER_BUILD', MoveCornerBuild(), transitions={'done':'TURN_360', 'failed':'RETURN_HOME'})
        smach.StateMachine.add('TURN_360', Turn360(), transitions={'done':'EXPLORE', 'failed':'RETURN_HOME'})
        smach.StateMachine.add('EXPLORE', Explore(), transitions={'done':'ASCEND_AGAIN', 'failed':'RETURN_HOME'})
        smach.StateMachine.add('ASCEND_AGAIN', AscendAgain(), transitions={'done':'POP_COORDINATE'})
        smach.StateMachine.add('RETURN_HOME', ReturnHome(), transitions={'land': 'LAND'})
        smach.StateMachine.add('LAND', Land(), transitions={'done': 'done'})

    # FSM Viewer
    sis = smach_ros.IntrospectionServer('fsm_server', sm, '/SM_ROOT')
    sis.start()

    try:
        outcome = sm.execute()
    except rospy.ROSInterruptException:
        rospy.loginfo("[FSM] FSM ROS kapandığı için durduruldu")

    # FSM bittiğinde launch kapatılıyor
    launch_proc.terminate()
    rospy.loginfo("[FSM] Global launch dosyası kapatıldı")

    rospy.spin()
    sis.stop()
