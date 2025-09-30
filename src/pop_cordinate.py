#!/usr/bin/env python3
import rospy
import yaml
import math
import sys
import os

# YAML dosya yolu
YAML_FILE = "/home/furkan/catkin_ws_staj/src/firefly_control/data/detected_buildings.yaml"

def load_and_process_first_building():
    if not os.path.exists(YAML_FILE):
        rospy.logwarn(f"{YAML_FILE} bulunamadı.")
        return None

    # Dosyayı oku
    with open(YAML_FILE, 'r') as f:
        data = yaml.safe_load(f) or {}

    if not data:
        rospy.logwarn("YAML dosyası boş.")
        return None

    # İlk building'i al (sıra key sırasına göre)
    first_building_id = sorted(data.keys())[0]
    building = data[first_building_id]

    # Koordineleri al
    x0 = building['x0']
    y0 = building['y0']
    x1 = building['x1']
    y1 = building['y1']

    rospy.loginfo(f"Processed building {first_building_id}: ({x0}, {y0}) - ({x1}, {y1})")

    # Dosyadan ilk elemanı sil
    data.pop(first_building_id)
    with open(YAML_FILE, 'w') as f:
        yaml.safe_dump(data, f)

    return x0, y0, x1, y1

if __name__ == "__main__":
    rospy.init_node("pop_cordinate_node")
    rospy.loginfo("pop_cordinate node started.")

    coords = load_and_process_first_building()
    if coords:
        x0, y0, x1, y1 = coords
        rospy.loginfo(f"Coordinates set: x0={x0}, y0={y0}, x1={x1}, y1={y1}")

        # 🔹 Param server'a yaz
        rospy.set_param("/building_coords", {
            "x0": x0, "y0": y0,
            "x1": x1, "y1": y1
        })
        rospy.loginfo("Coordinates written to /building_coords")
        sys.exit(0)
    else:
        rospy.logwarn("No building to process. Deleting /building_coords param if exists.")
        if rospy.has_param("/building_coords"):
            rospy.delete_param("/building_coords")
            rospy.loginfo("/building_coords param deleted.")
        sys.exit(1)
